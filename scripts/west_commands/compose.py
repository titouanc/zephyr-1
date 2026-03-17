# Copyright (c) 2026 Titouan Christophe
#
# SPDX-License-Identifier: Apache-2.0

import argparse
import logging
import os
import re
import ipaddress
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from threading import RLock
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml
from tqdm import tqdm

from west.commands import WestCommand
from west import log
from west.util import west_topdir, WestNotFound

from zephyr_ext_common import ZEPHYR_BASE

# Default IP allocation for apps on a network
MAC_PREFIX = "00:00:5E:00:53"


@dataclass
class NetworkConfig:
    name: str
    host_veth: bool
    ipv4: Optional[ipaddress.IPv4Network] = ipaddress.ip_network("192.0.2.0/24")
    ipv6: Optional[ipaddress.IPv6Network] = ipaddress.ip_network("2001:db8::/56")


@dataclass
class ApplicationConfig:
    name: str
    source: Path
    network: Optional[str]
    board: str = "native_sim"
    extra_build_snippets: List[str] = field(default_factory=list)
    extra_build_args: List[str] = field(default_factory=list)
    extra_run_args: List[str] = field(default_factory=list)


@dataclass
class AppNetworkInfo:
    network: str
    iface: str
    mac: str
    ipv4: Optional[str] = None
    ipv6: Optional[str] = None


@dataclass
class ComposeContext:
    name: str
    fs_name: str
    networks: Dict[str, NetworkConfig]
    applications: Dict[str, ApplicationConfig]
    app_list: List[str]
    app_network_info: Dict[str, AppNetworkInfo]
    topdir: Path
    compose_path: Path


def _load_compose_file(path: Path) -> dict:
    if not path.is_file():
        raise FileNotFoundError(f"Compose file not found: {path}")
    with open(path) as f:
        data = yaml.safe_load(f)
    if not data:
        raise ValueError("Compose file is empty")
    return data


def _resolve_source(topdir: Path, source: str) -> Path:
    """Resolve application source path (relative to workspace or absolute)."""
    p = Path(source)
    if not p.is_absolute():
        p = topdir / p
    return p.resolve()


def _sanitize_path_component(value: str, *, default: str) -> str:
    """
    Convert an arbitrary name into a safe, space-free path component.
    Keeps [A-Za-z0-9._-], converts everything else (including whitespace) to '_'.
    """
    s = str(value or "").strip()
    s = re.sub(r"\s+", "_", s)
    s = re.sub(r"[^A-Za-z0-9._-]+", "_", s)
    s = s.strip("._-")
    return s or default


def _parse_compose(data: dict, compose_path: Path) -> tuple[str, Dict[str, NetworkConfig], Dict[str, ApplicationConfig], List[str]]:
    """
    Parse compose dict. Returns (name, networks, applications).
    networks: { net_name: { "host_veth": bool } }
    applications: {
        app_name: {
            "source": Path,
            "network": Optional[str],
            "extra_build": dict,
        }
    }
    """
    name = data.get("name", "compose")
    networks = data.get("networks") or {}
    apps = data.get("applications") or {}
    fs_name = _sanitize_path_component(name, default="compose")

    net_config: Dict[str, NetworkConfig] = {}
    for net_name, net_cfg in networks.items():
        # Network options:
        # - host-veth: bool
        # - ipv4: CIDR string (e.g. "192.0.2.0/24") or false to disable IPv4
        # - ipv6: CIDR string (e.g. "2001:db8::/64") or false to disable IPv6

        if net_cfg is None:
            net_cfg = {}

        opts = {
            "name": net_name,
            "host_veth": bool(net_cfg.get("host-veth", False)),
        }

        if "ipv4" in net_cfg:
            opts["ipv4"] = None if net_cfg["ipv4"] is False else ipaddress.ip_network(net_cfg["ipv4"])
        if "ipv6" in net_cfg:
            opts["ipv6"] = None if net_cfg["ipv6"] is False else ipaddress.ip_network(net_cfg["ipv6"])

        net_config[net_name] = NetworkConfig(**opts)

    app_list: List[str] = list(apps.keys())
    app_config: Dict[str, ApplicationConfig] = {}
    for app_name, app_cfg in apps.items():
        if app_cfg is None:
            app_cfg = {}
        src = app_cfg.get("source")
        if not src:
            raise ValueError(f"application '{app_name}' has no 'source'")
        net_name = app_cfg.get("network")
        board = app_cfg.get("board") or "native_sim"

        extra_build = app_cfg.get("extra-build") or {}
        extra_run = app_cfg.get("extra-run") or {}

        extra_build_args_raw = extra_build.get("args", [])
        if extra_build_args_raw is None:
            extra_build_args_raw = []
        if not isinstance(extra_build_args_raw, list):
            raise ValueError(
                f"application '{app_name}' field 'extra-build.args' must be a list"
            )
        if any(not isinstance(a, str) for a in extra_build_args_raw):
            raise ValueError(
                f"application '{app_name}' field 'extra-build.args' must be a list of strings"
            )

        extra_run_args_raw = extra_run.get("args", [])
        if extra_run_args_raw is None:
            extra_run_args_raw = []
        if not isinstance(extra_run_args_raw, list):
            raise ValueError(
                f"application '{app_name}' field 'extra-run.args' must be a list"
            )
        if any(not isinstance(a, str) for a in extra_run_args_raw):
            raise ValueError(
                f"application '{app_name}' field 'extra-run.args' must be a list of strings"
            )

        extra_build_config = extra_build.get("config", {})
        if extra_build_config is None:
            extra_build_config = {}
        if not isinstance(extra_build_config, dict):
            raise ValueError(
                f"application '{app_name}' field 'extra-build.config' must be a mapping"
            )
        if any(not isinstance(k, str) for k in extra_build_config.keys()):
            raise ValueError(
                f"application '{app_name}' field 'extra-build.config' keys must be strings"
            )
        extra_build_config_args = [f"-D{k}={v}" for k, v in extra_build_config.items()]

        app_config[app_name] = ApplicationConfig(
            name=app_name,
            source=Path(src),
            network=net_name,
            board=board,
            extra_build_snippets=extra_build.get("snippets") or [],
            extra_build_args=list(extra_build_args_raw) + extra_build_config_args,
            extra_run_args=list(extra_run_args_raw),
        )

    return name, fs_name, net_config, app_config, app_list


def _assign_network_info(
    net_config: Dict[str, NetworkConfig],
    app_config: Dict[str, ApplicationConfig],
    app_list: List[str],
) -> Dict[str, AppNetworkInfo]:
    """
    Assign interface names, IPv4, IPv6, MAC per application.
    Each application may be connected to at most a single network.
    Returns: {
        app_name: {
            "network": Optional[str],
            "iface": str,
            "ipv4": str,
            "ipv6": str,
            "mac": str,
        }
    }
    """
    # For each network, assign indices to apps that use it. Each application
    # may be attached to at most a single network (singular "network" key).
    net_app_indices: Dict[str, List[tuple[str, int]]] = {}  # net_name -> [ (app_name, index) ]
    for net_name in net_config:
        idx = 0
        net_app_indices[net_name] = []
        for app_name in app_list:
            app_net = app_config[app_name].network
            if app_net == net_name:
                net_app_indices[net_name].append((app_name, idx))
                idx += 1

    info: Dict[str, AppNetworkInfo] = {}
    for app_name in app_list:
        app = app_config[app_name]
        app_net = app.network
        if not app_net:
            continue
        pairs = net_app_indices.get(app_net, [])
        idx = next((i for a, i in pairs if a == app_name), 0)
        # Interface naming: {net}tap{idx} for TAP, {net}br for bridge, {net}veth0 for veth
        iface = f"{app_net}tap{idx}"
        net_opts = net_config[app_net]
        ipv4_net = net_opts.ipv4
        ipv6_net = net_opts.ipv6
        # Use stdlib ipaddress for deterministic, validated allocation
        ipv4_host = None
        ipv6_host = None
        if ipv4_net is not None:
            try:
                ipv4_host = ipaddress.ip_address(
                    int(ipv4_net.network_address) + (idx + 2)
                )
            except ValueError as exc:
                raise ValueError(
                    f"IPv4 address allocation overflow for app '{app_name}' on network '{app_net}'"
                ) from exc
            if ipv4_host not in ipv4_net:
                raise ValueError(
                    f"Allocated IPv4 {ipv4_host} for app '{app_name}' is outside {ipv4_net}"
                )

        if ipv6_net is not None:
            try:
                ipv6_host = ipaddress.ip_address(
                    int(ipv6_net.network_address) + (idx + 2)
                )
            except ValueError as exc:
                raise ValueError(
                    f"IPv6 address allocation overflow for app '{app_name}' on network '{app_net}'"
                ) from exc
            if ipv6_host not in ipv6_net:
                raise ValueError(
                    f"Allocated IPv6 {ipv6_host} for app '{app_name}' is outside {ipv6_net}"
                )

        mac = f"{MAC_PREFIX}:{idx + 2:02x}"
        info[app_name] = AppNetworkInfo(
            network=app_net,
            iface=iface,
            mac=mac,
            ipv4=str(ipv4_host) if ipv4_host is not None else None,
            ipv6=str(ipv6_host) if ipv6_host is not None else None,
        )
    return info


def _substitute_vars(value: str, app_network_info: Dict[str, AppNetworkInfo]) -> str:
    """Replace ${app:ipv4}, ${app:ipv6}, ${app:mac} in value."""
    if not isinstance(value, str):
        return value

    def repl(m):
        app, key = m.group(1), m.group(2)
        data = app_network_info.get(app)
        if not data:
            return m.group(0)
        return getattr(data, key, m.group(0)) or m.group(0)

    return re.sub(r"\$\{([^:]+):(ipv4|ipv6|mac)\}", repl, value)


def _substitute_arg_list(args: List[str], app_network_info: Dict[str, AppNetworkInfo]) -> List[str]:
    return [_substitute_vars(a, app_network_info) if isinstance(a, str) else a for a in (args or [])]


class Compose(WestCommand):
    def __init__(self):
        super().__init__(
            "compose",
            "manage multi-app Zephyr contexts defined in west-compose.yml",
            "Build, run, and manage networks for multiple Zephyr applications.",
            accepts_unknown_args=False,
        )

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(
            self.name,
            help=self.help,
            description=self.description,
        )
        parser.add_argument(
            "-f",
            "--file",
            default="west-compose.yml",
            metavar="FILE",
            help="Path to compose file (default: west-compose.yml)",
        )
        subparsers = parser.add_subparsers(dest="subcmd", help="Subcommand", required=True)

        def add_app_arg(p):
            p.add_argument("application", nargs="?", help="Application name")

        p = subparsers.add_parser("show", help="Show context and per-app network info")
        add_app_arg(p)
        p = subparsers.add_parser("up", help="Bring up networks (bridge, TAP, veth)")
        p = subparsers.add_parser("down", help="Tear down networks")
        p = subparsers.add_parser("build", help="Build all applications")
        add_app_arg(p)
        p = subparsers.add_parser("run", help="Build (if needed) and run all applications")
        add_app_arg(p)
        return parser

    def _compose_path(self, args) -> Path:
        return Path(args.file).resolve()

    def _load_context(self, args) -> ComposeContext:
        path = self._compose_path(args)
        data = _load_compose_file(path)
        name, fs_name, net_config, app_config, app_list = _parse_compose(data, path)
        compose_dir = path.parent
        try:
            topdir = Path(west_topdir(Path.cwd()))
        except WestNotFound:
            topdir = Path.cwd()
        # Resolve source paths
        for app in app_config.values():
            app.source = _resolve_source(compose_dir, str(app.source))
        app_network_info = _assign_network_info(net_config, app_config, app_list)
        return ComposeContext(
            name=name,
            fs_name=fs_name,
            networks=net_config,
            applications=app_config,
            app_list=app_list,
            app_network_info=app_network_info,
            topdir=topdir,
            compose_path=path,
        )

    def do_run(self, args, _):
        match args.subcmd:
            case "show":
                self._cmd_show(args)
            case "up":
                self._cmd_up(args)
            case "down":
                self._cmd_down(args)
            case "build":
                self._cmd_build(args)
            case "run":
                self._cmd_run(args)
            case _:
                self.die(f"Unknown subcommand: {args.subcmd}")

    def _get_apps_and_nets_to_process(self, ctx: ComposeContext, args) -> tuple[List[str], List[str]]:
        app_list = ctx.app_list
        net_list = list(ctx.networks.keys())

        if hasattr(args, "application") and args.application:
            app_name = args.application
            if app_name not in ctx.applications:
                self.die(f"Application '{app_name}' not found in compose file.")

            app_list = [app_name]
            app_cfg = ctx.applications.get(app_name)
            if app_cfg and app_cfg.network:
                net_list = [app_cfg.network]
            else:
                net_list = []

        return app_list, net_list

    def _cmd_show(self, args):
        ctx = self._load_context(args)
        app_list, _ = self._get_apps_and_nets_to_process(ctx, args)
        self.inf(f"Context {ctx.name}")
        board = "native_sim"
        for app_name in app_list:
            app = ctx.applications[app_name]
            src = app.source
            try:
                rel = src.relative_to(ctx.topdir)
            except ValueError:
                rel = src
            self.inf(f" - {app_name} ({board} - {rel})")
            ninfo = ctx.app_network_info.get(app_name)
            if ninfo and ninfo.network:
                net_name = ninfo.network
                net_opts = ctx.networks[net_name]
                parts = []
                ipv4_host = ninfo.ipv4
                ipv4_net = net_opts.ipv4
                if ipv4_host and ipv4_net is not None:
                    parts.append(f"{ipv4_host}/{ipv4_net.prefixlen}")
                ipv6_host = ninfo.ipv6
                ipv6_net = net_opts.ipv6
                if ipv6_host and ipv6_net is not None:
                    parts.append(f"{ipv6_host}/{ipv6_net.prefixlen}")
                addr_str = " ".join(parts) if parts else "-"
                self.inf(
                    f"    - ({net_name}) {ninfo.iface} :: [{ninfo.mac}] {addr_str}"
                )

    def _run_ip(self, cmd: list, check: bool = True) -> None:
        cmdline = ["sudo", "ip", *cmd]
        log.dbg("Running:", " ".join(cmdline))
        subprocess.check_call(cmdline)

    def _cmd_up(self, args):
        ctx = self._load_context(args)
        _, net_list = self._get_apps_and_nets_to_process(ctx, args)

        log.inf(f"Bringing up context {ctx.name}")
        for net_name in net_list:
            net_cfg = ctx.networks[net_name]
            bridge = f"{net_name}br"
            # Create bridge
            log.inf(f"Creating BRIDGE interface {bridge}")
            self._run_ip(["link", "add", "name", bridge, "type", "bridge"], check=False)
            self._run_ip(["link", "set", bridge, "up"])
            # TAP per app on this network
            for app_name in ctx.app_list:
                ninfo = ctx.app_network_info.get(app_name)
                if not ninfo or ninfo.network != net_name:
                    continue
                iface = ninfo.iface
                ipv4 = ninfo.ipv4
                ipv6 = ninfo.ipv6
                log.inf(f"Creating TAP interface {iface} bridged to {bridge}")
                self._run_ip(["tuntap", "add", iface, "mode", "tap"], check=False)
                self._run_ip(["link", "set", iface, "master", bridge], check=False)
                net_opts = net_cfg
                if ipv4 and net_opts.ipv4 is not None:
                    self._run_ip(
                        [
                            "addr",
                            "add",
                            f"{ipv4}/{net_opts.ipv4.prefixlen}",
                            "dev",
                            iface,
                        ]
                    )
                if ipv6 and net_opts.ipv6 is not None:
                    self._run_ip(
                        [
                            "addr",
                            "add",
                            f"{ipv6}/{net_opts.ipv6.prefixlen}",
                            "dev",
                            iface,
                        ]
                    )
                self._run_ip(["link", "set", iface, "up"])
            if net_cfg.host_veth:
                veth = f"{net_name}veth0"
                log.inf(f"Creating VETH interface {veth} bridged to {bridge}")
                self._run_ip(["link", "add", veth, "type", "veth", "peer", "name", veth + "p"], check=False)
                self._run_ip(["link", "set", veth, "master", bridge])
                self._run_ip(["link", "set", veth, "up"])
                self._run_ip(["link", "set", veth + "p", "up"])
            log.inf(f"Network {net_name} up")

    def _cmd_down(self, args):
        ctx = self._load_context(args)
        _, net_list = self._get_apps_and_nets_to_process(ctx, args)

        for net_name in reversed(net_list):
            net_cfg = ctx.networks[net_name]
            bridge = f"{net_name}br"
            if net_cfg.host_veth:
                veth = f"{net_name}veth0"
                log.inf(f"Removing VETH interface {veth}")
                self._run_ip(["link", "del", veth], check=False)
            for app_name in ctx.app_list:
                ninfo = ctx.app_network_info.get(app_name)
                if not ninfo or ninfo.network != net_name:
                    continue
                iface = ninfo.iface
                log.inf(f"Removing TAP interface {iface}")
                self._run_ip(["link", "del", iface], check=False)
            log.inf(f"Removing BRIDGE interface {bridge}")
            self._run_ip(["link", "del", bridge], check=False)

    def _build_dir(self, ctx: ComposeContext, app_name: str) -> Path:
        safe_app = _sanitize_path_component(app_name, default="app")
        return ctx.compose_path.parent / ".compose" / ctx.fs_name / safe_app

    def _cmd_build(self, args):
        ctx = self._load_context(args)
        app_list, _ = self._get_apps_and_nets_to_process(ctx, args)
        log.inf(f"Building applications for context {ctx.name}")
        self._build_all(ctx, app_list)

    def _build_all(self, ctx: ComposeContext, app_list: Optional[List[str]] = None):
        if app_list is None:
            app_list = ctx.app_list
        topdir = ctx.topdir

        tqdm.set_lock(RLock())

        def build_one(app_name: str, bar: tqdm) -> tuple[str, int, str]:
            app = ctx.applications[app_name]
            source = app.source
            if not source.is_dir():
                return app_name, 2, f"Source not found: {source}"

            build_dir = self._build_dir(ctx, app_name)
            build_dir.mkdir(parents=True, exist_ok=True)
            app_board = app.board or "native_sim"

            # West build options before '--'
            snippet_args: List[str] = []
            for snippet in app.extra_build_snippets:
                snippet_args.extend(["--snippet", snippet])

            cmake_args = _substitute_arg_list(app.extra_build_args, ctx.app_network_info)
            cmd = [
                "west",
                "build",
                "--board",
                app_board,
                "--build-dir",
                str(build_dir),
                str(source),
                *snippet_args,
                "--",
                *cmake_args,
            ]
            log.dbg("Running:", " ".join(cmd))
            out_lines: List[str] = []
            last_step: int | None = None
            p = subprocess.Popen(
                cmd,
                cwd=str(topdir),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            assert p.stdout is not None
            try:
                for line in iter(p.stdout.readline, ""):
                    if not line:
                        break
                    out_lines.append(line)
                    m = re.search(r"\[(\d+)\s*/\s*(\d+)\]", line)
                    if m:
                        step = int(m.group(1))
                        total = int(m.group(2))
                        if bar.total != total:
                            bar.total = total
                        if last_step is None or step >= last_step:
                            bar.n = step
                            last_step = step
                        bar.refresh()
                    else:
                        # Fallback for phases which don't emit [x/y]
                        if bar.total is None:
                            bar.update(1)
                rc = p.wait()
            finally:
                try:
                    p.stdout.close()
                except Exception:
                    pass
                bar.close()
            return app_name, rc, "".join(out_lines)

        max_workers = min(len(app_list) or 1, os.cpu_count() or 1)
        failures: List[tuple[str, int, str]] = []
        bars = [
            tqdm(
                total=None,
                desc=app_name,
                position=i,
                leave=True,
                unit="step",
                dynamic_ncols=True,
            )
            for i, app_name in enumerate(app_list)
        ]
        with ThreadPoolExecutor(max_workers=max_workers) as ex:
            futures = {
                ex.submit(build_one, app_name, bars[i]): app_name
                for i, app_name in enumerate(app_list)
            }
            for fut in as_completed(futures):
                app_name, rc, out = fut.result()
                if rc != 0:
                    failures.append((app_name, rc, out))

        if failures:
            for app_name, rc, out in failures:
                log.err(f"Build failed for {app_name} (exit code {rc})")
                if out.strip():
                    log.err(out.rstrip())
            self.die(f"{len(failures)}/{len(app_list)} builds failed")

    def _exe_for_app(self, ctx: ComposeContext, app_name: str) -> Path | None:
        build_dir = self._build_dir(ctx, app_name)
        runners_yaml = build_dir / "zephyr" / "runners.yaml"
        if not runners_yaml.is_file():
            return None
        try:
            with open(runners_yaml) as f:
                data = yaml.safe_load(f)
        except Exception:
            return None
        cfg = (data or {}).get("config") or {}
        exe = cfg.get("exe_file") or cfg.get("elf_file")
        if not exe:
            return None
        path = build_dir / "zephyr" / exe
        if path.is_file():
            return path
        path = build_dir / exe
        return path if path.is_file() else None

    def _cmd_run(self, args):
        ctx = self._load_context(args)
        app_list, _ = self._get_apps_and_nets_to_process(ctx, args)
        log.inf(f"Building applications for context {ctx.name}")
        self._build_all(ctx, app_list)
        log.inf(f"Running applications for context {ctx.name}")
        import threading
        procs = []
        for app_name in app_list:
            exe = self._exe_for_app(ctx, app_name)
            if not exe:
                self.die(f"No executable for {app_name}; run 'west compose build' first")
            app = ctx.applications[app_name]
            run_args = list(app.extra_run_args)
            if app.board == "native_sim":
                ninfo = ctx.app_network_info.get(app_name)
                if ninfo and ninfo.network:
                    net_cfg = ctx.networks[ninfo.network]
                    if ninfo.iface:
                        run_args.append(f"-eth-if={ninfo.iface}")
                    if ninfo.mac:
                        run_args.append(f"-mac-addr={ninfo.mac}")
                    if ninfo.ipv4 and net_cfg.ipv4:
                        run_args.append(f"-ipv4-addr={ninfo.ipv4}")

                        # Calculate netmask from prefixlen
                        run_args.append(f"-ipv4-nm={str(net_cfg.ipv4.netmask)}")

            env = os.environ.copy()
            p = subprocess.Popen(
                [str(exe), *run_args],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                env=env,
                cwd=str(exe.parent),
            )
            procs.append((app_name, p))

        def read_stream(name, stream):
            for line in iter(stream.readline, ""):
                if line:
                    sys.stdout.write(f"{name} | {line}")
                    sys.stdout.flush()
            stream.close()

        threads = [
            threading.Thread(target=read_stream, args=(app_name, p.stdout), daemon=True)
            for app_name, p in procs
        ]
        for t in threads:
            t.start()
        try:
            for _, p in procs:
                p.wait()
        except KeyboardInterrupt:
            for _, p in procs:
                p.terminate()
            for _, p in procs:
                p.wait(timeout=2)
            raise
