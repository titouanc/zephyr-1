# Copyright (c) 2025 The Zephyr Project Contributors
#
# SPDX-License-Identifier: Apache-2.0

"""West compose command for managing multi-application Zephyr environments."""

from __future__ import annotations

# West loads extension modules via importlib without registering them in
# sys.modules.  Python >=3.14's dataclass decorator expects the module to be
# present there, so we register ourselves before any @dataclass is evaluated.
import sys as _sys
if __name__ not in _sys.modules:
    import types as _types
    _sys.modules[__name__] = _types.ModuleType(__name__)

import argparse
import concurrent.futures
import ipaddress
import re
import shlex
import shutil
import subprocess
import sys
import textwrap
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml
from tqdm import tqdm
from west.commands import WestCommand

try:
    from yaml import CSafeLoader as SafeLoader
except ImportError:
    from yaml import SafeLoader

DEFAULT_BOARD = "native_sim"
DEFAULT_IPV4_SUBNET = "192.0.2.0/24"
DEFAULT_IPV6_SUBNET = "2001:db8::/64"
MAC_PREFIX = (0x00, 0x00, 0x5E, 0x00, 0x53)

NINJA_PROGRESS_RE = re.compile(r"\[(\d+)/(\d+)\]")
VAR_REF_RE = re.compile(r"\$\{(\w+):(\w+)\}")


def _slugify(text: str) -> str:
    """Turn *text* into a filesystem-safe, lowercase, hyphen-separated slug."""
    text = text.lower().strip()
    text = re.sub(r"[^\w\s-]", "", text)
    return re.sub(r"[\s_]+", "-", text)


@dataclass
class AppConfig:
    """Resolved configuration for a single application."""

    name: str
    source: str
    board: str = DEFAULT_BOARD
    network: str | None = None
    extra_build: dict[str, Any] = field(default_factory=dict)
    extra_run: dict[str, Any] = field(default_factory=dict)
    ipv4: str | None = None
    ipv6: str | None = None
    mac: str | None = None
    tap_name: str | None = None


@dataclass
class NetworkConfig:
    """Configuration for a single network."""

    name: str
    host_veth: bool = False
    ipv4_net: ipaddress.IPv4Network | None = None
    ipv6_net: ipaddress.IPv6Network | None = None

    @property
    def bridge_name(self) -> str:
        return f"{self.name}br"

    @property
    def veth_name(self) -> str | None:
        return f"{self.name}veth0" if self.host_veth else None

    @property
    def veth_br_name(self) -> str | None:
        return f"{self.name}veth0br" if self.host_veth else None


@dataclass
class ComposeConfig:
    """Fully parsed and resolved compose configuration."""

    project_name: str
    compose_dir: Path
    networks: dict[str, NetworkConfig] = field(default_factory=dict)
    apps: dict[str, AppConfig] = field(default_factory=dict)

    @classmethod
    def from_yaml(cls, yaml_data: dict[str, Any], compose_dir: Path) -> ComposeConfig:
        """Parse a compose YAML dict into a fully resolved config."""
        config = cls(
            project_name=yaml_data.get("name", "unnamed"),
            compose_dir=Path(compose_dir),
        )
        config._parse_networks(yaml_data.get("networks", {}))
        config._parse_apps(yaml_data.get("applications", {}))
        config._resolve_variables()
        return config

    def _parse_networks(self, networks_data: dict[str, Any]) -> None:
        for name, data in networks_data.items():
            ipv4_raw = data.get("ipv4", DEFAULT_IPV4_SUBNET)
            ipv6_raw = data.get("ipv6", DEFAULT_IPV6_SUBNET)

            self.networks[name] = NetworkConfig(
                name=name,
                host_veth=data.get("host-veth", False),
                ipv4_net=(
                    ipaddress.IPv4Network(ipv4_raw, strict=False)
                    if ipv4_raw and ipv4_raw is not False
                    else None
                ),
                ipv6_net=(
                    ipaddress.IPv6Network(ipv6_raw, strict=False)
                    if ipv6_raw and ipv6_raw is not False
                    else None
                ),
            )

    def _parse_apps(self, apps_data: dict[str, Any]) -> None:
        # per-network counters: offset 1 is reserved for host veth
        addr_offset: dict[str, int] = {
            n: (2 if nc.host_veth else 1) for n, nc in self.networks.items()
        }
        tap_index: dict[str, int] = {n: 0 for n in self.networks}

        for app_name, app_data in apps_data.items():
            net_name: str | None = app_data.get("network")
            net_cfg = self.networks.get(net_name) if net_name else None

            ipv4 = ipv6 = mac = tap_name = None

            if net_cfg and net_name is not None:
                offset = addr_offset[net_name]
                addr_offset[net_name] = offset + 1

                if net_cfg.ipv4_net:
                    ipv4 = str(net_cfg.ipv4_net.network_address + offset)
                if net_cfg.ipv6_net:
                    ipv6 = str(net_cfg.ipv6_net.network_address + offset)

                mac_bytes = MAC_PREFIX + (offset,)
                mac = ":".join(f"{b:02X}" for b in mac_bytes)

                idx = tap_index[net_name]
                tap_index[net_name] = idx + 1
                tap_name = f"{net_name}tap{idx}"

            self.apps[app_name] = AppConfig(
                name=app_name,
                source=app_data.get("source", ""),
                board=app_data.get("board", DEFAULT_BOARD),
                network=net_name,
                extra_build=dict(app_data.get("extra-build", {})),
                extra_run=dict(app_data.get("extra-run", {})),
                ipv4=ipv4,
                ipv6=ipv6,
                mac=mac,
                tap_name=tap_name,
            )

    def _resolve_variables(self) -> None:
        """Resolve ``${app:property}`` references in extra-build config."""
        for app in self.apps.values():
            config = app.extra_build.get("config")
            if not config:
                continue
            app.extra_build["config"] = {
                k: VAR_REF_RE.sub(self._var_lookup, v) if isinstance(v, str) else v
                for k, v in config.items()
            }

    def _var_lookup(self, match: re.Match[str]) -> str:
        ref_app_name, prop = match.group(1), match.group(2)
        ref_app = self.apps.get(ref_app_name)
        if ref_app is None:
            raise ValueError(
                f"Unknown app '{ref_app_name}' in ${{{ref_app_name}:{prop}}}"
            )
        val = getattr(ref_app, prop, None)
        if val is None:
            raise ValueError(
                f"Unknown property '{prop}' for app '{ref_app_name}'"
            )
        return str(val)

    def build_dir(self, app_name: str) -> Path:
        return self.compose_dir / ".compose" / _slugify(self.project_name) / app_name

    def get_app(self, name: str) -> AppConfig:
        if name not in self.apps:
            avail = ", ".join(self.apps)
            raise ValueError(f"Unknown application '{name}'. Available: {avail}")
        return self.apps[name]


class Compose(WestCommand):
    def __init__(self) -> None:
        super().__init__(
            "compose",
            "",
            description="manage multi-application Zephyr environments",
            accepts_unknown_args=False,
        )

    def do_add_parser(self, parser_adder: argparse._SubParsersAction) -> argparse.ArgumentParser:
        parser = parser_adder.add_parser(
            self.name,
            formatter_class=argparse.RawDescriptionHelpFormatter,
            description=self.description,
            epilog=textwrap.dedent("""\
                subcommands:
                  show    print parsed configuration
                  up      set up network interfaces
                  down    tear down network interfaces
                  build   build applications (all or by name)
                  run     build & run applications
                  clean      clean application build directories
                  menuconfig open Kconfig TUI for an application
            """),
        )

        parser.add_argument(
            "-f", "--file",
            default="west-compose.yml",
            help="path to compose file (default: west-compose.yml)",
        )

        sub = parser.add_subparsers(
            dest="subcommand",
            metavar="<subcommand>",
            help="select a subcommand",
        )

        sub.add_parser("show", help="print parsed configuration")
        sub.add_parser("up", help="set up networks")
        sub.add_parser("down", help="tear down networks")

        build_p = sub.add_parser("build", help="build applications")
        build_p.add_argument(
            "app_name", nargs="?", help="application to build (omit for all)",
        )

        run_p = sub.add_parser("run", help="build & run applications")
        run_p.add_argument(
            "app_name", nargs="?", help="application to run (omit for all)",
        )

        clean_p = sub.add_parser("clean", help="clean build directories")
        clean_p.add_argument(
            "app_name", nargs="?", help="application to clean (omit for all)",
        )

        menuconfig_p = sub.add_parser(
            "menuconfig", help="open Kconfig TUI for an application",
        )
        menuconfig_p.add_argument("app_name", help="application to configure")

        return parser

    def _dbg_cmd(self, cmd: list[str]) -> None:
        """Log a command at debug verbosity (visible with ``west -v``)."""
        self.dbg(f"exec: {shlex.join(cmd)}")

    def _sudo(self, *args: str) -> None:
        """Run a command with sudo, logging it at debug level."""
        cmd = ["sudo", *args]
        self._dbg_cmd(cmd)
        subprocess.run(cmd, check=True)

    def do_run(self, args: argparse.Namespace, _: list[str]) -> None:
        if not args.subcommand:
            self.parser.print_help()
            return

        compose_file = Path(args.file)
        if not compose_file.is_absolute():
            compose_file = Path.cwd() / compose_file

        if not compose_file.exists():
            self.die(f"compose file not found: {compose_file}")

        with open(compose_file) as f:
            yaml_data: dict[str, Any] = yaml.load(f, Loader=SafeLoader)

        try:
            config = ComposeConfig.from_yaml(yaml_data, compose_file.parent)
        except (ValueError, KeyError) as exc:
            self.die(f"invalid compose file: {exc}")

        dispatch: dict[str, Any] = {
            "show": self._cmd_show,
            "up": self._cmd_up,
            "down": self._cmd_down,
            "build": self._cmd_build,
            "run": self._cmd_run,
            "clean": self._cmd_clean,
            "menuconfig": self._cmd_menuconfig,
        }
        dispatch[args.subcommand](config, args)

    def _cmd_show(self, config: ComposeConfig, args: argparse.Namespace) -> None:
        self.inf(f"Context: {config.project_name}")

        for net in config.networks.values():
            parts = [f"host-veth={'yes' if net.host_veth else 'no'}"]
            if net.ipv4_net:
                parts.append(f"ipv4={net.ipv4_net}")
            if net.ipv6_net:
                parts.append(f"ipv6={net.ipv6_net}")
            self.inf(f"  Network {net.name}: {', '.join(parts)}")

        for app in config.apps.values():
            self.inf(f"  Application {app.name}:")
            self.inf(f"    source:  {app.source}")
            self.inf(f"    board:   {app.board}")
            if app.network:
                net = config.networks[app.network]
                self.inf(f"    network: {app.network}")
                if app.tap_name:
                    self.inf(f"    iface:   {app.tap_name}")
                if app.mac:
                    self.inf(f"    mac:     {app.mac}")
                if app.ipv4 and net.ipv4_net:
                    self.inf(f"    ipv4:    {app.ipv4}/{net.ipv4_net.prefixlen}")
                if app.ipv6 and net.ipv6_net:
                    self.inf(f"    ipv6:    {app.ipv6}/{net.ipv6_net.prefixlen}")

    def _cmd_up(self, config: ComposeConfig, args: argparse.Namespace) -> None:
        self.inf(f"Bringing up context {config.project_name}")

        for net in config.networks.values():
            self.inf(f"Creating BRIDGE interface {net.bridge_name}")
            self._sudo("ip", "link", "add", "name", net.bridge_name, "type", "bridge")
            self._sudo("ip", "link", "set", net.bridge_name, "up")

            for app in config.apps.values():
                if app.network != net.name:
                    continue
                self.inf(
                    f"Creating TAP interface {app.tap_name} "
                    f"bridged to {net.bridge_name}"
                )
                self._sudo("ip", "tuntap", "add", "dev", app.tap_name, "mode", "tap")
                self._sudo("ip", "link", "set", app.tap_name, "master", net.bridge_name)
                self._sudo("ip", "link", "set", app.tap_name, "up")

            if net.host_veth:
                self.inf(
                    f"Creating VETH interface {net.veth_name} "
                    f"bridged to {net.bridge_name}"
                )
                self._sudo(
                    "ip", "link", "add", net.veth_name,
                    "type", "veth", "peer", "name", net.veth_br_name,
                )
                self._sudo("ip", "link", "set", net.veth_br_name, "master", net.bridge_name)
                self._sudo("ip", "link", "set", net.veth_name, "up")
                self._sudo("ip", "link", "set", net.veth_br_name, "up")

                if net.ipv4_net:
                    host_ip = str(net.ipv4_net.network_address + 1)
                    self._sudo(
                        "ip", "addr", "add",
                        f"{host_ip}/{net.ipv4_net.prefixlen}",
                        "dev", net.veth_name,
                    )
                if net.ipv6_net:
                    host_ip = str(net.ipv6_net.network_address + 1)
                    self._sudo(
                        "ip", "-6", "addr", "add",
                        f"{host_ip}/{net.ipv6_net.prefixlen}",
                        "dev", net.veth_name,
                    )

            self.inf(f"Network {net.name} up")

    def _cmd_down(self, config: ComposeConfig, args: argparse.Namespace) -> None:
        for net in config.networks.values():
            self.inf(f"Removing BRIDGE interface {net.bridge_name}")
            try:
                self._sudo("ip", "link", "del", net.bridge_name)
            except subprocess.CalledProcessError:
                self.wrn(f"failed to remove bridge {net.bridge_name}")

            if net.host_veth:
                self.inf(f"Removing VETH interface {net.veth_name}")
                try:
                    self._sudo("ip", "link", "del", net.veth_name)
                except subprocess.CalledProcessError:
                    pass

            for app in config.apps.values():
                if app.network != net.name:
                    continue
                self.inf(f"Removing TAP interface {app.tap_name}")
                try:
                    self._sudo("ip", "tuntap", "del", "dev", app.tap_name, "mode", "tap")
                except subprocess.CalledProcessError:
                    pass

    def _west_build_cmd(self, config: ComposeConfig, app: AppConfig) -> list[str]:
        """Return the ``west build`` command list for *app*."""
        build_dir = str(config.build_dir(app.name))
        source = str((config.compose_dir / app.source).resolve())

        cmd = ["west", "build", "-b", app.board, "-d", build_dir, source]

        cmake_args: list[str] = []

        for key, value in app.extra_build.get("config", {}).items():
            if not key.startswith("CONFIG_"):
                key = f"CONFIG_{key}"
            if isinstance(value, bool):
                value = "y" if value else "n"
            elif isinstance(value, str) and value not in "yn":
                value = f'"{value}"'
            cmake_args.append(f"-D{key}={value}")

        snippets: list[str] = app.extra_build.get("snippets", [])
        if snippets:
            cmake_args.append(f'-DSNIPPET={";".join(snippets)}')

        cmake_args.extend(app.extra_build.get("args", []))

        if cmake_args:
            cmd.append("--")
            cmd.extend(cmake_args)

        return cmd

    def _build_single(self, config: ComposeConfig, app: AppConfig) -> None:
        """Build one application with output going straight to the terminal."""
        cmd = self._west_build_cmd(config, app)
        self.inf(f"Building {app.name}")
        self._dbg_cmd(cmd)
        proc = subprocess.run(cmd, stderr=subprocess.PIPE, text=True)
        if proc.returncode:
            if proc.stderr:
                sys.stderr.write(proc.stderr)
            self.die(f"build failed for {app.name}")

    def _build_all_parallel(self, config: ComposeConfig) -> None:
        """Build every application in parallel, showing tqdm progress bars."""
        self.inf(f"Building applications for context {config.project_name}")

        apps = list(config.apps.values())
        lock = threading.Lock()

        bars: dict[str, tqdm[None]] = {}
        for idx, app in enumerate(apps):
            bars[app.name] = tqdm(
                total=0, desc=app.name, unit="step",
                position=idx, leave=True,
            )

        logs: dict[str, list[str]] = {}

        def _build_one(app: AppConfig) -> tuple[str, int]:
            cmd = self._west_build_cmd(config, app)
            self._dbg_cmd(cmd)
            bar = bars[app.name]
            last_n = 0
            output: list[str] = []

            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            assert proc.stdout is not None

            for line in proc.stdout:
                output.append(line)
                m = NINJA_PROGRESS_RE.search(line)
                if m:
                    current, total = int(m.group(1)), int(m.group(2))
                    with lock:
                        if bar.total != total:
                            bar.reset(total=total)
                        advance = current - last_n
                        if advance > 0:
                            bar.update(advance)
                            last_n = current

            proc.wait()

            with lock:
                if bar.total and bar.n < bar.total:
                    bar.update(bar.total - bar.n)
                bar.close()
                logs[app.name] = output

            return app.name, proc.returncode

        with concurrent.futures.ThreadPoolExecutor(max_workers=len(apps)) as pool:
            futures = {pool.submit(_build_one, a): a for a in apps}
            results: dict[str, int] = {}
            for fut in concurrent.futures.as_completed(futures):
                name, rc = fut.result()
                results[name] = rc

        # blank line after progress bars
        print()

        failed = [n for n, rc in results.items() if rc != 0]
        if failed:
            for name in failed:
                self.err(f"--- build log for {name} ---")
                for line in logs.get(name, []):
                    sys.stderr.write(line)
                self.err(f"--- end {name} ---")
            self.die(f'build failed for: {", ".join(failed)}')

    def _cmd_build(self, config: ComposeConfig, args: argparse.Namespace) -> None:
        if args.app_name:
            self._build_single(config, config.get_app(args.app_name))
        else:
            self._build_all_parallel(config)

    @staticmethod
    def _find_executable(config: ComposeConfig, app: AppConfig) -> Path | None:
        build_dir = config.build_dir(app.name)
        for name in ("zephyr.exe", "zephyr.elf"):
            path = build_dir / "zephyr" / name
            if path.exists():
                return path
        return None

    def _cmd_run(self, config: ComposeConfig, args: argparse.Namespace) -> None:
        if args.app_name:
            apps_to_run = [config.get_app(args.app_name)]
        else:
            apps_to_run = list(config.apps.values())

        # build first
        if len(apps_to_run) == 1:
            self._build_single(config, apps_to_run[0])
        else:
            self._build_all_parallel(config)

        # launch
        self.inf(f"Running applications for context {config.project_name}")
        prefix = len(apps_to_run) > 1
        processes: list[tuple[str, subprocess.Popen[str]]] = []
        stop_event = threading.Event()

        for app in apps_to_run:
            exe = self._find_executable(config, app)
            if exe is None:
                self.die(
                    f"no executable found for {app.name} "
                    f"(looked in {config.build_dir(app.name)})"
                )

            run_cmd = [str(exe)]

            if app.tap_name:
                run_cmd.append(f"-eth-if={app.tap_name}")
            if app.mac:
                run_cmd.append(f"-mac-addr={app.mac}")
            if app.ipv4 and app.network:
                net = config.networks[app.network]
                run_cmd.append(f"-ipv4-addr={app.ipv4}")
                if net.ipv4_net:
                    run_cmd.append(f"-ipv4-nm={net.ipv4_net.netmask}")
                    # gateway is the host veth address when available
                    if net.host_veth:
                        gw = str(net.ipv4_net.network_address + 1)
                        run_cmd.append(f"-ipv4-gw={gw}")

            run_cmd.extend(app.extra_run.get("args", []))

            self._dbg_cmd(run_cmd)
            proc = subprocess.Popen(
                run_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            processes.append((app.name, proc))

            def _reader(
                name: str,
                stream: Any,
                do_prefix: bool,
                stop: threading.Event,
            ) -> None:
                try:
                    for line in stream:
                        if stop.is_set():
                            break
                        text = line.rstrip("\n")
                        if do_prefix:
                            print(f"{name} | {text}", flush=True)
                        else:
                            print(text, flush=True)
                except (BrokenPipeError, ValueError):
                    pass

            threading.Thread(
                target=_reader,
                args=(app.name, proc.stdout, prefix, stop_event),
                daemon=True,
            ).start()

        try:
            for _, proc in processes:
                proc.wait()
        except KeyboardInterrupt:
            stop_event.set()
            for _, proc in processes:
                proc.terminate()
            for _, proc in processes:
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()

    def _cmd_clean(self, config: ComposeConfig, args: argparse.Namespace) -> None:
        if args.app_name:
            apps = [config.get_app(args.app_name)]
        else:
            apps = list(config.apps.values())

        for app in apps:
            build_dir = config.build_dir(app.name)
            if not build_dir.exists():
                self.inf(f"{app.name}: nothing to clean")
                continue
            self.inf(f"Cleaning {app.name}")
            clean_cmd = ["west", "build", "-d", str(build_dir), "-t", "clean"]
            self._dbg_cmd(clean_cmd)
            rc = subprocess.run(
                clean_cmd,
                capture_output=True,
                text=True,
            ).returncode
            if rc:
                self.wrn(
                    f"west clean failed for {app.name}, removing build directory"
                )
                shutil.rmtree(build_dir)

    def _cmd_menuconfig(self, config: ComposeConfig, args: argparse.Namespace) -> None:
        app = config.get_app(args.app_name)
        build_dir = config.build_dir(app.name)

        if not build_dir.exists():
            self.inf(f"Building {app.name} first (needed for menuconfig)")
            self._build_single(config, app)

        self.inf(f"Opening menuconfig for {app.name}")
        menuconfig_cmd = ["west", "build", "-d", str(build_dir), "-t", "menuconfig"]
        self._dbg_cmd(menuconfig_cmd)
        subprocess.run(menuconfig_cmd)
