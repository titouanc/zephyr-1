# Copyright (c) 2026 Titouan Christophe
#
# SPDX-License-Identifier: Apache-2.0

"""``west compose`` - orchestrate multiple Zephyr applications on a laptop.

Conceptually similar to ``docker compose``: a single YAML file describes a
group of applications and the virtual networks they share, and this command
builds, runs and tears them down on the developer's machine (native_sim or
an emulator such as QEMU).
"""

from __future__ import annotations

import argparse
import getpass
import hashlib
import ipaddress
import json
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path
from typing import Any, Self

import yaml
from west.commands import WestCommand

# Python 3.14's `@dataclass` introspects ``sys.modules[cls.__module__].__dict__``
# to resolve annotations, but west's extension loader executes this module
# without first registering it in ``sys.modules``. Self-register with our live
# globals so the introspection finds its namespace.
if __name__ not in sys.modules:

    class _SelfRef:
        pass

    _self = _SelfRef()
    _self.__dict__ = globals()
    _self.__name__ = __name__
    sys.modules[__name__] = _self
    del _self, _SelfRef


DEFAULT_FILE = "west-compose.yml"
DEFAULT_BOARD = "native_sim"

MAX_IFNAME_LEN = 15  # Linux constant (IFNAMSIZ - 1).
MAX_NET_NAME_LEN = 10  # Leaves room for "-host" / "tap<i>" suffixes.

SUPPORTED_NETWORK_TYPES = {"bridge"}
DEFAULT_IPV4_SUBNET = "192.0.2.0/24"
DEFAULT_IPV6_SUBNET = "2001:db8::/64"

USBIP_BUSID = "1-1"

# ${<app>:<prop>}
_SUBST_RE = re.compile(r"\$\{([^{}:]+):([^{}:]+)\}")
_SUPPORTED_APP_PROPS = ("ipv4", "ipv6", "mac")

_VALID_KEYS_TOP = {"name", "networks", "applications"}
_VALID_KEYS_NETWORK = {"type", "host-veth", "ipv4", "ipv6"}
_VALID_KEYS_APP = {"source", "board", "network", "extra-build", "extra-run"}
_VALID_KEYS_EXTRA_BUILD = {"args", "snippets", "config"}
_VALID_KEYS_EXTRA_RUN = {"args"}

_IDENT_RE = re.compile(r"^[a-z][a-z0-9_-]*$")

_NINJA_RE = re.compile(r"^\[(\d+)/(\d+)\]")
_PTY_RE = re.compile(r"(/dev/pts/\d+)")

# ANSI colors used by the run-output prefix.
_PALETTE = (31, 32, 33, 34, 35, 36, 91, 92, 93, 94, 95, 96)
_RESET = "\033[0m"

try:
    _IS_TTY = sys.stdout.isatty()
except Exception:
    _IS_TTY = False


class ConfigError(Exception):
    """Fatal configuration-file error."""


class PrivilegedCommandError(RuntimeError):
    """Raised when a privileged command fails."""


@dataclass
class Network:
    name: str
    type: str
    host_veth: bool
    ipv4_subnet: ipaddress.IPv4Network | None
    ipv6_subnet: ipaddress.IPv6Network | None

    # Derived interface names.
    bridge_iface: str = ""
    host_veth_iface: str = ""  # "" when host_veth is False
    host_bridge_side_iface: str = ""

    def host_ipv4(self) -> str | None:
        if not self.host_veth or self.ipv4_subnet is None:
            return None
        return str(self.ipv4_subnet.network_address + 1)

    def host_ipv6(self) -> str | None:
        if not self.host_veth or self.ipv6_subnet is None:
            return None
        return str(self.ipv6_subnet.network_address + 1)


@dataclass
class App:
    name: str
    source: Path
    board: str
    network: str | None
    extra_build_args: list[str] = field(default_factory=list)
    extra_build_snippets: list[str] = field(default_factory=list)
    extra_build_config: dict[str, str] = field(default_factory=dict)
    extra_run_args: list[str] = field(default_factory=list)

    # Derived - set by _allocate when a network is attached.
    iface: str = ""
    mac: str = ""
    ipv4: str = ""  # bare address, no prefix
    ipv6: str = ""
    ipv4_cidr: str = ""  # "a.b.c.d/<prefix>"
    ipv6_cidr: str = ""

    @cached_property
    def colorcode(self) -> str:
        hashed = int(hashlib.md5(self.name.encode()).hexdigest(), 16)
        return _PALETTE[hashed % len(_PALETTE)]

    @property
    def label(self) -> str:
        if _IS_TTY:
            return f"\033[{self.colorcode}m{self.name}\033[0m"
        return self.name


@dataclass
class Config:
    name: str
    slug: str
    compose_file: Path
    compose_dir: Path
    state_dir: Path
    networks: dict[str, Network] = field(default_factory=dict)
    apps: dict[str, App] = field(default_factory=dict)


@dataclass
class Context:
    config: Config
    compose_file: Path
    pristine: bool = False

    def build_dir(self, app: App) -> Path:
        return self.config.state_dir / app.name / "build"

    def app_dir(self, app: App) -> Path:
        return self.config.state_dir / app.name

    def run_log(self, app: App) -> Path:
        return self.app_dir(app) / "run.log"

    @property
    def state_file(self) -> Path:
        return self.config.state_dir / "state.json"


@dataclass
class State:
    path: Path
    networks_up: dict[str, bool] = field(default_factory=dict)
    pids: dict[str, int] = field(default_factory=dict)
    # Serialise concurrent save()s from multiple `run` threads.
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False, compare=False)

    @classmethod
    def load(cls, path: Path) -> Self:
        if not path.is_file():
            return cls(path=path)
        raw = json.loads(path.read_text() or "{}")
        return cls(
            path=path,
            networks_up=dict(raw.get("networks_up") or {}),
            pids={k: int(v) for k, v in (raw.get("pids") or {}).items()},
        )

    def save(self) -> None:
        with self._lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            # Snapshot under the lock so concurrent pids[...]=... from run
            # threads can't raise "dict changed size during iteration" inside
            # json.dumps.
            payload = json.dumps(
                {
                    "networks_up": dict(self.networks_up),
                    "pids": dict(self.pids),
                },
                indent=2,
                sort_keys=True,
            )
            # Unique tmp name per save so concurrent writers can't share a file.
            tmp = self.path.with_name(f"{self.path.name}.{os.getpid()}.{threading.get_ident()}.tmp")
            tmp.write_text(payload)
            tmp.replace(self.path)

    def alive_pids(self) -> dict[str, int]:
        return {name: pid for name, pid in self.pids.items() if _pid_alive(pid)}


# ============================================================================ #
# Configuration loader                                                         #
# ============================================================================ #


def load_config(path: Path) -> Config:
    """Parse, validate and resolve a compose file at ``path``."""
    path = Path(path).resolve()
    try:
        raw = yaml.safe_load(path.read_text())
    except yaml.YAMLError as e:
        raise ConfigError(f"{path}: YAML parse error: {e}") from e
    if not isinstance(raw, dict):
        raise ConfigError(f"{path}: top-level YAML must be a mapping")

    # Allow anchor-only helper keys (those starting with '.') per the full example.
    raw = {k: v for k, v in raw.items() if not (isinstance(k, str) and k.startswith("."))}

    _reject_unknown(raw, _VALID_KEYS_TOP, "<root>", path)

    name = raw.get("name")
    if not isinstance(name, str) or not name.strip():
        raise ConfigError(f"{path}: top-level `name` is required and must be a string")
    slug = slugify(name)
    if not slug:
        raise ConfigError(f"{path}: `name` slugifies to empty string")

    compose_dir = path.parent
    state_dir = compose_dir / ".compose" / slug

    cfg = Config(
        name=name,
        slug=slug,
        compose_file=path,
        compose_dir=compose_dir,
        state_dir=state_dir,
    )

    cfg.networks = _parse_networks(raw.get("networks"), path)
    cfg.apps = _parse_applications(raw.get("applications"), compose_dir, cfg.networks, path)

    _allocate(cfg)
    _substitute(cfg)

    return cfg


def slugify(name: str) -> str:
    s = re.sub(r"[^a-z0-9]+", "-", name.lower())
    return s.strip("-")


def _reject_unknown(d: dict, allowed: set[str], where: str, path: Path) -> None:
    unknown = sorted(set(d) - allowed)
    if unknown:
        raise ConfigError(
            f"{path}: unknown key(s) under {where}: {', '.join(unknown)} "
            f"(allowed: {', '.join(sorted(allowed))})"
        )


def _parse_networks(raw: Any, path: Path) -> dict[str, Network]:
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise ConfigError(f"{path}: `networks` must be a mapping")

    out: dict[str, Network] = {}
    for nname, nraw in raw.items():
        if not isinstance(nname, str) or not _IDENT_RE.match(nname):
            raise ConfigError(f"{path}: network name {nname!r} must be a lowercase identifier")
        if len(nname) > MAX_NET_NAME_LEN:
            raise ConfigError(
                f"{path}: network name {nname!r} exceeds {MAX_NET_NAME_LEN} characters "
                f"(Linux interface-name limit)"
            )
        nraw = nraw or {}
        if not isinstance(nraw, dict):
            raise ConfigError(f"{path}: network {nname!r} must be a mapping")
        _reject_unknown(nraw, _VALID_KEYS_NETWORK, f"networks.{nname}", path)

        ntype = nraw.get("type", "bridge")
        if ntype not in SUPPORTED_NETWORK_TYPES:
            raise ConfigError(
                f"{path}: network {nname!r}: unsupported type {ntype!r} "
                f"(supported: {', '.join(SUPPORTED_NETWORK_TYPES)})"
            )

        host_veth = bool(nraw.get("host-veth", False))
        ipv4 = _parse_subnet(
            nraw.get("ipv4", DEFAULT_IPV4_SUBNET),
            4,
            f"networks.{nname}.ipv4",
            path,
        )
        ipv6 = _parse_subnet(
            nraw.get("ipv6", DEFAULT_IPV6_SUBNET),
            6,
            f"networks.{nname}.ipv6",
            path,
        )

        out[nname] = Network(
            name=nname,
            type=ntype,
            host_veth=host_veth,
            ipv4_subnet=ipv4,
            ipv6_subnet=ipv6,
        )
    return out


def _parse_subnet(val: Any, family: int, where: str, path: Path):
    if val is None or val is False:
        return None
    if not isinstance(val, str):
        raise ConfigError(f"{path}: {where} must be a CIDR string or false")
    try:
        if family == 4:
            return ipaddress.IPv4Network(val, strict=False)
        return ipaddress.IPv6Network(val, strict=False)
    except (ValueError, ipaddress.AddressValueError) as e:
        raise ConfigError(f"{path}: {where}: invalid CIDR {val!r}: {e}") from e


def _parse_applications(
    raw: Any,
    compose_dir: Path,
    networks: dict[str, Network],
    path: Path,
) -> dict[str, App]:
    if raw is None or not isinstance(raw, dict) or not raw:
        raise ConfigError(f"{path}: `applications` must contain at least one entry")

    out: dict[str, App] = {}
    for aname, araw in raw.items():
        if not isinstance(aname, str) or not _IDENT_RE.match(aname):
            raise ConfigError(f"{path}: application name {aname!r} must be a lowercase identifier")
        araw = araw or {}
        if not isinstance(araw, dict):
            raise ConfigError(f"{path}: application {aname!r} must be a mapping")
        _reject_unknown(araw, _VALID_KEYS_APP, f"applications.{aname}", path)

        source = araw.get("source")
        if not isinstance(source, str) or not source:
            raise ConfigError(f"{path}: applications.{aname}.source is required")
        src_path = Path(source)
        if not src_path.is_absolute():
            src_path = (compose_dir / src_path).resolve()

        board = araw.get("board", DEFAULT_BOARD)
        if not isinstance(board, str):
            raise ConfigError(f"{path}: applications.{aname}.board must be a string")

        network = araw.get("network")
        if network is not None:
            if not isinstance(network, str):
                raise ConfigError(f"{path}: applications.{aname}.network must be a string")
            if network not in networks:
                raise ConfigError(
                    f"{path}: applications.{aname}.network: "
                    f"unknown network {network!r} "
                    f"(declared: {', '.join(networks) or '<none>'})"
                )

        app = App(
            name=aname,
            source=src_path,
            board=board,
            network=network,
        )

        eb = araw.get("extra-build") or {}
        if eb:
            if not isinstance(eb, dict):
                raise ConfigError(f"{path}: applications.{aname}.extra-build must be a mapping")
            _reject_unknown(eb, _VALID_KEYS_EXTRA_BUILD, f"applications.{aname}.extra-build", path)
            app.extra_build_args = _parse_str_list(
                eb.get("args"), f"applications.{aname}.extra-build.args", path
            )
            app.extra_build_snippets = _parse_str_list(
                eb.get("snippets"),
                f"applications.{aname}.extra-build.snippets",
                path,
            )
            app.extra_build_config = _parse_config_map(
                eb.get("config"),
                f"applications.{aname}.extra-build.config",
                path,
            )

        er = araw.get("extra-run") or {}
        if er:
            if not isinstance(er, dict):
                raise ConfigError(f"{path}: applications.{aname}.extra-run must be a mapping")
            _reject_unknown(er, _VALID_KEYS_EXTRA_RUN, f"applications.{aname}.extra-run", path)
            app.extra_run_args = _parse_str_list(
                er.get("args"), f"applications.{aname}.extra-run.args", path
            )

        out[aname] = app
    return out


def _parse_str_list(val: Any, where: str, path: Path) -> list[str]:
    if val is None:
        return []
    if not isinstance(val, list) or not all(isinstance(x, str) for x in val):
        raise ConfigError(f"{path}: {where} must be a list of strings")
    return list(val)


def _parse_config_map(val: Any, where: str, path: Path) -> dict[str, str]:
    if val is None:
        return {}
    if not isinstance(val, dict):
        raise ConfigError(f"{path}: {where} must be a mapping")
    out: dict[str, str] = {}
    for k, v in val.items():
        if not isinstance(k, str):
            raise ConfigError(f"{path}: {where}: keys must be strings")
        if isinstance(v, bool):
            out[k] = "y" if v else "n"
        elif isinstance(v, (int, str)):
            out[k] = str(v)
        else:
            raise ConfigError(f"{path}: {where}.{k}: value must be a string, int or bool")
    return out


def _allocate(cfg: Config) -> None:
    """Fill in derived interface names and addresses."""
    for net in cfg.networks.values():
        net.bridge_iface = net.name
        if net.host_veth:
            net.host_veth_iface = f"{net.name}-host"
            net.host_bridge_side_iface = f"{net.name}-br"
        for iface in (net.bridge_iface, net.host_veth_iface, net.host_bridge_side_iface):
            if iface and len(iface) > MAX_IFNAME_LEN:
                raise ConfigError(f"derived interface {iface!r} exceeds {MAX_IFNAME_LEN} chars")

    per_net_index: dict[str, int] = {}
    for app in cfg.apps.values():
        if app.network is None:
            continue
        idx = per_net_index.get(app.network, 0)
        per_net_index[app.network] = idx + 1
        net = cfg.networks[app.network]

        app.iface = f"{net.name}tap{idx}"
        if len(app.iface) > MAX_IFNAME_LEN:
            raise ConfigError(f"derived TAP interface {app.iface!r} exceeds {MAX_IFNAME_LEN} chars")

        low = 2 + idx
        if low > 0xFF:
            raise ConfigError(
                f"too many apps on network {net.name!r}: "
                f"{app.name} would receive an invalid low byte {low}"
            )
        app.mac = f"00:00:5E:00:53:{low:02X}"

        if net.ipv4_subnet is not None:
            addr = net.ipv4_subnet.network_address + low
            if addr not in net.ipv4_subnet:
                raise ConfigError(
                    f"application {app.name!r}: IPv4 {addr} does not fit "
                    f"in subnet {net.ipv4_subnet}"
                )
            app.ipv4 = str(addr)
            app.ipv4_cidr = f"{addr}/{net.ipv4_subnet.prefixlen}"

        if net.ipv6_subnet is not None:
            addr6 = net.ipv6_subnet.network_address + low
            if addr6 not in net.ipv6_subnet:
                raise ConfigError(
                    f"application {app.name!r}: IPv6 {addr6} does not fit "
                    f"in subnet {net.ipv6_subnet}"
                )
            app.ipv6 = str(addr6)
            app.ipv6_cidr = f"{addr6}/{net.ipv6_subnet.prefixlen}"


def _substitute(cfg: Config) -> None:
    """Resolve ``${app:prop}`` inside app string fields post-allocation."""
    for app in cfg.apps.values():
        app.extra_build_args = [
            _subst_str(s, cfg, f"applications.{app.name}.extra-build.args")
            for s in app.extra_build_args
        ]
        app.extra_run_args = [
            _subst_str(s, cfg, f"applications.{app.name}.extra-run.args")
            for s in app.extra_run_args
        ]
        app.extra_build_config = {
            k: _subst_str(v, cfg, f"applications.{app.name}.extra-build.config.{k}")
            for k, v in app.extra_build_config.items()
        }


def _subst_str(s: str, cfg: Config, where: str) -> str:
    def repl(m: re.Match) -> str:
        ref_app, prop = m.group(1).strip(), m.group(2).strip()
        if ref_app not in cfg.apps:
            raise ConfigError(
                f"{cfg.compose_file}: {where}: "
                f"substitution ${{{ref_app}:{prop}}} references "
                f"unknown application {ref_app!r}"
            )
        if prop not in _SUPPORTED_APP_PROPS:
            raise ConfigError(
                f"{cfg.compose_file}: {where}: "
                f"substitution ${{{ref_app}:{prop}}} references "
                f"unsupported property {prop!r} "
                f"(supported: {', '.join(_SUPPORTED_APP_PROPS)})"
            )
        value = getattr(cfg.apps[ref_app], prop)
        if not value:
            raise ConfigError(
                f"{cfg.compose_file}: {where}: "
                f"substitution ${{{ref_app}:{prop}}}: "
                f"application {ref_app!r} has no {prop} configured"
            )
        return value

    return _SUBST_RE.sub(repl, s)


def _pid_alive(pid: int) -> bool:
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _iface_exists(name: str) -> bool:
    rc = subprocess.run(
        ["ip", "-o", "link", "show", name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode
    return rc == 0


def _iface_is_up(name: str) -> bool:
    """True if the interface exists and carries IFF_UP."""
    p = subprocess.run(
        ["ip", "-o", "link", "show", name],
        capture_output=True,
        text=True,
    )
    if p.returncode != 0 or "<" not in p.stdout:
        return False
    flags = p.stdout.split("<", 1)[1].split(">", 1)[0].split(",")
    return "UP" in flags


def _current_user() -> str | None:
    try:
        return getpass.getuser()
    except Exception:
        return None


def _apps_on(cfg: Config, net_name: str) -> list[App]:
    return [a for a in cfg.apps.values() if a.network == net_name]


def _missing_ifaces_for_run(cfg: Config, targets: list[App]) -> list[str]:
    needed: set[str] = set()
    for app in targets:
        if app.network is None:
            continue
        net = cfg.networks[app.network]
        needed.add(net.bridge_iface)
        if net.host_veth:
            needed.add(net.host_veth_iface)
        needed.add(app.iface)
    return sorted(i for i in needed if not _iface_exists(i))


def _qstring(v: str) -> str:
    """Kconfig string value, passed as the raw arg to ``-DCONFIG_X=...``."""
    return f'"{v}"'


_SUBCOMMANDS = (
    "show",
    "up",
    "down",
    "build",
    "run",
    "clean",
    "menuconfig",
    "attach-usb",
    "console",
)


class Compose(WestCommand):
    def __init__(self) -> None:
        super().__init__(
            "compose",
            "orchestrate multiple Zephyr applications",
            (
                "Orchestrate a group of Zephyr applications declared in a "
                "YAML file, conceptually similar to `docker compose`"
            ),
            accepts_unknown_args=False,
        )

    def do_add_parser(self, parser_adder) -> argparse.ArgumentParser:
        parser = parser_adder.add_parser(
            self.name,
            help=self.help,
            description=self.description,
            formatter_class=argparse.RawDescriptionHelpFormatter,
        )
        parser.add_argument(
            "-f",
            "--file",
            metavar="FILE",
            help=f"path to the compose YAML file (default: ./{DEFAULT_FILE})",
        )
        parser.add_argument(
            "-p",
            "--pristine",
            action="store_true",
            help="pristine build (forwarded to `west build -p always`)",
        )
        sub = parser.add_subparsers(dest="action", required=True, metavar="ACTION")
        for name in _SUBCOMMANDS:
            sp = sub.add_parser(name, help=f"`west compose {name}`")
            if name in ("build", "run", "clean", "attach-usb"):
                sp.add_argument("app", nargs="?", help="application name (optional)")
            elif name in ("menuconfig", "console"):
                sp.add_argument("app", help="application name")
        return parser

    def do_run(self, args, _unknown) -> None:
        file_arg = Path(args.file).resolve() if args.file else Path.cwd() / DEFAULT_FILE
        if not file_arg.is_file():
            self.die(f"compose file not found: {file_arg}")

        try:
            config = load_config(file_arg)
        except ConfigError as e:
            self.die(str(e))

        ctx = Context(config=config, compose_file=file_arg, pristine=args.pristine)

        actions = {
            "show": self.action_show,
            "up": self.action_up,
            "down": self.action_down,
            "build": self.action_build,
            "run": self.action_run,
            "clean": self.action_clean,
            "menuconfig": self.action_menuconfig,
            "attach-usb": self.action_attach_usb,
            "console": self.action_console,
        }
        app = getattr(args, "app", None)
        try:
            rc = actions[args.action](ctx, app=app)
        except KeyboardInterrupt:
            self.err("interrupted")
            rc = 130
        sys.exit(rc or 0)

    def _run_sudo(self, argv: list[str], *, check: bool = True) -> int:
        """Print then run ``sudo argv``. Return the exit code."""
        full = ["sudo", *argv]
        self.inf(f"$ {shlex.join(full)}")
        rc = subprocess.run(full).returncode
        if check and rc != 0:
            raise PrivilegedCommandError(f"command failed with status {rc}: {shlex.join(full)}")
        return rc

    def _network_up(self, net: Network, apps_on_net: list[App]) -> None:
        """Bring up bridge, host veth (if any) and one TAP per app."""
        if not _iface_exists(net.bridge_iface):
            self._run_sudo(["ip", "link", "add", net.bridge_iface, "type", "bridge"])
        self._run_sudo(["ip", "link", "set", "dev", net.bridge_iface, "up"])

        if net.host_veth:
            if not _iface_exists(net.host_veth_iface):
                self._run_sudo(
                    [
                        "ip",
                        "link",
                        "add",
                        net.host_veth_iface,
                        "type",
                        "veth",
                        "peer",
                        "name",
                        net.host_bridge_side_iface,
                    ]
                )
            self._run_sudo(
                [
                    "ip",
                    "link",
                    "set",
                    "dev",
                    net.host_bridge_side_iface,
                    "master",
                    net.bridge_iface,
                ]
            )
            self._run_sudo(["ip", "link", "set", "dev", net.host_bridge_side_iface, "up"])
            self._run_sudo(["ip", "link", "set", "dev", net.host_veth_iface, "up"])

            host_v4 = net.host_ipv4()
            if host_v4 is not None and net.ipv4_subnet is not None:
                self._run_sudo(
                    [
                        "ip",
                        "addr",
                        "replace",
                        f"{host_v4}/{net.ipv4_subnet.prefixlen}",
                        "dev",
                        net.host_veth_iface,
                    ]
                )
            host_v6 = net.host_ipv6()
            if host_v6 is not None and net.ipv6_subnet is not None:
                self._run_sudo(
                    [
                        "ip",
                        "addr",
                        "replace",
                        f"{host_v6}/{net.ipv6_subnet.prefixlen}",
                        "dev",
                        net.host_veth_iface,
                    ]
                )

        user = _current_user()
        for app in apps_on_net:
            if not _iface_exists(app.iface):
                cmd = ["ip", "tuntap", "add", "dev", app.iface, "mode", "tap"]
                if user:
                    cmd += ["user", user]
                self._run_sudo(cmd)
            self._run_sudo(
                [
                    "ip",
                    "link",
                    "set",
                    "dev",
                    app.iface,
                    "master",
                    net.bridge_iface,
                ]
            )
            self._run_sudo(["ip", "link", "set", "dev", app.iface, "up"])

    def _network_down(self, net: Network, apps_on_net: list[App]) -> None:
        for app in apps_on_net:
            if _iface_exists(app.iface):
                self._run_sudo(["ip", "link", "del", app.iface])
        if net.host_veth and _iface_exists(net.host_veth_iface):
            # Removing one side of a veth pair removes the other too.
            self._run_sudo(["ip", "link", "del", net.host_veth_iface])
        if _iface_exists(net.bridge_iface):
            self._run_sudo(["ip", "link", "del", net.bridge_iface])

    def _build_command(self, ctx: Context, app: App) -> list[str]:
        """Render the `west build` command line for a single application."""
        cmd = ["west", "build", "-b", app.board, "-d", str(ctx.build_dir(app))]
        if ctx.pristine:
            cmd += ["-p", "always"]
        for snippet in app.extra_build_snippets:
            cmd += ["-S", snippet]
        cmd.append(str(app.source))

        auto = self._auto_config(ctx, app)
        effective = auto | {
            k: (_qstring(v) if isinstance(v, str) and v != 'y' else v)
            for k, v in app.extra_build_config.items()
        }

        for k, v in app.extra_build_config.items():
            if k in auto and auto[k] != v:
                self.wrn(
                    f"{app.name}: user overrides auto-injected "
                    f"CONFIG_{k} (auto={auto[k]!r}, user={v!r})"
                )

        if effective or app.extra_build_args:
            cmd.append("--")
        for k, v in effective.items():
            cmd.append(f"-DCONFIG_{k}={v}")
        cmd += app.extra_build_args
        return cmd

    def _auto_config(self, ctx: Context, app: App) -> dict[str, str]:
        """Auto-injected Kconfig for network-attached apps. User overrides win."""
        out: dict[str, str] = {}
        if app.network is None:
            return out
        net = ctx.config.networks[app.network]

        if app.board == "native_sim":
            out["ETH_NATIVE_TAP_DRV_NAME"] = _qstring(app.iface)
            if app.mac:
                out["ETH_NATIVE_TAP_RANDOM_MAC"] = "n"
                out["ETH_NATIVE_TAP_MAC_ADDR"] = _qstring(app.mac.lower())

        if app.ipv4:
            out["NET_CONFIG_SETTINGS"] = "y"
            out["NET_CONFIG_NEED_IPV4"] = "y"
            out["NET_CONFIG_MY_IPV4_ADDR"] = _qstring(app.ipv4)
            assert net.ipv4_subnet is not None
            out["NET_CONFIG_MY_IPV4_NETMASK"] = _qstring(str(net.ipv4_subnet.netmask))
            host_v4 = net.host_ipv4()
            if host_v4:
                out["NET_CONFIG_MY_IPV4_GW"] = _qstring(host_v4)

        if app.ipv6:
            out["NET_CONFIG_SETTINGS"] = "y"
            out["NET_CONFIG_NEED_IPV6"] = "y"
            out["NET_CONFIG_MY_IPV6_ADDR"] = _qstring(app.ipv6)

        return out

    def _build_one(self, ctx: Context, app_name: str) -> int:
        """Build one app synchronously, streaming output to the terminal."""
        app = self._require_app(ctx, app_name)
        ctx.build_dir(app).parent.mkdir(parents=True, exist_ok=True)
        cmd = self._build_command(ctx, app)
        self.inf(f"$ {shlex.join(cmd)}")
        return subprocess.run(cmd).returncode

    def _build_all(self, ctx: Context) -> int:
        """Build every app in parallel with a tqdm progress bar per app."""
        try:
            from tqdm import tqdm
        except ImportError:
            self.err(
                "`tqdm` is required for parallel builds. "
                "Install it (pip install tqdm) or use `west compose build APP` "
                "to build apps one at a time."
            )
            return 1

        apps = list(ctx.config.apps.values())
        for app in apps:
            ctx.build_dir(app).parent.mkdir(parents=True, exist_ok=True)

        bars: dict[str, Any] = {}
        bufs: dict[str, list[str]] = {}
        procs: dict[str, subprocess.Popen] = {}
        cancelled = threading.Event()
        io_lock = threading.Lock()

        for i, app in enumerate(apps):
            bars[app.name] = tqdm(
                total=1,
                desc=app.name,
                position=i,
                leave=True,
                bar_format="{desc:>16}: {n_fmt}/{total_fmt} |{bar}| {elapsed}",
            )
            bufs[app.name] = []

        def build_app(app: App) -> int:
            cmd = self._build_command(ctx, app)
            with io_lock:
                tqdm.write(f"$ {shlex.join(cmd)}")
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            procs[app.name] = proc
            assert proc.stdout is not None
            for line in proc.stdout:
                bufs[app.name].append(line)
                m = _NINJA_RE.match(line)
                if m:
                    step, total = int(m.group(1)), int(m.group(2))
                    bar = bars[app.name]
                    with io_lock:
                        bar.total = total
                        bar.n = step
                        bar.refresh()
            rc = proc.wait()
            if rc != 0 and not cancelled.is_set():
                cancelled.set()
                with io_lock:
                    tqdm.write(
                        f"\nbuild failed for {app.name!r} (exit {rc}); cancelling other builds\n"
                    )
                    tqdm.write("".join(bufs[app.name]))
                for other_name, other_proc in procs.items():
                    if other_name != app.name and other_proc.poll() is None:
                        other_proc.terminate()
            elif rc == 0:
                with io_lock:
                    bar = bars[app.name]
                    bar.n = bar.total
                    bar.refresh()
            return rc

        with ThreadPoolExecutor() as pool:
            rcs = list(pool.map(build_app, apps))
        for bar in bars.values():
            bar.close()
        self.inf("")
        return 0 if all(rc == 0 for rc in rcs) and not cancelled.is_set() else 1

    # -- run --------------------------------------------------------------- #

    def _run_command(self, ctx: Context, app: App) -> list[str]:
        """Render the run command for an application."""
        if app.board == "native_sim":
            exe = ctx.build_dir(app) / "zephyr" / "zephyr.exe"
            return [str(exe), *app.extra_run_args]
        return ["west", "build", "-t", "run", "-d", str(ctx.build_dir(app))]

    def _run_apps(self, ctx: Context, apps: list[App]) -> int:
        """Launch ``apps`` in YAML order and stream their output."""
        if not apps:
            return 0

        single = len(apps) == 1
        io_lock = threading.Lock()
        procs: dict[str, subprocess.Popen] = {}
        state = State.load(ctx.state_file)

        def reap(app: App) -> int:
            cmd = self._run_command(ctx, app)
            ctx.app_dir(app).mkdir(parents=True, exist_ok=True)
            log_fh = open(ctx.run_log(app), "w", encoding="utf-8")
            with io_lock:
                self.inf(f"$ {shlex.join(cmd)}")
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            procs[app.name] = proc
            state.pids[app.name] = proc.pid
            state.save()

            assert proc.stdout is not None
            for line in proc.stdout:
                log_fh.write(line)
                log_fh.flush()
                with io_lock:
                    if single:
                        self.inf(line, end="")
                    else:
                        self.inf(f"{app.label} | {line}", end="")
            rc = proc.wait()
            log_fh.close()
            state.pids.pop(app.name, None)
            state.save()
            return rc

        results: dict[str, int] = {}
        threads: list[threading.Thread] = []

        def worker(app: App) -> None:
            results[app.name] = reap(app)

        for app in apps:
            t = threading.Thread(target=worker, args=(app,), daemon=True, name=f"run-{app.name}")
            t.start()
            threads.append(t)

        def forward(sig, _frame):
            with io_lock:
                self.inf(f"\nforwarding signal {sig} to {len(procs)} app(s)")
            for p in procs.values():
                if p.poll() is None:
                    try:
                        p.send_signal(sig)
                    except ProcessLookupError:
                        pass

        prev_int = signal.signal(signal.SIGINT, forward)
        prev_term = signal.signal(signal.SIGTERM, forward)
        try:
            for t in threads:
                t.join()
        finally:
            signal.signal(signal.SIGINT, prev_int)
            signal.signal(signal.SIGTERM, prev_term)

        return 0 if all(rc == 0 for rc in results.values()) else 1

    def _open_console(self, ctx: Context, app_name: str) -> int:
        if app_name not in ctx.config.apps:
            self.err(f"unknown application {app_name!r}")
            return 1
        app = ctx.config.apps[app_name]
        log = ctx.run_log(app)
        if not log.is_file():
            self.err(
                f"no run log for {app_name!r} - is the app running (`west compose run {app_name}`)?"
            )
            return 1

        pty = None
        for line in log.read_text(errors="replace").splitlines():
            m = _PTY_RE.search(line)
            if m:
                pty = m.group(1)
        if pty is None:
            self.err(
                f"could not find a PTY path in {log}; "
                f"the app must print a line like 'connected to pseudotty: /dev/pts/N'."
            )
            return 1

        if shutil.which("picocom") is None:
            self.err(f"picocom not installed. PTY is at: {pty} - attach with your preferred tool.")
            return 1

        os.execvp("picocom", ["picocom", "-q", pty])
        return 1  # unreachable

    def _attach_apps(self, ctx: Context, apps: list[App]) -> int:
        if shutil.which("usbip") is None:
            self.err("`usbip` is not installed on the host.")
            return 1

        rc_total = 0
        for app in apps:
            if not app.ipv4:
                self.err(f"application {app.name!r} has no IPv4 address; attach-usb requires one.")
                rc_total = 1
                continue
            try:
                self._run_sudo(["usbip", "attach", "-r", app.ipv4, "-b", USBIP_BUSID])
            except PrivilegedCommandError as e:
                self.err(str(e))
                rc_total = 1
        return rc_total

    def _require_app(self, ctx: Context, app_name: str) -> App:
        if app_name not in ctx.config.apps:
            self.die(
                f"unknown application {app_name!r} "
                f"(declared: {', '.join(ctx.config.apps) or '<none>'})"
            )
        return ctx.config.apps[app_name]

    def _select_apps(self, cfg: Config, app: str | None) -> list[App]:
        if app is None:
            return list(cfg.apps.values())
        if app not in cfg.apps:
            self.die(f"unknown application {app!r}")
        return [cfg.apps[app]]

    def action_show(self, ctx: Context, *, app: str | None = None) -> int:
        cfg = ctx.config
        self.inf(f"Context: {cfg.name}")
        for net in cfg.networks.values():
            host_veth = "yes" if net.host_veth else "no"
            ipv4 = str(net.ipv4_subnet) if net.ipv4_subnet else "no"
            ipv6 = str(net.ipv6_subnet) if net.ipv6_subnet else "no"
            status = "up" if _iface_exists(net.bridge_iface) else "down"
            self.inf(
                f"  Network {net.name}: host-veth={host_veth}, "
                f"ipv4={ipv4}, ipv6={ipv6}, status={status}"
            )
        for a in cfg.apps.values():
            self.inf(f"  Application {a.name}:")
            try:
                src = a.source.relative_to(cfg.compose_dir)
            except ValueError:
                src = a.source
            self.inf(f"    source:  {src}")
            self.inf(f"    board:   {a.board}")
            if a.network:
                iface_status = "up" if _iface_is_up(a.iface) else "down"
                self.inf(f"    network: {a.network}")
                self.inf(f"    iface:   {a.iface} ({iface_status})")
                if a.mac:
                    self.inf(f"    mac:     {a.mac}")
                if a.ipv4_cidr:
                    self.inf(f"    ipv4:    {a.ipv4_cidr}")
                if a.ipv6_cidr:
                    self.inf(f"    ipv6:    {a.ipv6_cidr}")
        return 0

    def action_up(self, ctx: Context, *, app: str | None = None) -> int:
        state = State.load(ctx.state_file)
        ctx.config.state_dir.mkdir(parents=True, exist_ok=True)
        for net in ctx.config.networks.values():
            self.inf(f"bringing up network {net.name!r}")
            self._network_up(net, _apps_on(ctx.config, net.name))
            state.networks_up[net.name] = True
            state.save()
        return 0

    def action_down(self, ctx: Context, *, app: str | None = None) -> int:
        state = State.load(ctx.state_file)
        alive = state.alive_pids()
        if alive:
            listing = ", ".join(f"{n} (pid {p})" for n, p in alive.items())
            self.err(
                f"refusing to tear down: application(s) still running: {listing}. "
                f"Stop them first (Ctrl-C the `west compose run`) and retry."
            )
            return 1
        for net in reversed(list(ctx.config.networks.values())):
            self.inf(f"tearing down network {net.name!r}")
            self._network_down(net, _apps_on(ctx.config, net.name))
            state.networks_up.pop(net.name, None)
            state.save()
        return 0

    def action_build(self, ctx: Context, *, app: str | None = None) -> int:
        if app is None:
            return self._build_all(ctx)
        return self._build_one(ctx, app)

    def action_run(self, ctx: Context, *, app: str | None = None) -> int:
        if app is None:
            targets = list(ctx.config.apps.values())
        else:
            if app not in ctx.config.apps:
                self.err(f"unknown application {app!r}")
                return 1
            targets = [ctx.config.apps[app]]

        missing = _missing_ifaces_for_run(ctx.config, targets)
        if missing:
            self.err(
                f"cannot start: missing network interface(s): "
                f"{', '.join(missing)}. "
                f"Run `west compose up` first (this requires root "
                f"and is intentionally kept separate from running applications)."
            )
            return 1

        rc = self.action_build(ctx, app=app)
        if rc != 0:
            return rc
        return self._run_apps(ctx, targets)

    def action_clean(self, ctx: Context, *, app: str | None = None) -> int:
        targets = self._select_apps(ctx.config, app)
        rc_total = 0
        for a in targets:
            if not ctx.build_dir(a).is_dir():
                self.inf(f"skipping {a.name!r}: no build dir at {ctx.build_dir(a)}")
                continue
            cmd = ["west", "build", "-d", str(ctx.build_dir(a)), "-t", "clean"]
            self.inf(f"$ {shlex.join(cmd)}")
            rc = subprocess.run(cmd).returncode
            if rc != 0:
                rc_total = rc
        return rc_total

    def action_menuconfig(self, ctx: Context, *, app: str | None = None) -> int:
        if app is None or app not in ctx.config.apps:
            self.err("menuconfig requires a valid application name")
            return 1
        a = ctx.config.apps[app]
        if not ctx.build_dir(a).is_dir():
            self.err(f"no build dir for {app!r} - run `west compose build {app}` first.")
            return 1
        cmd = ["west", "build", "-d", str(ctx.build_dir(a)), "-t", "menuconfig"]
        self.inf(f"$ {shlex.join(cmd)}")
        return subprocess.run(cmd).returncode

    def action_attach_usb(self, ctx: Context, *, app: str | None = None) -> int:
        targets = self._select_apps(ctx.config, app)
        return self._attach_apps(ctx, targets)

    def action_console(self, ctx: Context, *, app: str | None = None) -> int:
        if app is None:
            self.err("console requires an application name")
            return 1
        return self._open_console(ctx, app)
