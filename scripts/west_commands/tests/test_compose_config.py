# Copyright (c) 2026 Titouan Christophe
#
# SPDX-License-Identifier: Apache-2.0

"""Tests for west compose configuration file parsing"""

from __future__ import annotations

import textwrap
from pathlib import Path

import pytest

from compose import (
    DEFAULT_BOARD,
    DEFAULT_IPV4_SUBNET,
    DEFAULT_IPV6_SUBNET,
    MAX_NET_NAME_LEN,
    ConfigError,
    load_config,
    slugify,
)


def _write(tmp_path: Path, content: str, name: str = "west-compose.yml") -> Path:
    p = tmp_path / name
    p.write_text(textwrap.dedent(content))
    return p


# ---------------------------------------------------------------- slugify

@pytest.mark.parametrize(
    "raw,expected",
    [
        ("Network echo samples", "network-echo-samples"),
        ("hello", "hello"),
        ("  trim me  ", "trim-me"),
        ("A---B", "a-b"),
        ("abc.def/ghi", "abc-def-ghi"),
    ],
)
def test_slugify(raw: str, expected: str) -> None:
    assert slugify(raw) == expected


def test_empty_slug_rejected(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: "---"
        applications:
          a:
            source: .
    """)
    with pytest.raises(ConfigError, match="slugifies to empty"):
        load_config(p)


# ---------------------------------------------------------------- minimal

def test_minimal_config(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Minimal
        applications:
          solo:
            source: ./my-app
    """)
    cfg = load_config(p)
    assert cfg.name == "Minimal"
    assert cfg.slug == "minimal"
    assert cfg.compose_file == p.resolve()
    assert cfg.compose_dir == p.parent.resolve()
    assert cfg.state_dir == (p.parent / ".compose" / "minimal").resolve()

    app = cfg.apps["solo"]
    assert app.board == DEFAULT_BOARD
    assert app.network is None
    assert app.source == (p.parent / "my-app").resolve()
    assert app.ipv4 == ""
    assert app.iface == ""


# ---------------------------------------------------------------- substitution

def test_substitution_across_apps(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Echo
        networks:
          zeth:
            host-veth: true
            ipv4: 192.0.2.0/24
            ipv6: 2001:db8::/64
        applications:
          server:
            source: ./srv
            network: zeth
          client:
            source: ./cli
            network: zeth
            extra-build:
              config:
                NET_CONFIG_PEER_IPV4_ADDR: ${server:ipv4}
                NET_CONFIG_PEER_IPV6_ADDR: ${server:ipv6}
                NET_CONFIG_PEER_MAC:       ${server:mac}
    """)
    cfg = load_config(p)
    client = cfg.apps["client"]
    assert client.extra_build_config["NET_CONFIG_PEER_IPV4_ADDR"] == "192.0.2.2"
    assert client.extra_build_config["NET_CONFIG_PEER_IPV6_ADDR"] == "2001:db8::2"
    assert client.extra_build_config["NET_CONFIG_PEER_MAC"] == "00:00:5E:00:53:02"


def test_substitution_in_extra_run_args(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Run args
        networks:
          zeth:
            ipv4: 192.0.2.0/24
        applications:
          server:
            source: .
            network: zeth
          client:
            source: .
            network: zeth
            extra-run:
              args:
                - --peer=${server:ipv4}
    """)
    cfg = load_config(p)
    assert cfg.apps["client"].extra_run_args == ["--peer=192.0.2.2"]


def test_substitution_unknown_app(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Bad ref
        applications:
          a:
            source: .
            extra-build:
              config:
                FOO: ${nope:ipv4}
    """)
    with pytest.raises(ConfigError, match="unknown application 'nope'"):
        load_config(p)


def test_substitution_unsupported_property(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Bad prop
        networks:
          zeth:
            ipv4: 192.0.2.0/24
        applications:
          a:
            source: .
            network: zeth
            extra-build:
              config:
                FOO: ${a:bogus}
    """)
    with pytest.raises(ConfigError, match="unsupported property 'bogus'"):
        load_config(p)


def test_substitution_missing_value(tmp_path: Path) -> None:
    # App `a` has no ipv6 (network explicitly disables it) → ${a:ipv6} unresolvable.
    p = _write(tmp_path, """
        name: Missing
        networks:
          zeth:
            ipv4: 192.0.2.0/24
            ipv6: false
        applications:
          a:
            source: .
            network: zeth
          b:
            source: .
            extra-build:
              config:
                FOO: ${a:ipv6}
    """)
    with pytest.raises(ConfigError, match="has no ipv6 configured"):
        load_config(p)


# ---------------------------------------------------------------- allocation

def test_allocation_two_apps_on_bridge_with_host_veth(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Echo
        networks:
          zeth:
            host-veth: true
            ipv4: 192.0.2.0/24
            ipv6: 2001:db8::/64
        applications:
          server:
            source: .
            network: zeth
          client:
            source: .
            network: zeth
    """)
    cfg = load_config(p)
    net = cfg.networks["zeth"]

    assert net.bridge_iface == "zeth"
    assert net.host_veth_iface == "zeth-host"
    assert net.host_bridge_side_iface == "zeth-br"
    assert net.host_ipv4() == "192.0.2.1"
    assert net.host_ipv6() == "2001:db8::1"

    server = cfg.apps["server"]
    client = cfg.apps["client"]
    assert server.iface == "zethtap0"
    assert client.iface == "zethtap1"
    assert server.mac == "00:00:5E:00:53:02"
    assert client.mac == "00:00:5E:00:53:03"
    assert server.ipv4 == "192.0.2.2" and server.ipv4_cidr == "192.0.2.2/24"
    assert client.ipv4 == "192.0.2.3" and client.ipv4_cidr == "192.0.2.3/24"
    assert server.ipv6 == "2001:db8::2" and server.ipv6_cidr == "2001:db8::2/64"


def test_default_subnets_when_unspecified(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Defaulted
        networks:
          zeth:
            host-veth: true
        applications:
          a:
            source: .
            network: zeth
    """)
    cfg = load_config(p)
    net = cfg.networks["zeth"]
    assert str(net.ipv4_subnet) == DEFAULT_IPV4_SUBNET
    assert str(net.ipv6_subnet) == DEFAULT_IPV6_SUBNET
    assert cfg.apps["a"].ipv4 == "192.0.2.2"


def test_false_disables_addressing(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: V4only
        networks:
          zeth:
            ipv4: 10.0.0.0/24
            ipv6: false
        applications:
          a:
            source: .
            network: zeth
    """)
    cfg = load_config(p)
    assert cfg.networks["zeth"].ipv6_subnet is None
    assert cfg.apps["a"].ipv6 == ""


def test_allocation_without_host_veth(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Noveth
        networks:
          net1:
            ipv4: 10.0.0.0/24
        applications:
          one:
            source: .
            network: net1
    """)
    cfg = load_config(p)
    net = cfg.networks["net1"]
    assert net.bridge_iface == "net1"
    assert net.host_veth_iface == ""
    assert net.host_bridge_side_iface == ""
    assert net.host_ipv4() is None
    assert cfg.apps["one"].ipv4 == "10.0.0.2"


def test_allocation_multiple_networks_independent(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Multi
        networks:
          neta:
            ipv4: 10.0.0.0/24
          netb:
            ipv4: 10.0.1.0/24
        applications:
          a1:
            source: .
            network: neta
          b1:
            source: .
            network: netb
          a2:
            source: .
            network: neta
    """)
    cfg = load_config(p)
    assert cfg.apps["a1"].iface == "netatap0"
    assert cfg.apps["a2"].iface == "netatap1"
    assert cfg.apps["b1"].iface == "netbtap0"
    assert cfg.apps["a1"].ipv4 == "10.0.0.2"
    assert cfg.apps["a2"].ipv4 == "10.0.0.3"
    assert cfg.apps["b1"].ipv4 == "10.0.1.2"


def test_network_name_too_long_rejected(tmp_path: Path) -> None:
    long_name = "a" * (MAX_NET_NAME_LEN + 1)
    p = _write(tmp_path, f"""
        name: Long
        networks:
          {long_name}:
            ipv4: 10.0.0.0/24
        applications:
          x:
            source: .
            network: {long_name}
    """)
    with pytest.raises(ConfigError, match="exceeds"):
        load_config(p)


# ---------------------------------------------------------------- schema errors

def test_missing_name(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        applications:
          a:
            source: .
    """)
    with pytest.raises(ConfigError, match="top-level `name` is required"):
        load_config(p)


def test_missing_applications(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Empty
    """)
    with pytest.raises(ConfigError, match="`applications` must contain"):
        load_config(p)


def test_unknown_top_level_key(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Bad
        applications:
          a:
            source: .
        bogus: 1
    """)
    with pytest.raises(ConfigError, match="unknown key.*bogus"):
        load_config(p)


def test_unknown_app_key(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Bad
        applications:
          a:
            source: .
            mystery: 1
    """)
    with pytest.raises(ConfigError, match="unknown key.*mystery"):
        load_config(p)


def test_unknown_network_key(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Bad
        networks:
          zeth:
            mystery: 1
        applications:
          a:
            source: .
    """)
    with pytest.raises(ConfigError, match="unknown key.*mystery"):
        load_config(p)


def test_unsupported_network_type(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Bad type
        networks:
          zeth:
            type: can
        applications:
          a:
            source: .
    """)
    with pytest.raises(ConfigError, match="unsupported type"):
        load_config(p)


def test_unknown_network_reference(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Bad ref
        networks:
          zeth:
            ipv4: 10.0.0.0/24
        applications:
          a:
            source: .
            network: nope
    """)
    with pytest.raises(ConfigError, match="unknown network 'nope'"):
        load_config(p)


def test_invalid_cidr(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Bad cidr
        networks:
          zeth:
            ipv4: "not-a-cidr"
        applications:
          a:
            source: .
    """)
    with pytest.raises(ConfigError, match="invalid CIDR"):
        load_config(p)


def test_bad_types(tmp_path: Path) -> None:
    # extra-build.args must be a list of strings.
    p = _write(tmp_path, """
        name: Bad
        applications:
          a:
            source: .
            extra-build:
              args: "string not list"
    """)
    with pytest.raises(ConfigError, match="must be a list of strings"):
        load_config(p)


def test_relative_path_resolution(tmp_path: Path) -> None:
    sub = tmp_path / "sub"
    sub.mkdir()
    (sub / "west-compose.yml").write_text(textwrap.dedent("""
        name: Rel
        applications:
          a:
            source: ./here
    """))
    cfg = load_config(sub / "west-compose.yml")
    assert cfg.apps["a"].source == (sub / "here").resolve()


# ---------------------------------------------------------------- YAML anchors

def test_yaml_anchors_and_merge_keys(tmp_path: Path) -> None:
    p = _write(tmp_path, """
        name: Anchor merge
        .common: &common
          NET_CONFIG_NEED_IPV6: n
          NET_IPV6: n
        applications:
          one:
            source: .
            extra-build:
              config:
                <<: *common
                EXTRA_KEY: 1
    """)
    cfg = load_config(p)
    cfg_map = cfg.apps["one"].extra_build_config
    assert cfg_map["NET_CONFIG_NEED_IPV6"] == "n"
    assert cfg_map["NET_IPV6"] == "n"
    assert cfg_map["EXTRA_KEY"] == "1"
