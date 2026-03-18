.. _west-compose:

Multi-app environments: ``west compose``
########################################

The ``compose`` command manages multi-application Zephyr environments. It
handles building, running, and networking multiple Zephyr applications that
need to communicate with each other, similar in spirit to Docker Compose but
for Zephyr firmware images running on simulated or emulated boards.

.. tip:: Run ``west compose -h`` for a quick overview.

.. only:: html

   .. contents::
      :local:

Overview
********

A typical workflow looks like this:

.. code-block:: bash

   # Inspect the parsed configuration
   west compose show

   # Create network interfaces on the host
   west compose up

   # Build and run all applications
   west compose run

   # Tear down the network when done
   west compose down

All subcommands read a YAML compose file (default: :file:`west-compose.yml` in
the current directory). Use ``-f`` to specify a different file:

.. code-block:: bash

   west compose -f my-compose.yml show

Compose file format
*******************

The compose file is a YAML document with three top-level keys:

.. code-block:: yaml

   name: Network echo samples

   networks:
     zeth:
       host-veth: true

   applications:
     server:
       source: samples/net/sockets/echo_server
       network: zeth

     client:
       source: samples/net/sockets/echo_client
       network: zeth
       extra-build:
         config:
           NET_CONFIG_PEER_IPV4_ADDR: ${server:ipv4}

``name``
========

A human-readable name for the project. This is used in log messages and to
derive the build directory path (slugified to a filesystem-safe form).

``networks``
============

A map of network names to their configuration. Each network creates a Linux
bridge with TAP interfaces for connected applications.

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Key
     - Default
     - Description
   * - ``host-veth``
     - ``false``
     - When ``true``, create a VETH pair connecting the host to the bridge.
       The host side is assigned the first address in each subnet (e.g.
       ``192.0.2.1``).
   * - ``ipv4``
     - ``192.0.2.0/24``
     - IPv4 subnet for the network. Set to ``false`` to disable IPv4.
   * - ``ipv6``
     - ``2001:db8::/64``
     - IPv6 subnet for the network. Set to ``false`` to disable IPv6.

``applications``
================

A map of application names to their configuration.

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Key
     - Default
     - Description
   * - ``source``
     - (required)
     - Path to the Zephyr application source directory, relative to the
       compose file location.
   * - ``board``
     - ``native_sim``
     - Board to build for.
   * - ``network``
     - (none)
     - Name of the network to attach this application to.
   * - ``extra-build``
     - (none)
     - Additional build configuration (see below).
   * - ``extra-run``
     - (none)
     - Additional runtime configuration (see below).

``extra-build`` accepts the following sub-keys:

``config``
   A map of Kconfig options to values. Keys without a ``CONFIG_`` prefix have
   it added automatically. Boolean values are converted to ``y``/``n``.

   .. code-block:: yaml

      extra-build:
        config:
          NET_CONFIG_PEER_IPV4_ADDR: ${server:ipv4}
          NET_IPV6: n

``snippets``
   A list of :ref:`snippets <snippets>` to include in the build.

   .. code-block:: yaml

      extra-build:
        snippets:
          - usbip-native-sim

``args``
   A list of extra CMake arguments passed verbatim after ``--``.

   .. code-block:: yaml

      extra-build:
        args:
          - -DEXTRA_DTC_OVERLAY_FILE=app.overlay

``extra-run`` accepts:

``args``
   A list of extra command-line arguments passed to the application executable.

   .. code-block:: yaml

      extra-run:
        args:
          - --device_id=42

Variable interpolation
======================

Configuration values support ``${app_name:property}`` references that are
resolved after all applications have been assigned their addresses. Available
properties are ``ipv4``, ``ipv6``, and ``mac``.

For example, ``${server:ipv4}`` resolves to the IPv4 address assigned to the
``server`` application.

Address assignment
==================

Applications attached to a network are assigned addresses sequentially from
the network's subnet:

- If ``host-veth`` is enabled, offset 1 (e.g. ``192.0.2.1``) is reserved for
  the host.
- Applications receive the next offsets in definition order (e.g.
  ``192.0.2.2``, ``192.0.2.3``, ...).
- MAC addresses follow the ``00:00:5E:00:53:XX`` pattern where ``XX``
  corresponds to the address offset.
- Each application gets a dedicated TAP interface named
  ``<network>tap<index>`` (e.g. ``zethtap0``, ``zethtap1``).

Build directories
=================

Build artifacts are stored under :file:`.compose/<slugified-name>/<app>/`
relative to the compose file. For example, a project named "Network echo
samples" stores the ``server`` build in
:file:`.compose/network-echo-samples/server/`.

Full example
============

.. code-block:: yaml

   name: Network echo samples

   .common-config: &common-config
     ETH_NATIVE_TAP_RANDOM_MAC: n
     NET_CONFIG_NEED_IPV6: n
     NET_IPV6: n

   networks:
     zeth:
       host-veth: true
       ipv4: 192.0.2.0/24
       ipv6: 2001:db8::/56

   applications:
     server:
       source: samples/net/sockets/echo_server
       board: native_sim
       network: zeth
       extra-build:
         config:
           <<: *common-config

     client:
       source: samples/net/sockets/echo_client
       board: native_sim
       network: zeth
       extra-run:
         args:
           - --device_id=42
       extra-build:
         snippets:
           - usbip-native-sim
         args:
           - -DEXTRA_DTC_OVERLAY_FILE=app.overlay
         config:
           <<: *common-config
           NET_CONFIG_PEER_IPV4_ADDR: ${server:ipv4}

Subcommands
***********

``west compose show``
=====================

Print the fully resolved configuration, including assigned addresses and
interface names:

.. code-block:: none

   $ west compose show
   Context: Network echo samples
     Network zeth: host-veth=yes, ipv4=192.0.2.0/24, ipv6=2001:db8::/64
     Application server:
       source:  samples/net/sockets/echo_server
       board:   native_sim
       network: zeth
       iface:   zethtap0
       mac:     00:00:5E:00:53:02
       ipv4:    192.0.2.2/24
       ipv6:    2001:db8::2/64
     Application client:
       source:  samples/net/sockets/echo_client
       board:   native_sim
       network: zeth
       iface:   zethtap1
       mac:     00:00:5E:00:53:03
       ipv4:    192.0.2.3/24
       ipv6:    2001:db8::3/64

``west compose up``
===================

Create the network infrastructure on the host. This requires ``sudo``
privileges and creates the following Linux network interfaces for each
configured network:

- A **bridge** (e.g. ``zethbr``)
- A **TAP** interface per application (e.g. ``zethtap0``, ``zethtap1``),
  attached to the bridge
- A **VETH** pair if ``host-veth`` is enabled (e.g. ``zethveth0``), with the
  host-side endpoint assigned the first IP in each subnet

.. code-block:: none

   $ west compose up
   Bringing up context Network echo samples
   Creating BRIDGE interface zethbr
   Creating TAP interface zethtap0 bridged to zethbr
   Creating TAP interface zethtap1 bridged to zethbr
   Creating VETH interface zethveth0 bridged to zethbr
   Network zeth up

``west compose down``
=====================

Remove all network interfaces created by ``up``:

.. code-block:: none

   $ west compose down
   Removing BRIDGE interface zethbr
   Removing VETH interface zethveth0
   Removing TAP interface zethtap0
   Removing TAP interface zethtap1

``west compose build [APP_NAME]``
=================================

Build applications.

When called **without arguments**, all applications are built in parallel. A
progress bar (powered by `tqdm <https://tqdm.github.io/>`_) is displayed for
each application, driven by ninja's ``[N/M]`` progress output:

.. code-block:: none

   $ west compose build
   Building applications for context Network echo samples
   server: 100%|████████████████████████████████| 145/145 [00:20<00:00]
   client: 100%|████████████████████████████████| 159/159 [00:21<00:00]

When called **with an application name**, that single application is built
with its output printed directly to the terminal:

.. code-block:: bash

   west compose build server

Under the hood, each build invokes ``west build`` with the appropriate
``-b BOARD``, ``-d BUILD_DIR``, source directory, and any extra CMake
arguments derived from ``extra-build``.

``west compose run [APP_NAME]``
===============================

Build (if needed) and then run applications.

Applications are rebuilt before running to ensure the binaries are up to date.
For native_sim applications, the runtime command automatically includes
network configuration flags:

- ``-eth-if=<tap_name>`` --- TAP interface to use
- ``-mac-addr=<mac>`` --- assigned MAC address
- ``-ipv4-addr=<addr>`` --- assigned IPv4 address
- ``-ipv4-nm=<netmask>`` --- subnet mask
- ``-ipv4-gw=<gateway>`` --- host VETH address (when ``host-veth`` is enabled)

Any arguments from ``extra-run.args`` are appended after these.

When **multiple applications** are running, each output line is prefixed with
the application name:

.. code-block:: none

   server | *** Booting Zephyr OS build v4.3.0 ***
   server | [00:00:00.000,000] <inf> net_config: IPv4 address: 192.0.2.2
   client | *** Booting Zephyr OS build v4.3.0 ***
   client | [00:00:00.000,000] <inf> net_config: IPv4 address: 192.0.2.3

When a **single application** is specified, output is printed without a
prefix.

Press :kbd:`Ctrl+C` to terminate all running applications.

``west compose clean [APP_NAME]``
=================================

Clean build directories. Without an argument, all applications are cleaned.
With an argument, only the named application is cleaned.

Cleaning delegates to ``west build -t clean``. If that fails (e.g. the build
directory is corrupted), the directory is removed entirely.

``west compose menuconfig APP_NAME``
====================================

Open the Kconfig interactive configuration menu (menuconfig) for the given
application. If the application has not been built yet, it is built first.

.. code-block:: bash

   west compose menuconfig server

This is equivalent to running ``west build -d <build_dir> -t menuconfig``.

Debugging
*********

Pass ``-v`` to ``west`` to enable verbose output. This logs every subprocess
command that ``west compose`` executes:

.. code-block:: none

   $ west -v compose run server
   Building server
   exec: west build -b native_sim -d .compose/network-echo-samples/server ...
   ...
   Running applications for context Network echo samples
   exec: .compose/network-echo-samples/server/zephyr/zephyr.exe -eth-if=zethtap0 ...

Prerequisites
*************

- The ``tqdm`` Python package is required for parallel builds
  (``pip install tqdm``).
- The ``up`` and ``down`` subcommands require Linux with ``sudo`` access to
  manage network interfaces (bridge, TAP, VETH).
- Applications using the ``native_sim`` board are directly executed. Other
  board targets may require additional runner support.
