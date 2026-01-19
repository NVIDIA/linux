.. SPDX-License-Identifier: GPL-2.0

====================================
MCTP Statistics and Monitoring
====================================

Overview
========

The MCTP stack provides comprehensive statistics at multiple layers for debugging,
monitoring, and troubleshooting:

* **Socket layer:** Protocol-level stats (routing, fragmentation, drops)
* **Device layer:** Transport-specific stats (I2C/USB/SPI errors, retries)
* **Queue layer:** Transmit queue stats (backlog, congestion)

**Available Interfaces:**

* **Per-socket stats:** Socket option ``MCTP_OPT_SOCK_STATS`` (C API)
* **Global stats:** Generic Netlink family ``"mctp"`` (binary)
* **Socket list:** ``/proc/net/mctp/sockets`` (human-readable)
* **Global view:** ``/proc/net/mctp/stats`` (human-readable)
* **Device stats:** ``ethtool -S`` (transport-specific)
* **Queue stats:** ``tc -s qdisc`` (congestion monitoring)

Statistics Types
================

Per-Socket Statistics
---------------------

Each MCTP socket maintains its own set of statistics accessible via the
``MCTP_OPT_SOCK_STATS`` socket option.

**Available Metrics:**

* **TX (Transmit) Counters:**
  - ``tx_bytes``: Total bytes transmitted
  - ``tx_packets``: Total packets transmitted
  - ``tx_messages``: Total messages transmitted
  - ``tx_errors``: Transmission errors
  - ``tx_drops``: Dropped transmissions

* **RX (Receive) Counters:**
  - ``rx_bytes``: Total bytes received
  - ``rx_packets``: Total packets received
  - ``rx_messages``: Total messages received
  - ``rx_errors``: Reception errors
  - ``rx_drops``: Dropped receptions

* **TX Drop Reasons:**
  - ``tx_dropped_no_route``: No route to destination (EHOSTUNREACH)
  - ``tx_dropped_mtu_exceeded``: Message exceeds interface MTU (EMSGSIZE)
  - ``tx_dropped_no_memory``: Failed to allocate socket buffer/key (ENOMEM)
  - ``tx_dropped_queue_full``: Device transmit queue full (ENOBUFS)
  - ``tx_dropped_device_down``: Network device is down (ENETDOWN)
  - ``tx_dropped_tag_exhaustion``: No local tags available (EBUSY)
  - ``tx_dropped_permission``: Permission denied (EPERM/EACCES)

* **RX Drop Reasons:**
  - ``rx_dropped_no_route``: Packet for unknown local socket/binding
  - ``rx_dropped_no_memory``: Failed to allocate reassembly buffer
  - ``rx_dropped_seq_mismatch``: Sequence number error or missing SOM
  - ``rx_dropped_tag_mismatch``: Message tag mismatch (unexpected tag)
  - ``rx_dropped_queue_full``: Socket receive queue full (application slow)
  - ``rx_dropped_invalid_header``: Malformed MCTP header (version, addressing)
  - ``rx_dropped_permission``: Socket filter/BPF drop
  - ``rx_dropped_timeout``: Reassembly timed out (missing fragments)

* **Timestamps:**
  - ``last_tx_time``: Last transmission time (nanoseconds since boot)
  - ``last_rx_time``: Last reception time (nanoseconds since boot)

* **Connection Info:**
  - ``num_active_keys``: Number of active MCTP keys
  - ``bind_net``: Bound network ID
  - ``bind_addr``: Bound EID address
  - ``bind_type``: Bound message type

Global Statistics
-----------------

System-wide statistics are available via two interfaces:

1. **Generic Netlink** (primary, programmatic access)
2. **``/proc/net/mctp/stats``** (convenience, human-readable)

The Netlink interface provides structured binary data suitable for monitoring
tools, while the /proc interface is convenient for debugging.

Per-Socket List
---------------

A list of all active MCTP sockets with their individual statistics is available
via **``/proc/net/mctp/sockets``** (similar to ``/proc/net/tcp``).

This shows each socket's PID, EID, type, network, state, and detailed statistics,
allowing system administrators to identify which applications are using MCTP
and their traffic patterns.

**Available Metrics:**

* **Socket Counts:**
  - ``num_sockets``: Total active sockets
  - ``num_bound_sockets``: Total bound sockets

* **Aggregate TX/RX Counters:** Same as per-socket, but summed across all sockets

* **Aggregate Drop Reasons:** Same as per-socket, but summed across all sockets

Network Device (Netdev) Statistics
===================================

In addition to socket-layer statistics, each MCTP network device (binding) provides
detailed hardware/transport-layer statistics via **ethtool**. These stats track
errors and events at the physical transport layer (I2C, USB, SPI, etc.).

**Key Differences:**

* **Socket stats:** Track data flow at the MCTP protocol layer (route.c, af_mctp.c)
* **Netdev stats:** Track operations at the hardware/transport layer (drivers)

Accessing Netdev Stats
-----------------------

Use the standard ``ethtool -S`` command:

.. code-block:: bash

    # Show statistics for a specific MCTP device
    ethtool -S mctp0
    
    # Show stats for all MCTP devices
    for dev in mctp*; do echo "=== $dev ==="; ethtool -S $dev; done

Available Metrics by Binding Type
----------------------------------

Each MCTP binding type provides transport-specific statistics:

MCTP over I2C (mctp-i2c)
~~~~~~~~~~~~~~~~~~~~~~~~

* **Error Counters:**
  - ``tx_errors_i2c``: I2C bus errors during transmit
  - ``rx_errors_i2c``: I2C bus errors during receive
  - ``pec_errors``: Packet Error Code (PEC) failures
  - ``nak_recv``: I2C NAK (not acknowledged) received
  - ``arbitration_lost``: I2C arbitration lost
  - ``bus_error``: I2C bus protocol errors

* **Retry Tracking:**
  - ``tx_retries``: Number of transmission retries
  - ``max_retries_hit``: Times maximum retry count was reached

* **Traffic:**
  - ``tx_bytes``: Bytes transmitted
  - ``rx_bytes``: Bytes received
  - ``tx_packets``: Packets transmitted
  - ``rx_packets``: Packets received

MCTP over USB (mctp-usb)
~~~~~~~~~~~~~~~~~~~~~~~~~

* **URB Management:**
  - ``tx_urb_submit_errors``: URB submission failures
  - ``rx_urb_submit_errors``: RX URB submission failures
  - ``tx_urb_completion_errors``: TX URB completion errors
  - ``rx_urb_completion_errors``: RX URB completion errors

* **Drop Reasons:**
  - ``tx_dropped_no_mem``: TX drops due to memory allocation failure
  - ``tx_dropped_no_route``: TX drops due to no route
  - ``rx_dropped_early_exit``: RX drops in early validation
  - ``rx_dropped_packet_length``: RX drops due to invalid length

* **Error Types:**
  - ``tx_errors_eagain``: EAGAIN errors (resource temporarily unavailable)
  - ``tx_errors_enodev``: ENODEV errors (device disconnected)
  - ``tx_errors_eshutdown``: ESHUTDOWN errors (device shutdown)
  - ``tx_errors_other``: Other transmission errors

* **Traffic:**
  - ``tx_bytes``: Bytes transmitted
  - ``rx_bytes``: Bytes received
  - ``tx_packets``: Packets transmitted
  - ``rx_packets``: Packets received

MCTP over I3C (mctp-i3c)
~~~~~~~~~~~~~~~~~~~~~~~~~

* **Device Management:**
  - ``devices_added``: I3C devices dynamically added
  - ``devices_removed``: I3C devices removed
  - ``ibi_events``: In-Band Interrupt events received

* **Per-Device Stats:**
  - ``device_X_tx_bytes``: TX bytes per I3C device
  - ``device_X_rx_bytes``: RX bytes per I3C device
  - ``device_X_tx_packets``: TX packets per I3C device
  - ``device_X_rx_packets``: RX packets per I3C device
  - ``device_X_errors``: Errors per I3C device

* **Global Traffic:**
  - ``tx_bytes``: Total bytes transmitted
  - ``rx_bytes``: Total bytes received
  - ``tx_packets``: Total packets transmitted
  - ``rx_packets``: Total packets received

MCTP over Serial (mctp-serial)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* **Protocol Tracking:**
  - ``state_idle``: Time in idle state
  - ``state_receiving``: Time in receiving state
  - ``state_escape``: Time processing escape sequences
  - ``fcs_errors``: Frame Check Sequence errors

* **Traffic:**
  - ``tx_bytes``: Bytes transmitted
  - ``rx_bytes``: Bytes received
  - ``tx_packets``: Packets transmitted
  - ``rx_packets``: Packets received

MCTP over SPI (mctp-spi)
~~~~~~~~~~~~~~~~~~~~~~~~~

* **Interrupt Tracking:**
  - ``gpio_interrupts``: GPIO interrupts received
  - ``spurious_interrupts``: Spurious interrupts

* **SPI Errors:**
  - ``spi_transfer_errors``: SPI transfer failures
  - ``tx_errors``: Transmission errors
  - ``rx_errors``: Reception errors

* **Traffic:**
  - ``tx_bytes``: Bytes transmitted
  - ``rx_bytes``: Bytes received
  - ``tx_packets``: Packets transmitted
  - ``rx_packets``: Packets received

Example Output
--------------

**MCTP over USB:**

.. code-block:: text

    # ethtool -S mctp0
    NIC statistics:
         tx_bytes: 45678
         rx_bytes: 67890
         tx_packets: 234
         rx_packets: 345
         tx_urb_submit_errors: 0
         rx_urb_submit_errors: 0
         tx_urb_completion_errors: 0
         rx_urb_completion_errors: 2
         tx_dropped_no_mem: 0
         tx_dropped_no_route: 0
         rx_dropped_early_exit: 1
         rx_dropped_packet_length: 0
         tx_errors_eagain: 0
         tx_errors_enodev: 0
         tx_errors_eshutdown: 0
         tx_errors_other: 0

**MCTP over I2C:**

.. code-block:: text

    # ethtool -S mctp0
    NIC statistics:
         tx_bytes: 12345
         rx_bytes: 23456
         tx_packets: 100
         rx_packets: 150
         tx_errors_i2c: 2
         rx_errors_i2c: 1
         pec_errors: 0
         nak_recv: 2
         arbitration_lost: 0
         bus_error: 0
         tx_retries: 5
         max_retries_hit: 1

Integration with tc (Traffic Control)
--------------------------------------

In addition to ethtool statistics, you can use ``tc`` to monitor device queue
statistics:

.. code-block:: bash

    # Show queue statistics (dropped packets, backlog)
    tc -s qdisc show dev mctp0
    
    # Example output:
    # qdisc pfifo_fast 0: root refcnt 2 bands 3 priomap ...
    #  Sent 12345 bytes 100 pkt (dropped 5, overlimits 0 requeues 0)
    #  backlog 1024b 5p requeues 0

**Key metrics from tc:**

* ``dropped``: Packets dropped at device queue (queue full)
* ``backlog``: Current queue depth (bytes and packets)
* ``overlimits``: Times queue limit was exceeded

**Note:** ``tc`` tracks the **transmit queue** at the device layer, while socket-layer
``tx_dropped_queue_full`` tracks the **device queue full** condition from the socket's perspective.

Complete Statistics Stack
--------------------------

MCTP provides statistics at three layers:

.. code-block:: text

    ┌─────────────────────────────────────────┐
    │  Application Layer                      │
    │  └─ getsockopt(MCTP_OPT_SOCK_STATS)    │  ← Per-socket stats
    │     What: Protocol-level stats          │
    │     Tool: Application API               │
    └─────────────────────────────────────────┘
                      ↓
    ┌─────────────────────────────────────────┐
    │  Socket Layer (MCTP Stack)              │
    │  └─ /proc/net/mctp/sockets             │  ← Socket list + stats
    │  └─ /proc/net/mctp/stats               │  ← Global aggregates
    │     What: Routing, fragmentation        │
    │     Tool: procfs, Netlink               │
    └─────────────────────────────────────────┘
                      ↓
    ┌─────────────────────────────────────────┐
    │  Network Device Layer (Bindings)        │
    │  └─ ethtool -S mctp0                   │  ← Transport stats
    │  └─ tc -s qdisc show dev mctp0         │  ← Queue stats
    │     What: I2C/USB/SPI hardware errors   │
    │     Tool: ethtool, tc                   │
    └─────────────────────────────────────────┘

**When to use each:**

* **Socket stats:** Diagnose application or protocol issues (drops, fragmentation)
* **Netdev stats:** Diagnose hardware/transport issues (I2C errors, USB failures)
* **tc stats:** Diagnose queue/congestion issues (backlog, device queue full)

Usage Examples
==============

Per-Socket Statistics (C API)
------------------------------

.. code-block:: c

    #include <sys/socket.h>
    #include <linux/mctp.h>

    int fd;
    struct mctp_sock_stats_info stats;
    socklen_t len = sizeof(stats);

    fd = socket(AF_MCTP, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        return 1;
    }

    if (getsockopt(fd, SOL_MCTP, MCTP_OPT_SOCK_STATS, &stats, &len) < 0) {
        perror("getsockopt");
        return 1;
    }

    printf("TX: %llu bytes, %llu packets, %llu messages\\n",
           stats.tx_bytes, stats.tx_packets, stats.tx_messages);
    printf("RX: %llu bytes, %llu packets, %llu messages\\n",
           stats.rx_bytes, stats.rx_packets, stats.rx_messages);
    printf("TX Drops: %llu (no route: %llu, tag exhaust: %llu)\\n",
           stats.tx_drops,
           stats.tx_dropped_no_route, stats.tx_dropped_tag_exhaustion);
    printf("RX Drops: %llu (seq mismatch: %llu, timeout: %llu)\\n",
           stats.rx_drops,
           stats.rx_dropped_seq_mismatch, stats.rx_dropped_timeout);

    close(fd);

Global Statistics via Netlink (C API)
--------------------------------------

.. code-block:: c

    #include <linux/genetlink.h>
    #include <linux/mctp.h>
    #include <libmnl/libmnl.h>

    /* Using libmnl (netlink library) */
    struct mnl_socket *nl;
    struct nlmsghdr *nlh;
    struct genlmsghdr *genl;
    int family_id;

    /* Open netlink socket */
    nl = mnl_socket_open(NETLINK_GENERIC);
    mnl_socket_bind(nl, 0, MNL_SOCKET_AUTOPID);

    /* Resolve family ID for "mctp" */
    family_id = genl_ctrl_resolve(nl, "mctp");

    /* Build request */
    nlh = mnl_nlmsg_put_header(buf);
    nlh->nlmsg_type = family_id;
    nlh->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
    nlh->nlmsg_seq = time(NULL);

    genl = mnl_nlmsg_put_extra_header(nlh, sizeof(struct genlmsghdr));
    genl->cmd = MCTP_CMD_GET_STATS;
    genl->version = 1;

    /* Send and receive */
    mnl_socket_sendto(nl, nlh, nlh->nlmsg_len);
    ret = mnl_socket_recvfrom(nl, buf, sizeof(buf));

    /* Parse response - will contain MCTP_ATTR_STATS with
     * struct mctp_global_stats
     */
    struct mctp_global_stats *stats;
    /* ... parse netlink attributes ... */

    printf("Global TX: %llu packets, %llu bytes\n",
           stats->tx_packets, stats->tx_bytes);

Global Statistics via /proc (Shell)
------------------------------------

.. code-block:: bash

    # View all global statistics (human-readable)
    cat /proc/net/mctp/stats

    # Monitor statistics in real-time
    watch -n 1 cat /proc/net/mctp/stats

**Example Output:**

.. code-block:: text

    Sockets:
      Active: 3
      Bound:  2

    TX Statistics:
      Bytes:    45678
      Packets:  234
      Messages: 156
      Errors:   0
      Drops:    5

    RX Statistics:
      Bytes:    67890
      Packets:  345
      Messages: 234
      Errors:   0
      Drops:    2

    TX Drop Reasons:
      No route:      2
      MTU exceeded:  0
      No memory:     0
      Queue full:    0
      Device down:   0
      Tag exhaust:   3
      Permission:    0

    RX Drop Reasons:
      No route:      0
      No memory:     0
      Seq mismatch:  0
      Tag mismatch:  0
      Queue full:    0
      Invalid hdr:   0
      Permission:    0
      Timeout:       2

Per-Socket Statistics via /proc (Shell)
----------------------------------------

.. code-block:: bash

    # View all MCTP sockets with their stats
    cat /proc/net/mctp/sockets

**Example Output:**

.. code-block:: text

    Socket List:
      PID    Net  Type     Local EID  Peer EID   State      TX Pkts   RX Pkts
      ---    ---  ----     ---------  --------   -----      -------   -------
      100    5    PLDM     255        20         BOUND      57        14
        TX Drops: No Route: 2, MTU Exceeded: 0, No Memory: 0, Queue Full: 0, Device Down: 0, Tag Exhaust: 0, Permission: 0
        RX Drops: No Route: 0, No Memory: 0, Seq Mismatch: 0, Tag Mismatch: 0, Queue Full: 0, Invalid Hdr: 0, Permission: 0, Timeout: 1

      250    5    SPDM     255        *          BOUND      1024      1024
        TX Drops: No Route: 0, MTU Exceeded: 0, No Memory: 0, Queue Full: 0, Device Down: 0, Tag Exhaust: 0, Permission: 0
        RX Drops: No Route: 0, No Memory: 0, Seq Mismatch: 5, Tag Mismatch: 0, Queue Full: 0, Invalid Hdr: 0, Permission: 0, Timeout: 0

Each socket entry shows:
- **PID:** Process ID of the application that created the socket
- **Type:** Message type (e.g., PLDM, SPDM)
- **Peer EID:** Destination EID (for connected sockets) or `*` (for listeners)
- **Detailed Drops:** Indented lines showing specific TX and RX drop counts

.. code-block:: bash

    # Monitor per-socket activity
    watch -n 1 cat /proc/net/mctp/sockets

    # Find which PID is dropping packets
    cat /proc/net/mctp/sockets | grep -B 1 "RX Drops.*Timeout: [1-9]"

Integration with Monitoring Tools
==================================

HMC Dumps
---------

Global statistics, per-socket information, and network device statistics should
be included in HMC (Hardware Management Console) dumps for complete post-mortem
analysis across all layers of the MCTP stack.

Complete HMC dump script:

.. code-block:: bash

    #!/bin/bash
    # mctp_hmc_dump.sh - Complete MCTP statistics for HMC dumps
    
    DUMP_FILE="/var/log/hmc_dump_mctp_$(date +%Y%m%d_%H%M%S).log"
    
    echo "=== MCTP HMC Dump - $(date) ===" >> "$DUMP_FILE"
    echo "" >> "$DUMP_FILE"
    
    # Socket layer statistics
    if [ -f /proc/net/mctp/stats ]; then
        echo "=== MCTP Global Statistics (Socket Layer) ===" >> "$DUMP_FILE"
        cat /proc/net/mctp/stats >> "$DUMP_FILE"
        echo "" >> "$DUMP_FILE"
    fi
    
    if [ -f /proc/net/mctp/sockets ]; then
        echo "=== MCTP Active Sockets ===" >> "$DUMP_FILE"
        cat /proc/net/mctp/sockets >> "$DUMP_FILE"
        echo "" >> "$DUMP_FILE"
    fi
    
    # Network device statistics
    echo "=== MCTP Network Device Statistics ===" >> "$DUMP_FILE"
    for dev in $(ip link show type mctp 2>/dev/null | grep -o 'mctp[0-9]*'); do
        echo "--- Device: $dev ---" >> "$DUMP_FILE"
        
        # Device status
        ip link show dev "$dev" >> "$DUMP_FILE" 2>&1
        echo "" >> "$DUMP_FILE"
        
        # Ethtool statistics (transport-specific)
        if command -v ethtool >/dev/null 2>&1; then
            ethtool -S "$dev" >> "$DUMP_FILE" 2>&1
            echo "" >> "$DUMP_FILE"
        fi
        
        # Traffic control queue statistics
        if command -v tc >/dev/null 2>&1; then
            tc -s -d qdisc show dev "$dev" >> "$DUMP_FILE" 2>&1
            echo "" >> "$DUMP_FILE"
        fi
    done
    
    # MCTP routing table
    echo "=== MCTP Routing Table ===" >> "$DUMP_FILE"
    ip mctp route show >> "$DUMP_FILE" 2>&1
    echo "" >> "$DUMP_FILE"
    
    # MCTP neighbors
    echo "=== MCTP Neighbors ===" >> "$DUMP_FILE"
    ip mctp neigh show >> "$DUMP_FILE" 2>&1
    
    echo "MCTP HMC dump saved to: $DUMP_FILE"

This captures:

* **Socket layer:** Global stats, per-socket stats with drop reasons
* **Device layer:** Transport-specific errors (ethtool), queue stats (tc)
* **Network config:** Routes, neighbors, device states

**Example captured data:**

.. code-block:: text

    === MCTP Global Statistics (Socket Layer) ===
    Sockets:
      Active: 3
      Bound:  2
    TX Statistics:
      Packets:  234
      Drops:    5
    TX Drop Reasons:
      Tag exhaust:   3
      No route:      2
    
    === MCTP Active Sockets ===
    Socket List:
      PID    Net  Type     Local EID  Peer EID   State      TX Pkts   RX Pkts
      ---    ---  ----     ---------  --------   -----      -------   -------
      100    5    PLDM     255        20         BOUND      57        14
        TX Drops: No Route: 2, MTU Exceeded: 0, No Memory: 0, Queue Full: 0, Device Down: 0, Tag Exhaust: 0, Permission: 0
        RX Drops: No Route: 0, No Memory: 0, Seq Mismatch: 0, Tag Mismatch: 0, Queue Full: 0, Invalid Hdr: 0, Permission: 0, Timeout: 1
    
    === MCTP Network Device Statistics ===
    --- Device: mctp0 ---
    NIC statistics:
         tx_bytes: 45678
         tx_packets: 234
         tx_urb_submit_errors: 0
         rx_urb_completion_errors: 2
    
    qdisc pfifo_fast 0: root
     Sent 45678 bytes 234 pkt (dropped 0)
     backlog 0b 0p

This provides complete visibility from application down to hardware for debugging
any MCTP communication issue.

Custom Monitoring
-----------------

Applications can periodically query per-socket statistics to monitor their
own MCTP communication health:

.. code-block:: c

    void monitor_mctp_health(int fd) {
        struct mctp_sock_stats_info stats;
        socklen_t len = sizeof(stats);
        
        if (getsockopt(fd, SOL_MCTP, MCTP_OPT_SOCK_STATS, &stats, &len) == 0) {
            // Check for excessive drops
            if (stats.tx_drops > 100 || stats.rx_drops > 100) {
                log_warning("High MCTP drop rate detected");
            }
            
            // Check for sequence mismatches (fragmentation issues)
            if (stats.rx_dropped_seq_mismatch > 10) {
                log_warning("MCTP fragmentation issues detected");
            }
            
            // Check for queue full (application slow to read)
            if (stats.rx_dropped_queue_full > 20) {
                log_warning("MCTP socket receive queue full - application too slow");
            }
            
            // Check for device issues
            if (stats.tx_dropped_device_down > 0) {
                log_error("MCTP device is down");
            }
            
            // Check for timeout (reassembly failure)
            if (stats.rx_dropped_timeout > 5) {
                log_error("MCTP reassembly timeout - check I2C bus health");
            }
        }
    }

Troubleshooting
===============

High Drop Rates
---------------

If you observe high drop rates in the statistics:

1. **Identify problem socket:**

   .. code-block:: bash

       # Find which socket has drops
       cat /proc/net/mctp/sockets | grep -B 1 "Drops.*: [1-9]"
       # Shows sockets with non-zero drops

2. **No Route Drops:** Check routing configuration
   
   .. code-block:: bash
   
       ip mctp route show
       # Verify route exists for destination EID

3. **Sequence Mismatches / Timeouts:** May indicate packet loss or reordering
   - Check physical layer (I2C, USB, etc.)
   - Verify MTU settings
   - Check with: ``grep "Seq mismatch" /proc/net/mctp/stats`` or ``grep "Timeout" /proc/net/mctp/stats``

4. **Memory Drops:** System under memory pressure
   - Check system memory availability
   - Reduce MCTP traffic load

5. **MTU Exceeded:** Messages too large for configured MTU
   - Increase MTU if possible
   - Fragment messages at application layer

6. **Tag Exhaustion:** Too many concurrent requests
   - Reduce concurrency or increase timeout
   - Check for stuck requests

Zero Statistics
---------------

If all statistics show zero:

1. Verify MCTP module is loaded: ``lsmod | grep mctp``
2. Check that statistics module is compiled: ``ls /proc/net/mctp/``
3. Ensure socket is actually sending/receiving data

Performance Considerations
==========================

The statistics implementation uses:

* **Per-socket stats:** Spinlock-protected counters (minimal overhead)
* **Global stats:** Atomic operations (lockless, very low overhead)

Statistics updates add negligible overhead to the MCTP data path and are
safe to use in production environments.

Quick Reference
===============

Socket Layer Statistics
------------------------

**Per-Socket Stats (C API):**

::

    #include <linux/mctp.h>
    
    int fd = socket(AF_MCTP, SOCK_DGRAM, 0);
    struct mctp_sock_stats_info stats;
    socklen_t len = sizeof(stats);
    getsockopt(fd, SOL_MCTP, MCTP_OPT_SOCK_STATS, &stats, &len);
    
    // Access: tx_bytes, rx_packets, rx_dropped_seq_mismatch, etc.

**Per-Socket List (Shell):**

::

    cat /proc/net/mctp/sockets
    # Shows: PID, EID, type, network, state, TX/RX stats, drop reasons

**Global Stats (Netlink):**

::

    Family: "mctp"
    Command: MCTP_CMD_GET_STATS
    Returns: MCTP_ATTR_STATS (struct mctp_global_stats)

**Global Stats (Shell):**

::

    cat /proc/net/mctp/stats
    # Shows: System-wide aggregated stats with split TX/RX drop reasons

Device Layer Statistics
------------------------

**Transport-Specific Stats (ethtool):**

::

    ethtool -S mctp0
    # Shows: I2C/USB/SPI errors, retries, per-transport metrics
    
    # For all MCTP devices:
    for dev in mctp*; do echo "=== $dev ==="; ethtool -S $dev; done

**Queue Stats (tc):**

::

    tc -s qdisc show dev mctp0
    # Shows: Queue depth (backlog), dropped packets, queue full events
    
    # Real-time monitoring:
    watch -n 1 'tc -s qdisc show dev mctp0'

Complete System View
--------------------

**All layers at once:**

::

    # Socket layer
    cat /proc/net/mctp/stats
    cat /proc/net/mctp/sockets
    
    # Device layer
    ethtool -S mctp0
    tc -s qdisc show dev mctp0
    
    # Network config
    ip mctp route show
    ip link show type mctp

Statistics Hierarchy
--------------------

::

    Application → getsockopt(MCTP_OPT_SOCK_STATS)
           ↓
    Socket Layer → /proc/net/mctp/{stats,sockets}
           ↓
    Device Layer → ethtool -S mctp0
           ↓
    Queue Layer → tc -s qdisc show dev mctp0
           ↓
    Hardware (I2C/USB/SPI bus)

See Also
========

**MCTP Documentation:**

* Documentation/networking/mctp.rst - MCTP protocol overview
* include/uapi/linux/mctp.h - MCTP UAPI definitions and structures

**Related Tools:**

* ``ethtool(8)`` - Display and change Ethernet device settings (including statistics)
* ``tc(8)`` - Traffic control (queue management and statistics)
* ``ip-mctp(8)`` - MCTP route and neighbor management

**Driver-Specific:**

* drivers/net/mctp/mctp-i2c.c - I2C binding with PEC, retry stats
* drivers/net/mctp/mctp-usb.c - USB binding with URB tracking
* drivers/net/mctp/mctp-i3c.c - I3C binding with IBI events, per-device stats
* drivers/net/mctp/mctp-serial.c - Serial binding with FCS tracking
* drivers/net/mctp/mctp-spi.c - SPI binding with GPIO interrupt tracking
