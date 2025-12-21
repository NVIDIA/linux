Virtual I2C Mux Architecture
1. Purpose and Scope
This document describes a Remote I2C FRU Bridge architecture that allows FRU EEPROM devices physically connected to one system to be exposed to another system’s IPMI / FRU stack as if they were locally attached I2C devices.
The design is:
General-purpose at the I2C transport level
Specifically intended for FRU EEPROM access
Compatible with stock Linux I2C drivers (at24) and OpenBMC IPMI/FRU services
Transparent to user-space and IPMI layers
This architecture is intended for platforms where:
One device (“I2C Host”) runs IPMI services
Another device (“I2C Client”) physically owns the FRU EEPROMs
The two devices are connected by an I2C link

2. High-Level Overview
The solution presents FRU EEPROMs to the I2C Host as standard Linux I2C devices by introducing:
• A Virtual Mux Host Adapter on the Host
• A Virtual Mux Client on the Client
All I2C transactions issued by standard kernel drivers (e.g. at24) on the Host are forwarded across the Link I2C Bus to the Client, executed on the Client’s Downstream I2C Bus, and returned transparently.
+------------------+                       +------------------+
|     I2C Host     |                       |    I2C Client    |
|                  |                       |                  |
|  IPMI / FRU      |                       |  FRU EEPROM(s)   |
|  at24 EEPROM     |                       |  Downstream I2C  |
|        |         |                       |        |         |
|  Virtual Mux     |                       |  I2C Master      |
|  Adapter Driver  |                       |                  |
|        |         |                       |  Virtual Mux     |
|        |         |                       |  Client          |
|  Link I2C Master |<---- Link I2C ---->   |  I2C Slave       |
+------------------+                       +------------------+

3. Design Goals
Functional Goals
Allow unmodified OpenBMC FRU/IPMI stack to access remote FRU EEPROMs
Preserve I2C transaction semantics, including:
Multi-message transfers
Repeated START conditions
Support multiple FRU devices (0x50, 0x51, etc.)
Non-Goals
High-performance I2C tunneling
General peripheral virtualization beyond FRU EEPROMs
Hot-plug or dynamic bus discovery

4. Host Side Architecture (Virtual Mux Host Adapter)
Responsibilities
Register a virtual I2C adapter with the Linux I2C core
Accept I2C transfers from kernel clients (i2c_transfer)
Serialize and forward transfers to the Client
Deserialize responses and complete the original transfer
Integration Points
Implements struct i2c_algorithm
Uses .master_xfer() as the forwarding hook
Appears as a normal /dev/i2c-X adapter
Standard drivers (at24) bind normally
Key Characteristics
No FRU or EEPROM awareness
Transaction-level forwarding only
All policy and protocol logic lives in the driver

5. Client Side Architecture (Virtual Mux Client)
Responsibilities
Register as an I2C target/slave on the Link I2C Bus
Receive serialized I2C transaction requests
Execute transactions on a local I2C master adapter
Return status and read data to the Host
Execution Model
Incoming data captured in I2C slave callbacks
Actual downstream i2c_transfer() executed in process context (workqueue)
Supports atomic multi-message replay
Downstream Constraints
EEPROM addressing width (1-byte vs 2-byte) is handled naturally by at24
Client does not interpret FRU format
Client does not cache data (optional future enhancement)
Security and mapping
Client enforces a whitelist mapping from virtual address → (downstream bus, downstream addr) configured via Device Tree. The host never selects downstream bus/address.

6. I2C Transaction Forwarding Model
Preserved Semantics
Entire struct i2c_msg[] array forwarded as a single unit
Read/write flags preserved
Repeated START behavior preserved
NACK / timeout errors propagated
Error Handling
Downstream errors mapped to Linux -errno
Errors returned verbatim to Host’s i2c_transfer()
IPMI FRU stack sees standard failures

7. Communication Protocol
The Host and Client communicate using a simple framed binary protocol over the Link I2C Bus.
Design Requirements
Framing to allow variable-length transactions
Message integrity (optional CRC)
Transaction correlation (sequence number)
Protocol Layers
Transport: I2C
Payload: Remote I2C transaction frame
Semantics: Linux i2c_transfer() equivalent
The protocol is intentionally minimal and private to this implementation.

Virtual Mux Protocol
Design constraints (why the protocol looks like this)
Link transport is I2C master (Host) ↔ target/slave (Client).
I2C doesn’t give you “packet boundaries” beyond START/STOP.
The Host must be able to send variable-size requests and then fetch variable-size responses.
We must support multi-message transfers (struct i2c_msg[]) because EEPROM reads are typically:
write offset bytes (no STOP) + repeated-start + read N bytes
To keep this robust, the protocol uses two operations per transaction:
SUBMIT request (Host writes a frame to Client)
FETCH response (Host reads back the response frame)
This avoids timing issues and doesn’t require clock stretching.

Link bus behavior (wire level)
Submit phase (Host → Client)
Host performs an I2C write to the Client’s bridge address (e.g. 0x2a) containing a Request Frame.
Fetch phase (Host ← Client)
Host performs an I2C read from the same address and reads out a Response Frame.
If the Host reads “too early”, Client returns a response with status -EAGAIN (or a “not-ready” code) until it finishes the downstream transfer.

Framing
All frames are self-length-delimited so the receiver knows when it has a complete message.
All multi-byte integers are little-endian.
Common header
offset size  field
0      2     magic
2      1     version
3      1     msg_type
4      1     seq
5      1     header_len      (bytes, including common header)
6      2     total_len       (bytes, entire frame including CRC if present)
8      2     flags
10     2     reserved

magic:
request frames: 0x4952 ("RI")
response frames: 0x4F52 ("RO")
version: 0x03
seq: Host increments each SUBMIT; response echoes it.
flags:
bit0: CRC16 present (optional)
bit1: RESERVED
bit2: RESERVED
total_len lets the receiver know when the full frame is received.
CRC (optional but recommended):
CRC16-CCITT over the entire frame excluding the CRC field itself
CRC placed at the end: last 2 bytes of frame

Message types
Request: SUBMIT_XFER (msg_type = 0x01)
Payload layout
Immediately after the common header:
offset size field
...   1    nmsgs
...   1    retry_hint       (0 = default)
...   2    timeout_ms_hint  (0 = default)
...   4    client_cookie    (optional; Host can set 0)
...   var  msg descriptors...

Per-message descriptor
For i = 0..nmsgs-1:
u8   addr_7bit        (0x00..0x7f)
u8   msg_flags        bit0=READ (1), bit1=NO_STOP (1) reserved (see note)
u16  len
u8   data[len]        present only for WRITE msgs

Note on STOP / repeated-start semantics:
Linux i2c_transfer() treats an array of msgs as one transaction; repeated-start behavior is implied.
We don’t need an explicit NO_STOP flag if we always execute all nmsgs in one downstream i2c_transfer().
So you can keep msg_flags only as READ/WRITE and ignore NO_STOP. (I left the bit in case you ever need it.)
Constraints for FRU usage (recommended enforcement):
nmsgs <= 4
each len <= 256 (or 512)
total request size must fit within a bounded buffer (e.g., 1–4 KB)

Address selection and security:
- Host uses only virtual 7-bit I2C addresses in addr_7bit.
- Client maps each virtual address to a specific downstream bus and 7-bit device address according to its Device Tree mapping. Requests for unmapped virtual addresses are rejected with -EPERM.
Response: XFER_RESULT (msg_type = 0x81)
Payload layout
Immediately after the common header:
s16  status           (0=OK, negative Linux errno on failure)
u8   nread_msgs
u8   reserved
u16  reserved2
var  read payloads...

Read payloads
For each READ msg in the original request, in the same order:
u16  len
u8   data[len]

If status != 0, nread_msgs may be 0 and no payloads follow.

Response: NOT_READY (msg_type = 0x82) (optional)
Client may return a header-only NOT_READY frame when the result isn’t ready yet.
Details:
- Header only, total_len = sizeof(header)
- seq = 0 (don’t-care)
- flags = 0 (no CRC, no payload)
- No payload bytes follow


Host transaction flow (exact)
Build SUBMIT_XFER(seq=N, msgs=[...])
i2c_master_send(link_client, frame, total_len)
Poll fetch (typically a few tries; FRU access is slow anyway):
1) Read header first (12 bytes). If msg_type == NOT_READY, sleep briefly and retry header read.
2) If XFER_RESULT, read the remaining payload bytes per total_len, then validate seq and CRC (if present).
On success:
Copy returned read data into the corresponding original i2c_msg[].buf
Return nmsgs from .master_xfer() (Linux convention)

Client behavior (exact)
On receiving a complete SUBMIT frame (detected by total_len):
Validate magic/version/CRC
Store request + seq
Schedule workqueue to execute:
Convert descriptors into struct i2c_msg[]
Run i2c_transfer(downstream_adap, msgs, nmsgs)
Capture errno + any read buffers
Build response frame XFER_RESULT(seq=N, status, read_payloads)
On Host read:
If latest response for that seq is ready, stream it out
Else return NOT_READY (header-only)
Recommended simplification: allow only one outstanding transaction at a time (serialize). That’s completely fine for FRU.

Sizes (practical defaults)
Max request: 2048 bytes
Max response: 4096 bytes (FRU reads can be larger; but typical is small chunks)
Typical at24 read: offset write (1–2 bytes) + read (16–32 bytes), repeated.
If you need larger reads, the Host driver can naturally chunk them (at24 already does).

Mapping to struct i2c_msg (downstream execution)
For each descriptor:
addr = addr_7bit
if READ: flags = I2C_M_RD, len = len, buf = temp read buf
if WRITE: flags = 0, len = len, buf = pointer into request payload
Then call i2c_transfer() once with the msg array.

Device Tree Bindings (current)
Host (virtual-mux-host):
- compatible = "virtual-mux-host"
- reg = <link_address> (e.g., 0x51)
- virtual-bus-num = <X> (fixed virtual adapter number)
- use-crc16; (optional boolean)
- status = "disabled" by default; enable via overlay or sysfs new_device when ready

Client (virtual-mux-client):
- compatible = "virtual-mux-client"
- Child nodes describe whitelist mappings:
  map@<virt_addr> {
    reg = <0x50>;                 // virtual address visible to host
    downstream-bus = <1>;         // client local bus number
    downstream-addr = <0x50>;     // 7-bit address on that bus
  };
- use-crc16; (optional boolean)
All host requests to unmapped virtual addresses are denied (-EPERM).



9. Scalability and Limits
Aspect
Supported
Multiple FRU EEPROMs
Yes
Multiple transactions
Yes
EEPROM sizes
Any supported by at24
Concurrent access
Serialized
Bus sharing
Link bus may be shared if address-unique

10. Security and Isolation
No authentication or encryption (assumes trusted internal bus)
Client only executes I2C operations for addresses explicitly whitelisted in DT
No memory or register exposure beyond I2C

11. Summary
This architecture provides a transparent, kernel-level mechanism to expose remote FRU EEPROMs over I2C without modifying OpenBMC’s IPMI or FRU services. It leverages existing Linux I2C infrastructure and cleanly separates Host and Client responsibilities while preserving I2C semantics required by standard EEPROM drivers. The host only uses virtual addresses; the client enforces mapping and security via Device Tree.
