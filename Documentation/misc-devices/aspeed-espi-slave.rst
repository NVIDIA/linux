.. SPDX-License-Identifier: GPL-2.0

==========
eSPI Slave
==========

:Author: Haiyue Wang <haiyue.wang@linux.intel.com>

The PCH (**eSPI master**) provides the eSPI to support connection of a
BMC (**eSPI slave**) to the platform.

The LPC and eSPI interfaces are mutually exclusive. Both use the same
pins, but on power-up, a HW strap determines if the eSPI or the LPC bus
is operational. Once selected, it’s not possible to change to the other
interface.

``eSPI Channels and Supported Transactions``
 +------+---------------------+----------------------+--------------------+
 | CH # | Channel             | Posted Cycles        | Non-Posted Cycles  |
 +======+=====================+======================+====================+
 |  0   | Peripheral          | Memory Write,        | Memory Read,       |
 |      |                     | Completions          | I/O Read/Write     |
 +------+---------------------+----------------------+--------------------+
 |  1   | Virtual Wire        | Virtual Wire GET/PUT | N/A                |
 +------+---------------------+----------------------+--------------------+
 |  2   | Out-of-Band Message | SMBus Packet GET/PUT | N/A                |
 +------+---------------------+----------------------+--------------------+
 |  3   | Flash Access        | N/A                  | Flash Read, Write, |
 |      |                     |                      | Erase              |
 +------+---------------------+----------------------+--------------------+
 |  N/A | General             | Register Accesses    | N/A                |
 +------+---------------------+----------------------+--------------------+

Virtual Wire Channel (Channel 1) Overview
-----------------------------------------

The Virtual Wire channel uses a standard message format to communicate
several types of signals between the components on the platform::

 - Sideband and GPIO Pins: System events and other dedicated signals
   between the PCH and eSPI slave. These signals are tunneled between the
   two components over eSPI.

 - Serial IRQ Interrupts: Interrupts are tunneled from the eSPI slave to
   the PCH. Both edge and triggered interrupts are supported.

When PCH runs on eSPI mode, from BMC side, the following VW messages are
done in firmware::

 1. SLAVE_BOOT_LOAD_DONE / SLAVE_BOOT_LOAD_STATUS
 2. SUS_ACK
 3. OOB_RESET_ACK
 4. HOST_RESET_ACK

``eSPI Virtual Wires (VW)``
 +----------------------+---------+---------------------------------------+
 |Virtual Wire          |PCH Pin  |Comments                               |
 |                      |Direction|                                       |
 +======================+=========+=======================================+
 |SUS_WARN#             |Output   |PCH pin is a GPIO when eSPI is enabled.|
 |                      |         |eSPI controller receives as VW message.|
 +----------------------+---------+---------------------------------------+
 |SUS_ACK#              |Input    |PCH pin is a GPIO when eSPI is enabled.|
 |                      |         |eSPI controller receives as VW message.|
 +----------------------+---------+---------------------------------------+
 |SLAVE_BOOT_LOAD_DONE  |Input    |Sent when the BMC has completed its    |
 |                      |         |boot process as an indication to       |
 |                      |         |eSPI-MC to continue with the G3 to S0  |
 |                      |         |exit.                                  |
 |                      |         |The eSPI Master waits for the assertion|
 |                      |         |of this virtual wire before proceeding |
 |                      |         |with the SLP_S5# deassertion.          |
 |                      |         |The intent is that it is never changed |
 |                      |         |except on a G3 exit - it is reset on a |
 |                      |         |G3 entry.                              |
 +----------------------+---------+---------------------------------------+
 |SLAVE_BOOT_LOAD_STATUS|Input    |Sent upon completion of the Slave Boot |
 |                      |         |Load from the attached flash. A stat of|
 |                      |         |1 indicates that the boot code load was|
 |                      |         |successful and that the integrity of   |
 |                      |         |the image is intact.                   |
 +----------------------+---------+---------------------------------------+
 |HOST_RESET_WARN       |Output   |Sent from the MC just before the Host  |
 |                      |         |is about to enter reset. Upon receiving|
 |                      |         |, the BMC must flush and quiesce its   |
 |                      |         |upstream Peripheral Channel request    |
 |                      |         |queues and assert HOST_RESET_ACK VWire.|
 |                      |         |The MC subsequently completes any      |
 |                      |         |outstanding posted transactions or     |
 |                      |         |completions and then disables the      |
 |                      |         |Peripheral Channel via a write to      |
 |                      |         |the Slave's Configuration Register.    |
 +----------------------+---------+---------------------------------------+
 |HOST_RESET_ACK        |Input    |ACK for the HOST_RESET_WARN message    |
 +----------------------+---------+---------------------------------------+
 |OOB_RESET_WARN        |Output   |Sent from the MC just before the OOB   |
 |                      |         |processor is about to enter reset. Upon|
 |                      |         |receiving, the BMC must flush and      |
 |                      |         |quiesce its OOB Channel upstream       |
 |                      |         |request queues and assert OOB_RESET_ACK|
 |                      |         |VWire. The-MC subsequently completes   |
 |                      |         |any outstanding posted transactions or |
 |                      |         |completions and then disables the OOB  |
 |                      |         |Channel via a write to the Slave's     |
 |                      |         |Configuration Register.                |
 +----------------------+---------+---------------------------------------+
 |OOB_RESET_ACK         |Input    |ACK for OOB_RESET_WARN message         |
 +----------------------+---------+---------------------------------------+

`Intel C620 Series Chipset Platform Controller Hub
<https://www.intel.com/content/www/us/en/chipsets/c620-series-chipset-datasheet.html>`_

   -- 17. Enhanced Serial Peripheral Interface


`Enhanced Serial Peripheral Interface (eSPI)
- Interface Base Specification (for Client and Server Platforms)
<https://www.intel.com/content/dam/support/us/en/documents/software/chipset-software/327432-004_espi_base_specification_rev1.0.pdf>`_
