.. SPDX-License-Identifier: GPL-2.0

Kernel driver peci-dimmtemp
===========================

<<<<<<< HEAD
:Copyright: |copy| 2018-2019 Intel Corporation

=======
>>>>>>> origin/linux_6.1.15_upstream
Supported chips:
	One of Intel server CPUs listed below which is connected to a PECI bus.
		* Intel Xeon E5/E7 v3 server processors
			Intel Xeon E5-14xx v3 family
			Intel Xeon E5-24xx v3 family
			Intel Xeon E5-16xx v3 family
			Intel Xeon E5-26xx v3 family
			Intel Xeon E5-46xx v3 family
			Intel Xeon E7-48xx v3 family
			Intel Xeon E7-88xx v3 family
		* Intel Xeon E5/E7 v4 server processors
			Intel Xeon E5-16xx v4 family
			Intel Xeon E5-26xx v4 family
			Intel Xeon E5-46xx v4 family
			Intel Xeon E7-48xx v4 family
			Intel Xeon E7-88xx v4 family
		* Intel Xeon Scalable server processors
			Intel Xeon D family
			Intel Xeon Bronze family
			Intel Xeon Silver family
			Intel Xeon Gold family
			Intel Xeon Platinum family

<<<<<<< HEAD
	Addresses scanned: PECI client address 0x30 - 0x37
	Datasheet: Available from http://www.intel.com/design/literature.htm

Author:
	Jae Hyun Yoo <jae.hyun.yoo@linux.intel.com>
=======
	Datasheet: Available from http://www.intel.com/design/literature.htm

Author: Jae Hyun Yoo <jae.hyun.yoo@linux.intel.com>
>>>>>>> origin/linux_6.1.15_upstream

Description
-----------

<<<<<<< HEAD
This driver implements a generic PECI hwmon feature which provides Digital
Thermal Sensor (DTS) thermal readings of DIMM components that are accessible
using the PECI Client Command Suite via the processor PECI client.
=======
This driver implements a generic PECI hwmon feature which provides
Temperature sensor on DIMM readings that are accessible via the processor PECI interface.
>>>>>>> origin/linux_6.1.15_upstream

All temperature values are given in millidegree Celsius and will be measurable
only when the target CPU is powered on.

<<<<<<< HEAD
``sysfs`` interface
-------------------
=======
Sysfs interface
-------------------

>>>>>>> origin/linux_6.1.15_upstream
======================= =======================================================

temp[N]_label		Provides string "DIMM CI", where C is DIMM channel and
			I is DIMM index of the populated DIMM.
temp[N]_input		Provides current temperature of the populated DIMM.
temp[N]_max		Provides thermal control temperature of the DIMM.
temp[N]_crit		Provides shutdown temperature of the DIMM.
<<<<<<< HEAD
=======

>>>>>>> origin/linux_6.1.15_upstream
======================= =======================================================

Note:
	DIMM temperature attributes will appear when the client CPU's BIOS
	completes memory training and testing.
