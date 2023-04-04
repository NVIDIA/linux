<<<<<<< HEAD
# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the PECI core drivers.
#

# Core functionality
obj-$(CONFIG_PECI)		+= peci-core.o
obj-$(CONFIG_PECI_CHARDEV)	+= peci-dev.o

# Hardware specific bus drivers
obj-y				+= busses/
=======
# SPDX-License-Identifier: GPL-2.0-only

# Core functionality
peci-y := core.o request.o device.o sysfs.o
obj-$(CONFIG_PECI) += peci.o
peci-cpu-y := cpu.o
obj-$(CONFIG_PECI_CPU) += peci-cpu.o

# Hardware specific bus drivers
obj-y += controller/
>>>>>>> origin/linux_6.1.15_upstream
