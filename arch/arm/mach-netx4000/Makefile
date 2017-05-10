# netX4000 Architecture Makefile
#
# (C) Copyright 2014
# Hilscher Gesellschaft fuer Systemautomation mbH, <www.hilscher.com>
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
#

# netx4000_regdef.h brings typedefs for all modules,
# which throws a warning about unused typedefs
CFLAGS:=$(filter-out -Wunused-local-typedefs,$(CFLAGS))

obj-$(CONFIG_ARCH_NETX4000) += lowlevel.o generic.o ddr400-init.o ddr600-init.o device-label.o
obj-$(CONFIG_CACHE_L2X0) += cache-l2x0.o
