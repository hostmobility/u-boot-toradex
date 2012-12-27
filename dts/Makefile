#
# Copyright (c) 2011 The Chromium OS Authors.
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundatio; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

include $(TOPDIR)/config.mk

LIB	= $(obj)libdts.o

# We use DEV_TREE_SRC as the name of the device tree, if defined
# If not, we use CONFIG_DEFAULT_DEVICE_TREE

ifeq ($(DEV_TREE_SRC),)
$(if $(CONFIG_DEFAULT_DEVICE_TREE),,\
$(error Please define CONFIG_DEFAULT_DEVICE_TREE in your board header \
	file or define DEV_TREE_SRC in your environment))
DEVICE_TREE = $(subst ",,$(CONFIG_DEFAULT_DEVICE_TREE))
else
DEVICE_TREE = $(DEV_TREE_SRC)
endif

all:	$(obj).depend $(LIB)

# Use a constant name for this so we can access it from C code.
# objcopy doesn't seem to allow us to set the symbol name independently of
# the filename.
DT_BIN	:= $(obj)dt.dtb

$(DT_BIN): $(TOPDIR)/board/$(BOARDDIR)/$(DEVICE_TREE).dts
	$(DTC) -R 4 -p 0x1000 -O dtb -o ${DT_BIN} $<

process_lds = \
	$(1) | sed -r -n 's/^OUTPUT_$(2)[ ("]*([^")]*).*/\1/p'

# Run the compiler and get the link script from the linker
GET_LDS = $(CC) $(CFLAGS) $(LDFLAGS) -Wl,--verbose 2>&1

$(obj)dt.o: $(DT_BIN)
	# We want the output format and arch.
	# We also hope to win a prize for ugliest Makefile / shell interaction
	# We look in the LDSCRIPT first.
	# Then try the linker which should give us the answer.
	# Then check it worked.
	oformat=`$(call process_lds,cat $(LDSCRIPT),FORMAT)` ;\
	oarch=`$(call process_lds,cat $(LDSCRIPT),ARCH)` ;\
	\
	[ -z $${oformat} ] && \
		oformat=`$(call process_lds,$(GET_LDS),FORMAT)` ;\
	[ -z $${oarch} ] && \
		oarch=`$(call process_lds,$(GET_LDS),ARCH)` ;\
	\
	[ -z $${oformat} ] && \
		echo "Cannot read OUTPUT_FORMAT from lds file $(LDSCRIPT)" && \
		exit 1 || true ;\
	[ -z $${oarch} ] && \
		echo "Cannot read OUTPUT_ARCH from lds file $(LDSCRIPT)" && \
		exit 1 || true ;\
	\
	cd $(dir ${DT_BIN}) && \
	$(OBJCOPY) -I binary -O $${oformat} -B $${oarch} \
		$(notdir ${DT_BIN}) $@
	rm $(DT_BIN)

OBJS-$(CONFIG_OF_EMBED)	:= dt.o

COBJS	:= $(OBJS-y)

OBJS	:= $(addprefix $(obj),$(COBJS))

binary:	$(DT_BIN)

$(LIB):	$(OBJS) $(DTB)
	$(call cmd_link_o_target, $(OBJS))

clean:
	rm -f $(OBJS) $(LIB)
	rm -f $(DT_BIN)

distclean:	clean

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
