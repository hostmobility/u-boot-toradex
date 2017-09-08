# Copyright (c) 2017 Host Mobiltiy AB
# SPDX-License-Identifier:      GPL-2.0+

obj-y	+= mx4_t30.o

ifeq ($(CONFIG_SPL_BUILD),)
obj-y	+= ../mx4_common/mx4_common.o
obj-y	+= ../mx4_common/cmd_mx4_pic.o
obj-y	+= ../mx4_common/cmd_mx4_product.o
endif
