/*
 * Copyright 2013-2015 Host Mobility AB
 *
 * Configuration settings for the Toradex VF50/VF61 module on MX-4 V61 board.
 *
 * Based on vf610twr.h:
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx4-vf-common.h"

#undef MX4_CONSOLE_PORT
#define MX4_CONSOLE_PORT "ttyLP2"

#define MX4_PRODUCT_TYPE "c61"

#endif /* __CONFIG_H */
