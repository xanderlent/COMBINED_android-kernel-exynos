/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *
 * Boojin Kim <boojin.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _MAILBOX_IPC_TYPE_H
#define _MAILBOX_IPC_TYPE_H

#define CIPC_RAW_READL(a)      (*(volatile unsigned int *)(a))
#define CIPC_RAW_WRITEL(v, a)  (*(volatile unsigned int *)(a) = ((unsigned int)v))

#define CIPC_TRUE 1
#define CIPC_FALSE 0
#define CIPC_NULL 0
#define CIPC_ERR -1

#ifndef CU64
typedef unsigned long long CU64;
#endif

#ifndef CU32
typedef unsigned long int CU32;
#endif
#endif
