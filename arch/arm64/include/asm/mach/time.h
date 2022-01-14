/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __ASM_ARM64_MACH_TIME_H
#define __ASM_ARM64_MACH_TIME_H

typedef void (*clock_access_fn)(struct timespec64 *);
extern int register_persistent_clock(clock_access_fn read_persistent);

#endif
