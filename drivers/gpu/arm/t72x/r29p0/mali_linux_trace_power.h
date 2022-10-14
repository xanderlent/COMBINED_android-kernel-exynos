/* SPDX-License-Identifier: GPL-2.0
 *
 * (C) COPYRIGHT 2011-2019 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 *
 */

#if !defined(_TRACE_MALI_POWER_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MALI_POWER_H
#include <linux/tracepoint.h>
#undef TRACE_SYSTEM
#define TRACE_SYSTEM power
#define TRACE_INCLUDE_FILE mali_linux_trace_power
/**
 * gpu_frequency - Reports frequency changes in GPU clock domains
 * @state:  New frequency
 * @gpu_id: GPU clock domain
 */
TRACE_EVENT(gpu_frequency,
	TP_PROTO(unsigned int state, unsigned int gpu_id),
	TP_ARGS(state, gpu_id),
	TP_STRUCT__entry(
		__field(unsigned int, state)
		__field(unsigned int, gpu_id)
	),
	TP_fast_assign(
		entry->state = state;
		__entry->gpu_id = gpu_id;
	),
	TP_printk("state=%lu gpu_id=%lu", __entry->state, __entry->gpu_id)
);
#endif /* _TRACE_MALI_POWER_H */

#undef TRACE_INCLUDE_PATH
#undef linux
#define TRACE_INCLUDE_PATH .

/* This part must be outside protection */
#include <trace/define_trace.h>
