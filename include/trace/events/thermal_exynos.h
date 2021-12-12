/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM thermal_exynos

#if !defined(_TRACE_THERMAL_EXYNOS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_THERMAL_EXYNOS_H

#include <linux/tracepoint.h>

TRACE_EVENT(thermal_exynos_power_cpu_get_power,
	TP_PROTO(const struct cpumask *cpus, unsigned long freq, u32 *load,
		size_t load_len, u32 dynamic_power, u32 static_power),

	TP_ARGS(cpus, freq, load, load_len, dynamic_power, static_power),

	TP_STRUCT__entry(
		__bitmask(cpumask, num_possible_cpus())
		__field(unsigned long, freq          )
		__dynamic_array(u32,   load, load_len)
		__field(size_t,        load_len      )
		__field(u32,           dynamic_power )
		__field(u32,           static_power  )
	),

	TP_fast_assign(
		__assign_bitmask(cpumask, cpumask_bits(cpus),
			num_possible_cpus());
		__entry->freq = freq;
		memcpy(__get_dynamic_array(load), load,
			load_len * sizeof(*load));
		__entry->load_len = load_len;
		__entry->dynamic_power = dynamic_power;
		__entry->static_power = static_power;
	),

	TP_printk("cpus=%s freq=%lu load={%s} dynamic_power=%d static_power=%d",
		__get_bitmask(cpumask), __entry->freq,
		__print_array(__get_dynamic_array(load), __entry->load_len, 4),
		__entry->dynamic_power, __entry->static_power)
);

TRACE_EVENT(thermal_exynos_power_cpu_limit,
	TP_PROTO(const struct cpumask *cpus, unsigned int freq,
		unsigned long cdev_state, u32 power),

	TP_ARGS(cpus, freq, cdev_state, power),

	TP_STRUCT__entry(
		__bitmask(cpumask, num_possible_cpus())
		__field(unsigned int,  freq      )
		__field(unsigned long, cdev_state)
		__field(u32,           power     )
	),

	TP_fast_assign(
		__assign_bitmask(cpumask, cpumask_bits(cpus),
				num_possible_cpus());
		__entry->freq = freq;
		__entry->cdev_state = cdev_state;
		__entry->power = power;
	),

	TP_printk("cpus=%s freq=%u cdev_state=%lu power=%u",
		__get_bitmask(cpumask), __entry->freq, __entry->cdev_state,
		__entry->power)
);

TRACE_EVENT(thermal_exynos_power_gpu_get_power,
	TP_PROTO(unsigned long freq, u32 load, u32 dynamic_power, u32 static_power),

	TP_ARGS(freq, load, dynamic_power, static_power),

	TP_STRUCT__entry(
		__field(unsigned long, freq          )
		__field(u32,           load )
		__field(u32,           dynamic_power )
		__field(u32,           static_power  )
	),

	TP_fast_assign(
		__entry->freq = freq;
		__entry->load = load;
		__entry->dynamic_power = dynamic_power;
		__entry->static_power = static_power;
	),

	TP_printk("freq=%lu load=%d dynamic_power=%d static_power=%d",
		__entry->freq, __entry->load, __entry->dynamic_power, __entry->static_power)
);

TRACE_EVENT(thermal_exynos_power_gpu_limit,
	TP_PROTO(unsigned int freq, unsigned long cdev_state, u32 power),

	TP_ARGS(freq, cdev_state, power),

	TP_STRUCT__entry(
		__field(unsigned int,  freq      )
		__field(unsigned long, cdev_state)
		__field(u32,           power     )
	),

	TP_fast_assign(
		__entry->freq = freq;
		__entry->cdev_state = cdev_state;
		__entry->power = power;
	),

	TP_printk("freq=%u cdev_state=%lu power=%u",
		__entry->freq, __entry->cdev_state,
		__entry->power)
);
#endif /* _TRACE_THERMAL_EXYNOS_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
