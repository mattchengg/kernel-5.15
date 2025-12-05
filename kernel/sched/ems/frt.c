#include "ems.h"

#include <trace/events/ems.h>
#include <trace/events/ems_debug.h>

static DEFINE_PER_CPU(cpumask_var_t, frt_local_cpu_mask);

#define cpu_selected(cpu)	(cpu >= 0)
#define AVG_IDLE_THRESHOLD	200000

static bool frt_task_fits_capacity(struct task_struct *p, int cpu)
{
	unsigned long capacity_orig = capacity_cpu_orig(cpu);
	unsigned int min_cap, max_cap;

	min_cap = uclamp_eff_value(p, UCLAMP_MIN);
	max_cap = uclamp_eff_value(p, UCLAMP_MAX);

	return capacity_orig >= min(min_cap, max_cap);
}

static inline bool frt_can_sync_to_cur_cpu(struct rq *cur_rq, struct task_struct *p,
					   int cur_cpu)
{
	if (!cpu_active(cur_cpu))
		return false;

	if (!cpumask_test_cpu(cur_cpu, p->cpus_ptr))
		return false;

	if (!cpumask_test_cpu(cur_cpu, ecs_available_cpus()))
		return false;

	if (p->prio > cur_rq->rt.highest_prio.next)
		return false;

	if (cur_rq->rt.rt_nr_running > 2)
		return false;

	return true;
}

int frt_get_scale_freq_capacity(int cpu)
{
	int ratio = emstune_frt_scale_freq_threshold(cpu);

	if (!ratio)
		return ratio;

	return SCHED_CAPACITY_SCALE * ratio / 100;
}

/*****************************************************************************/
/*				SELECT WAKEUP CPU			     */
/*****************************************************************************/
int frt_wakeup_fast_track(struct task_struct *p, int index)
{
	struct pe_list *pe_list = get_pe_list(index);
	const struct cpumask *ecs_mask = ecs_available_cpus();
	int cluster, cpu;
	int best_cpu = -1;

	for (cluster = 0; cluster < pe_list->num_of_cpus; cluster++) {
		for_each_cpu_and(cpu, ecs_mask, &pe_list->cpus[cluster]) {
			unsigned long irq_load = ml_cpu_irq_load(cpu);
			unsigned long extra_util = cpu_util_rt(cpu_rq(cpu)) + cpu_util_dl(cpu_rq(cpu)) + irq_load;
			unsigned long total_util = min(ml_cpu_util(cpu) + extra_util, capacity_cpu_orig(cpu));
			int overutilized = cpu_overutilized_rt(cpu, total_util);
			int nr_running = cpu_rq(cpu)->rt.rt_nr_running;
			int scale_freq = frt_get_scale_freq_capacity(cpu);

			trace_ems_frt_fast_track(p, cpu, total_util, overutilized, nr_running,
						cpu_rq(cpu)->avg_idle, arch_scale_freq_capacity(cpu),
						scale_freq);

			if (!cpu_active(cpu))
				continue;

			if (!cpumask_test_cpu(cpu, p->cpus_ptr))
				continue;

			if (overutilized)
				continue;

			/* avoid this cpu, if there is a rt task */
			if (nr_running)
				continue;

			/* avoid this cpu, if there is a tex task */
			if (ems_rq_nr_launch(cpu_rq(cpu)) > 0)
				continue;

			if (cpu_rq(cpu)->avg_idle < AVG_IDLE_THRESHOLD)
				continue;

			if (scale_freq && arch_scale_freq_capacity(cpu) > scale_freq)
				continue;

			best_cpu = cpu;
			break;
		}

		if (cpu_selected(best_cpu))
			goto out;
	}

out:
	return best_cpu;
}

int frt_get_init_index(struct task_struct *p)
{
	int index = cpuctl_task_group_idx(p);
	int pe_list_size = get_pe_list_size();

	/* In case of single cluster, init_index should be zero. */
	if (pe_list_size == 1)
		return pe_list_size - 1;

	return emstune_frt_boost_enabled(index);
}

int __frt_find_lowest_rq(struct task_struct *p, struct cpumask *lowest_mask, int ret, bool select_rq)
{
	int index = frt_get_init_index(p);
	struct pe_list *pe_list = get_pe_list(index);
	unsigned long task_util = ml_task_util(p);
	unsigned long best_cpu_util = ULONG_MAX;
	unsigned long max_spare_cap = 0;
	unsigned int best_exit_latency = UINT_MAX;
	unsigned int exit_latency = UINT_MAX;
	int cluster, cpu;
	int best_cpu = -1;
	int max_spare_cpu = -1;
	int prev_cpu = task_cpu(p);
	int fast_track = 0;
	bool best_tex_stat = true;
	struct cpumask spared_lowest_mask;

	cpumask_and(&spared_lowest_mask, lowest_mask, ecs_available_cpus());

	if (!select_rq) {
		cpu = frt_wakeup_fast_track(p, index);
		if (cpu_selected(cpu)) {
			best_cpu = cpu;
			fast_track = 1;
			goto out;
		}
	}

	if (!ret)
		return prev_cpu;

	rcu_read_lock();
	for (cluster = 0; cluster < pe_list->num_of_cpus; cluster++) {
		for_each_cpu_and(cpu, &spared_lowest_mask, &pe_list->cpus[cluster]) {
			unsigned long cpu_util = 0;
			unsigned long cpu_total_util = 0;
			unsigned long irq_load = ml_cpu_irq_load(cpu);
			unsigned long extra_util = cpu_util_rt(cpu_rq(cpu)) + cpu_util_dl(cpu_rq(cpu)) + irq_load;
			unsigned long capacity_orig = capacity_cpu_orig(cpu);
			unsigned long irq_thr = (capacity_orig * MLT_IRQ_LOAD_RATIO) / 100;
			unsigned long spare;
			bool is_tex;

			if (!cpu_active(cpu))
				continue;

			cpu_total_util = min(ml_cpu_util(cpu) + extra_util, capacity_orig);
			spare = capacity_orig - cpu_total_util;
			if (spare > max_spare_cap) {
				max_spare_cap = spare;
				max_spare_cpu = cpu;
			}

			trace_ems_frt_lowest_rq_stat(p, cpu, spare, irq_load,
					get_tex_level(cpu_rq(cpu)->curr),
					available_idle_cpu(cpu),
					get_idle_exit_latency(cpu_rq(cpu)));

			if (irq_load >= irq_thr)
				continue;

			if (!available_idle_cpu(cpu)) {
				if (cpu_overutilized_rt(cpu, cpu_total_util))
					continue;

				cpu_util = ml_cpu_util(cpu);
			}

			is_tex = get_tex_level(cpu_rq(cpu)->curr) != NOT_TEX;
			if (is_tex && !best_tex_stat)
				continue;

			if ((best_tex_stat && is_tex) || (!best_tex_stat && !is_tex))
				if (cpu_util > best_cpu_util)
					continue;

			if (best_cpu_util == cpu_util && best_cpu == prev_cpu)
				continue;

			exit_latency = get_idle_exit_latency(cpu_rq(cpu));

			if (cpu != prev_cpu && best_cpu_util == cpu_util) {
				if (best_exit_latency < exit_latency)
					continue;

				if (best_exit_latency == exit_latency)
					continue;
			}

			best_exit_latency = exit_latency;
			best_cpu_util = cpu_util;
			best_cpu = cpu;
			best_tex_stat = is_tex;
		}

		if (cpu_selected(best_cpu))
			break;
	}

	if (!cpu_selected(best_cpu)) {
		if (cpu_selected(max_spare_cpu)) {
			best_cpu = max_spare_cpu;
		} else {
			if (cpumask_test_cpu(prev_cpu, &spared_lowest_mask))
				best_cpu = prev_cpu;
			else
				best_cpu = !cpumask_empty(&spared_lowest_mask) ?
					cpumask_last(&spared_lowest_mask) : cpumask_first(lowest_mask);
		}
	}

	rcu_read_unlock();

out:
	trace_ems_frt_lowest_rq(p, task_util, index, lowest_mask, ecs_available_cpus(), &spared_lowest_mask,
				best_cpu, max_spare_cpu, best_exit_latency, best_cpu_util,
				max_spare_cap, fast_track);
	return best_cpu;
}

int frt_find_lowest_rq(struct task_struct *p, struct cpumask *lowest_mask, int ret)
{
	return __frt_find_lowest_rq(p, lowest_mask, ret, false);
}

int frt_select_task_rq_rt(struct task_struct *p, int prev_cpu,
			  int sd_flag, int wake_flags)
{
	struct cpumask *lowest_mask = NULL;
	struct rq *rq, *cur_rq;
	bool sync = !!(wake_flags & WF_SYNC);
	int target_cpu = -1, cur_cpu;
	int cpu, ret;

	/* For anything but wake ups, just return the task_cpu */
	if (sd_flag != SD_BALANCE_WAKE && sd_flag != SD_BALANCE_FORK)
		goto out;

	cur_cpu = raw_smp_processor_id();
	cur_rq = cpu_rq(cur_cpu);

	/* Handle sync flag */
	if (sync && frt_can_sync_to_cur_cpu(cur_rq, p, cur_cpu)) {
		target_cpu = cur_cpu;
		goto out;
	}

	rq = cpu_rq(prev_cpu);

	rcu_read_lock();

	lowest_mask = this_cpu_cpumask_var_ptr(frt_local_cpu_mask);
	ret = cpupri_find_fitness(&task_rq(p)->rd->cpupri, p,
				lowest_mask, frt_task_fits_capacity);

	cpu = frt_wakeup_fast_track(p, frt_get_init_index(p));
	if (cpu_selected(cpu)) {
		target_cpu = cpu;
		rcu_read_unlock();
		update_rt_stat(target_cpu);
		trace_frt_select_task_rq(p, target_cpu, "rt-fast-track");
		return target_cpu;
	}

	cpu = __frt_find_lowest_rq(p, lowest_mask, ret, true);
	if (likely(cpu >= 0)) {
		if (p->prio < cpu_rq(cpu)->rt.highest_prio.curr)
			target_cpu = cpu;
		else if (cpumask_test_cpu(prev_cpu, lowest_mask))
			target_cpu = prev_cpu;
		else
			target_cpu = cpu;
	}

	rcu_read_unlock();

out:
	update_rt_stat(target_cpu);

	if (lowest_mask)
		trace_frt_select_task_rq_mask(p, target_cpu, lowest_mask, "rt-wakeup-mask");
	else
		trace_frt_select_task_rq(p, target_cpu, "rt-wakeup");

	return target_cpu;
}

void frt_init(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		if (!zalloc_cpumask_var_node(&per_cpu(frt_local_cpu_mask, cpu),
					GFP_KERNEL, cpu_to_node(cpu))) {
			pr_err("failed to alloc frt_local_cpu_mask\n");
		}
	}

	pr_info("%s: frt initialized complete!\n", __func__);
}
