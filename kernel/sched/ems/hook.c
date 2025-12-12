/*
 * tracepoint hook handling
 *
 * Copyright (C) 2018 Samsung Electronics Co., Ltd
 * Park Bumgyu <bumgyu.park@samsung.com>
 */

#include "ems.h"

#include <trace/events/sched.h>
#include <trace/events/ems_debug.h>
#include <trace/events/power.h>
#include <trace/hooks/sched.h>
#include <trace/hooks/cpuidle.h>
#include <trace/hooks/binder.h>
#include <trace/hooks/cgroup.h>
#include <trace/hooks/sys.h>
#include <trace/hooks/topology.h>
#include <trace/hooks/cpufreq.h>

#include "../../../drivers/android/binder_trace.h"

/******************************************************************************
 * tracepoint of Android vendor hook                                          *
 ******************************************************************************/
static void ems_hook_select_task_rq_fair(void *data,
			struct task_struct *p, int prev_cpu,
			int sd_flag, int wake_flags, int *new_cpu)
{
	struct ems_dslt_rq *dslt_rq = ems_rq_dslt(task_rq(p));
	int cpu;

	if (!(sd_flag & SD_BALANCE_FORK)) {
		u64 now = sched_clock();
		mlt_update_task(p, MLT_STATE_NOCHANGE, now, false, false);

		dslt_sync_entity_load_avg(dslt_rq, &p->se);
	}

	/* skip calling dynamic_fast_release_cpus from Ontime */
	if (wake_flags)
		ecs_enqueue_update(prev_cpu, p);

	cpu = ems_select_task_rq_fair(p, prev_cpu, sd_flag, wake_flags);

	*new_cpu = cpu;
	ems_target_cpu(p) = cpu;
}

static void ems_hook_select_task_rq_rt(void *data,
			struct task_struct *p, int prev_cpu,
			int sd_flag, int wake_flags, int *new_cpu)
{
	int cpu;

	if (!(sd_flag & SD_BALANCE_FORK)) {
		u64 now = sched_clock();
		mlt_update_task(p, MLT_STATE_NOCHANGE, now, false, false);
	}

	cpu = frt_select_task_rq_rt(p, prev_cpu, sd_flag, wake_flags);

	*new_cpu = cpu;
	ems_target_cpu(p) = cpu;
}

static void ems_hook_select_fallback_rq(void *data,
			int cpu, struct task_struct *p, int *new_cpu)
{
	int target_cpu = ems_target_cpu(p);

	if (target_cpu >= 0 && cpu_active(target_cpu))
		*new_cpu = target_cpu;
	else
		*new_cpu = -1;

	ems_target_cpu(p) = -1;
}

static void ems_hook_find_lowest_rq(void *data,
			struct task_struct *p, struct cpumask *local_cpu_mask,
			int ret, int *lowest_cpu)
{
	*lowest_cpu = frt_find_lowest_rq(p, local_cpu_mask, ret);
}

u64 ems_tick_update_t;
DEFINE_RAW_SPINLOCK(ems_tick_lock);
static bool ems_tick_updater(void)
{
	unsigned long flags;
	u64 now = jiffies;
	bool ret = false;

	if (!raw_spin_trylock_irqsave(&ems_tick_lock, flags))
		return ret;

	if (ems_tick_update_t >= now)
		goto unlock;

	ems_tick_update_t = now;
	ret = true;

unlock:
	raw_spin_unlock_irqrestore(&ems_tick_lock, flags);
	return ret;
}

static void ems_hook_scheduler_tick(void *data, struct rq *rq)
{
	pago_tick(rq);

	if (ems_tick_updater()) {
		mhdvfs();
		mlt_tick();
		profile_sched_data();
		gsc_update_tick();
		ecs_update();
		monitor_sysbusy();
		pago_update();
	}

	halo_tick(rq);

	tex_update(rq);

	lb_tick(rq);

	ontime_migration();

	ecs_retry_sparing(rq);
}

static void ems_hook_enqueue_task(void *data,
			struct rq *rq, struct task_struct *p, int flags)
{
	int dvfs_flags = 0;

	u64 cur = ktime_get_ns();

	mlt_enqueue_task(rq, p);

	profile_enqueue_task(rq, p, &dvfs_flags);

	tex_enqueue_task(p, cpu_of(rq));

	freqboost_enqueue_task(p, cpu_of(rq), flags);

	lb_enqueue_misfit_task(p, rq);

	ems_enqueue_ts(p) = cur;

	taskgroup_enqueue_task(rq, p, &dvfs_flags);

	if (dvfs_flags)
		ego_update_freq(rq, sched_clock(), dvfs_flags);
}

static void ems_hook_after_enqueue_task(void *data,
			struct rq *rq, struct task_struct *p, int flags)
{
	dslt_enqueue_task(rq, p, flags);
}

static void ems_hook_dequeue_task(void *data,
			struct rq *rq, struct task_struct *p, int flags)
{
	int dvfs_flags = 0;

	mlt_dequeue_task(rq, p);

	profile_dequeue_task(rq, p, &dvfs_flags);

	tex_dequeue_task(p, cpu_of(rq));

	freqboost_dequeue_task(p, cpu_of(rq), flags);

	lb_dequeue_misfit_task(p, rq);

	dslt_util_est_dequeue(rq, p);

	sysbusy_dequeue_task(p);

	if (dvfs_flags)
		ego_update_freq(rq, sched_clock(), dvfs_flags);
}

static void ems_hook_after_dequeue_task(void *data,
			struct rq *rq, struct task_struct *p, int flags)
{
	dslt_dequeue_task(rq, p, flags);
}

static void ems_hook_can_migrate_task(void *data,
			struct task_struct *p, int dst_cpu, int *can_migrate)
{
	lb_can_migrate_task(p, dst_cpu, can_migrate);
}

static void ems_hook_find_busiest_queue(void *data, int dst_cpu, struct sched_group *group,
		struct cpumask *env_cpus, struct rq **busiest, int *done)
{
	lb_find_busiest_queue(dst_cpu, group, env_cpus, busiest, done);
}

static void ems_hook_cpu_cgroup_attach(void *data, struct cgroup_taskset *tset)
{
	gsc_cpu_cgroup_attach(tset);
}

static void ems_hook_rebalance_domains(void *data,
			struct rq *rq, int *continue_balancing)
{
	*continue_balancing = !lb_rebalance_domains(rq);
}

static void ems_hook_nohz_balancer_kick(void *data,
			struct rq *rq, unsigned int *flags, int *done)
{
	lb_nohz_balancer_kick(rq, flags, done);
}

static void ems_hook_newidle_balance(void *data,
			struct rq *this_rq, struct rq_flags *rf,
			int *pulled_task, int *done)
{
	lb_newidle_balance(this_rq, rf, pulled_task, done);
}

static void ems_hook_new_task_stats(void *data, struct task_struct *p)
{
	u64 cur = ktime_get_ns();

	ems_last_waked(p) = cur;
	ems_enqueue_ts(p) = cur;

	mlt_update_load(task_rq(p), p, sched_clock(), NEW_TASK);
}

static void ems_hook_find_new_ilb(void *data, struct cpumask *nohz_idle_cpus_mask, int *ilb)
{
	*ilb = lb_find_new_ilb(nohz_idle_cpus_mask);
}

static void ems_hook_sched_fork_init(void *data, struct task_struct *p)
{
	tex_task_init(p);
	mlt_init_task(p);
	ems_task_misfited(p) = 0;
	ems_launch_task(p) = false;
}

static void ems_wake_up_new_task(void *unused, struct task_struct *p)
{
	ntu_apply(p);
	dslt_new_task_wakeup(p);
	tex_task_init(p);
	gsc_init_new_task(p);

	ems_yield_cnt(p) = 0;
	ems_yield_last(p) = 0;
	ems_busy_waiting(p) = false;
}

static void ems_flush_task(void *unused, struct task_struct *p)
{
	gsc_flush_task(p);
}

#define ems_for_each_sched_entity(se) \
	for (; (se); (se) = (se)->parent)
static void ems_hook_replace_next_task_fair(void *data, struct rq *rq,
					    struct task_struct **p_ptr,
					    struct sched_entity **se_ptr,
					    bool *repick,
					    bool simple,
					    struct task_struct *prev)
{
	tex_replace_next_task_fair(rq, p_ptr, se_ptr, repick, simple, prev);

	if (*repick && simple) {
		ems_for_each_sched_entity(*se_ptr)
			set_next_entity(cfs_rq_of(*se_ptr), *se_ptr);
	}
}

static void
ems_hook_schedule(void *data, struct task_struct *prev,
			struct task_struct *next, struct rq *rq)
{
	if (prev == next)
		return;

	slack_timer_cpufreq(rq->cpu,
		get_sched_class(next) == EMS_SCHED_IDLE,
		get_sched_class(prev) == EMS_SCHED_IDLE);

	mlt_task_switch(rq->cpu, prev, next);

	dslt_switch_sensitivity(rq->cpu, prev, next);
}

void ems_hook_check_preempt_wakeup(void *data, struct rq *rq, struct task_struct *p,
		bool *preempt, bool *nopreempt, int wake_flags, struct sched_entity *se,
		struct sched_entity *pse, int next_buddy_marked, unsigned int granularity)
{
	tex_check_preempt_wakeup(rq, p, preempt, nopreempt);
}

static void ems_hook_update_misfit_status(void *data, struct task_struct *p,
					struct rq *rq, bool *need_update)
{
	lb_update_misfit_status(p, rq, need_update);
}

/******************************************************************************
 * built-in tracepoint                                                        *
 ******************************************************************************/
#if defined (CONFIG_SCHED_EMS_DEBUG)
static void ems_hook_pelt_cfs_tp(void *data, struct cfs_rq *cfs_rq)
{
	trace_sched_load_cfs_rq(cfs_rq);
}

static void ems_hook_pelt_rt_tp(void *data, struct rq *rq)
{
	trace_sched_load_rt_rq(rq);
}

static void ems_hook_pelt_dl_tp(void *data, struct rq *rq)
{
	trace_sched_load_dl_rq(rq);
}

static void ems_hook_pelt_irq_tp(void *data, struct rq *rq)
{
#ifdef CONFIG_HAVE_SCHED_AVG_IRQ
	trace_sched_load_irq(rq);
#endif
}

static void ems_hook_pelt_se_tp(void *data, struct sched_entity *se)
{
	trace_sched_load_se(se);
}

static void ems_hook_sched_overutilized_tp(void *data,
			struct root_domain *rd, bool overutilized)
{
	trace_sched_overutilized(overutilized);
}
#endif
/*
static void ems_hook_binder_wake_up_ilocked(void *data, struct task_struct *p,
		bool sync, struct binder_proc *proc)
{
	if (!p)
		return;

	if ((is_top_app_task(current) && is_rt_task(p->group_leader)) ||
	    (is_top_app_task(p) && is_rt_task(current->group_leader)))
		ems_binder_task(p) = 1;

	if (current->signal && is_important_task(current))
		ems_binder_task(p) = 1;
}
*/
static void ems_hook_binder_transaction_received(void *data, struct binder_transaction *t)
{
	ems_binder_task(current) = 0;
}

#define BINDER_ONE_WAY 0x01
static void ems_hook_binder_set_priority(void *data, struct binder_transaction *t,
		struct task_struct *task)
{
	bool oneway;

	if (!t)
		return;

	oneway = !!(t->flags & BINDER_ONE_WAY);

	if (t && !oneway && is_boosted_tex_task(current))
		ems_boosted_tex(task) = 1;

	if (t && oneway) {
		if ((is_top_app_task(current) && is_rt_task(task->group_leader)) ||
				(is_top_app_task(task) && is_rt_task(current->group_leader)))
			ems_binder_task(task) = 1;

		if (current->signal && is_important_task(current))
			ems_binder_task(task) = 1;
	}
}

static void ems_hook_binder_restore_priority(void *data, struct binder_transaction *t,
		struct task_struct *p)
{
	if (t && ems_boosted_tex(p))
		ems_boosted_tex(p) = 0;
}

static void ems_hook_do_sched_yield(void *data, struct rq *rq)
{
	struct task_struct *curr = rq->curr;

	lockdep_assert_held(&rq->__lock);

	tex_do_yield(curr);

	sysbusy_do_yield(curr);
}

static void ems_hook_try_to_wake_up(void *data, struct task_struct *p)
{
	struct rq *rq = task_rq(p);
	struct rq_flags rf;

	rq_lock_irqsave(rq, &rf);
	mlt_update_load(rq, p, sched_clock(), WAKEUP_TASK);
	rq_unlock_irqrestore(rq, &rf);

	ems_last_waked(p) = ktime_get_ns();

	ems_yield_cnt(p) = 0;
	ems_yield_last(p) = 0;
	ems_busy_waiting(p) = false;
}

static void ems_hook_syscall_prctl_finished(void *data, int option, struct task_struct *p)
{
	if (option != PR_SET_NAME)
		return;

	if (strcmp(p->comm, "RenderThread"))
		return;

	if (cpuctl_task_group_idx(p) != CGROUP_TOPAPP)
		return;

	ems_render(p) = 1;
}

static void ems_hook_enter_idle(void *data, int *state,
		struct cpuidle_device *dev)
{
	pago_idle_enter(dev->cpu);
}

#ifdef CONFIG_SCHED_EMS_TASK_GROUP
static void ems_hook_dup_task_struct(void *data, struct task_struct *tsk, struct task_struct *orig)
{
        struct ems_task_struct *v_tsk, *v_orig;

        v_tsk = (struct ems_task_struct *)tsk->android_vendor_data1;
        v_orig = (struct ems_task_struct *)orig->android_vendor_data1;
        v_tsk->cgroup = v_orig->cgroup;
}

static void ems_uclamp_eff_get(void *data, struct task_struct *p, enum uclamp_id clamp_id,
                                  struct uclamp_se *uclamp_max, struct uclamp_se *uclamp_eff,
                                  int *ret)
{
	struct uclamp_se uc_req = ems_uclamp_tg_restrict(p, clamp_id);
	*ret = 1;

	/* System default restrictions always apply */
	if (unlikely(uc_req.value > uclamp_max->value)) {
		*uclamp_eff = *uclamp_max;
		return;
	}

	*uclamp_eff = uc_req;
	return;
}
#endif

static void ems_hook_finish_prio_fork(void *data, struct task_struct *p)
{
	dslt_init_entity_runnable_average(p);
}

static void ems_hook_post_init_entity_util_avg(void *data, struct sched_entity *se)
{
	dslt_post_init_entity_util_avg(se);
}

static void ems_hook_util_est_update(void *data, struct cfs_rq *cfs_rq, struct task_struct *p,
					bool task_sleep, int *ret)
{
	dslt_util_est_update(cfs_rq, p, task_sleep);
}

static void ems_hook_update_load_avg(void *data, u64 now, struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	dslt_update_load_avg(now, cfs_rq, se);
}

static void ems_hook_attach_entity_load_avg(void *data, struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	dslt_attach_entity_load_avg(cfs_rq, se);
}

static void ems_hook_detach_entity_load_avg(void *data, struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	dslt_detach_entity_load_avg(cfs_rq, se);
}

static void ems_hook_set_task_cpu(void *data, struct task_struct *p, unsigned int new_cpu)
{
	if (get_sched_class(p) == EMS_SCHED_FAIR) {
		if (!task_on_rq_migrating(p))
			dslt_migrate_se_dslt_lag(&p->se);

		ems_dslt_task(p)->avg.last_update_time = 0;
	}

	dslt_set_task_rq_fair(&p->se, p->se.cfs_rq, task_group(p)->cfs_rq[new_cpu]);
}

static void ems_hook_remove_entity_load_avg(void *data, struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	dslt_remove_entity_load_avg(cfs_rq, se);
}

static void ems_hook_update_blocked_fair(void *data, struct rq *rq)
{
	dslt_update_blocked_fair(rq);
}

static void ems_hook_tick_entry(void *data, struct rq *rq)
{
	u64 now = sched_clock();
	int dvfs_flags = 0;

	mlt_update_task(rq->curr, MLT_STATE_NOCHANGE, now, true, true);
	mlt_update_cpu(rq->cpu, MLT_STATE_NOCHANGE, now, true, true);

	mlt_update_load(rq, rq->curr, now, UPDATE_TASK);

	sysbusy_update_busy_wait(rq);

	profile_update_htask(rq, &dvfs_flags);

	if (dvfs_flags)
		ego_update_freq(rq, now, dvfs_flags);
}

static void ems_hook_is_cpu_allowed(void *data, struct task_struct *p, int cpu, bool *allowed)
{
	if (!cpumask_test_cpu(cpu, ecs_available_cpus())) {
		cpumask_t allowed_mask;

		/* default reject for sparing cpu */
		*allowed = false;

		/* sparing cpus are not allowed */
		cpumask_and(&allowed_mask, cpu_active_mask, p->cpus_ptr);
		cpumask_and(&allowed_mask, &allowed_mask, ecs_available_cpus());

		if (!(p->flags & PF_KTHREAD)) {
			/* All affinity cpus are inactive or sparing -> Allow this cpu */
			if (cpumask_empty(&allowed_mask)) {
				*allowed = true;
			}

			return;
		}

		/* for kthread, dying cpus are not allowed */
		cpumask_andnot(&allowed_mask, &allowed_mask, cpu_dying_mask);
		/* All affinity cpus inactive or sparing or dying -> Allow this cpu */
		if (cpumask_empty(&allowed_mask)) {
			*allowed = true;
		}
	}
}

static void ems_hook_set_cpus_allowed_by_task(void *data,
		const struct cpumask *cpu_valid_mask, const struct cpumask *new_mask,
		struct task_struct *p, unsigned int *dest_cpu)
{
	/* allow kthreads to change affinity */
	if (p->flags & PF_KTHREAD)
		return;

	if (!cpumask_test_cpu(*dest_cpu, ecs_available_cpus()) && !p->migration_disabled) {
		cpumask_t allowed_mask;

		/* remove ecs cpus from valid mask */
		cpumask_and(&allowed_mask, cpu_valid_mask, ecs_available_cpus());
		cpumask_and(&allowed_mask, &allowed_mask, new_mask);

		/* do not modify dest_cpu if there are no allowed cpus */
		if (!cpumask_empty(&allowed_mask))
			*dest_cpu = cpumask_any_and_distribute(&allowed_mask, new_mask);
	}
}

static void ems_hook_update_cpu_capacity(void *data, int cpu, unsigned long *capacity)
{
	unsigned long rt_contrib = arch_scale_cpu_capacity(cpu) - *capacity;
	struct rq *rq = cpu_rq(cpu);

	ems_update_cpu_capacity(cpu);
	*capacity = max((rq->cpu_capacity_orig - rt_contrib), (unsigned long)0);
}

static void ems_hook_cpu_frequency_limits(void *data, struct cpufreq_policy *policy)
{
	ego_update_frequency_limits(policy);
}

static void ems_hook_cpu_capacity_show(void *data, unsigned long *capacity, int cpu)
{
	emstune_capacity_fixup(capacity, cpu);
}

static void ems_hook_show_max_freq(void *data,
		struct cpufreq_policy *policy, unsigned int *max_freq)
{
	emstune_maxfreq_fixup(max_freq, policy->cpu);
}

int hook_init(void)
{
	int ret;

	ret = register_trace_android_rvh_select_task_rq_fair(ems_hook_select_task_rq_fair, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_select_task_rq_rt(ems_hook_select_task_rq_rt, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_select_fallback_rq(ems_hook_select_fallback_rq, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_find_lowest_rq(ems_hook_find_lowest_rq, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_scheduler_tick(ems_hook_scheduler_tick, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_enqueue_task(ems_hook_enqueue_task, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_after_enqueue_task(ems_hook_after_enqueue_task, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_dequeue_task(ems_hook_dequeue_task, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_after_dequeue_task(ems_hook_after_dequeue_task, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_cpu_cgroup_attach(ems_hook_cpu_cgroup_attach, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_sched_rebalance_domains(ems_hook_rebalance_domains, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_sched_newidle_balance(ems_hook_newidle_balance, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_new_task_stats(ems_hook_new_task_stats, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_find_new_ilb(ems_hook_find_new_ilb, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_sched_fork_init(ems_hook_sched_fork_init, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_replace_next_task_fair(ems_hook_replace_next_task_fair, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_schedule(ems_hook_schedule, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_check_preempt_wakeup(ems_hook_check_preempt_wakeup, NULL);
	if (ret)
		return ret;
/*
	ret = register_trace_android_vh_binder_wakeup_ilocked(ems_hook_binder_wake_up_ilocked, NULL);
	if (ret)
		return ret;
*/
	ret = register_trace_binder_transaction_received(ems_hook_binder_transaction_received, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_binder_set_priority(ems_hook_binder_set_priority, NULL);
	if (ret)
		return ret;
	ret = register_trace_android_vh_binder_restore_priority(ems_hook_binder_restore_priority, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_update_misfit_status(ems_hook_update_misfit_status, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_sched_nohz_balancer_kick(ems_hook_nohz_balancer_kick, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_can_migrate_task(ems_hook_can_migrate_task, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_find_busiest_queue(ems_hook_find_busiest_queue, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_do_sched_yield(ems_hook_do_sched_yield, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_try_to_wake_up(ems_hook_try_to_wake_up, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_syscall_prctl_finished(ems_hook_syscall_prctl_finished, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_cpu_idle_enter(ems_hook_enter_idle, NULL);
	if (ret)
		return ret;

#ifdef CONFIG_SCHED_EMS_TASK_GROUP
	ret = register_trace_android_vh_dup_task_struct(ems_hook_dup_task_struct, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_uclamp_eff_get(ems_uclamp_eff_get, NULL);
	if (ret)
		return ret;
#endif

#if defined (CONFIG_SCHED_EMS_DEBUG)
	WARN_ON(register_trace_pelt_cfs_tp(ems_hook_pelt_cfs_tp, NULL));
	WARN_ON(register_trace_pelt_rt_tp(ems_hook_pelt_rt_tp, NULL));
	WARN_ON(register_trace_pelt_dl_tp(ems_hook_pelt_dl_tp, NULL));
	WARN_ON(register_trace_pelt_irq_tp(ems_hook_pelt_irq_tp, NULL));
	WARN_ON(register_trace_pelt_se_tp(ems_hook_pelt_se_tp, NULL));
	WARN_ON(register_trace_sched_overutilized_tp(ems_hook_sched_overutilized_tp, NULL));
#endif
	register_trace_android_rvh_wake_up_new_task(ems_wake_up_new_task, NULL);
	register_trace_android_rvh_flush_task(ems_flush_task, NULL);

	/* dslt */
	register_trace_android_rvh_finish_prio_fork(ems_hook_finish_prio_fork, NULL);
	register_trace_android_rvh_post_init_entity_util_avg(ems_hook_post_init_entity_util_avg, NULL);
	register_trace_android_rvh_util_est_update(ems_hook_util_est_update, NULL);
	register_trace_android_rvh_update_load_avg(ems_hook_update_load_avg, NULL);
	register_trace_android_rvh_attach_entity_load_avg(ems_hook_attach_entity_load_avg, NULL);
	register_trace_android_rvh_detach_entity_load_avg(ems_hook_detach_entity_load_avg, NULL);
	register_trace_android_rvh_set_task_cpu(ems_hook_set_task_cpu, NULL);
	register_trace_android_rvh_remove_entity_load_avg(ems_hook_remove_entity_load_avg, NULL);
	register_trace_android_rvh_update_blocked_fair(ems_hook_update_blocked_fair, NULL);

	register_trace_android_rvh_tick_entry(ems_hook_tick_entry, NULL);

	register_trace_android_rvh_is_cpu_allowed(ems_hook_is_cpu_allowed, NULL);
	register_trace_android_rvh_set_cpus_allowed_by_task(ems_hook_set_cpus_allowed_by_task, NULL);
	register_trace_android_rvh_update_cpu_capacity(ems_hook_update_cpu_capacity, NULL);
	register_trace_cpu_frequency_limits(ems_hook_cpu_frequency_limits, NULL);
	register_trace_android_rvh_cpu_capacity_show(ems_hook_cpu_capacity_show, NULL);
	register_trace_android_rvh_show_max_freq(ems_hook_show_max_freq, NULL);

	return 0;
}
