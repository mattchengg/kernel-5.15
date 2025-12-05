/*
 * Scheduling in busy system feature for Exynos Mobile Scheduler (EMS)
 *
 * Copyright (C) 2020 Samsung Electronics Co., Ltd
 * Choonghoon Park <choong.park@samsung.com>
 */

#include "ems.h"

#include <trace/events/ems.h>
#include <trace/events/ems_debug.h>

struct sysbusy {
	raw_spinlock_t lock;
	u64 last_update_time;
	u64 activated_ts;
	int monitor_interval;
	int boost_duration;
	enum sysbusy_state state;
	struct work_struct work;
	int emstune_level;
	int enabled;
} sysbusy;

struct sysbusy_states {
	struct kobject kobj;
	int monitor_interval;		/* tick (1 tick = 4ms) */
	int boost_duration;		/* tick (1 tick = 4ms) */
} sysbusy_states[NUM_OF_SYSBUSY_STATE];

struct somac_env {
	struct kthread_work	work;
	struct kthread_worker	*worker;
	struct task_struct	*src_p;
	struct task_struct	*dst_p;
	struct rq		*src_rq;
	struct rq		*dst_rq;
	int			src_cpu;
	int			dst_cpu;
};
static struct somac_env __percpu *somac_env;

struct sysbusy_stat {
	int count;
	u64 last_time;
	u64 time_in_state;
} sysbusy_stats[NUM_OF_SYSBUSY_STATE];

/******************************************************************************
 * sysbusy notify                                                             *
 ******************************************************************************/
int sysbusy_monitor_interval(int state)
{
	return sysbusy_states[state].monitor_interval;
}

static RAW_NOTIFIER_HEAD(sysbusy_chain);

static int sysbusy_notify(int next_state, int event)
{
	int ret;

	ret = raw_notifier_call_chain(&sysbusy_chain, event, &next_state);

	return notifier_to_errno(ret);
}

int sysbusy_register_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&sysbusy_chain, nb);
}
EXPORT_SYMBOL_GPL(sysbusy_register_notifier);

int sysbusy_unregister_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&sysbusy_chain, nb);
}
EXPORT_SYMBOL_GPL(sysbusy_unregister_notifier);

/******************************************************************************
 *			            SOMAC			              *
 ******************************************************************************/
int sysbusy_on_somac(void)
{
	return (sysbusy.state == SYSBUSY_STATE2) || (sysbusy.state == SYSBUSY_STATE3);
}

static void sysbusy_migrate_swap_work(struct kthread_work *work)
{
	struct somac_env *env = container_of(work, struct somac_env, work);
	struct rq *src_rq = env->src_rq;
	struct rq *dst_rq = env->dst_rq;
	struct task_struct *src_p = env->src_p;
	struct task_struct *dst_p = env->dst_p;
	unsigned long flags;

	migrate_swap(env->src_p, env->dst_p, env->dst_cpu, env->src_cpu);

	put_task_struct(src_p);
	put_task_struct(dst_p);

	local_irq_save(flags);
	double_rq_lock(src_rq, dst_rq);

	src_rq->active_balance = 0;
	dst_rq->active_balance = 0;

	ems_clear_migrated(src_rq);
	ems_clear_migrated(dst_rq);

	double_rq_unlock(src_rq, dst_rq);
	local_irq_restore(flags);
}

static bool sysbusy_migrate_swap(int src_cpu, int dst_cpu)
{
	struct somac_env *env = per_cpu_ptr(somac_env, src_cpu);
	struct rq *src_rq = cpu_rq(src_cpu);
	struct rq *dst_rq = cpu_rq(dst_cpu);
	struct task_struct *src_p = src_rq->curr;
	struct task_struct *dst_p = dst_rq->curr;
	unsigned long flags;

	local_irq_save(flags);
	double_rq_lock(src_rq, dst_rq);

	if (!cpumask_test_cpu(dst_cpu, src_p->cpus_ptr))
		goto fail;

	if (!cpumask_test_cpu(src_cpu, dst_p->cpus_ptr))
		goto fail;

	env->src_cpu = src_cpu;
	env->src_rq = src_rq;
	env->src_p = src_p;
	ems_set_migrated(src_rq);
	src_rq->active_balance = 1;
	get_task_struct(src_p);

	env->dst_cpu = dst_cpu;
	env->dst_rq = dst_rq;
	env->dst_p = dst_p;
	ems_set_migrated(dst_rq);
	dst_rq->active_balance = 1;
	get_task_struct(dst_p);

	double_rq_unlock(src_rq, dst_rq);
	local_irq_restore(flags);

	kthread_queue_work(env->worker, &env->work);

	return true;

fail:
	double_rq_unlock(src_rq, dst_rq);
	local_irq_restore(flags);

	return false;
}

static bool sysbusy_migrate_runnable_task(int src_cpu, int dst_cpu)
{
	struct rq *src_rq = cpu_rq(src_cpu);
	struct rq *dst_rq = cpu_rq(dst_cpu);
	struct task_struct *p, *pulled_task = NULL;
	unsigned long flags;

	raw_spin_lock_irqsave(&src_rq->__lock, flags);
	list_for_each_entry_reverse(p, &src_rq->cfs_tasks, se.group_node) {
		if (task_current(src_rq, p))
			continue;

		if (!cpumask_test_cpu(dst_cpu, p->cpus_ptr))
			continue;

		if (cpuctl_task_group_idx(p) != CGROUP_TOPAPP)
			continue;

		pulled_task = p;
		break;
	}

	if (!pulled_task || !available_idle_cpu(dst_cpu)) {
		raw_spin_unlock_irqrestore(&src_rq->__lock, flags);
		return 0;
	}

	update_rq_clock(src_rq);
	detach_task(src_rq, dst_rq, p);

	raw_spin_unlock_irqrestore(&src_rq->__lock, flags);

	raw_spin_lock_irqsave(&dst_rq->__lock, flags);
	update_rq_clock(dst_rq);
	attach_task(dst_rq, pulled_task);
	raw_spin_unlock_irqrestore(&dst_rq->__lock, flags);

	return 1;
}

static int somac_interval = 4; /* 4tick = 16ms */

/******************************************************************************
 *			       sysbusy schedule			              *
 ******************************************************************************/
int sysbusy_activated(void)
{
	return sysbusy.state > SYSBUSY_STATE0;
}

static enum sysbusy_state determine_sysbusy_state(u64 now)
{
	struct system_profile_data system_data;
	int prev = sysbusy.state, next;

	get_system_sched_data(&system_data);

	/* sysbusy is only activated when emstune level 0 */
	if (sysbusy.emstune_level > 0) {
		next = SYSBUSY_STATE0;
		goto out;
	}

	if ((system_data.busy_cpu_count == 1) && (system_data.heavy_task_count == 1)) {
		if (check_busy(system_data.heavy_task_util_sum, system_data.cpu_util_sum)) {
			if (sysbusy_states[SYSBUSY_STATE1].boost_duration > 0) {
				next = SYSBUSY_STATE1;
				goto out;
			}
		}
	}

	if (system_data.avg_nr_run_sum >= num_online_cpus() &&
		(system_data.heavy_task_count >= (num_online_cpus() >> 1))) {
		if (sysbusy_states[SYSBUSY_STATE3].boost_duration > 0) {
			next = SYSBUSY_STATE3;
			goto out;
		}
	}

	if (system_data.heavy_task_count >= num_online_cpus()) {
		if (sysbusy_states[SYSBUSY_STATE3].boost_duration > 0) {
			next = SYSBUSY_STATE3;
			goto out;
		}
	}

	if (system_data.avg_nr_heavy_sum >= num_possible_cpus()) {
		if (sysbusy_states[SYSBUSY_STATE2].boost_duration > 0) {
			next = SYSBUSY_STATE2;
			goto out;
		}
	}

	/* not sysbusy or no heavy task */
	next = SYSBUSY_STATE0;

out:
	if (next != SYSBUSY_STATE0)
		sysbusy.activated_ts = now;

	trace_sysbusy_determine_state(prev, next, system_data.busy_cpu_count,
			system_data.heavy_task_util_sum, system_data.cpu_util_sum,
			system_data.avg_nr_run_sum, system_data.heavy_task_count,
			system_data.avg_nr_heavy_sum);

	return next;
}

static void
update_sysbusy_stat(int old_state, int next_state, unsigned long now)
{
	sysbusy_stats[old_state].count++;
	sysbusy_stats[old_state].time_in_state +=
		now - sysbusy_stats[old_state].last_time;

	sysbusy_stats[next_state].last_time = now;
}

static void change_sysbusy_state(int next_state, u64 now)
{
	int old_state = sysbusy.state;

	if (old_state == next_state)
		return;

	/* release sysbusy */
	if (next_state == SYSBUSY_STATE0) {
		if ((sysbusy.emstune_level == 0) &&
		    (now - sysbusy.activated_ts <= sysbusy.boost_duration))
			return;
	}

	sysbusy.monitor_interval = sysbusy_states[next_state].monitor_interval;
	sysbusy.boost_duration = sysbusy_states[next_state].boost_duration;
	sysbusy.state = next_state;

	schedule_work(&sysbusy.work);
	update_sysbusy_stat(old_state, next_state, now);
	trace_sysbusy_state(old_state, next_state);
}

int sysbusy_boost_task(struct task_struct *p)
{
	int grp_idx = cpuctl_task_group_idx(p);

	if (sysbusy.state != SYSBUSY_STATE1)
		return 0;

	if (grp_idx != CGROUP_TOPAPP)
		return 0;

	return is_heavy_task_util(ml_task_util_est(p));
}

void monitor_sysbusy(void)
{
	enum sysbusy_state next_state;
	u64 now = jiffies;
	unsigned long flags;

	if (!raw_spin_trylock_irqsave(&sysbusy.lock, flags))
		return;

	if (!sysbusy.enabled) {
		raw_spin_unlock_irqrestore(&sysbusy.lock, flags);
		return;
	}

	if (now < sysbusy.last_update_time + sysbusy.monitor_interval) {
		raw_spin_unlock_irqrestore(&sysbusy.lock, flags);
		return;
	}

	sysbusy.last_update_time = now;

	next_state = determine_sysbusy_state(now);

	change_sysbusy_state(next_state, now);
	raw_spin_unlock_irqrestore(&sysbusy.lock, flags);
}

void sysbusy_dequeue_task(struct task_struct *p)
{
	if (get_sched_class(p) != EMS_SCHED_FAIR)
		return;

	if (cpuctl_task_group_idx(p) != CGROUP_TOPAPP)
		return;

	if (READ_ONCE(p->__state) == TASK_RUNNING)
		return;

	ems_yield_cnt(p) = 0;
}

void sysbusy_do_yield(struct task_struct *p)
{
	if (get_sched_class(p) != EMS_SCHED_FAIR)
		return;

	if (cpuctl_task_group_idx(p) != CGROUP_TOPAPP)
		return;

	ems_yield_cnt(p) += 1;
}

#define BUSY_WAITING_COUNT	(1000)
void sysbusy_update_busy_wait(struct rq *rq)
{
	struct task_struct *p = rq->curr;
	u64 prev, curr, delta;

	if (get_sched_class(p) != EMS_SCHED_FAIR)
		return;

	if (cpuctl_task_group_idx(p) != CGROUP_TOPAPP)
		return;

	prev = ems_yield_last(p);
	curr = ems_yield_cnt(p);

	delta = curr - prev;
	if ((s64)delta < 0)
		delta = 0;

	ems_busy_waiting(p) = delta > BUSY_WAITING_COUNT;
	if (ems_busy_waiting(p))
		trace_sysbusy_busy_waiting(cpu_of(rq), p, delta);

	ems_yield_last(p) = curr;
}

/**********************************************************************
 *			    SYSFS support			      *
 **********************************************************************/
static struct kobject *sysbusy_kobj;

static ssize_t show_somac_interval(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 10, "%d\n", somac_interval);
}

static ssize_t store_somac_interval(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int input;

	if (sscanf(buf, "%d", &input) != 1)
		return -EINVAL;

	somac_interval = input;

	return count;
}

static struct kobj_attribute somac_interval_attr =
__ATTR(somac_interval, 0644, show_somac_interval, store_somac_interval);

static ssize_t show_sysbusy_stat(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int i, count = 0;

	for (i = SYSBUSY_STATE0; i < NUM_OF_SYSBUSY_STATE; i++)
		count += snprintf(buf + count, PAGE_SIZE - count,
				"[state%d] count:%d time_in_state=%ums\n",
				i, sysbusy_stats[i].count,
				jiffies_to_msecs(sysbusy_stats[i].time_in_state));

	return count;
}

static struct kobj_attribute sysbusy_stat_attr =
__ATTR(sysbusy_stat, 0644, show_sysbusy_stat, NULL);

static ssize_t show_sysbusy_control(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 10, "%d\n", sysbusy.enabled);
}

static ssize_t store_sysbusy_control(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int input;
	unsigned long flags;

	if (sscanf(buf, "%d", &input) != 1)
		return -EINVAL;

	/* Make input 0 or 1 */
	input = !!input;

	/* No state change, just return */
	if (sysbusy.enabled == input)
		return count;

	raw_spin_lock_irqsave(&sysbusy.lock, flags);

	/* Store input value to enabled with sysbusy.lock */
	sysbusy.enabled = input;

	/* If sysbusy is disabled, transition to SYSBUSY_STATE0 */
	if (!sysbusy.enabled)
		change_sysbusy_state(SYSBUSY_STATE0, jiffies);

	raw_spin_unlock_irqrestore(&sysbusy.lock, flags);

	return count;
}

static struct kobj_attribute sysbusy_control_attr =
__ATTR(sysbusy_control, 0644, show_sysbusy_control, store_sysbusy_control);

struct sysbusy_state_attr {
	struct attribute attr;
	ssize_t (*show)(struct kobject *, char *);
	ssize_t (*store)(struct kobject *, const char *, size_t count);
};

#define sysbusy_state_attr_rw(name)							\
static struct sysbusy_state_attr name##_attr =						\
__ATTR(name, 0644, show_##name, store_##name)

#define sysbusy_state_show(name)							\
static ssize_t show_##name(struct kobject *k, char *buf)				\
{											\
	struct sysbusy_states *state = container_of(k, struct sysbusy_states, kobj);	\
											\
	return sprintf(buf, "%d\n", state->name);					\
}											\

#define sysbusy_state_store(name)							\
static ssize_t store_##name(struct kobject *k, const char *buf, size_t count)		\
{											\
	struct sysbusy_states *state = container_of(k, struct sysbusy_states, kobj);	\
	unsigned int data;								\
											\
	if (!sscanf(buf, "%d", &data))							\
		return -EINVAL;								\
											\
	if (data <= 0)									\
		data = 0;								\
											\
	state->name = data;								\
	return count;									\
}

sysbusy_state_show(monitor_interval);
sysbusy_state_store(monitor_interval);
sysbusy_state_attr_rw(monitor_interval);

sysbusy_state_show(boost_duration);
sysbusy_state_store(boost_duration);
sysbusy_state_attr_rw(boost_duration);

static ssize_t show(struct kobject *kobj, struct attribute *at, char *buf)
{
	struct sysbusy_state_attr *fvattr = container_of(at,
			struct sysbusy_state_attr, attr);

	return fvattr->show(kobj, buf);
}

static ssize_t store(struct kobject *kobj, struct attribute *at,
		const char *buf, size_t count)
{
	struct sysbusy_state_attr *fvattr = container_of(at,
			struct sysbusy_state_attr, attr);

	return fvattr->store(kobj, buf, count);
}

static const struct sysfs_ops sysbusy_state_sysfs_ops = {
	.show	= show,
	.store	= store,
};

static struct attribute *sysbusy_state_attrs[] = {
	&monitor_interval_attr.attr,
	&boost_duration_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(sysbusy_state);

static struct kobj_type ktype_sysbusy_state = {
	.sysfs_ops	= &sysbusy_state_sysfs_ops,
	.default_groups	= sysbusy_state_groups,
};

static int sysbusy_state_sysfs_init(struct kobject *sysbusy_kobj) {
	int i;

	for (i = 0; i < NUM_OF_SYSBUSY_STATE; i++) {
		struct sysbusy_states *state = &sysbusy_states[i];

		if (kobject_init_and_add(&state->kobj, &ktype_sysbusy_state, sysbusy_kobj,
					"%s", sysbusy_state_names[i])) {
			pr_err("%s: failed to init sysbusy state kobject\n", __func__);
			return -ENOMEM;
		}
	}

	return 0;
}

static int sysbusy_sysfs_init(struct kobject *ems_kobj)
{
	int ret;

	sysbusy_kobj = kobject_create_and_add("sysbusy", ems_kobj);
	if (!sysbusy_kobj) {
		pr_info("%s: fail to create node\n", __func__);
		return -EINVAL;
	}

	ret = sysfs_create_file(sysbusy_kobj, &somac_interval_attr.attr);
	if (ret)
		pr_warn("%s: failed to create somac sysfs\n", __func__);

	ret = sysfs_create_file(sysbusy_kobj, &sysbusy_stat_attr.attr);
	if (ret)
		pr_warn("%s: failed to create somac sysfs\n", __func__);

	ret = sysfs_create_file(sysbusy_kobj, &sysbusy_control_attr.attr);
	if (ret)
		pr_warn("%s: failed to create sysbusy sysfs\n", __func__);

	ret = sysbusy_state_sysfs_init(sysbusy_kobj);
	if (ret)
		pr_warn("%s: failed to create sysbusy state sysfs\n", __func__);

	return ret;
}

/**********************************************************************
 *			    Initialization			      *
 **********************************************************************/
static void sysbusy_boost_fn(struct work_struct *work)
{
	int cur_state;
	unsigned long flags;

	raw_spin_lock_irqsave(&sysbusy.lock, flags);
	cur_state = sysbusy.state;
	raw_spin_unlock_irqrestore(&sysbusy.lock, flags);

	if (cur_state > SYSBUSY_STATE0) {
		/*
		 * if sysbusy is activated, notifies whether it is
		 * okay to trigger sysbusy boost.
		 */
		if (sysbusy_notify(cur_state, SYSBUSY_CHECK_BOOST))
			return;
	}

	sysbusy_notify(cur_state, SYSBUSY_STATE_CHANGE);
}

static bool need_active_balance(int src_cpu, int dst_cpu)
{
	struct rq *src_rq, *dst_rq;
	unsigned long util;

	if (src_cpu < 0 || dst_cpu < 0)
		return false;

	src_rq = cpu_rq(src_cpu);
	dst_rq = cpu_rq(dst_cpu);

	if (!available_idle_cpu(dst_cpu)) {
		trace_sysbusy_need_active_balance(src_cpu, dst_cpu, "not-idle");
		return false;
	}

	if (ems_rq_cluster_idx(src_rq) >= ems_rq_cluster_idx(dst_rq)) {
		trace_sysbusy_need_active_balance(src_cpu, dst_cpu, "to-slower");
		return false;
	}

	if (ems_busy_waiting(src_rq->curr)) {
		trace_sysbusy_need_active_balance(src_cpu, dst_cpu, "busy-waiting");
		return false;
	}

	if (get_sched_class(src_rq->curr) != EMS_SCHED_FAIR) {
		trace_sysbusy_need_active_balance(src_cpu, dst_cpu, "not-fair");
		return false;
	}

	if (cpuctl_task_group_idx(src_rq->curr) != CGROUP_TOPAPP) {
		trace_sysbusy_need_active_balance(src_cpu, dst_cpu, "not-topapp");
		return false;
	}

	util = ml_task_util_est(src_rq->curr);
	if (!is_heavy_task_util(util)) {
		trace_sysbusy_need_active_balance(src_cpu, dst_cpu, "not-htask");
		return false;
	}

	trace_sysbusy_need_active_balance(src_cpu, dst_cpu, "need-active-balance");

	return true;
}

int sysbusy_shuffle(struct rq *src_rq)
{
	int cpu;
	int src_cpu = cpu_of(src_rq);
	u64 cur, run, max_run = 0;
	int slow_max_cpu = -1;
	int fast_max_cpu = -1;
	int busy_cpu = -1, idle_cpu = -1;
	int nr_running;
	struct rq *rq;

	if (!sysbusy.enabled)
		return true;

	for_each_cpu(cpu, ecs_available_cpus()) {
		rq = cpu_rq(cpu);
		nr_running = rq->nr_running;

		if (!cpu_active(cpu))
			continue;

		if (rq->active_balance || ems_rq_migrated(rq))
			continue;

		if (available_idle_cpu(cpu))
			idle_cpu = cpu;
		else if (nr_running > 1)
			busy_cpu = cpu;
	}

	if (busy_cpu >= 0 && idle_cpu >= 0) {
		if (sysbusy_migrate_runnable_task(src_cpu, idle_cpu))
			trace_sysbusy_somac_reason(src_cpu, idle_cpu, "BUSY -> IDLE");
		return true;
	}

	if (need_active_balance(src_cpu, idle_cpu)) {
		if (lb_migrate_running_task(src_cpu, idle_cpu, src_rq->curr)) {
			trace_sysbusy_somac_reason(src_cpu, idle_cpu, "ACTIVE-BALANCE");
			return true;
		}
		return false;
	}

	if (!cpumask_test_cpu(src_cpu, cpu_slowest_mask()))
		return true;

	cur = ktime_get_ns();

	for_each_cpu_and(cpu, cpu_slowest_mask(), ecs_available_cpus()) {
		rq = cpu_rq(cpu);

		if (!cpu_active(cpu))
			continue;

		if (rq->active_balance || ems_rq_migrated(rq))
			continue;

		if (!ems_rq_nr_misfited(rq) || get_sched_class(rq->curr) != EMS_SCHED_FAIR)
			continue;

		if (ems_busy_waiting(rq->curr))
			continue;

		run = cur - ems_enqueue_ts(rq->curr);
		if (run > max_run) {
			max_run = run;
			slow_max_cpu = cpu;
		}
	}

	if (slow_max_cpu != src_cpu)
		return true;

	max_run = 0;

	for_each_cpu(cpu, ecs_available_cpus()) {
		struct rq *rq = cpu_rq(cpu);

		if (cpumask_test_cpu(cpu, cpu_slowest_mask()))
			continue;

		if (!cpu_active(cpu))
			continue;

		if (rq->active_balance || ems_rq_migrated(rq))
			continue;

		if (get_sched_class(rq->curr) != EMS_SCHED_FAIR)
			continue;

		if (rq->nr_running > 1)
			continue;

		if (ems_busy_waiting(rq->curr)) {
			fast_max_cpu = cpu;
			break;
		}

		run = cur - ems_enqueue_ts(rq->curr);

		if (run < (somac_interval * TICK_NSEC))
			continue;

		if (run > max_run) {
			max_run = run;
			fast_max_cpu = cpu;
		}
	}

	if (fast_max_cpu == -1)
		return true;

	if (sysbusy_migrate_swap(slow_max_cpu, fast_max_cpu))
		trace_sysbusy_somac_reason(slow_max_cpu, fast_max_cpu, "ROTATION");

	return true;
}

static int sysbusy_parse_dt(void)
{
	struct device_node *dn, *child;

	dn = of_find_node_by_path("/ems/sysbusy");
	if (!dn) {
		pr_err("%s: failed to find device node\n", __func__);
		return -EINVAL;
	}

	for_each_child_of_node(dn, child) {
		struct sysbusy_states *state;
		int idx;

		if (of_property_read_s32(child, "state", &idx)) {
			pr_err("%s: state property is omitted\n", __func__);
			return -EINVAL;
		}

		state = &sysbusy_states[idx];

		if (of_property_read_s32(child, "monitor-interval", &state->monitor_interval))
			state->monitor_interval = 1;

		if (of_property_read_s32(child, "boost-duration", &state->boost_duration))
			state->boost_duration = 0;
	}

	return 0;
}

/******************************************************************************
 *                       emstune mode update notifier                         *
 ******************************************************************************/
static int sysbusy_emstune_update_callback(struct notifier_block *nb,
				unsigned long val, void *v)
{
	struct emstune_set *cur_set = (struct emstune_set *)v;
	unsigned long flags;

	raw_spin_lock_irqsave(&sysbusy.lock, flags);

	sysbusy.emstune_level = cur_set->level;

	if (sysbusy.emstune_level > 0)
		change_sysbusy_state(SYSBUSY_STATE0, jiffies);

	raw_spin_unlock_irqrestore(&sysbusy.lock, flags);

	return NOTIFY_OK;
}

static struct notifier_block sysbusy_emstune_update_notifier = {
	.notifier_call = sysbusy_emstune_update_callback,
};

int sysbusy_init(struct kobject *ems_kobj)
{
	int ret;
	int cpu;

	ret = sysbusy_parse_dt();
	if (ret) {
		pr_err("%s: failed to parse dt\n", __func__);
		return -EINVAL;
	}

	somac_env = alloc_percpu(struct somac_env);
	if (!somac_env) {
		pr_err("failed to allocate somac_env\n");
		return -ENOMEM;
	}

	for_each_possible_cpu(cpu) {
		struct somac_env *env = per_cpu_ptr(somac_env, cpu);

		env->worker = kthread_create_worker_on_cpu(cpu, 0, "sysbusy/%u", cpu);
		if (IS_ERR(env->worker)) {
			pr_err("%s: failed to create worker\n", __func__);
			return PTR_ERR(env->worker);
		}

		sched_set_fifo(env->worker->task);
		kthread_init_work(&env->work, sysbusy_migrate_swap_work);
	}

	raw_spin_lock_init(&sysbusy.lock);
	INIT_WORK(&sysbusy.work, sysbusy_boost_fn);
	emstune_register_notifier(&sysbusy_emstune_update_notifier);

	ret = sysbusy_sysfs_init(ems_kobj);
	if (ret)
		pr_err("failed to init sysfs for sysbusy\n");

	if (!ret)
		sysbusy.enabled = 1;

	pr_info("SYSBUSY initialized\n");

	return ret;
}
