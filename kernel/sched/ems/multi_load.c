/*
 * Multi-purpose Load tracker
 *
 * Copyright (C) 2023 Samsung Electronics Co., Ltd
 * S.LSI, SoC Dev, APSW Team, CPUSW&Performance Part
 */
#include "ems.h"

#include <trace/events/ems.h>
#include <trace/events/ems_debug.h>

static bool mlt_disabled = true;
struct mlt_cluster mlt_clusters[MLT_MAX_CLUSTER_NUM];

const char *mlt_load_flag_name[] = {
	"new_task",
	"wakeup_task",
	"update_task",
	"put_prev_task",
	"pick_next_task",
};

/******************************************************************************
 *                           MULTI LOAD for TASK                              *
 ******************************************************************************/
/*
 * ml_task_util - task util
 *
 * Task utilization. The calculation is the same as the task util of cfs.
 */
unsigned long ml_task_util(struct task_struct *p)
{
	struct dslt_avg *dsa = &ems_dslt_task(p)->avg;
	return READ_ONCE(dsa->runnable_avg);
}

/*
 * ml_task_util_est - task util with util-est
 *
 * Task utilization with util-est, The calculation is the same as
 * task_util_est of cfs.
 */
unsigned long _ml_task_util_est(struct task_struct *p)
{
	struct dslt_avg *dsa = &ems_dslt_task(p)->avg;
	return READ_ONCE(dsa->util_est) & ~UTIL_AVG_UNCHANGED;
}

unsigned long ml_task_util_est(struct task_struct *p)
{
	return max(ml_task_util(p), _ml_task_util_est(p));
}

/******************************************************************************
 *                            MULTI LOAD for CPU                              *
 ******************************************************************************/
/*
 * ml_cpu_util - cpu utilization
 */
unsigned long ml_cpu_util(int cpu)
{
	struct ems_dslt_rq *drq = ems_rq_dslt(cpu_rq(cpu));
	unsigned int util;

	util = READ_ONCE(drq->avg.runnable_avg);
	util = max(util, READ_ONCE(drq->avg.util_est));

	return min_t(unsigned long, util, capacity_cpu_orig(cpu));
}
EXPORT_SYMBOL_GPL(ml_cpu_util);

/*
 * Remove and clamp on negative, from a local variable.
 *
 * A variant of sub_positive(), which does not use explicit load-store
 * and is thus optimized for local variable updates.
 */
#define lsub_positive(_ptr, _val) do {				\
	typeof(_ptr) ptr = (_ptr);				\
	*ptr -= min_t(typeof(*ptr), *ptr, _val);		\
} while (0)

unsigned long ml_cpu_irq_load(int cpu)
{
#ifdef CONFIG_HAVE_SCHED_AVG_IRQ
	struct rq *rq = cpu_rq(cpu);

	return rq->avg_irq.util_avg;
#else
	return 0;
#endif
}

/*
 * ml_cpu_util_without - cpu utilization without waking task
 */
unsigned long ml_cpu_util_without(int cpu, struct task_struct *p)
{
	struct dslt_avg  *_avg = &ems_rq_dslt(cpu_rq(cpu))->avg;
	struct ems_dslt_task *dtsk = ems_dslt_task(p);
	unsigned long cpu_util = READ_ONCE(_avg->runnable_avg);
	unsigned long cpu_util_est = READ_ONCE(_avg->util_est);

	if (cpu == task_cpu(p)) {
		unsigned long task_util, task_util_est;

		if (READ_ONCE(dtsk->avg.last_update_time)) {
			task_util = max(ml_task_util(p), (unsigned long)1);
			lsub_positive(&cpu_util, task_util);
		}

		if (task_on_rq_queued(p) || current == p) {
			task_util_est = max(_ml_task_util_est(p), (unsigned long)1);
			lsub_positive(&cpu_util_est, task_util_est);
		}
	}

	cpu_util = max(cpu_util, cpu_util_est);

	return min_t(unsigned long, cpu_util, capacity_cpu_orig(cpu));
}

/*
 * ml_cpu_util_with - cpu utilization with waking task
 *
 * When task is queued,
 * 1) task_cpu(p) == cpu
 *    => The contribution is already present on target cpu
 * 2) task_cpu(p) != cpu
 *    => The contribution is not present on target cpu
 *
 * When task is not queued,
 * 3) task_cpu(p) == cpu
 *    => The contribution is already applied to util_avg,
 *       but is not applied to util_est
 * 4) task_cpu(p) != cpu
 *    => The contribution is not present on target cpu
 */
unsigned long ml_cpu_util_with(struct task_struct *p, int cpu)
{
	struct dslt_avg *_avg = &ems_rq_dslt(cpu_rq(cpu))->avg;
	unsigned long cpu_util = READ_ONCE(_avg->runnable_avg);
	unsigned long cpu_util_est = READ_ONCE(_avg->util_est);

	if (cpu != task_cpu(p)) {
		cpu_util += max(ml_task_util(p), (unsigned long)1);
		cpu_util_est += max(_ml_task_util_est(p), (unsigned long)1);
	} else if (!task_on_rq_queued(p)) {
		cpu_util_est += max(_ml_task_util_est(p), (unsigned long)1);
	}

	cpu_util = max(cpu_util, cpu_util_est);

	return cpu_util;
}

/******************************************************************************
 *                     New task utilization init                              *
 ******************************************************************************/
static int ntu_ratio[CGROUP_COUNT];

void ntu_apply(struct task_struct *p)
{
	struct sched_entity *se = &p->se;
	struct sched_avg *sa = &se->avg;
	struct sched_entity *pse = &current->se;
	struct sched_avg *psa = &pse->avg;
	int grp_idx = cpuctl_task_group_idx(task_of(se));
	int ratio = ntu_ratio[grp_idx];

	/*
	 * The new task has inherited parent's util_avg and runnable_avg
	 * via ratio for each cgroups.
	 */
	sa->util_avg = psa->util_avg * ratio / 100;
	sa->runnable_avg = psa->runnable_avg * ratio / 100;
}

static int
ntu_emstune_notifier_call(struct notifier_block *nb,
					   unsigned long val, void *v)
{
	struct emstune_set *cur_set = (struct emstune_set *)v;
	int i;

	for (i = 0; i < CGROUP_COUNT; i++)
		ntu_ratio[i] = cur_set->ntu.ratio[i];

	return NOTIFY_OK;
}

static struct notifier_block ntu_emstune_notifier = {
	.notifier_call = ntu_emstune_notifier_call,
};

void ntu_init(struct kobject *ems_kobj)
{
	int i;

	for (i = 0; i < CGROUP_COUNT; i++)
		ntu_ratio[i] = 0;

	emstune_register_notifier(&ntu_emstune_notifier);
}

/******************************************************************************
 *                           MULTI LOAD for UARCH			      *
 ******************************************************************************/
static bool uarch_supported;
struct mlt_cpu __percpu *pcpu_mlt;		/* active ratio tracking */
static inline int part_move_period(int period, int count);

enum pmu_event {
	CPU_CYCLE,
	INST_RETIRED,
	NUM_OF_PMU_DATA,
};

int pmu_event_id[NUM_OF_PMU_DATA] = {
	ARMV8_PMUV3_PERFCTR_CPU_CYCLES,
	ARMV8_PMUV3_PERFCTR_INST_RETIRED,
};

static int mlt_cpu_pmu_event_tbl[MLT_RAW_CPU_NUM] = {
	CPU_CYCLE,
	INST_RETIRED,
};

static int mlt_tsk_pmu_event_tbl[MLT_RAW_TSK_NUM] = {
	CPU_CYCLE,
	INST_RETIRED,
};

struct pmu_data {
	int				cpu;
	struct perf_event	*pe[NUM_OF_PMU_DATA];
	/* pmu event data */
};
static DEFINE_PER_CPU(struct pmu_data, pmu_data);

static u64 read_pmu_events(struct pmu_data *data, int idx)
{
	u64 ret = 0;

	perf_event_read_local(data->pe[idx], &ret, NULL, NULL);

	return ret;
}

static int activate_pmu_events(struct pmu_data *data)
{
	struct perf_event *pe;
	struct perf_event_attr *pe_attr;
	int i;

	pe_attr = kzalloc(sizeof(struct perf_event_attr), GFP_KERNEL);
	if (!pe_attr) {
		pr_err("failed to allocate perf event attr\n");
		return -ENOMEM;
	}

	pe_attr->size = (u32)sizeof(struct perf_event_attr);
	pe_attr->type = PERF_TYPE_RAW;
	pe_attr->pinned = 1;
	pe_attr->config1 = 1;	/* enable 64-bit counter */

	for (i = 0; i < NUM_OF_PMU_DATA; i++) {
		pe_attr->config = pmu_event_id[i];

		pe = perf_event_create_kernel_counter(pe_attr, data->cpu, NULL, NULL, NULL);
		if (IS_ERR(pe)) {
			pr_err("failed to create kernel perf event. CPU=%d\n", data->cpu);
			kfree(pe_attr);
			return -ENOMEM;
		}
		perf_event_enable(pe);
		data->pe[i] = pe;
	}

	kfree(pe_attr);

	return 0;
}

static int mlt_pmu_init(void) {
	int cpu;

	for_each_possible_cpu(cpu) {
		struct pmu_data *data = &per_cpu(pmu_data, cpu);
		data->cpu = cpu;
		if (activate_pmu_events(data)) {
			pr_err("failed to enable pmu events. CPU=%d\n", cpu);
			return -ENOMEM;
		}
		pr_info("cpu%d: mlt pmu event init complete.\n", cpu);
	}

	return 0;
}

static u64 get_cpu_uarch(enum mlt_uarch_cpu uarch_id, struct mlt_uarch_raw *raw)
{
	u64 ret = 0;

	switch (uarch_id) {
	case MLT_UARCH_CPU_IPC:
		if (likely(raw[MLT_RAW_CPU_CPU_CYCLE].contrib))
			ret = (raw[MLT_RAW_CPU_INST_RETIRED].contrib << 10) /
					raw[MLT_RAW_CPU_CPU_CYCLE].contrib;
		break;
	default:
		break;
	}

	return ret;
}

static u64 get_tsk_uarch(enum mlt_uarch_tsk uarch_id, struct mlt_uarch_raw *raw)
{
	u64 ret = 0;

	switch (uarch_id) {
	case MLT_UARCH_TSK_IPC:
		if (likely(raw[MLT_RAW_TSK_CPU_CYCLE].contrib))
			ret = (raw[MLT_RAW_TSK_INST_RETIRED].contrib << 10) /
					raw[MLT_RAW_TSK_CPU_CYCLE].contrib;
		break;
	default :
		break;
	}

	return ret;
}

static void
mlt_update_uarch_recent(struct mlt_env *env, u64 *contrib,
			int num_of_raw, int num_of_uarch)
{
	int idx;

	if (env->part->state == INACTIVE)
		return;

	for (idx = 0; idx < num_of_raw; idx++)
		env->raw[idx].contrib += contrib[idx];

	for (idx = 0; idx < num_of_uarch; idx++) {
		if (env->is_task)
			env->uarch[idx].recent = (u32) get_tsk_uarch(idx, env->raw);
		else
			env->uarch[idx].recent = (u32) get_cpu_uarch(idx, env->raw);
	}

}

static void
mlt_uarch_update_recent_elapsed(struct mlt_env *env, int num_of_raw,
				int num_of_uarch, int cur_period, u32 *val)
{
	int idx;

	for (idx = 0; idx < num_of_raw; idx++)
		env->raw[idx].contrib = 0;

	for (idx = 0; idx < num_of_uarch; idx++) {
		env->uarch[idx].periods[cur_period] = env->uarch[idx].recent;
		val[idx] = env->uarch[idx].periods[cur_period];
		env->uarch[idx].recent = 0;
	}
}

static void
mlt_uarch_update_periods(struct mlt_env *env, int num_of_uarch,
			int cur_period, int count, u32 *val)
{
	int idx;

	while (count--) {
		cur_period = part_move_period(cur_period, 1);

		if (env->part->state == ACTIVE) {
			for (idx = 0; idx < num_of_uarch; idx++)
				env->uarch[idx].periods[cur_period] = val[idx];
		} else {
			for (idx = 0; idx < num_of_uarch; idx++)
				env->uarch[idx].periods[cur_period] = 0;
		}
	}
}

static int
_mlt_update_uarch(struct mlt_env *env, u64 *contrib, u64 *remain,
			int num_of_raw, int num_of_uarch, int prd_cnt)
{
	int period = env->is_task ? env->part->active_period : env->part->cur_period;

	/* (1) update recent period */
	mlt_update_uarch_recent(env, contrib, num_of_raw, num_of_uarch);
	if (prd_cnt) {
		u32 val[MLT_MAX_RAW_NUM] = { 0, };
		int count = prd_cnt;

		/* (2) store recent period to period history */
		period = part_move_period(period, 1);
		mlt_uarch_update_recent_elapsed(env, num_of_raw,
						num_of_uarch, period, val);

		/* (3) update fully elapsed period */
		count--;
		count = min(count, MLT_PERIOD_COUNT);

		if (env->is_task && env->part->state == INACTIVE)
			return count;

		mlt_uarch_update_periods(env, num_of_uarch, period, count, val);

		/* (4) store remain time to recent period */
		mlt_update_uarch_recent(env, remain, num_of_raw, num_of_uarch);
	}

	return 0;
}

static void
mlt_update_uarch_raw(struct mlt_env *env, int *raw_tbl, int num_of_raw,
			u64 *contrib, u64 *remain, u64 remain_t, u64 now,
			u64 period_count)
{
	struct pmu_data *data;
	u64 remain_ratio = 0;
	int idx, last_period;
	bool valid = likely(now > env->part->last_updated);

	data = &per_cpu(pmu_data, smp_processor_id());
	if (unlikely(!data))
		return;

	if (likely(valid) && period_count)
		remain_ratio = (remain_t << 10) / (now - env->part->last_updated);

	last_period = part_move_period(env->part->cur_period, period_count);
	for (idx = 0; idx < num_of_raw; idx++) {
		u64 val = read_pmu_events(data, raw_tbl[idx]);

		if (likely(valid) && env->part->state == ACTIVE)
			contrib[idx] = max((u64) 0, (val - env->raw[idx].last));

		if (remain_ratio) {
			remain[idx] = ((contrib[idx] * remain_ratio) >> 10);
			contrib[idx] = max((u64) 0, (contrib[idx] - remain[idx]));
		}

		env->raw[idx].last = val;
	}
}

static void
mlt_update_uarch(struct mlt_env *env, u64 now, u64 remain_t, int prd_cnt)
{
	int *raw_tbl = env->is_task ? mlt_tsk_pmu_event_tbl : mlt_cpu_pmu_event_tbl;
	int num_of_raw = env->is_task ? MLT_RAW_TSK_NUM : MLT_RAW_CPU_NUM;
	int num_of_uarch = env->is_task ? MLT_UARCH_TSK_NUM : MLT_UARCH_CPU_NUM;
	u64 contrib[MLT_MAX_RAW_NUM] = { 0, }, remain[MLT_MAX_RAW_NUM] = { 0 , };
	int idle_prd_cnt;

	if (env->update_uarch)
		mlt_update_uarch_raw(env, raw_tbl, num_of_raw,
				contrib, remain, remain_t, now, prd_cnt);

	idle_prd_cnt = _mlt_update_uarch(env, contrib, remain,
					num_of_raw, num_of_uarch, prd_cnt);

	if (env->is_task) {
		prd_cnt = prd_cnt - idle_prd_cnt;
		env->part->active_period = part_move_period(env->part->active_period, prd_cnt);
	}
}

/******************************************************************************
 *                             Multi load tracking                            *
 ******************************************************************************/
static inline int part_move_period(int period, int count)
{
	return (period + count + MLT_PERIOD_COUNT) % MLT_PERIOD_COUNT;
}

static void update_part_periods(struct mlt_part *part,
			int cur_period, int count, int value)
{
	while (count--) {
		cur_period = part_move_period(cur_period, 1);
		if (part->state == ACTIVE)
			part->periods[cur_period] = value;
		else
			part->periods[cur_period] = 0;
	}
}

static void part_recent_elapsed(struct mlt_part *part, int cur_period)
{
	part->periods[cur_period] = part->recent;
	part->contrib = 0;
	part->recent = 0;
}

static void update_part_recent(struct mlt_part *part,
				u64 contrib, u64 period_size)
{
	if (part->state == INACTIVE)
		return;

	part->contrib += contrib;
	part->recent= div64_u64(
		part->contrib << SCHED_CAPACITY_SHIFT, period_size);
}

static void __update_part(struct mlt_part *part,
			int period, int period_count, u64 contrib,
			u64 remain, u64 period_size, u64 period_value)
{
	/*
	 * [before]
	 *           last_updated                                 now
	 *                 ↓                                       ↓
	 *                 |<---1,2--->|<-------3------->|<---4--->|
	 * timeline --|----------------|-----------------|-----------------|-->
	 *               recent period
	 *
	 * Update recent period first(1) and quit sequence if the recent
	 * period has not elapsed, otherwise store recent period in the period
	 * history(2). If more than 1 period has elapsed, fill in the elapsed
	 * period with an appropriate value (running = 1024, idle = 0) and
	 * store in the period history(3). And store remain time in new recent
	 * period(4).
	 *
	 * [after]
	 *                                                   last_updated
	 *                                                         ↓
	 *                                                         |
	 * timeline --|----------------|-----------------|-----------------|-->
	 *                                 last period      recent period
	 */

	/* (1) update recent period */
	update_part_recent(part, contrib, period_size);

	if (period_count) {
		int count = period_count;

		/* (2) store recent period to period history */
		period = part_move_period(period, 1);
		part_recent_elapsed(part, period);
		count--;

		/* (3) update fully elapsed period */
		count = min(count, MLT_PERIOD_COUNT);
		update_part_periods(part, period, count, period_value);

		/* (4) store remain time to recent period */
		update_part_recent(part, remain, period_size);
	}
}

static void mlt_update(struct mlt_env *env, u64 now)
{
	u64 contrib = 0, remain = 0;
	int period_count = 0;
	struct mlt_part *part = env->part;

	/* to sync-up period_start with sched tick */
	if (unlikely(part->period_start == ULLONG_MAX)) {
		if (env->tick)
			part->period_start = now - MLT_PERIOD_SIZE;
		else
			return;
	}

	if (likely(now > part->last_updated)) {
		contrib = min(now, part->period_start + MLT_PERIOD_SIZE);
		contrib -= part->last_updated;
	}

	if (likely(now > part->period_start)) {
		period_count = div64_u64_rem(now - part->period_start,
					MLT_PERIOD_SIZE, &remain);

		/*
		 * forcely update period_count when sched tick occured
		 * and remain is greater than 3.75ms
		 */
		if (env->tick && remain > MLT_PERIOD_SIZE_MIN) {
			period_count++;
			remain = 0;
		}
	}

	__update_part(part, part->cur_period, period_count,
			contrib, remain, MLT_PERIOD_SIZE,
			SCHED_CAPACITY_SCALE);

	if (uarch_supported)
		mlt_update_uarch(env, now, remain, period_count);

	part->cur_period = part_move_period(part->cur_period, period_count);
	part->period_start += MLT_PERIOD_SIZE * period_count;
	part->last_updated = now;

	if (env->next_state != MLT_STATE_NOCHANGE)
		env->part->state = env->next_state;
}

/******************************************************************************
 *                           MULTI LOAD for RUNNABLE			      *
 ******************************************************************************/
static void mlt_update_runnable_ctrb(struct rq *rq)
{
	int cpu = cpu_of(rq);
	struct mlt_runnable *runnable = &per_cpu_ptr(pcpu_mlt, cpu)->runnable;
	u64 delta, now = sched_clock();

	if (unlikely(now < runnable->last_updated))
		return;

	delta = now - runnable->last_updated;

	runnable->nr_run = rq->nr_running;
	runnable->runnable_contrib += (runnable->nr_run * delta);

	runnable->nr_topapp = ems_rq_nr_topapp(rq);
	runnable->topapp_runnable_contrib += (runnable->nr_topapp * delta);

	runnable->contrib += delta;
	runnable->last_updated = now;
}

static void mlt_update_runnable(struct rq *rq)
{
	int cpu = cpu_of(rq);
	struct mlt_runnable *runnable = &per_cpu_ptr(pcpu_mlt, cpu)->runnable;

	mlt_update_runnable_ctrb(rq);

	runnable->runnable = (runnable->runnable_contrib * MLT_RUNNABLE_UNIT)
							/ runnable->contrib;
	runnable->topapp_runnable = (runnable->topapp_runnable_contrib * MLT_RUNNABLE_UNIT)
							/ runnable->contrib;
	trace_mlt_runnable_update(cpu, runnable);

	runnable->topapp_runnable_contrib = 0;
	runnable->runnable_contrib = 0;
	runnable->contrib = 0;
}

int mlt_get_runnable_cpu(int cpu)
{
	struct mlt_runnable *runnable = &per_cpu_ptr(pcpu_mlt, cpu)->runnable;
	return runnable->runnable;
}

int mlt_topapp_runnable(int cpu)
{
	struct mlt_runnable *runnable = &per_cpu_ptr(pcpu_mlt, cpu)->runnable;
	return runnable->topapp_runnable;
}

/******************************************************************************
 *                           MULTI LOAD for EXTERN API's		      *
 ******************************************************************************/
#define MLT_VALID_PERIOD(x)	((x) > -1 && (x) < MLT_PERIOD_COUNT)
#define MLT_VALID_STATE(x)	((x) > -1 && (x) < CSTATE_MAX)
#define MLT_VALID_LENGTH(x)	MLT_VALID_PERIOD((x - 1))
/* return task's average ratio of given length from cur_period */
int __mlt_avg_value(struct mlt_part *part, int length)
{
	int oldest_idx, idx = part->cur_period, value = 0;
	int cnt, corrected_idx = 0;

	if (unlikely(!MLT_VALID_LENGTH(length)))
		return 0;

	/*
	 * Recent correction
	 * If recent value is higher then oldest window value,
	 * replace small value window with recent velue to incerease performance
	 */
	oldest_idx = mlt_period_with_delta(idx, -(length - 1));
	if (part->periods[oldest_idx] < part->recent) {
		corrected_idx = 1;
		value = part->recent;
	}

	for (cnt = 0; cnt < (length - corrected_idx); cnt++) {
		value += part->periods[idx];
		idx = mlt_period_with_delta(idx, -1);
	}

	return value / length;
}

u64 __mlt_get_target_period_value(struct mlt_part *part, int target_period)
{
	if (unlikely(!MLT_VALID_PERIOD(target_period)))
		return 0;

	return part->periods[target_period];
}

u64 __mlt_get_value(struct mlt_part *part, int arg, enum mlt_value_type type)
{
	switch (type) {
	case PERIOD:
		return __mlt_get_target_period_value(part, arg);
	case AVERAGE:
		return __mlt_avg_value(part, arg);
	}
	return 0;
}

/* return task's ratio of target_period */
u64 mlt_task_value(struct task_struct *p, int arg, enum mlt_value_type type)
{
	struct mlt_part *part = &task_mlt(p)->part;

	if (unlikely(!part))
		return 0;

	return __mlt_get_value(part, arg, type);
}

/* return cpu's ratio of target_period */
u64 mlt_cpu_value(int cpu, int arg, enum mlt_value_type type, int state)
{
	struct mlt_part *part;

	part = &per_cpu_ptr(pcpu_mlt, cpu)->part;

	if (unlikely(!part))
		return 0;

	return __mlt_get_value(part, arg, type);
}

u32 mlt_tsk_uarch(struct task_struct *tsk, enum mlt_uarch_tsk type, int size)
{
	struct mlt_part *part = &task_mlt(tsk)->part;
	struct mlt_uarch *uarch;
	int c, p, cur_period;
	u32 div = 0, sum = 0;

	if (!uarch_supported)
		return 0;

	if (unlikely(!part))
		return 0;

	if (unlikely(type > MLT_UARCH_TSK_NUM))
		return 0;

	if (unlikely(size < 0 || size > MLT_PERIOD_COUNT))
		return 0;

	uarch = &task_mlt(tsk)->uarch[type];
	if (!size)
		return uarch->recent;

	cur_period = part->active_period;
	for (c = 0; c < size; c++) {
		p = part_move_period(cur_period, -c);

		if (uarch->periods[p]) {
			sum += uarch->periods[p];
			div++;
		}
	}

	if (div)
		return sum / div;
	else
		return uarch->recent;
}

u32 mlt_cpu_uarch(int cpu, enum mlt_uarch_cpu type, int size)
{
	struct mlt_part *part = &per_cpu_ptr(pcpu_mlt, cpu)->part;
	struct mlt_uarch *uarch;
	int c, p, cur_period;
	s32 sum = 0;

	if (!uarch_supported)
		return 0;

	if (unlikely(!part))
		return 0;

	if (unlikely(type > MLT_UARCH_CPU_NUM))
		return 0;

	if (unlikely(size < 0 || size > MLT_PERIOD_COUNT))
		return 0;

	uarch = &per_cpu_ptr(pcpu_mlt, cpu)->uarch[type];
	if (!size)
		return uarch->recent;

	cur_period = part->cur_period;
	for (c = 0; c < size; c++) {
		p = part_move_period(cur_period, -c);
		sum += uarch->periods[p];
	}

	return sum / size;
}

int mlt_runnable(struct rq *rq)
{
	int cpu = cpu_of(rq);
	struct mlt_runnable *runnable = &per_cpu_ptr(pcpu_mlt, cpu)->runnable;
	return runnable->runnable;
}

static bool mlt_check_topapp_task(struct task_struct *p)
{
	if (get_sched_class(p) != EMS_SCHED_FAIR)
		return false;

	return cpuctl_task_group_idx(p) == CGROUP_TOPAPP;
}

/* It will be called before increase nr_run */
void mlt_enqueue_task(struct rq *rq, struct task_struct *p)
{
	mlt_update_runnable_ctrb(rq);

	if (mlt_check_topapp_task(p)) {
		ems_topapp_task(p) = 1;
		ems_rq_nr_topapp(rq) += 1;
	}
	else
		ems_topapp_task(p) = 0;
}

/* It will be called before decrease nr_run */
void mlt_dequeue_task(struct rq *rq, struct task_struct *p)
{
	mlt_update_runnable_ctrb(rq);

	if (ems_topapp_task(p))
		ems_rq_nr_topapp(rq) -= 1;

	BUG_ON(ems_rq_nr_topapp(rq) < 0);
}

void mlt_update_cpu(int cpu, int next_state, u64 now,
				bool update_uarch, bool tick)
{
	struct mlt_env env = {
		.part = &per_cpu_ptr(pcpu_mlt, cpu)->part,
		.uarch = per_cpu_ptr(pcpu_mlt, cpu)->uarch,
		.raw = per_cpu_ptr(pcpu_mlt, cpu)->uarch_raw,
		.next_state = next_state,
		.update_uarch = update_uarch,
		.is_task = false,
		.tick = tick,
	};

	mlt_update(&env, now);
	trace_mlt_update_cpu_part(cpu, env.part);

	if (uarch_supported)
		trace_mlt_update_cpu_uarch(cpu, env.part, env.uarch);
}

void mlt_update_task(struct task_struct *p, int next_state, u64 now,
					bool update_uarch, bool tick)
{
	struct mlt_env env = {
		.part = &task_mlt(p)->part,
		.uarch = task_mlt(p)->uarch,
		.raw = task_mlt(p)->uarch_raw,
		.next_state = next_state,
		.update_uarch = update_uarch,
		.is_task = true,
		.tick = tick,
	};

	mlt_update(&env, now);
	trace_mlt_update_task_part(p, env.part);

	if (uarch_supported)
		trace_mlt_update_tsk_uarch(p, env.part, env.uarch);
}

void mlt_tick(void)
{
	u64 now = sched_clock();
	int cpu;

	for_each_cpu(cpu, cpu_active_mask) {
		struct rq *rq = cpu_rq(cpu);
		struct rq_flags rf;

		rq_lock_irqsave(rq, &rf);

		mlt_update_runnable(rq);

		if (rq->nohz_tick_stopped)
			mlt_update_cpu(cpu, MLT_STATE_NOCHANGE, now, false, true);

		rq_unlock_irqrestore(rq, &rf);
	}
}

void mlt_task_switch(int cpu, struct task_struct *prev,
				struct task_struct *next)
{
	struct rq *rq = cpu_rq(cpu);
	u64 now = sched_clock();

	if (!is_idle_task(prev))
		mlt_update_task(prev, INACTIVE, now, true, false);

	if (!is_idle_task(next))
		mlt_update_task(next, ACTIVE, now, true, false);

	/* cpu wake up from idle */
	if (is_idle_task(prev))
		mlt_update_cpu(cpu, ACTIVE, now, true, false);

	/* cpu go to idle */
	else if (is_idle_task(next))
		mlt_update_cpu(cpu, INACTIVE, now, true, false);

	if (prev != next) {
		mlt_update_load(rq, prev, now, PUT_PREV_TASK);
		mlt_update_load(rq, next, now, PICK_NEXT_TASK);
	} else
		mlt_update_load(rq, prev, now, UPDATE_TASK);
}

int mlt_cur_period(int cpu)
{
	struct mlt_part *part = &(per_cpu_ptr(pcpu_mlt, cpu)->part);

	return part->cur_period;
}

int mlt_prev_period(int period)
{
	period = period - 1;
	if (period < 0)
		return MLT_PERIOD_COUNT - 1;

	return period;
}

int mlt_period_with_delta(int idx, int delta)
{
	if (delta > 0)
		return (idx + delta) % MLT_PERIOD_COUNT;

	idx = idx + delta;
	if (idx < 0)
		return MLT_PERIOD_COUNT + idx;

	return idx;
}

/* return task's ratio of current period */
int mlt_task_cur_period(struct task_struct *p)
{
	struct mlt_part *part = &task_mlt(p)->part;
	return part->cur_period;
}

#define MSG_SIZE (1024 * VENDOR_NR_CPUS)
static char *sysfs_msg;
static ssize_t multi_load_read(struct file *file, struct kobject *kobj,
		struct bin_attribute *attr, char *buf,
		loff_t offset, size_t size)
{
	ssize_t msg_size, count = 0;
	int cpu;

	for_each_possible_cpu(cpu) {
		struct mlt_cpu *mlt_cpu = per_cpu_ptr(pcpu_mlt, cpu);
		struct mlt_part *part = &mlt_cpu->part;
		int idx, p, c, cur_period = part->cur_period;

		count += sprintf(sysfs_msg + count, "CPU%d :   Recent  |   Latest\n", cpu);
		count += sprintf(sysfs_msg + count, "time: %10d |", part->recent);

		for (c = 0; c < MLT_PERIOD_COUNT; c++) {
			p = part_move_period(cur_period, -c);
			count += sprintf(sysfs_msg + count, "%10d", part->periods[p]);
		}

		for (idx = 0; idx < MLT_UARCH_CPU_NUM; idx++) {
			struct mlt_uarch *uarch = &per_cpu_ptr(pcpu_mlt, cpu)->uarch[idx];
			count += sprintf(sysfs_msg + count, "\n%4s: %10u |",
					cpu_uarch_name[idx], uarch->recent);

			for (c = 0; c < MLT_PERIOD_COUNT; c++) {
				p = part_move_period(cur_period, -c);
				count += sprintf(sysfs_msg + count, "%10u", uarch->periods[p]);
			}
		}
		count += sprintf(sysfs_msg + count, "\n==================================================="
				"===================================\n");
	}

	msg_size = min_t(ssize_t, count, MSG_SIZE);
	msg_size = memory_read_from_buffer(buf, size, &offset, sysfs_msg, msg_size);

	return msg_size;
}
BIN_ATTR_RO(multi_load, 0);

static void mlt_init_tsk_uarch(struct mlt_uarch_raw *raw, struct mlt_uarch *uarch)
{
	memset(raw, 0, sizeof(struct mlt_uarch_raw) * MLT_RAW_TSK_NUM);
	memset(uarch, 0, sizeof(struct mlt_uarch) * MLT_UARCH_TSK_NUM);
}

static void mlt_init_runnable(struct mlt_runnable *runnable, u64 now)
{
	runnable->last_updated = now;
}

static void mlt_init_part_load(struct mlt_part *part)
{
	int i;

	for (i = 0; i < MLT_LOAD_PERIOD_COUNT; i++)
		part->load_history[i] = 0;
	part->hist_idx = 0;

	part->load_recent = 0;
	part->load_period_start = 0;

	part->last_load_updated = 0;
}

static void mlt_init_part(struct mlt_part *part, int cur_period,
				u64 period_start, u64 now)
{
	part->state = INACTIVE;
	part->cur_period = cur_period;
	part->active_period = cur_period;
	part->period_start = period_start;
	part->last_updated = now;

	mlt_init_part_load(part);
}

void mlt_init_task(struct task_struct *p)
{
	struct mlt_task *mlt_tsk= task_mlt(p);
	struct mlt_part *part;

	memset(mlt_tsk, 0, sizeof(struct mlt_task));
	part = &(per_cpu_ptr(pcpu_mlt, task_cpu(p))->part);

	mlt_init_part(&mlt_tsk->part, part->cur_period,
		part->period_start, part->last_updated);

	if (uarch_supported)
		mlt_init_tsk_uarch(mlt_tsk->uarch_raw, mlt_tsk->uarch);
}

static void mlt_init_cpu(struct mlt_cpu *mlt_cpu, int cpu, u64 now)
{
	mlt_cpu->cpu = cpu;
	mlt_init_part(&mlt_cpu->part, 0, ULLONG_MAX, now);
	mlt_init_runnable(&mlt_cpu->runnable, now);

	mlt_cpu->runtime_scale = SCHED_CAPACITY_SCALE;
}

#define DIV64_U64_ROUNDUP(X, Y) div64_u64((X) + (Y - 1), Y)
static void update_runtime_scale(struct rq *rq)
{
	int cpu = cpu_of(rq);
	struct mlt_cpu *mlt_cpu = per_cpu_ptr(pcpu_mlt, cpu);
	struct mlt_cluster *mcl;

	if (invalid_cluster_id(mlt_cpu->cluster))
		return;

	mcl = &mlt_clusters[mlt_cpu->cluster];

	mlt_cpu->runtime_scale = DIV64_U64_ROUNDUP(mcl->cur_freq *
			arch_scale_cpu_capacity(cpu), mcl->max_freq_org);
}

static u64 scale_runtime(struct rq *rq, u64 delta)
{
	struct mlt_cpu *mlt_cpu = per_cpu_ptr(pcpu_mlt, cpu_of(rq));

	delta = (delta * mlt_cpu->runtime_scale) >> SCHED_CAPACITY_SHIFT;

	return delta;
}

static void update_load_recent(struct rq *rq, struct mlt_part *part, u64 delta_ns, int type)
{
	u64 scaled_delta;

	if ((s64)delta_ns < 0)
		delta_ns = 0;

	scaled_delta = scale_runtime(rq, delta_ns);

	part->load_recent += scaled_delta;
	if (part->load_recent > MLT_LOAD_PERIOD_SIZE)
		part->load_recent = MLT_LOAD_PERIOD_SIZE;

	trace_mlt_update_load_recent(cpu_of(rq), scaled_delta, delta_ns,
			div64_u64(part->load_recent, MLT_LOAD_DIVISOR), part->load_recent, type);
}

static void update_load_history(struct mlt_part *part, u64 recent, u64 nr_periods)
{
	int i;
	u64 load;

	if (nr_periods > MLT_LOAD_PERIOD_COUNT)
		nr_periods = MLT_LOAD_PERIOD_COUNT;

	load = div64_u64(recent, MLT_LOAD_DIVISOR);
	for (i = 0; i < nr_periods; i++) {
		part->load_history[part->hist_idx] = load;
		part->hist_idx++;

		if (part->hist_idx >= MLT_LOAD_PERIOD_COUNT)
			part->hist_idx = 0;
	}

	part->load_recent = 0;
}

static bool need_update_load(struct rq *rq, struct mlt_part *part, int type, int flag)
{
	struct task_struct *p = part->task;

	if (type == MLT_TASK_LOAD_UPDATE) {
		if (get_sched_class(p) != EMS_SCHED_FAIR)
			return false;

		if (is_idle_task(p))
			return false;

		if (flag == UPDATE_TASK && !p->on_rq)
			return false;
	}

	if (type == MLT_CPU_LOAD_UPDATE) {
		/* we need to account sleep time */
		if (flag == PUT_PREV_TASK && is_idle_task(p))
			return true;

		/* we don't need to update load under sleep */
		if (is_idle_task(rq->curr))
			return false;
	}

	return true;
}

static bool wakeup_after_sleep(struct mlt_part *part, int type, int flag)
{
	struct task_struct *p = part->task;

	if (type == MLT_TASK_LOAD_UPDATE) {
		if (flag == WAKEUP_TASK)
			return true;
	}

	if (type == MLT_CPU_LOAD_UPDATE) {
		if (flag == PUT_PREV_TASK && is_idle_task(p))
			return true;
	}

	return false;
}

static void update_load(struct rq *rq, struct mlt_part *part, int type, u64 now, int flag)
{
	u64 last_updated = part->last_load_updated;
	u64 period_start = part->load_period_start;
	u64 period_end, nr_periods, delta_ns = 0;
	bool running = false;

	if (!need_update_load(rq, part, type, flag))
		return;

	running = !wakeup_after_sleep(part, type, flag);

	delta_ns = now - period_start;
	if ((s64)delta_ns < 0)
		delta_ns = 0;

	/* period is not fully elapsed */
	if (delta_ns < MLT_LOAD_PERIOD_SIZE) {
		if (running)
			update_load_recent(rq, part, now - last_updated, type);
		goto out;
	}

	/* 1) update first elapsed period */
	period_end = period_start + MLT_LOAD_PERIOD_SIZE;
	if (running)
		update_load_recent(rq, part, period_end - last_updated, type);
	update_load_history(part, part->load_recent, 1);

	/* 2) update fully elapsed period */
	period_start += MLT_LOAD_PERIOD_SIZE;
	nr_periods = div64_u64(now - period_start, MLT_LOAD_PERIOD_SIZE);
	if (nr_periods) {
		u64 recent = 0;

		if (running)
			recent = scale_runtime(rq, MLT_LOAD_PERIOD_SIZE);

		update_load_history(part, recent, nr_periods);
	}

	/* 3) update recent */
	period_start += nr_periods * MLT_LOAD_PERIOD_SIZE;
	if (running)
		update_load_recent(rq, part, now - period_start, type);

	part->load_period_start = period_start;
out:
	part->last_load_updated = now;

	trace_mlt_update_load(cpu_of(rq), part->task, delta_ns, running, type,
			mlt_load_flag_name[flag]);
}

void mlt_update_load(struct rq *rq, struct task_struct *p, u64 now, int flag)
{
	struct mlt_part *part = &task_mlt(p)->part;

	part->task = p;
	update_runtime_scale(rq);

	if (!part->load_period_start)
		part->load_period_start = now;

	if (!part->last_load_updated)
		part->last_load_updated = now;

	/* update task load */
	update_load(rq, part, MLT_TASK_LOAD_UPDATE, now, flag);
	trace_mlt_update_task_load(cpu_of(rq), p, part, mlt_load_flag_name[flag]);
}

void mlt_cpufreq_transition_notifier(int domain_id, unsigned int next_freq)
{
	struct cpufreq_policy *policy;
	struct mlt_cluster *mcl;
	int cpu;
	unsigned long flags;

	if (mlt_disabled)
		return;

	if (invalid_cluster_id(domain_id))
		return;

	mcl = &mlt_clusters[domain_id];
	policy = cpufreq_cpu_get_raw(cpumask_first(&mcl->cpus));
	if (!policy)
		return;

	if (mcl->cur_freq == next_freq)
		return;

	for_each_cpu(cpu, &mcl->cpus) {
		struct rq *rq = cpu_rq(cpu);

		raw_spin_lock_irqsave(&rq->__lock, flags);
		mlt_update_load(rq, rq->curr, sched_clock(), UPDATE_TASK);
		raw_spin_unlock_irqrestore(&rq->__lock, flags);
	}

	mcl->cur_freq = next_freq;
	mcl->max_freq = policy->max;
	mcl->max_freq_org = policy->cpuinfo.max_freq;
}
EXPORT_SYMBOL_GPL(mlt_cpufreq_transition_notifier);

unsigned long mlt_latest_task_load(struct task_struct *p)
{
	struct mlt_part *part = &task_mlt(p)->part;
	int idx = part->hist_idx - 1;

	if (idx < 0)
		idx = MLT_LOAD_PERIOD_COUNT - 1;

	return part->load_history[idx];
}

int mlt_init(struct kobject *ems_kobj, struct device_node *dn)
{
	int cpu;
	u64 now;
	struct task_struct *p;
	struct device_node *child;

	pcpu_mlt = alloc_percpu(struct mlt_cpu);
	if (!pcpu_mlt) {
		pr_err("failed to allocate mlt\n");
		return -ENOMEM;
	}

	if (of_property_read_bool(dn, "uarch-mlt") && !mlt_pmu_init())
		uarch_supported = true;

	now = sched_clock();
	for_each_possible_cpu(cpu) {
		struct mlt_cpu *mlt_cpu = per_cpu_ptr(pcpu_mlt, cpu);
		memset(mlt_cpu, 0, sizeof(struct mlt_cpu));
		mlt_init_cpu(mlt_cpu, cpu, now);
		ems_rq_nr_topapp(cpu_rq(cpu)) = 0;
	}

	dn = of_find_node_by_path("/ems/mlt");
	BUG_ON(!dn);

	for_each_child_of_node(dn, child) {
		int idx;
		const char *buf;
		struct mlt_cluster *mcl;

		BUG_ON(of_property_read_s32(child, "idx", &idx));
		BUG_ON(of_property_read_string(child, "cpus", &buf));
		BUG_ON(invalid_cluster_id(idx));

		mcl = &mlt_clusters[idx];
		mcl->idx = idx;
		cpulist_parse(buf, &mcl->cpus);

		for_each_cpu(cpu, &mcl->cpus) {
			struct mlt_cpu *mlt_cpu = per_cpu_ptr(pcpu_mlt, cpu);
			mlt_cpu->cluster = idx;
		}
	}

	rcu_read_lock();
	list_for_each_entry_rcu(p, &init_task.tasks, tasks) {
		get_task_struct(p);
		mlt_init_task(p);
		put_task_struct(p);
	}
	rcu_read_unlock();

	if (sysfs_create_bin_file(ems_kobj, &bin_attr_multi_load))
		pr_warn("failed to create multi_load\n");

	sysfs_msg = kcalloc(MSG_SIZE, sizeof(char), GFP_KERNEL);

	mlt_disabled = false;

	return 0;
}
