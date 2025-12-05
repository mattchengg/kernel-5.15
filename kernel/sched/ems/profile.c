/*
 * Scheduling status profiler for Exynos Mobile Scheduler
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd
 * Park Choonghoon <choong.park@samsung.com>
 */

#include "ems.h"

#include <trace/events/ems.h>
#include <trace/events/ems_debug.h>

static struct system_profile_data *system_profile_data;
static DEFINE_RWLOCK(profile_sched_lock);
#define get_cpu_profile(cpu, profile)	(&system_profile_data->cp[cpu][profile]);

char *fair_causes_name[END_OF_FAIR_CAUSES] = {
	"no-candidate",
	"only-one-candidate",
	"no-fit-cpu",
	"only-one-fit-cpu",
	"sysbusy",
	"task-express",
	"lowest-energy",
	"performance",
	"sync",
	"fast-track",
	"na"
};

struct sched_stat {
	unsigned int fair_sum;
	unsigned int rt_sum;
	unsigned int fair[END_OF_FAIR_CAUSES];
};
static DEFINE_PER_CPU(struct sched_stat, stats);

static unsigned int not_selected_fair;
static unsigned int not_selected_rt;

/*
 * Profile heavy task boost tunable parameter
 *
 * Start boost when operating (short_htask_ratio / 10)% of (window * 4 msec)
 * default : 90% of 8 msec
 */
#define DEFAULT_LONG_RATIO 700
#define DEFAULT_SHORT_RATIO 900
#define DEFAULT_WINDOW 2
struct htask_param {
	int long_heavy_task_ratio;	/* default : 70% */
	int short_heavy_task_ratio;	/* default : 90% */
	int htask_check_window_count;	/* default : 2	 */
};
static DEFINE_PER_CPU(struct htask_param, ht_params);

void update_fair_stat(int cpu, enum fair_causes i)
{
	struct sched_stat *stat;

	if (cpu < 0) {
		not_selected_fair++;
		return;
	}

	stat = per_cpu_ptr(&stats, cpu);

	if (!stat)
		return;

	stat->fair_sum++;
	stat->fair[i]++;
}

void update_rt_stat(int cpu)
{
	struct sched_stat *stat;

	if (cpu < 0) {
		not_selected_rt++;
		return;
	}

	stat = per_cpu_ptr(&stats, cpu);

	if (!stat)
		return;

	stat->rt_sum++;
}

static ssize_t sched_stat_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value, cpu;

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;

	for_each_possible_cpu(cpu) {
		struct sched_stat *stat = per_cpu_ptr(&stats, cpu);

		memset(stat, 0, sizeof(struct sched_stat));
	}

	return count;
}

static ssize_t sched_stat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sched_stat *stat;
	int ret = 0;
	int cpu, i;
	char *line = "--------------------------------------------------"
		"--------------------------------------------------"
		"---------------------";

	ret += sprintf(buf + ret, "%s\n", line);
	ret += sprintf(buf + ret, "%20s | ", "FAIR schedule");
	for_each_possible_cpu(cpu)
		ret += sprintf(buf + ret, "      CPU%d ", cpu);
	ret += sprintf(buf + ret, "\n%s\n", line);

	for (i = 0; i < END_OF_FAIR_CAUSES; i++) {
		ret += sprintf(buf + ret, "%20s | ", fair_causes_name[i]);
		for_each_possible_cpu(cpu) {
			stat = per_cpu_ptr(&stats, cpu);
			ret += sprintf(buf + ret, "%10u ", stat->fair[i]);
		}
		ret += sprintf(buf + ret, "\n");
	}

	ret += sprintf(buf + ret, "%s\n", line);
	ret += sprintf(buf + ret, "%20s | ", "total");
	for_each_possible_cpu(cpu) {
		stat = per_cpu_ptr(&stats, cpu);
		ret += sprintf(buf + ret, "%10u ", stat->fair_sum);
	}
	ret += sprintf(buf + ret, "\n%s\n", line);
	ret += sprintf(buf + ret, "not_selected: %u\n\n", not_selected_fair);

	ret += sprintf(buf + ret, "%s\n", line);
	ret += sprintf(buf + ret, "%20s | ", "RT schedule");
	for_each_possible_cpu(cpu)
		ret += sprintf(buf + ret, "      CPU%d ", cpu);

	ret += sprintf(buf + ret, "\n%s\n", line);
	ret += sprintf(buf + ret, "%20s | ", "total");
	for_each_possible_cpu(cpu) {
		stat = per_cpu_ptr(&stats, cpu);
		ret += sprintf(buf + ret, "%10u ", stat->rt_sum);
	}
	ret += sprintf(buf + ret, "\n%s\n", line);
	ret += sprintf(buf + ret, "not_selected: %u\n\n", not_selected_rt);

	return ret;
}
DEVICE_ATTR_RW(sched_stat);

/****************************************************************
 *		SYSBUSY base on Active Ratio			*
 ****************************************************************/
static bool is_longterm_htask(struct task_struct *p)
{
	int ratio, cpu = task_cpu(p);
	int grp = cpuctl_task_group_idx(p);
	struct htask_param *ht_param = per_cpu_ptr(&ht_params, cpu);
	bool htask_util, htask_ratio;

	if (grp != CGROUP_TOPAPP)
		return false;

	ratio = mlt_task_value(p, MLT_PERIOD_COUNT, AVERAGE);

	htask_ratio = ratio >= ht_param->long_heavy_task_ratio;
	htask_util = is_misfit_task_util(ml_task_util_est(p));

	return htask_ratio || htask_util;
}

/****************************************************************
 *		HEAVY TASK base on Active Ratio		*
 ****************************************************************/

/* heavy task should be included in TOP-APP or FOREGOUND */
static inline bool htask_enabled_grp(struct task_struct *p)
{
	int grp = cpuctl_task_group_idx(p);

	if (grp != CGROUP_TOPAPP && grp != CGROUP_FOREGROUND)
		return false;

	return true;
}

static bool is_heavy_task(struct task_struct *p)
{
	int ratio, cpu = task_cpu(p);
	struct htask_param *ht_param = per_cpu_ptr(&ht_params, cpu);

	if (!htask_enabled_grp(p))
		return false;

	ratio = mlt_task_value(p, ht_param->htask_check_window_count, AVERAGE);

	return ratio >= ht_param->short_heavy_task_ratio;
}

/* get_htask_ratio - if this task is heavy task, return heaviness ratio */
static int get_htask_ratio(struct task_struct *p)
{
	if (!is_heavy_task(p))
		return 0;

	return mlt_task_value(p, MLT_PERIOD_COUNT, AVERAGE);
}

static void profile_update_cpu_htask(int cpu, unsigned long hratio, unsigned long pid)
{
	struct cpu_profile *cs = get_cpu_profile(cpu, CPU_HTSK);

	cs->value = hratio;
	cs->data = pid;
}

static bool profile_has_htask(struct rq *rq, struct task_struct *skip)
{
	struct task_struct *p;
	int loop = 10;

	if (!rq->cfs.h_nr_running)
		return false;

	list_for_each_entry(p, &rq->cfs_tasks, se.group_node) {
		if (!loop)
			break;

		if (p == skip)
			continue;

		if (is_heavy_task(p)) {
			ems_rq_htask(rq) = (u64)p;
			return true;
		}

		loop--;
	}

	return false;
}

/*
 * profile_enqueue_task
 * check whether enqueued task is heavy or not, if it is true,
 * apply htask_ratio to this rq immediately to increase frequency latency
 */
void profile_enqueue_task(struct rq *rq, struct task_struct *p, int *flags)
{
	if (!is_heavy_task(p))
		return;

	ems_rq_htask(rq) = (u64)p;
	trace_profile_htask(cpu_of(rq), p, "enqueue");
	*flags |= HEAVY_TASK;
}

void profile_dequeue_task(struct rq *rq, struct task_struct *p, int *flags)
{
	struct task_struct *htask = (struct task_struct *)ems_rq_htask(rq);

	if (p != htask)
		return;

	ems_rq_htask(rq) = 0;
	trace_profile_htask(cpu_of(rq), p, "dequeue");

	if(profile_has_htask(rq, p)) {
		htask = (struct task_struct *)ems_rq_htask(rq);
		trace_profile_htask(cpu_of(rq), htask, "already-enqueued");
		*flags |= HEAVY_TASK;
	}
}

void profile_update_htask(struct rq *rq, int *flags)
{
	ems_rq_htask(rq) = 0;

	if (profile_has_htask(rq, NULL)) {
		struct task_struct *htask = (struct task_struct *)ems_rq_htask(rq);
		trace_profile_htask(cpu_of(rq), htask, "tick");
		*flags |= HEAVY_TASK;
	}
}

int profile_get_htask_ratio(int cpu)
{
	struct cpu_profile *cp = get_cpu_profile(cpu, CPU_HTSK);
	return cp->value;
}

int avg_nr_margin = MLT_RUNNABLE_ROUNDS_UNIT / 2;
static int get_avg_nr_heavy(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	int avg_nr;

	if (cpumask_test_cpu(cpu, cpu_slowest_mask()))
		return ems_rq_nr_misfited(rq);

	avg_nr = mlt_topapp_runnable(cpu);
	return (avg_nr + avg_nr_margin) / MLT_RUNNABLE_UNIT;
}

static ssize_t avg_nr_margin_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;

	avg_nr_margin = value;

	return count;
}

static ssize_t avg_nr_margin_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret += sprintf(buf + ret, "%d\n", avg_nr_margin);

	return ret;
}
DEVICE_ATTR_RW(avg_nr_margin);

/****************************************************************
 *			CPU UTIL				*
 ****************************************************************/
static void profile_update_cpu_util(int cpu, int *busy_cnt, unsigned long *util_sum)
{
	int cpu_util;
	struct cpu_profile *cs = get_cpu_profile(cpu, CPU_UTIL);

	cpu_util = ml_cpu_util(cpu) + cpu_util_rt(cpu_rq(cpu));
	(*util_sum) += cpu_util;

	if (check_busy(cpu_util, capacity_cpu(cpu))) {
		cs->data = PROFILE_CPU_BUSY;
		(*busy_cnt) += 1;
	} else {
		cs->data = PROFILE_CPU_IDLE;
	}

	cs->value = cpu_util;
}

/****************************************************************
 *			FPS					*
 ****************************************************************/
#define FPS_PROFILE_PERIOD_MS	99
#define FPS_WINDOW_CNT		10

struct ems_profiler {
	s64	profile_ms;

	int	idx;
	int	fps;

	u64	last_fence_cnt;
	s64	last_fence_ms;
	u64	cur_fps[FPS_WINDOW_CNT];

	ktime_t		start_time;
	ktime_t		end_time;

	void	(*get_fence_cnt)(u64 *cnt, ktime_t *time);
} profiler;

int profile_get_fps(void)
{
	return profiler.fps;
}

u64 ems_avg_val(u64 *data)
{
	int idx;
	u64 sum = 0;

	for (idx = 0; idx < FPS_WINDOW_CNT; idx++)
		sum += data[idx];

	return sum / FPS_WINDOW_CNT;
}

void ems_register_fence_cnt(void (*fn)(u64 *cnt, ktime_t *time))
{
	profiler.get_fence_cnt = fn;
}
EXPORT_SYMBOL_GPL(ems_register_fence_cnt);

#define MAX_VRR		120
static void update_fps_profiler(u64 now)
{
	ktime_t cur_time = 0, delta_ms;
	u64 cur_cnt;
	s64 cnt;
	int idx = profiler.idx;

	now = jiffies_to_msecs(now);

	if (ktime_sub(now, profiler.profile_ms) < FPS_PROFILE_PERIOD_MS)
		return;

	profiler.profile_ms = now;

	if (profiler.get_fence_cnt) {
		profiler.get_fence_cnt(&cur_cnt, &cur_time);
		cur_time = ktime_to_ms(cur_time);
		cnt = cur_cnt - profiler.last_fence_cnt;
		delta_ms = ktime_sub(cur_time, profiler.last_fence_ms);

		if (delta_ms > 0)
			profiler.cur_fps[idx] = cnt * MSEC_PER_SEC / delta_ms;
		else 
			profiler.cur_fps[idx] = 0;

		profiler.last_fence_ms = cur_time;
		profiler.last_fence_cnt = cur_cnt;
		profiler.fps = ems_avg_val(profiler.cur_fps);
	}

	if (++idx >= FPS_WINDOW_CNT)
		idx = 0;

	profiler.idx = idx;
}

/****************************************************************
 *			External APIs				*
 ****************************************************************/
static u64 last_profile_time;
static int profile_interval = 1; /* 1 tick = 4ms */
int profile_sched_data(void)
{
	unsigned long flags;
	unsigned long now = jiffies;
	unsigned long cpu_util_sum = 0;
	unsigned long heavy_task_util_sum = 0;
	int avg_nr_run_sum = 0;
	int avg_nr_heavy_sum = 0;
	int busy_cpu_count = 0;
	int heavy_task_count = 0;
	int cpu;

	if (!write_trylock_irqsave(&profile_sched_lock, flags))
		return -EBUSY;

	if (now < last_profile_time + profile_interval)
		goto unlock;

	last_profile_time = now;

	update_fps_profiler(now);

	for_each_cpu(cpu, cpu_active_mask) {
		struct rq *rq = cpu_rq(cpu);
		struct ems_dslt_rq *drq = ems_rq_dslt(rq);
		struct task_struct *p;
		unsigned long task_util;
		u64 max_hratio = 0, max_hratio_pid = 0;
		int track_count;
		int avg_nr_heavy;

		profile_update_cpu_util(cpu, &busy_cpu_count, &cpu_util_sum);

		raw_spin_rq_lock(rq);

		/* Explictly clear count */
		track_count = 0;
		avg_nr_run_sum += mlt_runnable(rq);

		avg_nr_heavy = get_avg_nr_heavy(cpu);
		avg_nr_heavy_sum += avg_nr_heavy;

		list_for_each_entry(p, &rq->cfs_tasks, se.group_node) {
			/* update heavy task base on active ratio */
			u64 task_hratio = get_htask_ratio(p);
			if (task_hratio > max_hratio) {
				max_hratio_pid = p->pid;
				max_hratio = task_hratio;
			}

			/* update heavy task base on cpu util */
			task_util = ml_task_util(p);
			if (is_longterm_htask(p)) {
				heavy_task_util_sum += task_util;
				heavy_task_count++;
			}

			if (++track_count >= TRACK_TASK_COUNT)
				break;
		}

		/* save heaviest task data */
		profile_update_cpu_htask(cpu, max_hratio, max_hratio_pid);

		trace_ems_cpu_profile(cpu, &system_profile_data->cp[cpu][0],
					rq->cfs.h_nr_running,
					rq->cfs.avg.runnable_avg,
					drq->avg.runnable_avg,
					avg_nr_heavy);

		raw_spin_rq_unlock(rq);
	}

	avg_nr_run_sum = (avg_nr_run_sum + MLT_RUNNABLE_UP_UNIT) / MLT_RUNNABLE_UNIT;;
	trace_ems_profile_tasks(busy_cpu_count, cpu_util_sum,
				heavy_task_count, heavy_task_util_sum,
				avg_nr_run_sum, avg_nr_heavy_sum);

	/* Fill profile data */
	system_profile_data->busy_cpu_count = busy_cpu_count;
	system_profile_data->heavy_task_count = heavy_task_count;
	system_profile_data->cpu_util_sum = cpu_util_sum;
	system_profile_data->avg_nr_run_sum = avg_nr_run_sum;
	system_profile_data->avg_nr_heavy_sum = avg_nr_heavy_sum;
	system_profile_data->heavy_task_util_sum = heavy_task_util_sum;

unlock:
	write_unlock_irqrestore(&profile_sched_lock, flags);

	return 0;
}

/* Caller MUST disable irq before calling this function. */
void get_system_sched_data(struct system_profile_data *data)
{
	read_lock(&profile_sched_lock);
	memcpy(data, system_profile_data, sizeof(struct system_profile_data));
	read_unlock(&profile_sched_lock);
}

/****************************************************************
 *		  sysbusy state change notifier			*
 ****************************************************************/
static int profile_sysbusy_notifier_call(struct notifier_block *nb,
					unsigned long val, void *v)
{
	enum sysbusy_state state = *(enum sysbusy_state *)v;

	if (val != SYSBUSY_STATE_CHANGE)
		return NOTIFY_OK;

	profile_interval = sysbusy_monitor_interval(state);

	return NOTIFY_OK;
}

static struct notifier_block profile_sysbusy_notifier = {
	.notifier_call = profile_sysbusy_notifier_call,
};

/****************************************************************
 *		  emstune htask change notifier			*
 ****************************************************************/
static int htask_emstune_notifier_call(struct notifier_block *nb,
		unsigned long val, void *v)
{
	struct emstune_set *cur_set = (struct emstune_set *)v;
	int cpu;

	for_each_possible_cpu(cpu) {
		struct htask_param *ht_param = per_cpu_ptr(&ht_params, cpu);

		ht_param->short_heavy_task_ratio = cur_set->cpufreq_gov.htask_ratio_threshold[cpu];
		ht_param->htask_check_window_count = cur_set->cpufreq_gov.window_count[cpu];
	}

	return NOTIFY_OK;
}

static struct notifier_block htask_emstune_notifier = {
	.notifier_call = htask_emstune_notifier_call,
};

/****************************************************************
 *			Initialization				*
 ****************************************************************/
static void htask_emstune_init(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		struct htask_param *ht_param = per_cpu_ptr(&ht_params, cpu);

		ht_param->long_heavy_task_ratio = DEFAULT_LONG_RATIO;
		ht_param->short_heavy_task_ratio = DEFAULT_SHORT_RATIO;
		ht_param->htask_check_window_count = DEFAULT_WINDOW;
	}

	emstune_register_notifier(&htask_emstune_notifier);
}

int profile_sched_init(struct kobject *ems_kobj)
{
	system_profile_data =
		kzalloc(sizeof(struct system_profile_data), GFP_KERNEL);
	if (!system_profile_data) {
		pr_err("Failed to allocate profile_system_data\n");
		return -ENOMEM;
	}

	sysbusy_register_notifier(&profile_sysbusy_notifier);
	htask_emstune_init();

	if (sysfs_create_file(ems_kobj, &dev_attr_sched_stat.attr))
		pr_warn("failed to create sched_stat\n");

	if (sysfs_create_file(ems_kobj, &dev_attr_avg_nr_margin.attr))
		pr_warn("failed to create avg_nr_margin\n");

	return 0;
}
