#include <trace/events/ems_debug.h>
#include "ems.h"
#include "dslt.h"

#define DEFAULT_RUNNABLE_SENSITIVITY 0
#define DEFAULT_RUNNING_SENSITIVITY	100

struct kobject *dslt_kobj;
int cgrp_runnable_sensitivity[CGROUP_COUNT];
int cgrp_running_sensitivity[CGROUP_COUNT];

struct kobject *util_est_kobj;
/*
 * Approximate:
 *   val * y^n,    where y^32 ~= 0.5 (~1 scheduling period)
 */
static u64 decay_load(u64 val, u64 n)
{
	unsigned int local_n;

	if (unlikely(n > LOAD_AVG_PERIOD * 63))
		return 0;

	/* after bounds checking we can collapse to 32-bit */
	local_n = n;

	/*
	 * As y^PERIOD = 1/2, we can combine
	 *    y^n = 1/2^(n/PERIOD) * y^(n%PERIOD)
	 * With a look-up table which covers y^n (n<PERIOD)
	 *
	 * To achieve constant time decay_load.
	 */
	if (unlikely(local_n >= LOAD_AVG_PERIOD)) {
		val >>= local_n / LOAD_AVG_PERIOD;
		local_n %= LOAD_AVG_PERIOD;
	}

	val = mul_u64_u32_shr(val, runnable_avg_yN_inv[local_n], 32);
	return val;
}

static u32 __accumulate_pelt_segments(u64 periods, u32 d1, u32 d3)
{
	u32 c1, c2, c3 = d3; /* y^0 == 1 */

	/*
	 * c1 = d1 y^p
	 */
	c1 = decay_load((u64)d1, periods);

	/*
	 *            p-1
	 * c2 = 1024 \Sum y^n
	 *            n=1
	 *
	 *              inf        inf
	 *    = 1024 ( \Sum y^n - \Sum y^n - y^0 )
	 *              n=0        n=p
	 */
	c2 = LOAD_AVG_MAX - decay_load(LOAD_AVG_MAX, periods) - 1024;

	return c1 + c2 + c3;
}

/*
 * Accumulate the three separate parts of the sum; d1 the remainder
 * of the last (incomplete) period, d2 the span of full periods and d3
 * the remainder of the (incomplete) current period.
 *
 *           d1          d2           d3
 *           ^           ^            ^
 *           |           |            |
 *         |<->|<----------------->|<--->|
 * ... |---x---|------| ... |------|-----x (now)
 *
 *                           p-1
 * u' = (u + d1) y^p + 1024 \Sum y^n + d3 y^0
 *                           n=1
 *
 *    = u y^p +					(Step 1)
 *
 *                     p-1
 *      d1 y^p + 1024 \Sum y^n + d3 y^0		(Step 2)
 *                     n=1
 */
static __always_inline u32
accumulate_sum(u64 delta, struct dslt_avg *sa,
	       unsigned long load, unsigned long sensitivity)
{
	u32 contrib = (u32)delta; /* p == 0 -> delta < 1024 */
	u64 periods;

	delta += sa->period_contrib;
	periods = delta / 1024; /* A period is 1024us (~1ms) */

	/*
	 * Step 1: decay old *_sum if we crossed period boundaries.
	 */
	if (periods) {
		sa->runnable_sum =
			decay_load(sa->runnable_sum, periods);

		/*
		 * Step 2
		 */
		delta %= 1024;
		if (load) {
			/*
			 * This relies on the:
			 *
			 * if (!load)
			 *	sensitivity = running = 0;
			 *
			 * clause from ___update_load_sum(); this results in
			 * the below usage of @contrib to disappear entirely,
			 * so no point in calculating it.
			 */
			contrib = __accumulate_pelt_segments(periods,
					1024 - sa->period_contrib, delta);
		}
	}
	sa->period_contrib = delta;

	if (sensitivity)
		sa->runnable_sum += ((sensitivity * contrib << SCHED_CAPACITY_SHIFT) / 100);

	return periods;
}

/*
 * We can represent the historical contribution to runnable average as the
 * coefficients of a geometric series.  To do this we sub-divide our runnable
 * history into segments of approximately 1ms (1024us); label the segment that
 * occurred N-ms ago p_N, with p_0 corresponding to the current period, e.g.
 *
 * [<- 1024us ->|<- 1024us ->|<- 1024us ->| ...
 *      p0            p1           p2
 *     (now)       (~1ms ago)  (~2ms ago)
 *
 * Let u_i denote the fraction of p_i that the entity was runnable.
 *
 * We then designate the fractions u_i as our co-efficients, yielding the
 * following representation of historical load:
 *   u_0 + u_1*y + u_2*y^2 + u_3*y^3 + ...
 *
 * We choose y based on the with of a reasonably scheduling period, fixing:
 *   y^32 = 0.5
 *
 * This means that the contribution to load ~32ms ago (u_32) will be weighted
 * approximately half as much as the contribution to load within the last ms
 * (u_0).
 *
 * When a period "rolls over" and we have new u_0`, multiplying the previous
 * sum again by y is sufficient to update:
 *   load_avg = u_0` + y*(u_0 + u_1*y + u_2*y^2 + ... )
 *            = u_0 + u_1*y + u_2*y^2 + ... [re-labeling u_i --> u_{i+1}]
 */
static int
___update_load_sum(u64 now, struct dslt_avg *sa,
		  unsigned long load, unsigned long sensitivity)
{
	u64 delta;

	delta = now - sa->last_update_time;
	/*
	 * This should only happen when time goes backwards, which it
	 * unfortunately does during sched clock init when we swap over to TSC.
	 */
	if ((s64)delta < 0) {
		sa->last_update_time = now;
		return 0;
	}

	/*
	 * Use 1024ns as the unit of measurement since it's a reasonable
	 * approximation of 1us and fast to compute.
	 */
	delta >>= 10;
	if (!delta)
		return 0;

	sa->last_update_time += delta << 10;

	/*
	 * running is a subset of sensitivity(weight) so running can't be set if
	 * sensitivity is clear. But there are some corner cases where the current
	 * se has been already dequeued but cfs_rq->curr still points to it.
	 * This means that weight will be 0 but not running for a sched_entity
	 * but also for a cfs_rq if the latter becomes idle. As an example,
	 * this happens during idle_balance() which calls
	 * update_blocked_averages().
	 *
	 * Also see the comment in accumulate_sum().
	 */
	if (!load)
		sensitivity = 0;

	/*
	 * Now we know we crossed measurement unit boundaries. The *_avg
	 * accrues by two steps:
	 *
	 * Step 1: accumulate *_sum since last_update_time. If we haven't
	 * crossed period boundaries, finish.
	 */
	if (!accumulate_sum(delta, sa, load, sensitivity))
		return 0;

	return 1;
}

/*
 * When syncing *_avg with *_sum, we must take into account the current
 * position in the PELT segment otherwise the remaining part of the segment
 * will be considered as idle time whereas it's not yet elapsed and this will
 * generate unwanted oscillation in the range [1002..1024[.
 *
 / The max value of *_sum varies with the position in the time segment and is
 * equals to :
 *
 *   LOAD_AVG_MAX*y + sa->period_contrib
 *
 * which can be simplified into:
 *
 *   LOAD_AVG_MAX - 1024 + sa->period_contrib
 *
 * because LOAD_AVG_MAX*y == LOAD_AVG_MAX-1024
 *
 * The same care must be taken when a sched entity is added, updated or
 * removed from a cfs_rq and we need to update dslt_avg. Scheduler entities
 * and the cfs rq, to which they are attached, have the same position in the
 * time segment because they use the same clock. This means that we can use
 * the period_contrib of cfs_rq when updating the dslt_avg of a sched_entity
 * if it's more convenient.
 */
static void
__dslt_update_load_avg(struct dslt_avg *sa)

{
	u32 divider = get_dslt_divider(sa);

	/*
	 * Step 2: update *_avg.
	 */
	sa->runnable_avg = div_u64(sa->runnable_sum, divider);
}

/*
 * sched_entity:
 *
 *   task:
 *     se_weight()   = se->load.weight
 *     se_runnable() = !!on_rq
 *
 *   group: [ see update_cfs_group() ]
 *     se_weight()   = tg->weight * grq->load_avg / tg->load_avg
 *     se_runnable() = grq->h_nr_running
 *
 *   runnable_sum = se_runnable() * runnable = grq->runnable_sum
 *   runnable_avg = runnable_sum
 *
 *   load_sum := runnable
 *   load_avg = se_weight(se) * load_sum
 *
 * cfq_rq:
 *
 *   runnable_sum = \Sum se->avg.runnable_sum
 *   runnable_avg = \Sum se->avg.runnable_avg
 *
 *   load_sum = \Sum se_weight(se) * se->avg.load_sum
 *   load_avg = \Sum se->avg.load_avg
 */

static inline long se_sensitivity(struct sched_entity *se, bool running, bool runnable)
{
	struct ems_dslt_task *dtsk = ems_dslt_task(task_of(se));
	struct dslt_avg *da = &dtsk->avg;

	if (!unlikely(entity_is_task(se)))
		return 0;

	if (running)
		return da->running_sensitivity;
	else if (runnable)
		return da->runnable_sensitivity;
	else
		return 0;
}

int dslt_update_load_avg_blocked_se(u64 now, struct sched_entity *se)
{
	struct ems_dslt_task *dslt_tsk = ems_dslt_task(task_of(se));

	if (___update_load_sum(now, &dslt_tsk->avg, 0, 0)) {
		__dslt_update_load_avg(&dslt_tsk->avg);
		trace_dslt_se(se, dslt_tsk);
		return 1;
	}

	return 0;
}

int dslt_update_load_avg_se(u64 now, struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct ems_dslt_task *dslt_tsk = ems_dslt_task(task_of(se));

	if (___update_load_sum(now, &dslt_tsk->avg, !!se->on_rq,
			se_sensitivity(se, cfs_rq->curr == se, se->on_rq))) {

		__dslt_update_load_avg(&dslt_tsk->avg);
		cfs_se_util_change(&dslt_tsk->avg);
		trace_dslt_se(se, dslt_tsk);
		return 1;
	}

	return 0;
}

int dslt_update_load_avg_cfs_rq(u64 now, struct cfs_rq *cfs_rq)
{
	struct ems_dslt_rq *dslt_rq = ems_rq_dslt(rq_of(cfs_rq));

	if (___update_load_sum(now, &dslt_rq->avg,
				scale_load_down(cfs_rq->load.weight),
				dslt_rq->sensitivity)) {

		__dslt_update_load_avg(&dslt_rq->avg);
		trace_dslt_cfs(cfs_rq, dslt_rq);
		return 1;
	}

	return 0;
}

/*
 * rt_rq:
 *
 *   util_sum = \Sum se->avg.util_sum but se->avg.util_sum is not tracked
 *   util_sum = cpu_scale * load_sum
 *   runnable_sum = util_sum
 *
 *   load_avg and runnable_avg are not supported and meaningless.
 *
 */
/*
int update_rt_rq_load_avg(u64 now, struct rq *rq, int running)
{
	if (___update_load_sum(now, &rq->avg_rt,
				running,
				running,
				running)) {

		___update_load_avg(&rq->avg_rt, 1);
		trace_pelt_rt_tp(rq);
		return 1;
	}

	return 0;
}
*/
/*
 * dl_rq:
 *
 *   util_sum = \Sum se->avg.util_sum but se->avg.util_sum is not tracked
 *   util_sum = cpu_scale * load_sum
 *   runnable_sum = util_sum
 *
 *   load_avg and runnable_avg are not supported and meaningless.
 *
 */
/*
int update_dl_rq_load_avg(u64 now, struct rq *rq, int running)
{
	if (___update_load_sum(now, &rq->avg_dl,
				running,
				running,
				running)) {

		___update_load_avg(&rq->avg_dl, 1);
		trace_pelt_dl_tp(rq);
		return 1;
	}

	return 0;
}
*/

static u64 dslt_rq_last_update_time(struct ems_dslt_rq *dslt_rq)
{
	return u64_u32_load_copy(dslt_rq->avg.last_update_time,
				 dslt_rq->last_update_time_copy);
}

/**
 * update_cfs_rq_load_avg - update the cfs_rq's load/util averages
 * @now: current time, as per cfs_rq_clock_pelt()
 * @cfs_rq: cfs_rq to update
 *
 * The cfs_rq avg is the direct sum of all its entities (blocked and runnable)
 * avg. The immediate corollary is that all (fair) tasks must be attached.
 *
 * cfs_rq->avg is used for task_h_load() and update_cfs_share() for example.
 *
 * Return: true if the load decayed or we removed load.
 *
 * Since both these conditions indicate a changed cfs_rq->avg.load we should
 * call update_tg_load_avg() when this function returns true.
 */
static inline int
dslt_update_cfs_rq_load_avg(u64 now, struct cfs_rq *cfs_rq)
{
	unsigned long removed_runnable = 0;
	struct ems_dslt_rq *dslt_rq = ems_rq_dslt(rq_of(cfs_rq));
	struct dslt_avg *sa = &dslt_rq->avg;
	int decayed = 0;

	if (dslt_rq->removed.nr) {
		unsigned long r;
		u32 divider = get_dslt_divider(&dslt_rq->avg);

		raw_spin_lock(&dslt_rq->removed.lock);
		swap(dslt_rq->removed.runnable_avg, removed_runnable);
		dslt_rq->removed.nr = 0;
		raw_spin_unlock(&dslt_rq->removed.lock);

		r = removed_runnable;
		sub_positive(&sa->runnable_avg, r);
		sub_positive(&sa->runnable_sum, r * divider);
		sa->runnable_sum = max_t(u32, sa->runnable_sum,
					      sa->runnable_avg * DSLT_MIN_DIVIDER);

		decayed = 1;
	}

	decayed |= dslt_update_load_avg_cfs_rq(now, cfs_rq);
	u64_u32_store_copy(sa->last_update_time,
			   dslt_rq->last_update_time_copy,
			   sa->last_update_time);

	return decayed;
}

static inline unsigned long dslt_task_util_est(struct ems_dslt_task *dslt_tsk)
{
	return READ_ONCE(dslt_tsk->avg.util_est) & ~UTIL_AVG_UNCHANGED;
}

static inline bool load_avg_is_decayed(struct sched_entity *se, struct cfs_rq *cfs_rq)
{
	struct ems_dslt_task *dslt_tsk = ems_dslt_task(task_of(se));

	if (dslt_tsk->avg.runnable_sum)
		return false;

	/*
	 * _avg must be null when _sum are null because _avg = _sum / divider
	 * Make sure that rounding and/or propagation of PELT values never
	 * break this.
	 */

	SCHED_WARN_ON(dslt_tsk->avg.runnable_avg);

	return true;
}

void dslt_sync_entity_load_avg(struct ems_dslt_rq *dslt_rq, struct sched_entity *se)
{
	u64 last_update_time;

	last_update_time = dslt_rq_last_update_time(dslt_rq);
	dslt_update_load_avg_blocked_se(last_update_time, se);
}

void dslt_update_blocked_fair(struct rq *rq)
{
	struct cfs_rq *cfs_rq = &rq->cfs;

	dslt_update_cfs_rq_load_avg(cfs_rq_clock_pelt(cfs_rq), cfs_rq);
}

void dslt_set_task_rq_fair(struct sched_entity *se,
			   struct cfs_rq *prev, struct cfs_rq *next)
{
	struct ems_dslt_task *dslt_tsk = ems_dslt_task(task_of(se));
	struct ems_dslt_rq * dslt_rq_p = ems_rq_dslt(rq_of(prev));
	struct ems_dslt_rq * dslt_rq_n = ems_rq_dslt(rq_of(next));
	u64 p_last_update_time;
	u64 n_last_update_time;

	/*
	 * We are supposed to update the task to "current" time, then its up to
	 * date and ready to go to new CPU/cfs_rq. But we have difficulty in
	 * getting what current time is, so simply throw away the out-of-date
	 * time. This will result in the wakee task is less decayed, but giving
	 * the wakee more load sounds not bad.
	 */
	if (!(dslt_tsk->avg.last_update_time && prev))
		return;

	p_last_update_time = dslt_rq_last_update_time(dslt_rq_p);
	n_last_update_time = dslt_rq_last_update_time(dslt_rq_n);

	dslt_update_load_avg_blocked_se(p_last_update_time, se);
	dslt_tsk->avg.last_update_time = n_last_update_time;
}

void dslt_migrate_se_dslt_lag(struct sched_entity *se)
{
	u64 throttled = 0, now, lut;
	struct cfs_rq *cfs_rq;
	struct rq *rq;
	struct ems_dslt_task *dslt_tsk;
	struct ems_dslt_rq *dslt_rq;
	bool is_idle;

	dslt_tsk = ems_dslt_task(task_of(se));

	cfs_rq = cfs_rq_of(se);
	rq = rq_of(cfs_rq);
	dslt_rq = ems_rq_dslt(rq);

	if (load_avg_is_decayed(se, cfs_rq))
		return;

	rcu_read_lock();
	is_idle = is_idle_task(rcu_dereference(rq->curr));
	rcu_read_unlock();

	/*
	 * The lag estimation comes with a cost we don't want to pay all the
	 * time. Hence, limiting to the case where the source CPU is idle and
	 * we know we are at the greatest risk to have an outdated clock.
	 */
	if (!is_idle)
		return;

	/*
	 * Estimated "now" is: last_update_time + cfs_idle_lag + rq_idle_lag, where:
	 *
	 *   last_update_time (the cfs_rq's last_update_time)
	 *	= cfs_rq_clock_pelt()@cfs_rq_idle
	 *      = rq_clock_pelt()@cfs_rq_idle
	 *        - cfs->throttled_clock_pelt_time@cfs_rq_idle
	 *
	 *   cfs_idle_lag (delta between rq's update and cfs_rq's update)
	 *      = rq_clock_pelt()@rq_idle - rq_clock_pelt()@cfs_rq_idle
	 *
	 *   rq_idle_lag (delta between now and rq's update)
	 *      = sched_clock_cpu() - rq_clock()@rq_idle
	 *
	 * We can then write:
	 *
	 *    now = rq_clock_pelt()@rq_idle - cfs->throttled_clock_pelt_time +
	 *          sched_clock_cpu() - rq_clock()@rq_idle
	 * Where:
	 *      rq_clock_pelt()@rq_idle is rq->clock_pelt_idle
	 *      rq_clock()@rq_idle      is rq->clock_idle
	 *      cfs->throttled_clock_pelt_time@cfs_rq_idle
	 *                              is cfs_rq->throttled_pelt_idle
	 */

#ifdef CONFIG_CFS_BANDWIDTH
	throttled = u64_u32_load(cfs_rq->throttled_pelt_idle);
	/* The clock has been stopped for throttling */
	if (throttled == U64_MAX)
		return;
#endif
	now = u64_u32_load(rq->clock_pelt_idle);
	/*
	 * Paired with _update_idle_rq_clock_pelt(). It ensures at the worst case
	 * is observed the old clock_pelt_idle value and the new clock_idle,
	 * which lead to an underestimation. The opposite would lead to an
	 * overestimation.
	 */
	smp_rmb();
	lut = dslt_rq_last_update_time(dslt_rq);

	now -= throttled;
	if (now < lut)
		/*
		 * cfs_rq->avg.last_update_time is more recent than our
		 * estimation, let's use it.
		 */
		now = lut;
	else
		now += sched_clock() - u64_u32_load(rq->clock_idle);

	dslt_update_load_avg_blocked_se(now, se);
}

/*
 * Task first catches up with cfs_rq, and then subtract
 * itself from the cfs_rq (task must be off the queue now).
 */
void dslt_remove_entity_load_avg(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct ems_dslt_rq *dslt_rq;
	struct ems_dslt_task *dslt_tsk;
        unsigned long flags;

	if (!entity_is_task(se))
		return;

	dslt_rq = ems_rq_dslt(rq_of(cfs_rq));
	dslt_tsk = ems_dslt_task(task_of(se));

	/*
         * tasks cannot exit without having gone through wake_up_new_task() ->
         * enqueue_task_fair() which will have added things to the cfs_rq,
         * so we can remove unconditionally.
         */

        dslt_sync_entity_load_avg(dslt_rq, se);

        raw_spin_lock_irqsave(&dslt_rq->removed.lock, flags);
        ++dslt_rq->removed.nr;
        dslt_rq->removed.runnable_avg    += dslt_tsk->avg.runnable_avg;
        raw_spin_unlock_irqrestore(&dslt_rq->removed.lock, flags);
}

void dslt_detach_entity_load_avg(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct ems_dslt_rq *dslt_rq;
	struct ems_dslt_task *dslt_tsk;

	if (!entity_is_task(se))
		return;

	dslt_rq = ems_rq_dslt(rq_of(cfs_rq));
	dslt_tsk = ems_dslt_task(task_of(se));

	sub_positive(&dslt_rq->avg.runnable_avg, dslt_tsk->avg.runnable_avg);
	sub_positive(&dslt_rq->avg.runnable_sum, dslt_tsk->avg.runnable_sum);
	/* See update_cfs_rq_load_avg() */
	dslt_rq->avg.runnable_sum = max_t(u32, dslt_rq->avg.runnable_sum,
                                      dslt_rq->avg.runnable_avg * DSLT_MIN_DIVIDER);
}

void dslt_attach_entity_load_avg(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct ems_dslt_rq *dslt_rq;
	struct ems_dslt_task *dslt_tsk;
	u32 divider;

	if (!entity_is_task(se))
		return;

	dslt_rq = ems_rq_dslt(rq_of(cfs_rq));
	dslt_tsk = ems_dslt_task(task_of(se));

	/*
	 * cfs_rq->avg.period_contrib can be used for both cfs_rq and se.
	 * See ___update_load_avg() for details.
	 */
	divider = get_dslt_divider(&dslt_rq->avg);

	/*
	 * When we attach the @se to the @cfs_rq, we must align the decay
	 * window because without that, really weird and wonderful things can
	 * happen.
	*/

	dslt_tsk->avg.last_update_time = dslt_rq->avg.last_update_time;
	dslt_tsk->avg.period_contrib = dslt_rq->avg.period_contrib;

	/*
	 * Hell(o) Nasty stuff.. we need to recompute _sum based on the new
	 * period_contrib. This isn't strictly correct, but since we're
	 * entirely outside of the PELT hierarchy, nobody cares if we truncate
	 * _sum a little.
	 */
	dslt_tsk->avg.runnable_sum = dslt_tsk->avg.runnable_avg * divider;

	dslt_rq->avg.runnable_avg += dslt_tsk->avg.runnable_avg;
	dslt_rq->avg.runnable_sum += dslt_tsk->avg.runnable_sum;
}

void dslt_update_load_avg(u64 now, struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct ems_dslt_rq *dslt_rq;
	struct ems_dslt_task *dslt_tsk;
	int decayed;

	if (!entity_is_task(se))
		return;

	dslt_rq = ems_rq_dslt(rq_of(cfs_rq));
	dslt_tsk = ems_dslt_task(task_of(se));

	if (dslt_tsk->avg.last_update_time)
		dslt_update_load_avg_se(now, cfs_rq, se);

	decayed = dslt_update_cfs_rq_load_avg(now, cfs_rq);
}

unsigned int dslt_util_est_enabled(struct task_struct *p)
{
	int cgroup_idx = cpuctl_task_group_idx(p);

	return util_est.enabled[cgroup_idx];
}

int dslt_util_est_min_weight_shift(void)
{
	return util_est.min_weight_shift;
}

/*
 * Check if a (signed) value is within a specified (unsigned) margin,
 * based on the observation that:
 *
 *     abs(x) < y := (unsigned)(x + y - 1) < (2 * y - 1)
 *
 * NOTE: this only works when value + margin < INT_MAX.
 */
static inline bool within_margin(int value, int margin)
{
	return ((unsigned int)(value + margin - 1) < (2 * margin - 1));
}

void dslt_util_est_update(struct cfs_rq *cfs_rq,
					struct task_struct *p, bool task_sleep)
{
	unsigned long last_ewma_diff = 0;
	struct ems_dslt_task *dslt_tsk = ems_dslt_task(p);
	unsigned long max_capacity = capacity_orig_of((cpu_of(rq_of(cfs_rq))));
	unsigned int enabled = dslt_util_est_enabled(p);
	unsigned int last_ewma = 0;
	unsigned long div_factor = max_capacity >> util_est.div_factor;
	unsigned long remain = 0;
	int shift = DEFAULT_UTIL_EST_WEIGHT_SHIFT;
	int min_weight_shift = dslt_util_est_min_weight_shift();
	unsigned int ewma, dequeued;

	if (!sched_feat(UTIL_EST))
		return;

	/*
	 * If the shift value was zero via util_est tunable knob,
	 * skip and reset the util_est.
	 */
	if (!enabled) {
		dslt_tsk->avg.util_est = 0;
		return;
	}

	/*
	 * Skip update of task's estimated utilization when the task has not
	 * yet completed an activation, e.g. being migrated.
	 */
	if (!task_sleep)
		return;

	/* Get current estimate of utilization */
	ewma = READ_ONCE(dslt_tsk->avg.util_est);

	/*
	 * If the PELT values haven't changed since enqueue time,
	 * skip the util_est update.
	 */
	if (ewma & UTIL_AVG_UNCHANGED)
		return;

	last_ewma = ewma;

	/* Get utilization at dequeue */
	dequeued = READ_ONCE(dslt_tsk->avg.runnable_avg);

	/*
	 * Reset EWMA on utilization increases, the moving average is used only
	 * to smooth utilization decreases.
	 */
	if (ewma <= dequeued) {
		ewma = dequeued;
		shift = -1;
		goto done;
	}

	/*
	 * Skip update of task's estimated utilization when its members are
	 * already ~1% close to its last activation value.
	 */
	last_ewma_diff = ewma - dequeued;
	if (last_ewma_diff < UTIL_EST_MARGIN) {
		shift = -1;
		goto done;
	}

	/*
	 * To avoid overestimation of actual task utilization, skip updates if
	 * we cannot grant there is idle time in this CPU.
	 */
	if (dequeued > max_capacity)
		return;

	/*
	 * Update Task's estimated utilization
	 *
	 * When *p completes an activation we can consolidate another sample
	 * of the task size. This is done by storing the current PELT value
	 * as ue.enqueued and by using this value to update the Exponential
	 * Weighted Moving Average (EWMA):
	 *
	 *  ewma(t) = w *  task_util(p) + (1-w) * ewma(t-1)
	 *          = w *  task_util(p) +         ewma(t-1)  - w * ewma(t-1)
	 *          = w * (task_util(p) -         ewma(t-1)) +     ewma(t-1)
	 *          = w * (      last_ewma_diff            ) +     ewma(t-1)
	 *          = w * (last_ewma_diff  +  ewma(t-1) / w)
	 *
	 * Where 'w' is the weight of new samples, which is configured to be
	 * 0.25, thus making w=1/4 ( >>= UTIL_EST_WEIGHT_SHIFT)
	 */

	remain = max_capacity - min(last_ewma_diff, max_capacity);

	if (remain < div_factor && !min_weight_shift) {
		shift = 0;
		goto skip_calc;
	}

	shift = remain / div_factor;
	shift = clamp(shift, min_weight_shift, MAX_UTIL_EST_WEIGHT_SHIFT);

skip_calc:
	ewma <<= shift;
	ewma  -= last_ewma_diff;
	ewma >>= shift;
done:
	ewma |= UTIL_AVG_UNCHANGED;
	WRITE_ONCE(dslt_tsk->avg.util_est, ewma);

	trace_dslt_util_est_update(p, last_ewma, ewma & ~UTIL_AVG_UNCHANGED, dequeued, shift);
}

void dslt_util_est_dequeue(struct rq *rq, struct task_struct *p)
{
	struct ems_dslt_rq *dslt_rq = ems_rq_dslt(rq);
	struct ems_dslt_task *dslt_tsk = ems_dslt_task(p);
        unsigned int enqueued;

	if (get_sched_class(p) != EMS_SCHED_FAIR)
		return;

        if (!sched_feat(UTIL_EST))
                return;

        /* Update root cfs_rq's estimated utilization */
        enqueued  = dslt_rq->avg.util_est;
        enqueued -= min_t(unsigned int, enqueued, dslt_task_util_est(dslt_tsk));
        WRITE_ONCE(dslt_rq->avg.util_est, enqueued);
}

void dslt_update_task_sensitivity(struct task_struct *p)
{
	int grp = cpuctl_task_group_idx(p);
	struct dslt_avg *da = &ems_dslt_task(p)->avg;

	/* When io_boost, no runnable sensitivity for random read perf */
	if (emstune_sched_io_boost())
		da->runnable_sensitivity = DEFAULT_RUNNABLE_SENSITIVITY;
	else
		da->runnable_sensitivity = cgrp_runnable_sensitivity[grp];
	da->running_sensitivity = cgrp_running_sensitivity[grp];
}

static void dslt_dequeue_update_sensitivity(struct ems_dslt_rq *drq, struct ems_dslt_task *dtsk,
						struct task_struct *p, int flags)
{
	struct dslt_avg *da = &dtsk->avg;
	int org_rq_sensitivity = drq->sensitivity;
	int curr_cgroup_idx = cpuctl_task_group_idx(p);

	if (flags && !(flags & DEQUEUE_SLEEP) && (flags != DEQUEUE_NOCLOCK)) {
		if (dtsk->cgroup_idx != curr_cgroup_idx) {
			if (task_current(task_rq(p), p)) {
				drq->sensitivity -= da->running_sensitivity;
				dslt_update_task_sensitivity(p);
				drq->sensitivity += da->running_sensitivity;
			} else {
				drq->sensitivity -= da->runnable_sensitivity;
				dslt_update_task_sensitivity(p);
				drq->sensitivity += da->runnable_sensitivity;
			}

		}
		goto out;
	}

	drq->sensitivity -= da->runnable_sensitivity;
out:
	trace_dslt_dequeue_sensitivity(p, drq, da, org_rq_sensitivity, current, flags);
}

static void dslt_enqueue_update_sensitivity(struct ems_dslt_rq *drq, struct ems_dslt_task *dtsk,
						struct task_struct *p, int flags)
{
	struct dslt_avg *da = &dtsk->avg;
	int org_rq_sensitivity = drq->sensitivity;

	if ((flags & ENQUEUE_WAKEUP)
		|| (flags & ENQUEUE_MIGRATED)
		|| (flags == ENQUEUE_NOCLOCK))
		drq->sensitivity += da->runnable_sensitivity;

	trace_dslt_enqueue_sensitivity(p, drq, da, org_rq_sensitivity, flags);
}

void dslt_dequeue_task(struct rq *rq, struct task_struct *p, int flags)
{
	struct ems_dslt_rq *dslt_rq = ems_rq_dslt(rq);
	struct ems_dslt_task *dslt_tsk = ems_dslt_task(p);

	dslt_dequeue_update_sensitivity(dslt_rq, dslt_tsk, p, flags);
}

static inline void dslt_util_est_enqueue(struct ems_dslt_rq *dslt_rq,
					struct ems_dslt_task *dslt_tsk, struct task_struct *p)
{
        unsigned int enqueued;

	if (get_sched_class(p) != EMS_SCHED_FAIR)
		return;

        if (!sched_feat(UTIL_EST))
                return;

        /* Update root cfs_rq's estimated utilization */
        enqueued  = dslt_rq->avg.util_est;
        enqueued += dslt_task_util_est(dslt_tsk);
        WRITE_ONCE(dslt_rq->avg.util_est, enqueued);
}

void dslt_enqueue_task(struct rq *rq, struct task_struct *p, int flags)
{
	struct ems_dslt_rq *dslt_rq = ems_rq_dslt(rq);
	struct ems_dslt_task *dslt_tsk = ems_dslt_task(p);

	dslt_update_task_sensitivity(p);

	dslt_enqueue_update_sensitivity(dslt_rq, dslt_tsk, p, flags);

	dslt_tsk->cgroup_idx = cpuctl_task_group_idx(p);

	dslt_util_est_enqueue(dslt_rq, dslt_tsk, p);
}

void dslt_switch_sensitivity(int cpu, struct task_struct *prev,
					struct task_struct *next)
{
	int delta = 0;
	struct dslt_avg *prev_sa = &ems_dslt_task(prev)->avg;
	struct dslt_avg *next_sa = &ems_dslt_task(next)->avg;
	struct ems_dslt_rq *dslt_rq = ems_rq_dslt(cpu_rq(cpu));
	int org_sensitivity = dslt_rq->sensitivity;

	delta = prev_sa->runnable_sensitivity - prev_sa->running_sensitivity;
	delta += (next_sa->running_sensitivity - next_sa->runnable_sensitivity);

	dslt_rq->sensitivity += delta;

	trace_dslt_switch_sensitivity(cpu, prev, next, org_sensitivity, dslt_rq, prev_sa, next_sa);

	if (is_idle_task(next)) {
		if (dslt_rq->sensitivity) {
			pr_info("dslt: sensitivity=%d, delta=%d, prev_runing=%d prev_runable=%d, next_runing=%d next_runable=%d\n",
					dslt_rq->sensitivity, delta, prev_sa->running_sensitivity, prev_sa->runnable_sensitivity,
					next_sa->running_sensitivity, next_sa->runnable_sensitivity);
		}
		dslt_rq->sensitivity = 0;
	}
}

void dslt_post_init_entity_util_avg(struct sched_entity *se)
{
	struct task_struct *p = task_of(se);
	struct ems_dslt_task *dslt_tsk = ems_dslt_task(p);
	struct dslt_avg *da = &dslt_tsk->avg;

	if (get_sched_class(p) != EMS_SCHED_FAIR) {
		da->last_update_time = cfs_rq_clock_pelt(cfs_rq_of(se));
		return;
	}

	da->runnable_avg = se->avg.util_avg;
}

void dslt_init_entity_runnable_average(struct task_struct *p)
{
	struct ems_dslt_task *dslt_tsk;

	dslt_tsk = ems_dslt_task(p);
	memset(&dslt_tsk->avg, 0, sizeof(dslt_tsk->avg));
}

static int dslt_util_est_emstune_notifier_call(struct notifier_block *nb,
				unsigned long val, void *v)
{
	struct emstune_set *cur_set = (struct emstune_set *)v;
	int cgroup;

	for (cgroup = 0; cgroup < CGROUP_COUNT; cgroup++) {
		util_est.enabled[cgroup] = cur_set->util_est.enabled[cgroup];
		cgrp_runnable_sensitivity[cgroup] = cur_set->dslt.runnable_sensitivity[cgroup];
		cgrp_running_sensitivity[cgroup] = cur_set->dslt.running_sensitivity[cgroup];
	}

	return NOTIFY_OK;
}

static struct notifier_block dslt_util_est_emstune_notifier = {
	.notifier_call = dslt_util_est_emstune_notifier_call,
};

#define SYSBUSY_RUNNING_SENSITIVITY	(200)
#define SYSBUSY_RUNNABLE_SENSITIVITY	(200)
static int dslt_sysbusy_notifier_call(struct notifier_block *nb,
		unsigned long val, void *v)
{
	enum sysbusy_state state = *(enum sysbusy_state *)v;
	int running, runnable;

	if (val != SYSBUSY_STATE_CHANGE)
		return NOTIFY_OK;

	if (state == SYSBUSY_STATE3) {
		running = SYSBUSY_RUNNING_SENSITIVITY;
		runnable = SYSBUSY_RUNNABLE_SENSITIVITY;
	} else {
		running = emstune_get_dslt_running(CGROUP_TOPAPP);
		runnable = emstune_get_dslt_runnable(CGROUP_TOPAPP);
	}

	cgrp_running_sensitivity[CGROUP_TOPAPP] = running;
	cgrp_runnable_sensitivity[CGROUP_TOPAPP] = runnable;

	return NOTIFY_OK;
}

static struct notifier_block dslt_sysbusy_notifier = {
	.notifier_call = dslt_sysbusy_notifier_call,
};

static ssize_t show_grp_sensitivity(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int i, ret = 0;

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "               cgroup     running  runnable\n");
	for (i = 0; i < CGROUP_COUNT; i++)
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d]%20s %6d  %6d\n",
					i, task_cgroup_name[i],
					cgrp_running_sensitivity[i],
					cgrp_runnable_sensitivity[i]);
	return ret;
}

static ssize_t store_grp_sensitivity(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int group, running, runnable;

	if (sscanf(buf, "%d %d %d", &group, &running, &runnable) != 3)
		return -EINVAL;

	if (runnable < 0 || running < 0 || group < 0 || group >= CGROUP_COUNT)
		return -EINVAL;

	cgrp_running_sensitivity[group] = running;
	cgrp_runnable_sensitivity[group] = runnable;

	return count;
}
static struct kobj_attribute cgrp_sensitivity_attr =
__ATTR(cgrp_sensitivity, 0644, show_grp_sensitivity, store_grp_sensitivity);

static ssize_t show_ue_div_factor(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "%d\n", util_est.div_factor);

	return ret;
}

static ssize_t store_ue_div_factor(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int div_factor;

	if (!sscanf(buf, "%d", &div_factor))
		return -EINVAL;

	if (div_factor > MAX_UTIL_EST_WEIGHT_SHIFT)
		return -EINVAL;

	util_est.div_factor = div_factor;

	return count;
}

static struct kobj_attribute ue_div_factor_attr =
__ATTR(ue_div_factor, 0644, show_ue_div_factor, store_ue_div_factor);

static ssize_t show_ue_min_weight_shift(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "%d\n", util_est.min_weight_shift);

	return ret;
}

static ssize_t store_ue_min_weight_shift(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int min_weight_shift;

	if (!sscanf(buf, "%d", &min_weight_shift))
		return -EINVAL;

	if (min_weight_shift < 0)
		return -EINVAL;

	util_est.min_weight_shift = min_weight_shift;

	return count;
}

static struct kobj_attribute ue_min_weight_shift_attr =
__ATTR(ue_min_weight_shift, 0644, show_ue_min_weight_shift, store_ue_min_weight_shift);

void dslt_new_task_wakeup(struct task_struct *p)
{
	struct sched_avg *sa = &p->se.avg;
	struct ems_dslt_task *dslt_tsk = ems_dslt_task(p);

	dslt_tsk->avg.runnable_avg = sa->runnable_avg;
	dslt_tsk->avg.runnable_sensitivity = DEFAULT_RUNNABLE_SENSITIVITY;
	dslt_tsk->avg.running_sensitivity = DEFAULT_RUNNING_SENSITIVITY;
}

static void dslt_init_task(struct task_struct *p, u64 now)
{
       struct sched_avg *sa = &p->se.avg;
       struct dslt_avg *da = &ems_dslt_task(p)->avg;

       da->last_update_time = sa->last_update_time;
       da->runnable_sum = sa->runnable_sum;
       da->period_contrib = sa->period_contrib;
       da->runnable_avg = sa->runnable_avg;
}

static void dslt_util_est_init(void)
{
	int cgroup;

	for (cgroup = 0; cgroup < CGROUP_COUNT; cgroup++)
		util_est.enabled[cgroup] = DEFAULT_UTIL_EST_ENABLED;

	util_est.div_factor = DEFAULT_UTIL_EST_DIV_FACTOR;
	util_est.min_weight_shift = MIN_UTIL_EST_WEIGHT_SHIFT;

	emstune_register_notifier(&dslt_util_est_emstune_notifier);
	sysbusy_register_notifier(&dslt_sysbusy_notifier);
}

int dslt_init(struct kobject *ems_kobj, struct device_node *dn)
{
	int i, cpu;
	struct task_struct *p;
	u64 now = sched_clock();

	dslt_util_est_init();

	rcu_read_lock();
	list_for_each_entry_rcu(p, &init_task.tasks, tasks) {
		get_task_struct(p);
		dslt_init_task(p, now);
		put_task_struct(p);
	}
	rcu_read_unlock();

	for_each_possible_cpu(cpu) {
		struct ems_dslt_rq *dslt_rq = ems_rq_dslt(cpu_rq(cpu));

		dslt_rq->cpu = cpu;
		raw_spin_lock_init(&dslt_rq->removed.lock);
	}


	dslt_kobj = kobject_create_and_add("dslt", ems_kobj);
	if (!dslt_kobj) {
		pr_info("%s: fail to create node\n", __func__);
		return -EINVAL;
	}

	util_est_kobj = kobject_create_and_add("util_est", ems_kobj);
	if (!util_est_kobj) {
		pr_info("%s: fail to create node\n", __func__);
		return -EINVAL;
	}

	if (sysfs_create_file(dslt_kobj, &cgrp_sensitivity_attr.attr))
		pr_warn("%s: failed to create cgrp sensitivity sysfs\n", __func__);

	if (sysfs_create_file(util_est_kobj, &ue_div_factor_attr.attr))
		pr_warn("%s: failed to create ue_div_factor sysfs\n", __func__);

	if (sysfs_create_file(util_est_kobj, &ue_min_weight_shift_attr.attr))
		pr_warn("%s: failed to create ue_min_weight_shift sysfs\n", __func__);

	for (i = 0; i < CGROUP_COUNT; i++) {
		cgrp_runnable_sensitivity[i] = DEFAULT_RUNNABLE_SENSITIVITY;
		cgrp_running_sensitivity[i] = DEFAULT_RUNNING_SENSITIVITY;
	}

	return 0;
}
