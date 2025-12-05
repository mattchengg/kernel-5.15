
#define MAX_UTIL_EST_WEIGHT_SHIFT		(7)
#define DEFAULT_UTIL_EST_ENABLED		(1)
#define DEFAULT_UTIL_EST_WEIGHT_SHIFT		(2)
#define MIN_UTIL_EST_WEIGHT_SHIFT		(1)
#define DEFAULT_UTIL_EST_DIV_FACTOR		(2)

struct {
	unsigned int enabled[CGROUP_COUNT];
	unsigned int div_factor;
	int min_weight_shift;
} util_est;

static const u32 runnable_avg_yN_inv[] __maybe_unused = {
	0xffffffff, 0xfa83b2da, 0xf5257d14, 0xefe4b99a, 0xeac0c6e6, 0xe5b906e6,
	0xe0ccdeeb, 0xdbfbb796, 0xd744fcc9, 0xd2a81d91, 0xce248c14, 0xc9b9bd85,
	0xc5672a10, 0xc12c4cc9, 0xbd08a39e, 0xb8fbaf46, 0xb504f333, 0xb123f581,
	0xad583ee9, 0xa9a15ab4, 0xa5fed6a9, 0xa2704302, 0x9ef5325f, 0x9b8d39b9,
	0x9837f050, 0x94f4efa8, 0x91c3d373, 0x8ea4398a, 0x8b95c1e3, 0x88980e80,
	0x85aac367, 0x82cd8698,
};

#define LOAD_AVG_PERIOD 32
#define LOAD_AVG_MAX 47742

/*
 * Unsigned subtract and clamp on underflow.
 *
 * Explicitly do a load-store to ensure the intermediate value never hits
 * memory. This allows lockless observations without ever seeing the negative
 * values.
 */
#define sub_positive(_ptr, _val) do {				\
	typeof(_ptr) ptr = (_ptr);				\
	typeof(*ptr) val = (_val);				\
	typeof(*ptr) res, var = READ_ONCE(*ptr);		\
	res = var - val;					\
	if (res > var)						\
		res = 0;					\
	WRITE_ONCE(*ptr, res);					\
} while (0)

#define DSLT_MIN_DIVIDER	(LOAD_AVG_MAX - 1024)
#define UTIL_EST_MARGIN 	(SCHED_CAPACITY_SCALE / 100)

static inline u32 get_dslt_divider(struct dslt_avg *avg)
{
	return DSLT_MIN_DIVIDER + avg->period_contrib;
}

static inline void cfs_se_util_change(struct dslt_avg *avg)
{
	unsigned int enqueued;

	if (!sched_feat(UTIL_EST))
		return;

	/* Avoid store if the flag has been already reset */
	enqueued = avg->util_est;
	if (!(enqueued & UTIL_AVG_UNCHANGED))
		return;

	/* Reset flag to report util_avg has been updated */
	enqueued &= ~UTIL_AVG_UNCHANGED;
	WRITE_ONCE(avg->util_est, enqueued);
}

static inline u64 rq_clock_pelt(struct rq *rq)
{
	lockdep_assert_rq_held(rq);
	assert_clock_updated(rq);

	return rq->clock_pelt - rq->lost_idle_time;
}

#ifdef CONFIG_CFS_BANDWIDTH
/* rq->task_clock normalized against any time this cfs_rq has spent throttled */
static inline u64 cfs_rq_clock_pelt(struct cfs_rq *cfs_rq)
{
	if (unlikely(cfs_rq->throttle_count))
		return cfs_rq->throttled_clock_pelt - cfs_rq->throttled_clock_pelt_time;

	return rq_clock_pelt(rq_of(cfs_rq)) - cfs_rq->throttled_clock_pelt_time;
}
#else
static inline u64 cfs_rq_clock_pelt(struct cfs_rq *cfs_rq)
{
	return rq_clock_pelt(rq_of(cfs_rq));
}
#endif
