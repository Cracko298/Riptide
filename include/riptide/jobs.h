/* SPDX-License-Identifier: MIT */
#ifndef RIPTIDE_JOBS_H
#define RIPTIDE_JOBS_H
#include "core.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct RiptideJobs RiptideJobs;
/** Job callback.
 * @param context Caller-owned payload; its lifetime spans submission through completion.
 * Work callbacks run on a worker; completion callbacks run during poll/destroy.
 * Publish results through the payload. Do not mutate live renderer/world state in work. */
typedef void (*RiptideJobFn)(void *context);
/** Create a bounded worker pool.
 * @param workers Worker count, [1, 16]. Ignored in RIPTIDE_NO_THREADS builds.
 * @param capacity Maximum outstanding jobs, including unpolled results, [1, 65536].
 * @return Owned pool or NULL. Allocation and thread startup happen here only. */
RiptideJobs *riptide_jobs_create(unsigned workers, unsigned capacity);
/** Submit work without blocking on queue space.
 * @param jobs Pool, owned by the calling main thread.
 * @param work Required worker callback.
 * @param complete Optional main-thread completion callback.
 * @param context Payload retained by pointer; caller keeps it alive until completion.
 * @return False if full/closing/invalid; ownership stays with the caller.
 * RIPTIDE_NO_THREADS runs work immediately but still defers complete until poll. */
bool riptide_jobs_submit(RiptideJobs *jobs, RiptideJobFn work, RiptideJobFn complete, void *context);
/** Deliver ready results on the calling thread.
 * @param jobs Pool; only its owner may poll.
 * @param budget Maximum completion callbacks/results to release.
 * @return Number of released results. Completion order may differ from submission order.
 * Completion callbacks may submit new work, but must not destroy or recursively poll the pool. */
unsigned riptide_jobs_poll(RiptideJobs *jobs, unsigned budget);
/** Drain accepted work, deliver remaining completions, and release the pool.
 * @param jobs Pool; NULL is accepted.
 * Blocks until running work returns. Work cannot be forcibly cancelled; callbacks
 * must terminate. Submissions during shutdown return false. */
void riptide_jobs_destroy(RiptideJobs *jobs);
#ifdef __cplusplus
}
#endif
#endif
