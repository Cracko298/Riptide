/* SPDX-License-Identifier: MIT */
#include "riptide/jobs.h"
#include <stdlib.h>
#ifndef RIPTIDE_NO_THREADS
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
typedef CRITICAL_SECTION Mutex;
typedef CONDITION_VARIABLE Condition;
typedef HANDLE Thread;
static bool lock_init(Mutex *m) {
    InitializeCriticalSection(m);
    return true;
}
static void lock_free(Mutex *m) {
    DeleteCriticalSection(m);
}
static void lock(Mutex *m) {
    EnterCriticalSection(m);
}
static void unlock(Mutex *m) {
    LeaveCriticalSection(m);
}
static bool condition_init(Condition *c) {
    InitializeConditionVariable(c);
    return true;
}
static void condition_free(Condition *c) {
    (void)c;
}
static void wake(Condition *c) {
    WakeAllConditionVariable(c);
}
static void wait(Condition *c, Mutex *m) {
    SleepConditionVariableCS(c, m, INFINITE);
}
#else
#include <pthread.h>
typedef pthread_mutex_t Mutex;
typedef pthread_cond_t Condition;
typedef pthread_t Thread;
static bool lock_init(Mutex *m) {
    return pthread_mutex_init(m, NULL) == 0;
}
static void lock_free(Mutex *m) {
    pthread_mutex_destroy(m);
}
static void lock(Mutex *m) {
    pthread_mutex_lock(m);
}
static void unlock(Mutex *m) {
    pthread_mutex_unlock(m);
}
static bool condition_init(Condition *c) {
    return pthread_cond_init(c, NULL) == 0;
}
static void condition_free(Condition *c) {
    pthread_cond_destroy(c);
}
static void wake(Condition *c) {
    pthread_cond_broadcast(c);
}
static void wait(Condition *c, Mutex *m) {
    pthread_cond_wait(c, m);
}
#endif
#else
typedef int Mutex;
static bool lock_init(Mutex *m) {
    *m = 0;
    return true;
}
static void lock_free(Mutex *m) {
    (void)m;
}
static void lock(Mutex *m) {
    (void)m;
}
static void unlock(Mutex *m) {
    (void)m;
}
#endif
typedef struct {
    RiptideJobFn work, complete;
    void *context;
    unsigned state;
    uint64_t ticket;
} Job;
struct RiptideJobs {
    Mutex mutex;
    Job *slots;
    unsigned capacity, worker_count;
    bool closing;
    uint64_t next_ticket;
#ifndef RIPTIDE_NO_THREADS
    Condition ready;
    Thread threads[16];
#endif
};
#ifndef RIPTIDE_NO_THREADS
static void worker_loop(RiptideJobs *p) {
    lock(&p->mutex);
    for (;;) {
        unsigned at = p->capacity;
        uint64_t oldest = UINT64_MAX;
        for (unsigned i = 0; i < p->capacity; i++)
            if (p->slots[i].state == 1 && p->slots[i].ticket < oldest) {
                at = i;
                oldest = p->slots[i].ticket;
            }
        if (at == p->capacity) {
            if (p->closing)
                break;
            wait(&p->ready, &p->mutex);
            continue;
        }
        Job *j = &p->slots[at];
        j->state = 2;
        RiptideJobFn fn = j->work;
        void *context = j->context;
        unlock(&p->mutex);
        fn(context);
        lock(&p->mutex);
        j->state = 3;
    }
    unlock(&p->mutex);
}
#ifdef _WIN32
static DWORD WINAPI worker_entry(void *p) {
    worker_loop(p);
    return 0;
}
#else
static void *worker_entry(void *p) {
    worker_loop(p);
    return NULL;
}
#endif
#endif
RiptideJobs *riptide_jobs_create(unsigned workers, unsigned capacity) {
    if (!workers || workers > 16 || !capacity || capacity > 65536)
        return NULL;
    RiptideJobs *p = calloc(1, sizeof *p);
    if (!p)
        return NULL;
    p->slots = calloc(capacity, sizeof *p->slots);
    p->capacity = capacity;
    if (!p->slots || !lock_init(&p->mutex)) {
        free(p->slots);
        free(p);
        return NULL;
    }
#ifndef RIPTIDE_NO_THREADS
    if (!condition_init(&p->ready)) {
        lock_free(&p->mutex);
        free(p->slots);
        free(p);
        return NULL;
    }
    for (unsigned i = 0; i < workers; i++) {
#ifdef _WIN32
        p->threads[i] = CreateThread(NULL, 0, worker_entry, p, 0, NULL);
        bool ok = p->threads[i] != NULL;
#else
        bool ok = pthread_create(&p->threads[i], NULL, worker_entry, p) == 0;
#endif
        if (!ok) {
            riptide_jobs_destroy(p);
            return NULL;
        }
        p->worker_count++;
    }
#else
    (void)workers;
#endif
    return p;
}
bool riptide_jobs_submit(RiptideJobs *p, RiptideJobFn work, RiptideJobFn complete, void *context) {
    if (!p || !work)
        return false;
    lock(&p->mutex);
    if (p->closing || p->next_ticket == UINT64_MAX) {
        unlock(&p->mutex);
        return false;
    }
    unsigned at = 0;
    while (at < p->capacity && p->slots[at].state)
        at++;
    if (at == p->capacity) {
        unlock(&p->mutex);
        return false;
    }
    p->slots[at] = (Job){work, complete, context, 1, p->next_ticket++};
#ifndef RIPTIDE_NO_THREADS
    wake(&p->ready);
    unlock(&p->mutex);
#else
    unlock(&p->mutex);
    work(context);
    p->slots[at].state = 3;
#endif
    return true;
}
unsigned riptide_jobs_poll(RiptideJobs *p, unsigned budget) {
    if (!p)
        return 0;
    unsigned count = 0;
    for (unsigned i = 0; i < p->capacity && count < budget; i++) {
        lock(&p->mutex);
        Job j = p->slots[i];
        if (j.state == 3)
            p->slots[i] = (Job){0};
        unlock(&p->mutex);
        if (j.state == 3) {
            count++;
            if (j.complete)
                j.complete(j.context);
        }
    }
    return count;
}
void riptide_jobs_destroy(RiptideJobs *p) {
    if (!p)
        return;
    lock(&p->mutex);
    p->closing = true;
#ifndef RIPTIDE_NO_THREADS
    wake(&p->ready);
#endif
    unlock(&p->mutex);
#ifndef RIPTIDE_NO_THREADS
    for (unsigned i = 0; i < p->worker_count; i++) {
#ifdef _WIN32
        WaitForSingleObject(p->threads[i], INFINITE);
        CloseHandle(p->threads[i]);
#else
        pthread_join(p->threads[i], NULL);
#endif
    }
    condition_free(&p->ready);
#endif
    riptide_jobs_poll(p, p->capacity);
    lock_free(&p->mutex);
    free(p->slots);
    free(p);
}
