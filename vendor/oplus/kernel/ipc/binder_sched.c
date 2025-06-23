// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#include <linux/seq_file.h>
#include <drivers/android/binder_internal.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <trace/hooks/binder.h>
#include <linux/random.h>
#include <linux/of.h>

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#endif
#include "binder_sched.h"
#define CREATE_TRACE_POINTS
#include "binder_sched_trace.h"
#if IS_ENABLED(CONFIG_ANDROID_BINDER_IPC_VIP_THREAD)
#include "vipthread/binder_vip.h"
#endif

/*All functions of the binder must be controlled by each
  bit of this switch. The binder uses bits 16-31.*/
#define FEATURE_ENABLE 0xffffffff
unsigned int dynamic_switch;
unsigned int g_bd_opt_enable;

#ifdef CONFIG_OPLUS_BINDER_REF_OPT
unsigned long *g_free_ref = NULL;
unsigned int g_ref_enable = 1;
#endif
struct kmem_cache *oplus_binder_struct_cachep = NULL;
unsigned int g_sched_enable = 1;
EXPORT_SYMBOL(g_sched_enable);
unsigned long long g_sched_debug = 0;

unsigned int g_async_ux_enable = 1;
unsigned int g_set_last_async_ux = 1;
unsigned int g_set_async_ux_after_pending = 1;
static unsigned int async_insert_queue = 1;
static unsigned int sync_insert_queue = 1;
static unsigned int async_ux_test = 0;
static unsigned int allow_accumulate_ux = 1;
int unset_async_ux_inrestore = 1;
int get_random_binder_task = 1;

static int insert_limit[NUM_INSERT_MAX] = {0};
#ifdef CONFIG_OPLUS_BINDER_REF_OPT
static struct binder_proc *system_server_proc = NULL;
#endif

#if (((LINUX_VERSION_CODE >= KERNEL_VERSION(5, 1, 0)) \
	&& (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))) \
	|| ((LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)) \
	&& (LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0))))
/* sync_use_t_vendordata can not be enabled in kernel5.15(SM7550) */
unsigned int sync_use_t_vendordata = 1;
#else
unsigned int sync_use_t_vendordata = 0;
#endif

#ifdef CONFIG_OPLUS_BINDER_REF_OPT
static DEFINE_SPINLOCK(binder_ref_lock);
#endif

#define trace_binder_debug(x...) \
	do { \
		if (g_sched_debug) \
			trace_printk(x); \
	} while (0)

#define oplus_binder_debug(debug_mask, x...) \
	do { \
		if (g_sched_debug & debug_mask) \
			pr_info(x); \
	} while (0)

static inline bool binder_opt_enable(unsigned int enable)
{
	return g_bd_opt_enable & enable;
}

void parse_dts_switch(void)
{
	struct device_node *np = NULL;
	int ret;

	np = of_find_node_by_name(NULL, "oplus_sync_ipc");

	if(np) {
		ret = of_property_read_u32(np, "disable", &dynamic_switch);
		if(ret) {
			pr_err("no oplus_sync_ipc disable!");
		} else {
			pr_err("oplus_sync_ipc : %d", dynamic_switch);
			return;
		}
	}

	dynamic_switch = FEATURE_ENABLE;
}

static inline struct oplus_binder_struct *alloc_oplus_binder_struct(void)
{
	if (!oplus_binder_struct_cachep) {
		return NULL;
	} else {
		return kmem_cache_alloc(oplus_binder_struct_cachep, GFP_ATOMIC);
	}
}

static inline void free_oplus_binder_struct(struct oplus_binder_struct *obs)
{
	if ((unsigned long long)obs == OBS_NOT_ASYNC_UX_VALUE) {
		obs = NULL;
	} else if (!oplus_binder_struct_cachep || IS_ERR_OR_NULL(obs)) {
		return;
	} else {
		memset(obs, 0, sizeof(struct oplus_binder_struct));
		kmem_cache_free(oplus_binder_struct_cachep, obs);
	}
}

static inline int is_obs_valid(struct oplus_binder_struct *obs)
{
	if ((unsigned long long)obs == OBS_NOT_ASYNC_UX_VALUE)
		return OBS_NOT_ASYNC_UX;
	else if (IS_ERR_OR_NULL(obs))
		return OBS_INVALID;
	else
		return OBS_VALID;
}

struct oplus_binder_struct *get_oplus_binder_struct(struct binder_transaction *t, bool alloc)
{
	struct oplus_binder_struct *obs = NULL;

	if (IS_ERR_OR_NULL(t)) {
		return NULL;
	}

	obs = (struct oplus_binder_struct *)(t->android_vendor_data1);
	if (!alloc) {
		trace_binder_get_obs(t, obs, alloc, "no alloc");
		return obs;
	}
	trace_binder_get_obs(t, obs, alloc, "before alloc");
	if (is_obs_valid(obs) == OBS_VALID) {
		return obs;
	} else if (alloc) {
		obs = alloc_oplus_binder_struct();
		t->android_vendor_data1 = (unsigned long long)obs;
		trace_binder_get_obs(t, obs, alloc, "after alloc");
		return obs;
	} else {
		return NULL;
	}
}

static void set_obs_not_async_ux(struct binder_transaction *t, struct oplus_binder_struct *obs)
{
	obs = (struct oplus_binder_struct *)OBS_NOT_ASYNC_UX_VALUE;
	if (!IS_ERR_OR_NULL(t)) {
		t->android_vendor_data1 = (unsigned long long)obs;
	}
}

static inline bool binder_is_sync_mode(u32 flags)
{
	return !(flags & TF_ONE_WAY);
}

static inline bool get_task_async_ux_sts(struct oplus_task_struct *ots)
{
	if (IS_ERR_OR_NULL(ots)) {
		return false;
	} else {
		return ots->binder_async_ux_sts;
	}
}

static inline void set_task_async_ux_sts(struct oplus_task_struct *ots, bool sts)
{
	if (IS_ERR_OR_NULL(ots)) {
		return;
	} else {
		ots->binder_async_ux_sts = sts;
	}
}

void set_task_async_ux_enable(pid_t pid, int enable)
{
	struct task_struct *task = NULL;
	struct oplus_task_struct *ots = NULL;

	if (unlikely(!g_async_ux_enable)) {
		return;
	}
	if (enable >= ASYNC_UX_ENABLE_MAX) {
		trace_binder_set_get_ux(task, pid, enable, "set, enable error");
		return;
	}

	if (pid == CURRENT_TASK_PID) {
		task = current;
	} else {
		if (pid < 0 || pid > PID_MAX_DEFAULT) {
			trace_binder_set_get_ux(task, pid, enable, "set, pid error");
			return;
		}
		task = find_task_by_vpid(pid);
		if (IS_ERR_OR_NULL(task)) {
			trace_binder_set_get_ux(NULL, pid, enable, "set, task null");
			return;
		}
	}
	ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots)) {
		trace_binder_set_get_ux(task, pid, enable, "set, ots null");
		return;
	}
	ots->binder_async_ux_enable = enable;

	trace_binder_set_get_ux(task, pid, enable, "set enable end");
	oplus_binder_debug(LOG_SET_ASYNC_UX, "(set_pid=%d task_pid=%d comm=%s) enable=%d ux_sts=%d set enable end\n",
		pid, task->pid, task->comm, ots->binder_async_ux_enable, ots->binder_async_ux_sts);
}

bool get_task_async_ux_enable(pid_t pid)
{
	struct task_struct *task = NULL;
	struct oplus_task_struct *ots = NULL;
	int enable = 0;

	if (unlikely(!g_async_ux_enable)) {
		return false;
	}

	if (pid == CURRENT_TASK_PID) {
		task = current;
	} else {
		if (pid < 0 || pid > PID_MAX_DEFAULT) {
			trace_binder_set_get_ux(task, pid, enable, "get, pid error");
			return false;
		}
		task = find_task_by_vpid(pid);
		if (IS_ERR_OR_NULL(task)) {
			trace_binder_set_get_ux(NULL, pid, enable, "get, task null");
			return false;
		}
	}

	ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots)) {
		trace_binder_set_get_ux(task, pid, enable, "get, ots null");
		return false;
	}
	enable = ots->binder_async_ux_enable;
	trace_binder_set_get_ux(task, pid, enable, "get end");
	return enable;
}

void get_all_tasks_async_ux_enable(void)
{
	struct task_struct *p = NULL;
	struct task_struct *t = NULL;
	struct oplus_task_struct *ots = NULL;
	bool async_ux_task = false;

	for_each_process_thread(p, t) {
		ots = get_oplus_task_struct(t);
		if (IS_ERR_OR_NULL(ots)) {
			pr_info("[async_ux_tasks] ots err, pid=%d tgid=%d comm=%s async_ux_enable=unknown\n",
				t->pid, t->tgid, t->comm);
			trace_binder_set_get_ux(t, INVALID_VALUE, INVALID_VALUE, "[async_ux_tasks] ots err");
		} else if (ots->binder_async_ux_enable) {
			async_ux_task = true;
			pr_info("[async_ux_tasks] pid=%d tgid=%d comm=%s async_ux_enable=%d\n",
				t->pid, t->tgid, t->comm, ots->binder_async_ux_enable);
			trace_binder_set_get_ux(t, INVALID_VALUE, ots->binder_async_ux_enable, "[async_ux_tasks]");
		}
	}
	if (!async_ux_task) {
		pr_info("[async_ux_tasks] no async_ux task\n");
		trace_binder_set_get_ux(NULL, INVALID_VALUE, INVALID_VALUE,
			"[async_ux_tasks] no task");
	}
}

static int list_count(struct list_head *head)
{
	struct list_head *pos = NULL;
	int count = 0;

	if (!head)
		return INVALID_VALUE;

	list_for_each(pos, head) {
		count++;
	}
	return count;
}

noinline int tracing_mark_write(const char *buf)
{
	trace_printk(buf);
	return 0;
}

static inline bool is_task_servicemg(struct task_struct *task)
{
	if (IS_ERR_OR_NULL(task)) {
		return false;
	}
	if (!strncmp(task->comm, "servicemanager", TASK_COMM_LEN)
				|| !strncmp(task->comm, "hwservicemanage", TASK_COMM_LEN)) {
		return true;
	} else {
		return false;
	}
}

static void binder_ux_state_systrace(struct task_struct *from, struct task_struct *target,
	int ux_state, int systrace_lvl, struct binder_transaction *t, struct binder_proc *proc)
{
	bool lvl0_enable = false;
	bool lvl1_enable = false;
	int from_pid = 0;
	int target_pid = 0;

	if (g_sched_debug & LOG_BINDER_SYSTRACE_LVL0) {
		lvl0_enable = true;
	}
	if (g_sched_debug & LOG_BINDER_SYSTRACE_LVL1) {
		lvl1_enable = true;
	}
	if (!lvl0_enable && !lvl1_enable) {
		return;
	} else if ((systrace_lvl == LOG_BINDER_SYSTRACE_LVL1) && !lvl1_enable) {
		return;
	} else {
		char buf[128] = {0};
		if (!IS_ERR_OR_NULL(from)) {
			from_pid = from->pid;
		}
		if (!IS_ERR_OR_NULL(target)) {
			target_pid = target->pid;
		}
		if ((g_sched_debug & LOG_BINDER_SYSTRACE_STATUS)) {
			unsigned long long inherit_ux = 0;
			int ux_type = INVALID_VALUE;
			int real_ux_state = INVALID_VALUE;
			int ux_depth = INVALID_VALUE;
			int proc_pid = INVALID_VALUE;
			int waiting_threads = INVALID_VALUE;
			int requested_threads = INVALID_VALUE;
			int requested_threads_started = INVALID_VALUE;
			int max_threads = INVALID_VALUE;
			unsigned long long t_vendordata = 127;	/* 0x7F */

			if (target) {
				inherit_ux = oplus_get_inherit_ux(target);
				ux_type = get_ux_state_type(target);
				real_ux_state = oplus_get_ux_state(target);
				ux_depth = oplus_get_ux_depth(target);
			}
			if (proc) {
				proc_pid = proc->tsk->pid;
				waiting_threads = list_count(&proc->waiting_threads);
				requested_threads = proc->requested_threads;
				requested_threads_started = proc->requested_threads_started;
				max_threads = proc->max_threads;
			}
			if (t && binder_is_sync_mode(t->flags)) {
				t_vendordata = t->android_vendor_data1;
			}

			memset(buf, 0, sizeof(buf));
			snprintf(buf, sizeof(buf), "C|9999|z_binder_inherit_ux|%lld\n", inherit_ux);
			tracing_mark_write(buf);
			memset(buf, 0, sizeof(buf));
			snprintf(buf, sizeof(buf), "C|9999|z_binder_ux_type|%d\n", ux_type);
			tracing_mark_write(buf);
			memset(buf, 0, sizeof(buf));
			snprintf(buf, sizeof(buf), "C|9999|z_binder_real_ux_state|%d\n", real_ux_state);
			tracing_mark_write(buf);
			memset(buf, 0, sizeof(buf));
			snprintf(buf, sizeof(buf), "C|9999|z_binder_ux_depth|%d\n", ux_depth);
			tracing_mark_write(buf);

			memset(buf, 0, sizeof(buf));
			snprintf(buf, sizeof(buf), "C|9999|z_binder_proc_pid|%d\n", proc_pid);
			tracing_mark_write(buf);
			memset(buf, 0, sizeof(buf));
			snprintf(buf, sizeof(buf), "C|9999|z_binder_waiting_threads|%d\n", waiting_threads);
			tracing_mark_write(buf);
			memset(buf, 0, sizeof(buf));
			snprintf(buf, sizeof(buf), "C|9999|z_binder_requested_threads|%d\n", requested_threads);
			tracing_mark_write(buf);
			memset(buf, 0, sizeof(buf));
			snprintf(buf, sizeof(buf), "C|9999|z_binder_requested_threads_started|%d\n", requested_threads_started);
			tracing_mark_write(buf);
			memset(buf, 0, sizeof(buf));
			snprintf(buf, sizeof(buf), "C|9999|z_binder_max_threads|%d\n", max_threads);
			tracing_mark_write(buf);
			memset(buf, 0, sizeof(buf));
			snprintf(buf, sizeof(buf), "C|9999|z_binder_sync_use_t_vendordata|%d\n", sync_use_t_vendordata);
			tracing_mark_write(buf);
			memset(buf, 0, sizeof(buf));
			snprintf(buf, sizeof(buf), "C|9999|z_binder_t_vendordata|%lld\n", t_vendordata);
			tracing_mark_write(buf);
		}
		snprintf(buf, sizeof(buf), "C|9999|z_binder_from|%d\n", from_pid);
		tracing_mark_write(buf);
		memset(buf, 0, sizeof(buf));
		snprintf(buf, sizeof(buf), "C|9999|z_binder_target|%d\n", target_pid);
		tracing_mark_write(buf);
		memset(buf, 0, sizeof(buf));
		snprintf(buf, sizeof(buf), "C|9999|z_binder_ux_state|%d\n", ux_state);
		tracing_mark_write(buf);
		memset(buf, 0, sizeof(buf));
		if (IS_ERR_OR_NULL(t)) {
			snprintf(buf, sizeof(buf), "C|9999|z_binder_vt_id|0\n");
		} else {
			snprintf(buf, sizeof(buf), "C|9999|z_binder_vt_id|%d\n", t->debug_id);
		}
		tracing_mark_write(buf);
	}
}

static inline void set_sync_t_ux_state(struct binder_transaction *t, bool enable,
	bool sync, bool is_servicemg)
{
	if (!sync_use_t_vendordata || !sync || !t || !is_servicemg) {
		return;
	}

	/*
	if t->android_vendor_data1 is occupied, stop use vendordata any more.
	*/
	if (t->android_vendor_data1 & CHECK_T_VENDORDATA_OCCUPIED) {
		sync_use_t_vendordata = 0;
		return;
	}

	t->android_vendor_data1 &= (~T_SYNC_UX_MASK);
	if (enable) {
		t->android_vendor_data1 |= T_IS_SYNC_UX;
		binder_ux_state_systrace(current, NULL, STATE_SET_T_VENDORDATA,
			LOG_BINDER_SYSTRACE_LVL0, t, NULL);
	} else {
		t->android_vendor_data1 |= T_NOT_SYNC_UX;
		binder_ux_state_systrace(current, NULL, STATE_UNSET_T_VENDORDATA,
			LOG_BINDER_SYSTRACE_LVL0, t, NULL);
	}
}

static inline bool is_sync_t_ux_state(struct binder_transaction *t,
	bool sync, bool is_servicemg)
{
	/* default return true */
	if (!sync_use_t_vendordata || !sync || !t || !is_servicemg) {
		return true;
	}
	if ((t->android_vendor_data1 & T_SYNC_UX_MASK) == T_IS_SYNC_UX)
		return true;
	else
		return false;
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)

static inline void sync_binder_set_inherit_ux(struct task_struct *thread_task, struct task_struct *from_task,
	bool sync, bool is_servicemg, struct binder_transaction *t, struct binder_proc *proc)
{
	int from_depth = oplus_get_ux_depth(from_task);
	int from_state = oplus_get_ux_state(from_task);
	int type = get_ux_state_type(thread_task);
	unsigned long long inherit_ux = 0;

	if (type != UX_STATE_NONE && type != UX_STATE_INHERIT) {
		trace_binder_inherit_ux(from_task, thread_task, from_depth, from_state,
			type, INVALID_VALUE, sync, "sync_set type not expected");
		binder_ux_state_systrace(current, thread_task, STATE_SYNC_TYPE_UNEXPECTED,
			LOG_BINDER_SYSTRACE_LVL1, t, proc);
		return;
	}
	if (from_task && test_set_inherit_ux(from_task)) {
		inherit_ux = oplus_get_inherit_ux(thread_task);
		if (!test_inherit_ux(thread_task, INHERIT_UX_BINDER)) {
			set_inherit_ux(thread_task, INHERIT_UX_BINDER, from_depth, from_state);
			set_sync_t_ux_state(t, true, true, is_servicemg);
			trace_binder_inherit_ux(from_task, thread_task, from_depth, from_state,
				type, INVALID_VALUE, sync, "sync_set ux set");
			binder_ux_state_systrace(current, thread_task, STATE_SYNC_SET_UX, LOG_BINDER_SYSTRACE_LVL0, t, proc);
		} else if (allow_accumulate_ux && is_servicemg && inherit_ux > 0 && inherit_ux < MAX_ACCUMULATED_UX) {
			set_inherit_ux(thread_task, INHERIT_UX_BINDER, from_depth, from_state);
			set_sync_t_ux_state(t, true, true, is_servicemg);

			binder_ux_state_systrace(current, thread_task, STATE_SYNC_SET_UX_AGAIN_SERVICEMG, LOG_BINDER_SYSTRACE_LVL0, t, proc);
		} else {
			reset_inherit_ux(thread_task, from_task, INHERIT_UX_BINDER);
			set_sync_t_ux_state(t, true, true, is_servicemg);
			if (is_servicemg) {
				binder_ux_state_systrace(current, thread_task, STATE_SYNC_RESET_UX_SERVICEMG, LOG_BINDER_SYSTRACE_LVL0, t, proc);
			} else {
				binder_ux_state_systrace(current, thread_task, STATE_SYNC_RESET_UX, LOG_BINDER_SYSTRACE_LVL0, t, proc);
			}
		}
	}  else if (from_task && test_task_is_rt(from_task)) { /* rt trans can be set as ux if binder thread is cfs class */
		inherit_ux = oplus_get_inherit_ux(thread_task);
		if (!test_inherit_ux(thread_task, INHERIT_UX_BINDER)) {
			set_inherit_ux(thread_task, INHERIT_UX_BINDER, from_depth, SA_TYPE_HEAVY);
			set_sync_t_ux_state(t, true, true, is_servicemg);
			trace_binder_inherit_ux(from_task, thread_task, from_depth,
				from_state, type, INVALID_VALUE, sync, "sync_set ux rt");
			binder_ux_state_systrace(current, thread_task, STATE_SYNC_RT_SET_UX, LOG_BINDER_SYSTRACE_LVL0, t, proc);
		} else if (allow_accumulate_ux && is_servicemg && inherit_ux > 0 && inherit_ux < MAX_ACCUMULATED_UX) {
			set_inherit_ux(thread_task, INHERIT_UX_BINDER, from_depth, SA_TYPE_HEAVY);
			set_sync_t_ux_state(t, true, true, is_servicemg);

			binder_ux_state_systrace(current, thread_task, STATE_SYNC_SET_UX_AGAIN_SERVICEMG, LOG_BINDER_SYSTRACE_LVL0, t, proc);
		} else {
			trace_binder_inherit_ux(from_task, thread_task, from_depth,
				from_state, type, INVALID_VALUE, sync, "sync_set rt none");
			if (is_servicemg) {
				binder_ux_state_systrace(current, thread_task, STATE_SYNC_RT_NOT_SET_SERVICEMG, LOG_BINDER_SYSTRACE_LVL0, t, proc);
			} else {
				binder_ux_state_systrace(current, thread_task, STATE_SYNC_RT_NOT_SET, LOG_BINDER_SYSTRACE_LVL0, t, proc);
			}
		}
	} else {
		trace_binder_inherit_ux(from_task, thread_task, from_depth, from_state,
			type, INVALID_VALUE, sync, "sync_set end do nothing");
		binder_ux_state_systrace(current, thread_task, STATE_SYNC_NOT_SET, LOG_BINDER_SYSTRACE_LVL1, t, proc);
	}
}

static inline void async_binder_set_inherit_ux(struct task_struct *thread_task,
	struct task_struct *from_task, bool sync, struct binder_transaction *t, struct binder_proc *proc)
{
	struct oplus_task_struct *ots = NULL;
	int type = 0;
	int ux_value = 0;

	if (unlikely(!g_async_ux_enable)) {
		return;
	}

	if (!thread_task) {
		return;
	}

	type = get_ux_state_type(thread_task);
	if (type != UX_STATE_NONE && type != UX_STATE_INHERIT) {
		trace_binder_inherit_ux(from_task, thread_task, INVALID_VALUE, INVALID_VALUE,
			type, INVALID_VALUE, sync, "async_set type not expected");
		return;
	}

	ots = get_oplus_task_struct(thread_task);
	if (unlikely(IS_ERR_OR_NULL(ots))) {
		trace_binder_inherit_ux(from_task, thread_task, INVALID_VALUE, INVALID_VALUE,
			type, INVALID_VALUE, sync, "async_set ots null");
		return;
	}

	if ((type == UX_STATE_INHERIT) && (ots->ux_depth > BINDER_UX_MAX_DEPTH)) {
		binder_ux_state_systrace(current, thread_task, STATE_MAX_DEPTH_NOT_SET_UX,
			LOG_BINDER_SYSTRACE_LVL0, t, proc);
		return;
	}

	trace_binder_inherit_ux(from_task, thread_task, ots->ux_depth, ots->ux_state,
		type, ots->binder_async_ux_sts, sync, "async_set before set");

	set_task_async_ux_sts(ots, true);
	ux_value = (ots->ux_state | SA_TYPE_HEAVY);
	set_inherit_ux(thread_task, INHERIT_UX_BINDER, ots->ux_depth, ux_value);

	trace_binder_inherit_ux(from_task, thread_task, ots->ux_depth, ots->ux_state,
		type, ots->binder_async_ux_sts, sync, "async_set after set");

	oplus_binder_debug(LOG_SET_ASYNC_UX, "async_set_ux after set, thread(pid=%d tgid=%d comm=%s) ux_sts=%d ux_state=%d ux_depth=%d inherit_ux=%lld\n",
		thread_task->pid, thread_task->tgid, thread_task->comm,
		ots->binder_async_ux_sts, ots->ux_state, ots->ux_depth, atomic64_read(&ots->inherit_ux));
	binder_ux_state_systrace(current, thread_task, STATE_ASYNC_SET_UX, LOG_BINDER_SYSTRACE_LVL0, t, proc);
}

static inline void binder_set_inherit_ux(struct task_struct *thread_task,
	struct task_struct *from_task, bool sync, bool is_servicemg,
	struct binder_transaction *t, struct binder_proc *proc)
{
	if (sync) {
		sync_binder_set_inherit_ux(thread_task, from_task, sync, is_servicemg, t, proc);
	} else {
		async_binder_set_inherit_ux(thread_task, from_task, sync, t, proc);
	}
}

static inline void binder_unset_inherit_ux(struct task_struct *thread_task,
	int unset_type, struct binder_transaction *t, struct binder_proc *proc)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(thread_task);
	bool is_servicemg = false;

	if (test_inherit_ux(thread_task, INHERIT_UX_BINDER)) {
		if (!IS_ERR_OR_NULL(ots)) {
			trace_binder_inherit_ux(NULL, thread_task, ots->ux_depth, ots->ux_state,
				INVALID_VALUE, ots->binder_async_ux_sts,
				unset_type, "unset_ux before unset");
		}

		if (sync_use_t_vendordata && (unset_type == SYNC_UNSET)) {
			is_servicemg = is_task_servicemg(thread_task);
			if (!is_sync_t_ux_state(t, unset_type, is_servicemg)) {
				binder_ux_state_systrace(current, thread_task, STATE_SYNC_T_NOT_UNSET_UX, LOG_BINDER_SYSTRACE_LVL0, t, proc);
				return;
			}
			set_sync_t_ux_state(t, false, true, is_servicemg);
		}
		unset_inherit_ux(thread_task, INHERIT_UX_BINDER);
		if (!IS_ERR_OR_NULL(ots)) {
			if (unset_type == SYNC_OR_ASYNC_UNSET) {
				set_task_async_ux_sts(ots, false);
			}
			trace_binder_inherit_ux(NULL, thread_task, ots->ux_depth, ots->ux_state,
				INVALID_VALUE, ots->binder_async_ux_sts, unset_type, "unset_ux after unset");
			if (unset_type == SYNC_OR_ASYNC_UNSET) {
				oplus_binder_debug(LOG_SET_ASYNC_UX, "sync || async_unset_ux after unset, thread(pid = %d tgid = %d comm = %s) \
					 ots_enable = %d ux_sts = %d ux_state = %d ux_depth = %d inherit_ux = %lld\n",
					thread_task->pid, thread_task->tgid, thread_task->comm, ots->binder_async_ux_enable,
					ots->binder_async_ux_sts, ots->ux_state, ots->ux_depth, atomic64_read(&ots->inherit_ux));
			} else {
				oplus_binder_debug(LOG_SET_SYNC_UX, "sync_unset_ux after unset, thread(pid = %d tgid = %d comm = %s) \
					 ux_state = %d ux_depth = %d inherit_ux = %lld\n",
					thread_task->pid, thread_task->tgid, thread_task->comm,
					ots->ux_state, ots->ux_depth, atomic64_read(&ots->inherit_ux));
			}
		}
		if (unset_type == SYNC_UNSET)
			binder_ux_state_systrace(current, thread_task, STATE_SYNC_UNSET_UX, LOG_BINDER_SYSTRACE_LVL0, t, proc);
		else
			binder_ux_state_systrace(current, thread_task, STATE_SYNC_OR_ASYNC_UNSET_UX, LOG_BINDER_SYSTRACE_LVL0, t, proc);
	} else {
		trace_binder_inherit_ux(NULL, thread_task, INVALID_VALUE, INVALID_VALUE,
			INVALID_VALUE, INVALID_VALUE, unset_type, "unset_ux do nothing");
	}
}

#else /* CONFIG_OPLUS_FEATURE_SCHED_ASSIST */
static inline void binder_set_inherit_ux(struct task_struct *thread_task, struct task_struct *from_task,
	bool sync, bool is_servicemg, struct binder_transaction *t, struct binder_proc *proc)
{
}

static inline void binder_unset_inherit_ux(struct task_struct *thread_task,
	int unset_type, struct binder_transaction *t, struct binder_proc *proc)
{
}
#endif

/* implement vender hook in driver/android/binder.c */
void android_vh_binder_restore_priority_handler(void *unused,
	struct binder_transaction *t, struct task_struct *task)
{
	/* Google commit "d1367b5" caused this priority pass issue on our kernel-5.15 project */
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 15, 0))
	if (t != NULL) {
		struct binder_priority *sp = &t->saved_priority;
		if (task->prio < MAX_RT_PRIO && !sp->prio && !sp->sched_policy) {
			sp->sched_policy = task->policy;
			sp->prio = task->prio;
		}
	}
#endif

	if (unlikely(!g_sched_enable))
		return;

	if (!is_task_servicemg(task)) {
		return;
	}

	if (t) {
		trace_binder_ux_task(1, INVALID_VALUE, INVALID_VALUE, task,
			INVALID_VALUE, t, NULL, "sync_ux unset binder_reply");
		binder_unset_inherit_ux(task, SYNC_UNSET, t, NULL);
	} else {
		trace_binder_ux_task(1, INVALID_VALUE, INVALID_VALUE, task,
			INVALID_VALUE, t, NULL, "sync_ux unset restore_prio t NULL");
		binder_unset_inherit_ux(task, SYNC_OR_ASYNC_UNSET, t, NULL);
	}
}

void android_vh_binder_wait_for_work_handler(void *unused,
			bool do_proc_work, struct binder_thread *tsk, struct binder_proc *proc)
{
	if (unlikely(!g_sched_enable))
		return;

	if (do_proc_work) {
		trace_binder_ux_task(1, INVALID_VALUE, INVALID_VALUE, tsk->task, INVALID_VALUE,
			NULL, NULL, "sync_ux unset wait_for_work");
		binder_unset_inherit_ux(tsk->task, SYNC_OR_ASYNC_UNSET, NULL, proc);
	}
}

void android_vh_sync_txn_recvd_handler(void *unused,
	struct task_struct *tsk, struct task_struct *from)
{
	if (unlikely(!g_sched_enable))
		return;

	trace_binder_ux_task(1, INVALID_VALUE, INVALID_VALUE, tsk, INVALID_VALUE,
		NULL, NULL, "sync_ux set txn_recvd");
	binder_set_inherit_ux(tsk, from, true, false, NULL, NULL);
}

#ifdef CONFIG_OPLUS_BINDER_PRIO_SKIP
static void android_vh_binder_priority_skip_handler(void *unused,
	struct task_struct *task, bool *skip)
{
	if (task->prio < MAX_RT_PRIO)
		*skip = true;
}
#endif

static int async_ux_test_debug(void)
{
	static unsigned int count = 0;
	unsigned int remainder = 0;
	int ret = 0;

	if (async_ux_test == ASYNC_UX_TEST_DISABLE) {
		return 0;
	}

	switch(async_ux_test) {
	case ASYNC_UX_RANDOM_LOW_INSERT_TEST:
		get_random_bytes(&count, sizeof(unsigned int));
		ret = (count % (2 * ASYNC_UX_ENABLE_MAX));
		break;
	case ASYNC_UX_RANDOM_HIGH_INSERT_TEST:
		get_random_bytes(&count, sizeof(unsigned int));
		ret = (count % ASYNC_UX_ENABLE_MAX);
		break;
	case ASYNC_UX_RANDOM_LOW_ENQUEUE_TEST:
		get_random_bytes(&count, sizeof(unsigned int));
		ret = (count % (2 * ASYNC_UX_ENABLE_MAX));
		if (ret > ASYNC_UX_ENABLE_ENQUEUE) {
			ret = 0;
		}
		break;
	case ASYNC_UX_RANDOM_HIGH_ENQUEUE_TEST:
		get_random_bytes(&count, sizeof(unsigned int));
		ret = (count % (ASYNC_UX_ENABLE_ENQUEUE + 1));
		break;
	case ASYNC_UX_INORDER_TEST:
		count++;
		remainder = count % 10;
		if (remainder == 1 || remainder == 5) {
			ret = ASYNC_UX_ENABLE_ENQUEUE;
		} else if (remainder == 2 || remainder == 6 || remainder == 8) {
			ret = ASYNC_UX_ENABLE_INSERT_QUEUE;
		} else {
			ret = ASYNC_UX_DISABLE;
		}
		break;
	default:
		ret = 0;
		break;
	}
	if (ret >= ASYNC_UX_ENABLE_MAX) {
		ret = 0;
	}
	return ret;
}

static bool is_allow_sf_binder_ux(struct task_struct *task)
{
	struct oplus_task_struct *ots = NULL;

	ots = get_oplus_task_struct(task);
	if (!IS_ERR_OR_NULL(ots) && test_bit(IM_FLAG_SURFACEFLINGER, &ots->im_flag)) {
		return true;
	} else {
		return false;
	}
}

static void android_vh_alloc_oem_binder_struct_handler(void *unused,
	struct binder_transaction_data *tr, struct binder_transaction *t, struct binder_proc *target_proc)
{
	struct oplus_binder_struct *obs = NULL;
	struct oplus_task_struct *ots = NULL;
	int async_ux_enable = 0, test_debug = 0;

	if (unlikely(!g_sched_enable) || unlikely(!g_async_ux_enable)) {
		return;
	}
	if (IS_ERR_OR_NULL(tr) || IS_ERR_OR_NULL(t)) {
		trace_binder_ux_enable(current, async_ux_enable, t,
			obs, "tr_buf t/tr err");
		return;
	}

	if (binder_is_sync_mode(tr->flags)) {
		return;
	}

	obs = get_oplus_binder_struct(t, false);
	if (is_obs_valid(obs) == OBS_NOT_ASYNC_UX) {
		trace_binder_ux_enable(current, async_ux_enable, t,
			obs, "tr NOT_ASYNC_UX");
		return;
	}

	if (is_obs_valid(obs) == OBS_VALID && obs->async_ux_enable) {
		trace_binder_ux_enable(current, obs->async_ux_enable, t,
			obs, "tr async_ux has enable");
		return;
	}

	ots = get_oplus_task_struct(current);
	if (IS_ERR_OR_NULL(ots)) {
		trace_binder_ux_enable(current, INVALID_VALUE, t,
			obs, "ots null");
		return;
	}

	if (ots->binder_async_ux_enable) {
		async_ux_enable = ots->binder_async_ux_enable;
	} else if (is_allow_sf_binder_ux(current)) {
		async_ux_enable = 1;
		binder_ux_state_systrace(current, target_proc ? target_proc->tsk : NULL,
			STATE_SF_ASYNC_IS_UX, LOG_BINDER_SYSTRACE_LVL0, t, NULL);
	}
	test_debug = async_ux_test_debug();
	if (async_ux_enable || test_debug) {
		obs = get_oplus_binder_struct(t, true);
		if (is_obs_valid(obs) == OBS_VALID) {
			obs->async_ux_enable = async_ux_enable ? async_ux_enable : test_debug;
			trace_binder_ux_enable(current, obs->async_ux_enable, t,
				obs, "tr async_ux enable");
		}
	} else {
		trace_binder_ux_enable(current, async_ux_enable, t,
				obs, "tr async_ux not enable");
		set_obs_not_async_ux(t, obs);
	}
}

/* sync mode unset_ux: pls refer to android_vh_sync_txn_recvd_handler / android_vh_binder_priority_skip_handler / android_vh_binder_wait_for_work_handler  */
static void sync_mode_unset_ux(struct binder_transaction *t,
				struct binder_proc *proc, struct binder_thread *thread, bool finished)
{
}

static void async_mode_unset_ux(struct binder_transaction *t,
				struct binder_proc *proc, struct binder_thread *thread, bool finished)
{
	struct oplus_binder_struct *obs = NULL;
	struct oplus_task_struct *ots = NULL;
	struct task_struct *task = NULL;

	if (unlikely(!g_sched_enable) || unlikely(!g_async_ux_enable)
		|| likely(unset_async_ux_inrestore)) {
		return;
	}

	if (IS_ERR_OR_NULL(thread) || IS_ERR_OR_NULL(thread->task)) {
		return;
	}

	task = thread->task;
	ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots)) {
		trace_binder_ux_task(false, INVALID_VALUE, INVALID_VALUE, task,
			INVALID_VALUE, t, NULL, "async_ux ots null, go");
	} else if (!get_task_async_ux_sts(ots)) {
		trace_binder_ux_task(false, INVALID_VALUE, INVALID_VALUE, task,
			INVALID_VALUE, t, NULL, "async_ux unset sts false");
		return;
	} else {
		trace_binder_ux_task(false, INVALID_VALUE, INVALID_VALUE, task,
			INVALID_VALUE, t, NULL, "async_ux unset sts true");
	}

	if (finished) {	/* t has been freed */
		binder_unset_inherit_ux(task, SYNC_OR_ASYNC_UNSET, t, proc);
		trace_binder_ux_task(false, INVALID_VALUE, INVALID_VALUE, task,
			INVALID_VALUE, t, NULL, "async_ux unset [finished]");
		return;
	}
	if (IS_ERR_OR_NULL(t)) {
		return;
	}
	obs = get_oplus_binder_struct(t, false);
	if (is_obs_valid(obs) != OBS_VALID) {
		trace_binder_ux_task(0, INVALID_VALUE, INVALID_VALUE, NULL, INVALID_VALUE,
			t, obs, "async_ux obs invalid");
		return;
	}
	if (obs->async_ux_enable == ASYNC_UX_DISABLE) {
		return;
	}
	obs->async_ux_enable = ASYNC_UX_DISABLE;
	binder_unset_inherit_ux(task, SYNC_OR_ASYNC_UNSET, t, proc);
	trace_binder_ux_task(false, INVALID_VALUE, INVALID_VALUE, task,
		obs->async_ux_enable, t, obs, "async_ux unset not-finished");
}

static void set_binder_thread_node(struct binder_transaction *t,
	struct task_struct *task, struct binder_buffer *buffer, bool sync, bool reset)
{
	struct oplus_task_struct *ots = NULL;
	struct binder_node *node = NULL;
	bool set_node = false;

	if (unlikely(!g_sched_enable) || unlikely(!g_async_ux_enable) || (!g_set_last_async_ux)) {
		return;
	}
	if (sync) {	/* don't use t->flags here because t maybe NULL */
		return;
	}
	if (IS_ERR_OR_NULL(task)) {
		return;
	}

	if (t && !IS_ERR_OR_NULL(t->buffer)) {
		node = t->buffer->target_node;
	}
	ots = get_oplus_task_struct(task);
	if (!IS_ERR_OR_NULL(ots)) {
		oplus_binder_debug(LOG_TRACK_ASYNC_NODE, "before, thread(pid:%d tgid:%d comm:%s) sync:%d, reset:%d, ots_node:0x%llx, node:0x%llx t:%d\n",
			task->pid, task->tgid, task->comm, sync, reset, (unsigned long long)ots->binder_thread_node,
			(unsigned long long)node, t ? t->debug_id : 0);
		if (reset) {
			ots->binder_thread_node = NULL;
			set_node = true;
			trace_set_thread_node(task, NULL, sync, "async reset");
		} else if (ots->binder_thread_node != node) {
			ots->binder_thread_node = node;
			set_node = true;
			trace_set_thread_node(task, node, sync, "async set");
		}
		oplus_binder_debug(LOG_TRACK_ASYNC_NODE, "after, thread(pid:%d tgid:%d comm:%s) sync:%d, reset:%d, ots_node:0x%llx, node:0x%llx, set_node:%d t:%d\n",
			task->pid, task->tgid, task->comm, sync, reset, (unsigned long long)ots->binder_thread_node, (unsigned long long)node,
			set_node,  t ? t->debug_id : 0);
	} else {
		trace_set_thread_node(task, NULL, sync, "ots null");
	}
}

static void set_thread_node_when_br_received(struct binder_transaction *t, struct binder_thread *thread)
{
	struct task_struct *task = NULL;

	if (!g_set_last_async_ux) {
		return;
	}
	if (IS_ERR_OR_NULL(t) || IS_ERR_OR_NULL(thread) || IS_ERR_OR_NULL(thread->task)) {
		return;
	}
	task = thread->task;
	trace_set_thread_node(task, NULL, INVALID_VALUE, "set when br_received");
	oplus_binder_debug(LOG_TRACK_ASYNC_NODE, "set node when transaction_received t=%d\n", t->debug_id);
	set_binder_thread_node(t, task, NULL, false, false);
}

static void android_vh_binder_transaction_received_handler(void *unused,
	struct binder_transaction *t, struct binder_proc *proc, struct binder_thread *thread, uint32_t cmd)
{
	if (unlikely(!g_sched_enable) || unlikely(!g_async_ux_enable)) {
		return;
	}
	if(!strncmp(proc->tsk->comm, SYSTEM_SERVER_NAME, TASK_COMM_LEN)) {
		if(t->debug_id == insert_limit[NUM_INSERT_ID1]) {
			insert_limit[NUM_INSERT_ID1] = 0;
		} else if (t->debug_id == insert_limit[NUM_INSERT_ID2]) {
			insert_limit[NUM_INSERT_ID2] = 0;
		}
	}
	if (binder_is_sync_mode(t->flags)) {
		return;
	}
	set_thread_node_when_br_received(t, thread);
}

static void android_vh_binder_buffer_release_handler(void *unused,
	struct binder_proc *proc, struct binder_thread *thread, struct binder_buffer *buffer, bool buffer_t_present)
{
	if (unlikely(!g_sched_enable) || unlikely(!g_async_ux_enable)) {
		return;
	}

	if (IS_ERR_OR_NULL(thread) || IS_ERR_OR_NULL(thread->task)) {
		return;
	}
	if (buffer->async_transaction) {
		async_mode_unset_ux(buffer->transaction, proc, thread, true);
		set_binder_thread_node(NULL, thread->task, buffer, false, true);
		trace_binder_free_buf(proc, thread, buffer, "async mode");
	} else {
		sync_mode_unset_ux(buffer->transaction, proc, thread, true);
		set_binder_thread_node(NULL, thread->task, buffer, true, true);
		trace_binder_free_buf(proc, thread, buffer, "sync mode");
	}
}

static void android_vh_free_oplus_binder_struct_handler(void *unused, struct binder_transaction *t)
{
	struct oplus_binder_struct *obs = (struct oplus_binder_struct *)(t->android_vendor_data1);

	if (unlikely(!g_async_ux_enable)) {
		return;
	}

	if (binder_is_sync_mode(t->flags)) {
		return;
	}
	trace_binder_t_obs(t, obs, "free_obs");
	free_oplus_binder_struct(obs);
	t->android_vendor_data1 = 0;
}

static bool binder_dynamic_enqueue_work_ilocked(struct binder_work *work,
		struct list_head *target_list, bool sync_insert)
{
	struct binder_work *w = NULL;
	struct binder_transaction *t = NULL;
	struct oplus_binder_struct *obs = NULL;
	bool insert = false;
	int i = 0;

	if (unlikely(!g_sched_enable) || unlikely(!g_async_ux_enable)) {
		return false;
	}

	trace_binder_ux_work(work, target_list, NULL, insert, i, "dynamic begin");
	if(insert_limit[NUM_INSERT_ID1] && insert_limit[NUM_INSERT_ID2]) {
		trace_binder_ux_work(work, target_list, NULL, insert, i, "dynamic break");
		return false;
	}

	BUG_ON(target_list == NULL);
	BUG_ON(work->entry.next && !list_empty(&work->entry));

	list_for_each_entry(w, target_list, entry) {
		i++;
		if (i > MAX_UX_IN_LIST) {
			insert = false;
			break;
		}
		if (IS_ERR_OR_NULL(w)) {
			break;
		}

		if (w->type != BINDER_WORK_TRANSACTION) {
			continue;
		}

		t = container_of(w, struct binder_transaction, work);
		if (IS_ERR_OR_NULL(t)) {
			break;
		}
		if (sync_insert) {
			if (!binder_is_sync_mode(t->flags)) {
				continue;
			}
			if (!t->from) {
				continue;
			}
			if ((test_task_ux(t->from->task) || test_task_is_rt(t->from->task))) {
				continue;
			}
			binder_ux_state_systrace(current, NULL, STATE_SYNC_INSERT_QUEUE, LOG_BINDER_SYSTRACE_LVL0, t, NULL);
		} else {
			if (binder_is_sync_mode(t->flags)) {
				continue;
			}
			obs = get_oplus_binder_struct(t, false);
			if (is_obs_valid(obs) != OBS_VALID) {
				insert = true;
				break;
			}
			if (obs->async_ux_enable) {
				continue;
			}
			binder_ux_state_systrace(current, NULL, STATE_ASYNC_INSERT_QUEUE, LOG_BINDER_SYSTRACE_LVL0, t, NULL);
		}

		insert = true;
		break;
	}

	if (insert && !IS_ERR_OR_NULL(w) && !IS_ERR_OR_NULL(&w->entry)) {
		list_add(&work->entry, &w->entry);
		if(!insert_limit[NUM_INSERT_ID1] && (t->debug_id != insert_limit[NUM_INSERT_ID2])) {
			insert_limit[NUM_INSERT_ID1] = t->debug_id;
		} else if (!insert_limit[NUM_INSERT_ID2] && (t->debug_id != insert_limit[NUM_INSERT_ID1])) {
			insert_limit[NUM_INSERT_ID2] = t->debug_id;
		}
	} else {
		list_add_tail(&work->entry, target_list);
	}
	trace_binder_ux_work(work, target_list, IS_ERR_OR_NULL(w) ? NULL : &w->entry, insert, i, "dynamic end");
	return true;
}

static void android_vh_binder_special_task_handler(void *unused, struct binder_transaction *t,
	struct binder_proc *proc, struct binder_thread *thread, struct binder_work *w,
	struct list_head *target_list, bool sync, bool *enqueue_task)
{
	struct oplus_binder_struct *obs = NULL;
	bool allow_sync_insert = false;

	if (unlikely(!g_sched_enable) || unlikely(!g_async_ux_enable)
		|| unlikely(!async_insert_queue)) {
		return;
	}

	if (sync) {
		/* called by binder_proc_transaction() when no binder_thread selected */
		if (sync_insert_queue && t && proc && (&proc->todo == target_list)) {
			if(proc->tsk && t->from
				&& (test_set_inherit_ux(t->from->task) || test_task_is_rt(t->from->task))
				&& !strncmp(proc->tsk->comm, SYSTEM_SERVER_NAME, TASK_COMM_LEN)) {
				allow_sync_insert = true;
				goto dynamic_enqueue;
			}
		}
		return;
	}

	if (unlikely(!async_insert_queue)) {
		return;
	}

	if (!w || !target_list) {
		return;
	}

	if (!t && w) {
		t = container_of(w, struct binder_transaction, work);
	}
	obs = get_oplus_binder_struct(t, false);
	if (is_obs_valid(obs) != OBS_VALID) {
		return;
	}
dynamic_enqueue:
	if ((obs && obs->async_ux_enable == ASYNC_UX_ENABLE_INSERT_QUEUE) || allow_sync_insert) {
		if (binder_dynamic_enqueue_work_ilocked(w, target_list, allow_sync_insert)) {
			/*
			if enqueue_task == false, binder_dynamic_enqueue_work_ilocked list_add_xxx is called,
			don't call binder.c binder_enqueue_work_ilocked() again.
			*/
			*enqueue_task = false;
		}
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)

static bool sync_mode_check_ux(struct binder_proc *proc,
		struct binder_transaction *t, struct task_struct *binder_th_task, bool sync)
{
	struct task_struct *binder_proc_task = proc->tsk;
	bool set_ux = true;

	if (unlikely(!g_sched_enable)) {
		return false;
	}

	if (is_task_servicemg(binder_proc_task)) {
		binder_set_inherit_ux(binder_proc_task, current, sync, true, t, proc);
		trace_binder_proc_thread(binder_proc_task, binder_th_task, sync, INVALID_VALUE, t, proc,
			"sync_ux set servicemg");
	}

	if (!binder_th_task)
		return false;

	trace_binder_proc_thread(binder_proc_task, binder_th_task, sync, INVALID_VALUE, t, proc,
		"sync_ux set ux");

	return set_ux;
}

#define CHECK_MAX_NODE_FOR_ASYNC_THREAD		400
static struct task_struct *get_same_node_task(struct binder_proc *proc,
	struct binder_node *node, struct binder_transaction *t)
{
	struct binder_thread *thread = NULL;
	struct task_struct *task = NULL;
	struct task_struct *select_task = NULL;
	struct oplus_task_struct *ots = NULL;
	struct rb_node *n = NULL;
	bool has_async = true;
	int count = 0;

	if (!proc || !node) {
		return NULL;
	}
	for (n = rb_first(&proc->threads); n != NULL; n = rb_next(n)) {
		thread = rb_entry(n, struct binder_thread, rb_node);
		task = thread->task;
		ots = get_oplus_task_struct(task);
		if (!IS_ERR_OR_NULL(ots) && (ots->binder_thread_node == node)) {
			select_task = task;
			goto end;
		}

		if (node->has_async_transaction == false) {
			has_async = false;
			break;
		}
		count++;
		if (count > CHECK_MAX_NODE_FOR_ASYNC_THREAD) {
			break;
		}
		if ((g_sched_debug & LOG_TRACK_LAST_ASYNC) && !IS_ERR_OR_NULL(ots) && task) {
			oplus_binder_debug(LOG_TRACK_LAST_ASYNC, "get_same_node t:%d proc(pid:%d tgid:%d comm:%s) task(pid:%d tgid:%d comm:%s) \
				max_threads:%d request:%d started:%d count:%d ots_node:%d node:%d\n",
				t ? t->debug_id : 0, proc->tsk->pid, proc->tsk->tgid, proc->tsk->comm, task->pid, task->tgid,
				task->comm, proc->max_threads, proc->requested_threads, proc->requested_threads_started,
				count, (ots->binder_thread_node ? ots->binder_thread_node->debug_id : 0), node->debug_id);
		}
	}

end:
	oplus_binder_debug(LOG_GET_LAST_ASYNC, "get_same_node end t:%d proc(pid:%d tgid:%d comm:%s) task(pid:%d tgid:%d comm:%s) \
		max_threads:%d request:%d started:%d count:%d ots_node:%d node:%d, has_async:%d node get_result:%d\n",
		t ? t->debug_id : 0, proc->tsk->pid, proc->tsk->tgid, proc->tsk->comm,
		select_task ? task->pid : 0, select_task ? task->tgid : 0,
		select_task ? task->comm : "null", proc->max_threads, proc->requested_threads,
		proc->requested_threads_started, count,
		(ots->binder_thread_node ? ots->binder_thread_node->debug_id : 0), node->debug_id,
		has_async, select_task ? true : false);

	return select_task;
}

static inline bool is_binder_thread(struct binder_proc *proc, struct task_struct *task)
{
	int proc_context = 0;
	bool ret = false;

	if (!task || !proc || !proc->context) {
		return false;
	}
	if (!strcmp(proc->context->name, BINDER_NAME)) {
		proc_context = PROC_CONTEXT_BINDER;
	} else if (!strcmp(proc->context->name, HWBINDER_NAME)) {
		proc_context = PROC_CONTEXT_HWBINDER;
	} else if (!strcmp(proc->context->name, VNDBINDER_NAME)) {
		proc_context = PROC_CONTEXT_VNDBINDER;
	}

	if ((proc_context == PROC_CONTEXT_BINDER &&
		(strstr(task->comm, BINDER_THREAD_NAME) && !strstr(task->comm, VNDBINER_THREAD_NAME)))
		|| (proc_context == PROC_CONTEXT_HWBINDER && strstr(task->comm, HWBINDER_THREAD_NAME))
		|| (proc_context == PROC_CONTEXT_VNDBINDER && strstr(task->comm, VNDBINER_THREAD_NAME))) {
		ret = true;
	} else {
		ret = false;
	}
	oplus_binder_debug(LOG_TRACK_LAST_ASYNC, "is_binder_thread:%d, proc_context:%d \
		proc(pid:%d comm:%s) task(pid:%d, tgid:%d comm:%s)\n", ret, proc_context,
		proc->tsk->pid, proc->tsk->comm, task->pid, task->tgid, task->comm);

	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
#define GET_STATE(t) (t->__state)
#else
#define GET_STATE(t) (t->state)
#endif

/* get same node task fail, maybe in proc->todo list, not in thread, select one (not_rt && not_ux && running) binder task */
static struct task_struct *get_proc_lowprio_binder_task(struct binder_proc *proc,
	struct binder_node *node, struct binder_transaction *t)
{
	struct binder_thread *thread = NULL;
	struct task_struct *task = NULL;
	struct task_struct *select_task = NULL;
	struct rb_node *n = NULL;
	int ux_type = 0;
	int binder_thread_count = 0;
	static int ux_thread = 0;
	static int not_ux_thread = 0;
	static int proc_allthread_ux = 0;
	static int proc_not_allthread_ux = 0;
	static int not_ux_not_running = 0;
	bool allthread_is_ux = false;
	bool has_async = true;
	int count = 0;

	if (!get_random_binder_task) {
		return NULL;
	}
	if (!proc) {
		return NULL;
	}

	/* for debug */
	if (g_sched_debug & LOG_TRACK_LAST_ASYNC) {
		for (n = rb_first(&proc->threads); n != NULL; n = rb_next(n)) {
			thread = rb_entry(n, struct binder_thread, rb_node);
			task = thread->task;
			if (is_binder_thread(proc, task)) {
				ux_type = get_ux_state_type(task);
				if (ux_type == UX_STATE_NONE) {
					ux_thread++;
				} else {
					not_ux_thread++;
				}
			}
			count++;
			if (count > CHECK_MAX_NODE_FOR_ASYNC_THREAD) {
				break;
			}
		}
		if (not_ux_thread)
			proc_not_allthread_ux++;
		else {
			proc_allthread_ux++;
			allthread_is_ux = true;
		}
	}

	count = 0;
	binder_thread_count = 0;
	for (n = rb_first(&proc->threads); n != NULL; n = rb_next(n)) {
		thread = rb_entry(n, struct binder_thread, rb_node);
		task = thread->task;
		if (is_binder_thread(proc, task)) {
			if (ux_type == UX_STATE_NONE) {
				if (GET_STATE(task) == TASK_RUNNING) {
					select_task = task;
					goto end;
				} else if (g_sched_debug & LOG_GET_LAST_ASYNC) {
					not_ux_not_running++;
				}
			}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
			oplus_binder_debug(LOG_TRACK_LAST_ASYNC, "get_binder_task t:%d proc(pid:%d tgid:%d comm:%s) task(pid:%d tgid:%d comm:%s) \
				max_threads:%d request:%d started:%d count:%d node_id:%d binder_thread_count:%d ux_type:%d ux_state:%d tsk_state:0x%x\n",
				t ? t->debug_id : 0, proc->tsk->pid, proc->tsk->tgid, proc->tsk->comm, task->pid, task->tgid, task->comm,
				proc->max_threads, proc->requested_threads, proc->requested_threads_started,
				count, node ? node->debug_id : 0, binder_thread_count, ux_type, oplus_get_ux_state(task), GET_STATE(task));
#else
			oplus_binder_debug(LOG_TRACK_LAST_ASYNC, "get_binder_task t:%d proc(pid:%d tgid:%d comm:%s) task(pid:%d tgid:%d comm:%s) \
				max_threads:%d request:%d started:%d count:%d node_id:%d binder_thread_count:%d ux_type:%d ux_state:%d tsk_state:0x%lx\n",
				t ? t->debug_id : 0, proc->tsk->pid, proc->tsk->tgid, proc->tsk->comm, task->pid, task->tgid, task->comm,
				proc->max_threads, proc->requested_threads, proc->requested_threads_started,
				count, node ? node->debug_id : 0, binder_thread_count, ux_type, oplus_get_ux_state(task), GET_STATE(task));
#endif
			if (g_sched_debug & LOG_BINDER_SYSTRACE_LVL1) {
				binder_ux_state_systrace(current, task, STATE_TASK_STRUCT_STATE + GET_STATE(task),
					LOG_BINDER_SYSTRACE_LVL1, t, proc);
			}
			binder_thread_count++;
			if (binder_thread_count >= proc->requested_threads_started) {
				break;
			}
		}

		if (node && (node->has_async_transaction == false)) {
			has_async = false;
			break;
		}
		count++;
		if (count > CHECK_MAX_NODE_FOR_ASYNC_THREAD) {
			break;
		}
	}

end:
	oplus_binder_debug(LOG_GET_LAST_ASYNC, "get_binder_task end t:%d proc(pid:%d tgid:%d comm:%s) task(pid:%d tgid:%d comm:%s) \
		max_threads:%d request:%d started:%d count:%d node_id:%d binder_thread_count:%d ux_state:%d, has_async:%d ux_thread:%d \
		not_ux_thread:%d proc_allthread_ux:%d proc_not_allthread_ux:%d allthread_is_ux:%d not_ux_not_running:%d task get_result:%d\n",
		t ? t->debug_id : 0, proc->tsk->pid, proc->tsk->tgid, proc->tsk->comm,
		select_task ? task->pid : 0, select_task ? task->tgid : 0, select_task ? task->comm : "null",
		proc->max_threads, proc->requested_threads, proc->requested_threads_started,
		count, node ? node->debug_id : 0, binder_thread_count, oplus_get_ux_state(select_task), has_async,
		ux_thread, not_ux_thread, proc_allthread_ux, proc_not_allthread_ux, allthread_is_ux, not_ux_not_running,
		select_task ? true : false);

	return select_task;
}

static struct task_struct *get_current_async_thread(struct binder_transaction *t, struct binder_proc *proc)
{
	struct task_struct *select_task = NULL;
	struct binder_node *node = NULL;
	ktime_t time = 0;
	static int get_same_node = 0;
	static int get_lowprio_binder = 0;
	static int notget_lowprio_binder = 0;
	static int get_task = 0;
	static int not_get = 0;

	if (unlikely(!g_set_last_async_ux)) {
		return NULL;
	}

	if (t && t->buffer) {
		node = t->buffer->target_node;
	}
	if (!node || !proc) {
		return NULL;
	}
	if (g_sched_debug & LOG_GET_LAST_ASYNC) {
		time = ktime_get();
	}

	select_task = get_same_node_task(proc, node, t);
	if (select_task) {
		if (g_sched_debug & LOG_GET_LAST_ASYNC) {
			get_same_node++;
		}
		goto end;
	}
	select_task = get_proc_lowprio_binder_task(proc, node, t);
	if (g_sched_debug & LOG_GET_LAST_ASYNC) {
		if (select_task)
			get_lowprio_binder++;
		else
			notget_lowprio_binder++;	/* maybe all rt /all ux / no binder thread / not running / has_async false  */
	}

end:
	if (g_sched_debug & LOG_GET_LAST_ASYNC) {
		time = ktime_get() - time;
		if (select_task)
			get_task++;
		else
			not_get++;
	}
	oplus_binder_debug(LOG_GET_LAST_ASYNC, "t:%d current(pid:%d tgid:%d comm:%s) proc(pid:%d tgid:%d comm:%s) \
		task(pid:%d tgid:%d comm:%s) max_threads:%d request:%d started:%d node:%d get_same_node:%d \
		get_lowprio_binder:%d notget_lowprio_binder:%d time:%lldns final get_result:%d get_task:%d not_get:%d\n",
		t ? t->debug_id : 0, current->pid, current->tgid, current->comm,
		proc->tsk->pid, proc->tsk->tgid, proc->tsk->comm,
		select_task ? select_task->pid : 0, select_task ? select_task->tgid : 0,
		select_task ? select_task->comm : "null", proc->max_threads, proc->requested_threads,
		proc->requested_threads_started, node->debug_id, get_same_node, get_lowprio_binder,
		notget_lowprio_binder, time, select_task ? true : false, get_task, not_get);

	return select_task;
}

static bool async_mode_check_ux(struct binder_proc *proc, struct binder_transaction *t,
		struct task_struct *binder_th_task, bool sync, bool pending_async,
		struct task_struct **last_task, bool *force_sync)
{
	struct oplus_binder_struct *obs = NULL;
	struct task_struct *ux_task = binder_th_task;
	bool set_ux = false;
	if (unlikely(!g_sched_enable)) {
		return false;
	}

	if (unlikely(!g_async_ux_enable)) {
		if (is_allow_sf_binder_ux(current)) {
			set_ux = true;
			*force_sync = true;
		}
		return set_ux;
	}
	obs = get_oplus_binder_struct(t, false);
	if (is_obs_valid(obs) != OBS_VALID) {
		set_ux = false;
		trace_binder_ux_task(sync, pending_async, set_ux, ux_task, INVALID_VALUE,
			t, obs, "async_ux obs invalid");
		goto end;
	}

	if (obs->async_ux_enable == ASYNC_UX_DISABLE) {
		set_ux = false;
		trace_binder_ux_task(sync, pending_async, set_ux, ux_task, obs->async_ux_enable,
			t, obs, "async_ux not enable");
		goto end;
	}

	if (ux_task) {
		set_ux = true;
		trace_binder_ux_task(sync, pending_async, set_ux, ux_task, obs->async_ux_enable,
			t, obs, "async_ux set ux");
		goto end;
	}

	/* pending_async, no binder_th_task */
	if (pending_async) {
		ux_task = get_current_async_thread(t, proc);
		if (ux_task) {
			*last_task = ux_task;
			set_ux = true;
			binder_ux_state_systrace(current, ux_task, STATE_ASYNC_SET_LAST_UX, LOG_BINDER_SYSTRACE_LVL0, t, proc);
		} else {
			set_ux = false;
			binder_ux_state_systrace(current, NULL, STATE_ASYNC_NOT_SET_LAST_UX, LOG_BINDER_SYSTRACE_LVL0, t, proc);
		}
		obs->pending_async = true;
		trace_binder_ux_task(sync, pending_async, set_ux, ux_task, obs->async_ux_enable,
			t, obs, "async_ux set last");
		goto end;
	} else {
		obs->async_ux_no_thread = true;
		binder_ux_state_systrace(current, NULL, STATE_ASYNC_NO_THREAD_NO_PENDING, LOG_BINDER_SYSTRACE_LVL0, t, proc);
	}
end:
	trace_binder_ux_task(sync, pending_async, set_ux, ux_task, INVALID_VALUE,
			t, obs, "async_ux end");
	return set_ux;
}


#else /* CONFIG_OPLUS_FEATURE_SCHED_ASSIST */

static bool sync_mode_check_ux(struct binder_proc *proc,
		struct binder_transaction *t, struct task_struct *binder_th_task, bool sync)
{
	return false;
}

static bool async_mode_check_ux(struct binder_proc *proc, struct binder_transaction *t,
	struct task_struct *binder_th_task, bool sync, bool pending_async,
	struct task_struct **last_task, bool *force_sync)
{
	return false;
}

#endif

static void android_vh_binder_set_priority_handler(void *unused,
	struct binder_transaction *t, struct task_struct *task)
{
	struct oplus_binder_struct *obs = NULL;
	struct oplus_task_struct *ots = NULL;
	bool current_is_client = false;

	if (unlikely(!g_sched_enable) || !g_set_async_ux_after_pending) {
		return;
	}
	if (IS_ERR_OR_NULL(t) || IS_ERR_OR_NULL(task)) {
		return;
	}
	if (binder_is_sync_mode(t->flags)) {
		return;
	}

	if (t->from && (t->from->task == current))
		current_is_client = true;
	if (!current_is_client &&
		(t->to_proc && t->to_proc->tsk && (t->to_proc->tsk->pid == current->tgid))) {
		oplus_binder_debug(LOG_TRACK_ASYNC_NODE, "set node when set_prio t=%d\n", t->debug_id);
		set_binder_thread_node(t, task, NULL, false, false);
	}

	obs = get_oplus_binder_struct(t, false);
	if (is_obs_valid(obs) != OBS_VALID) {
		return;
	}

	if (!obs->pending_async && !obs->async_ux_no_thread) {
		binder_ux_state_systrace(current, task, STATE_ASYNC_HAS_THREAD,
			LOG_BINDER_SYSTRACE_LVL1, t, NULL);
		return;
	}

	ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots)) {
		return;
	}
	if (oplus_get_ux_state(task) && get_task_async_ux_sts(ots)) {
		binder_ux_state_systrace(current, task, STATE_THREAD_WAS_ASYNC_UX,
			LOG_BINDER_SYSTRACE_LVL0, t, NULL);
		return;
	}

	binder_ux_state_systrace(current, task, STATE_ASYNC_SET_UX_AFTER_NO_THREAD,
		LOG_BINDER_SYSTRACE_LVL0, t, NULL);

	binder_set_inherit_ux(task, NULL, false, false, t, NULL);
	obs->pending_async = false;
	obs->async_ux_no_thread = false;

	oplus_binder_debug(LOG_SET_ASYNC_AFTER_PENDING, "thread(pid = %d tgid = %d comm = %s) \
		pending_async = %d async_ux_no_thread = %d set_async_after_nothread\n",
		task->pid, task->tgid, task->comm, obs->pending_async, obs->async_ux_no_thread);
}

static void android_vh_binder_proc_transaction_finish_handler(void *unused, struct binder_proc *proc,
		struct binder_transaction *t, struct task_struct *binder_th_task, bool pending_async, bool sync)
{
	struct task_struct *last_task = NULL;
	bool set_ux = false;
	bool force_sync = false;

	if (unlikely(!g_sched_enable))
		return;

	if (pending_async) {
		binder_ux_state_systrace(current, (proc ? proc->tsk : NULL),
			STATE_PENDING_ASYNC, LOG_BINDER_SYSTRACE_LVL0, t, proc);
	} else if (!binder_th_task) {
		binder_ux_state_systrace(current, (proc ? proc->tsk : NULL),
			STATE_NO_BINDER_THREAD, LOG_BINDER_SYSTRACE_LVL0, t, proc);
	}

	set_binder_thread_node(t, binder_th_task, NULL, sync, false);

	if (sync) {
		set_ux = sync_mode_check_ux(proc, t, binder_th_task, sync);
	} else {
		set_ux = async_mode_check_ux(proc, t, binder_th_task, sync,
			pending_async, &last_task, &force_sync);
	}

#if IS_ENABLED(CONFIG_ANDROID_BINDER_IPC_VIP_THREAD)
	if (t->flags & TF_TAXI_UX_WAY) {
		set_ux = true;
	}
#endif
	if (set_ux) {
		if (force_sync) {
			binder_set_inherit_ux(binder_th_task, current, true, false, t, proc);
		} else if (last_task) {
			binder_set_inherit_ux(last_task, current, sync, false, t, proc);
		} else {
			binder_set_inherit_ux(binder_th_task, current, sync, false, t, proc);
		}
	}

	if (last_task) {
		trace_binder_ux_task(sync, pending_async, set_ux, last_task,
			INVALID_VALUE, t, NULL, "ux t_finish last");
	} else {
		trace_binder_ux_task(sync, pending_async, set_ux, binder_th_task,
			INVALID_VALUE, t, NULL, "ux t_finish");
	}
}

#ifdef CONFIG_OPLUS_BINDER_REF_OPT
static void binder_alloc_desc_opt_init(struct binder_proc *proc)
{
	if (current_uid().val != SYSTEM_SERVER_UID) {
		return;
	}

	if (!system_server_proc && proc->tsk && (!strncmp(proc->tsk->comm, SYSTEM_SERVER_NAME, TASK_COMM_LEN))
		&& proc->context && proc->context->name
		&& (!strcmp(proc->context->name, BINDER_NAME))) {
			system_server_proc = proc;
			spin_lock(&binder_ref_lock);
			g_free_ref = bitmap_alloc(MAX_SYSTEM_SERVER_DESC, GFP_ATOMIC);
			if (g_free_ref) {
				bitmap_fill(g_free_ref, MAX_SYSTEM_SERVER_DESC);
				pr_info("alloc opt recognise system server ok\n");
			}
			spin_unlock(&binder_ref_lock);
	}
}

static void android_vh_binder_find_desc_handler(void *unused, struct binder_proc *proc,
		uint32_t *ref_desc, struct rb_node *nd_desc, bool *loop)
{
	struct rb_node **p = &proc->refs_by_node.rb_node;
	struct rb_node *parent = NULL;
	struct binder_ref *ref;
	uint32_t first_free_place;
	uint32_t tmp_desc;
	bool desc_add = true;

	if (unlikely(!g_ref_enable)) {
		return;
	}

	if (proc == system_server_proc) {
		spin_lock(&binder_ref_lock);
		if (g_free_ref) {
			first_free_place = find_first_bit(g_free_ref, MAX_SYSTEM_SERVER_DESC);
			if (first_free_place < MAX_SYSTEM_SERVER_DESC) {
				tmp_desc = *ref_desc;
				*ref_desc = first_free_place;
				desc_add = false;
				p = &proc->refs_by_desc.rb_node;
				trace_binder_ref_desc("obf find desc", first_free_place, proc);
				while (*p) {
					parent = *p;
					ref = rb_entry(parent, struct binder_ref, rb_node_desc);

					if (*ref_desc < ref->data.desc)
						p = &(*p)->rb_left;
					else if (*ref_desc > ref->data.desc)
						p = &(*p)->rb_right;
					else {
						desc_add = true;
						pr_err("binder ref double check fail! first_free_place=%u\n",
							first_free_place);
						kfree(g_free_ref);
						g_free_ref = NULL;
						*ref_desc = tmp_desc;
						break;
					}
				}
			}
		}

		if (desc_add) {
			spin_unlock(&binder_ref_lock);
			*loop = true;
			return;
		}

		rb_link_node(nd_desc, parent, p);
		rb_insert_color(nd_desc, &proc->refs_by_desc);

		if (g_free_ref && *ref_desc < MAX_SYSTEM_SERVER_DESC) {
			trace_binder_ref_desc("obf set desc", *ref_desc, proc);
			clear_bit(*ref_desc, g_free_ref);
		}
		spin_unlock(&binder_ref_lock);

		*loop = false;
	}
}

static void android_vh_binder_set_desc_bit_handler(void *unused, struct binder_proc *proc, uint32_t ref_desc)
{
	if (unlikely(!g_ref_enable)) {
		return;
	}

	if ((proc == system_server_proc) && ref_desc < MAX_SYSTEM_SERVER_DESC) {
		spin_lock(&binder_ref_lock);
		if(g_free_ref) {
			trace_binder_ref_desc("obf clear desc", ref_desc, NULL);
			set_bit(ref_desc, g_free_ref);
		}
		spin_unlock(&binder_ref_lock);
	}
}

static void android_vh_binder_desc_init_handler(void *unused, struct binder_proc *proc)
{
	if (unlikely(!g_ref_enable)) {
		return;
	}
	binder_alloc_desc_opt_init(proc);
}

static void android_vh_binder_free_proc_handler(void *unused,
						struct binder_proc *proc)
{
	if (unlikely(!g_ref_enable)) {
		return;
	}
	if (proc == system_server_proc) {
			spin_lock(&binder_ref_lock);
			if(g_free_ref) {
				kfree(g_free_ref);
				system_server_proc = NULL;
			}
			spin_unlock(&binder_ref_lock);
	}
}
#endif

void register_binder_sched_vendor_hooks(void)
{
	register_trace_android_vh_binder_restore_priority(
		android_vh_binder_restore_priority_handler, NULL);
	register_trace_android_vh_binder_wait_for_work(
		android_vh_binder_wait_for_work_handler, NULL);
	register_trace_android_vh_sync_txn_recvd(
		android_vh_sync_txn_recvd_handler, NULL);
#ifdef CONFIG_OPLUS_BINDER_PRIO_SKIP
	register_trace_android_vh_binder_priority_skip(
		android_vh_binder_priority_skip_handler, NULL);
#endif
	register_trace_android_vh_binder_proc_transaction_finish(
		android_vh_binder_proc_transaction_finish_handler, NULL);
	register_trace_android_vh_binder_special_task(
		android_vh_binder_special_task_handler, NULL);
	register_trace_android_vh_alloc_oem_binder_struct(
		android_vh_alloc_oem_binder_struct_handler, NULL);
	register_trace_android_vh_binder_transaction_received(
		android_vh_binder_transaction_received_handler, NULL);
	register_trace_android_vh_free_oem_binder_struct(
		android_vh_free_oplus_binder_struct_handler, NULL);
	register_trace_android_vh_binder_buffer_release(
		android_vh_binder_buffer_release_handler, NULL);
#ifdef CONFIG_OPLUS_BINDER_REF_OPT
	register_trace_android_vh_binder_find_desc(
		android_vh_binder_find_desc_handler, NULL);
	register_trace_android_vh_binder_set_desc_bit(
		android_vh_binder_set_desc_bit_handler, NULL);
	register_trace_android_vh_binder_desc_init(
		android_vh_binder_desc_init_handler, NULL);
	register_trace_android_vh_binder_free_proc(
		android_vh_binder_free_proc_handler, NULL);
#endif
	register_trace_android_vh_binder_set_priority(
		android_vh_binder_set_priority_handler, NULL);
}

static void init_oplus_binder_struct(void *ptr)
{
	struct oplus_binder_struct *obs = ptr;

	memset(obs, 0, sizeof(struct oplus_binder_struct));
}

void oplus_binder_sched_init(void)
{
	g_bd_opt_enable |= BD_BINDER_REF_OPT_ENABLE;

	parse_dts_switch();
	g_bd_opt_enable &= dynamic_switch;
	pr_err("g_bd_opt_enable : %d\n", g_bd_opt_enable);

	if(unlikely(!binder_opt_enable(BD_BINDER_REF_OPT_ENABLE))) {
#ifdef CONFIG_OPLUS_BINDER_REF_OPT
		g_ref_enable = 0;
#endif
	}

	oplus_binder_struct_cachep = kmem_cache_create("oplus_binder_struct",
		sizeof(struct oplus_binder_struct), 0, SLAB_PANIC|SLAB_ACCOUNT, init_oplus_binder_struct);

	register_binder_sched_vendor_hooks();
}

#ifdef CONFIG_OPLUS_BINDER_REF_OPT
module_param_named(binder_ref_enable, g_ref_enable, uint, 0660);
#endif
module_param_named(binder_sched_enable, g_sched_enable, uint, 0660);
module_param_named(binder_sched_debug, g_sched_debug, ullong, 0660);
module_param_named(binder_async_ux_test, async_ux_test, uint, 0660);
module_param_named(binder_ux_enable, g_async_ux_enable, int, 0664);
module_param_named(binder_async_insert_queue, async_insert_queue, int, 0664);
module_param_named(binder_sync_insert_queue, sync_insert_queue, uint, 0664);
module_param_named(binder_set_last_async_ux, g_set_last_async_ux, int, 0664);
module_param_named(binder_set_async_ux_after_pending, g_set_async_ux_after_pending, int, 0664);
module_param_named(binder_allow_accumulate_ux, allow_accumulate_ux, int, 0664);
module_param_named(binder_sync_use_t_vendordata, sync_use_t_vendordata, int, 0664);
module_param_named(binder_unset_async_ux_inrestore, unset_async_ux_inrestore, int, 0664);
module_param_named(get_random_binder_task, get_random_binder_task, int, 0664);

