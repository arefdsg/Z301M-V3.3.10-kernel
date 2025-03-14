/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include "ion_drv.h"
#include "mtk_ion.h"
#include "mt_smi.h"


#include "m4u.h"


#include <mt-plat/mt_gpio.h>
#ifdef CONFIG_MTK_CLKMGR
#include <mach/mt_clkmgr.h>
#endif

#include <linux/gpio.h>

#define mt_eint_set_hw_debounce(eint_num, ms) (void)0


#include "mtkfb.h"
#include "mtkfb_fence.h"
#include "disp_session.h"
#include "debug.h"
#include "disp_drv_log.h"
#include "disp_lcm.h"
#include "disp_utils.h"
#include "disp_drv_platform.h"
#include "display_recorder.h"
#include "primary_display.h"

#include "ddp_hal.h"
#include "ddp_dump.h"
#include "ddp_path.h"
#include "ddp_drv.h"
#include "ddp_ovl.h"
#include "ddp_manager.h"
#include "ddp_mmp.h"
#include "ddp_reg.h"
#include "ddp_debug.h"
#include "ddp_irq.h"

#include "cmdq_def.h"
#include "cmdq_record.h"
/* #include "cmdq_reg.h" */
#include "cmdq_core.h"
#ifdef DISP_ENABLE_SODI
#define DISP_REG_SODI_PA 0x10006b0c
/* for sodi reg addr define */
#include "mt_spm.h"
#include "mt_spm_idle.h"
#endif
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
#include "tz_cross/trustzone.h"
#include "tz_cross/ta_mem.h"
#include <tz_cross/tz_ddp.h>
#include "trustzone/kree/system.h"
#include "trustzone/kree/mem.h"
#endif
typedef void (*fence_release_callback) (unsigned int data);

unsigned int is_hwc_enabled = 0;


int primary_display_use_cmdq = CMDQ_DISABLE;
int primary_display_use_m4u = 1;
DISP_PRIMARY_PATH_MODE primary_display_mode = DIRECT_LINK_MODE;

static unsigned long dim_layer_mva;
/* wdma dump thread */
static unsigned int primary_dump_wdma;
static struct task_struct *primary_display_wdma_out;
static unsigned long dc_vAddr[DISP_INTERNAL_BUFFER_COUNT];
static disp_internal_buffer_info *decouple_buffer_info[DISP_INTERNAL_BUFFER_COUNT];
static RDMA_CONFIG_STRUCT decouple_rdma_config;
static WDMA_CONFIG_STRUCT decouple_wdma_config;
static disp_mem_output_config mem_config;

static unsigned int primary_session_id = MAKE_DISP_SESSION(DISP_SESSION_PRIMARY, 0);

/* primary display uses itself's abs macro */
#ifdef abs
#undef  abs
#define abs(a) (((a) < 0) ? -(a) : (a))
#endif

#define FRM_UPDATE_SEQ_CACHE_NUM (DISP_INTERNAL_BUFFER_COUNT+1)
static disp_frm_seq_info frm_update_sequence[FRM_UPDATE_SEQ_CACHE_NUM];
static unsigned int frm_update_cnt;
/* DDP_SCENARIO_ENUM ddp_scenario = DDP_SCENARIO_SUB_RDMA1_DISP; */
#ifdef DISP_SWITCH_DST_MODE
int primary_display_def_dst_mode = 0;
int primary_display_cur_dst_mode = 0;
#endif
#define ALIGN_TO(x, n)  \
	(((x) + ((n) - 1)) & ~((n) - 1))

int primary_trigger_cnt = 0;
#define PRIMARY_DISPLAY_TRIGGER_CNT (1)
unsigned int gEnterSodiAfterEOF = 0;

unsigned int WDMA0_FRAME_START_FLAG = 0;
unsigned int NEW_BUF_IDX = 0;
unsigned int ALL_LAYER_DISABLE_STEP = 0;
unsigned long long last_primary_trigger_time = 0xffffffffffffffff;

void enqueue_buffer(display_primary_path_context *ctx, struct list_head *head,
		    disp_internal_buffer_info *buf)
{
	if (ctx && head && buf) {
		list_add_tail(&buf->list, head);
		/* DISPMSG("enqueue_buffer, head=0x%08x, buf=0x%08x, mva=0x%08x\n", head, buf, buf->mva); */
	}
}

void reset_buffer(display_primary_path_context *ctx, disp_internal_buffer_info *buf)
{
	if (ctx && buf) {
		list_del_init(&buf->list);
		/* DISPMSG("reset_buffer, buf=0x%08x, mva=0x%08x\n", buf, buf->mva); */
	}
}

static struct ion_client *ion_client;

disp_internal_buffer_info *dequeue_buffer(display_primary_path_context *ctx,
					  struct list_head *head)
{

	disp_internal_buffer_info *buf = NULL;
	disp_internal_buffer_info *temp = NULL;
	unsigned int *pSrc;
	unsigned int src_mva;
	unsigned long size = 0;
	unsigned long tmp_size = 0;

	struct ion_mm_data mm_data;
	struct ion_handle *src_handle;

	if (ctx && head) {
		if (!list_empty(head)) {
			temp = list_entry(head->prev, disp_internal_buffer_info, list);
			/* DISPMSG("dequeue_buffer, head=0x%08x, buf=0x%08x, mva=0x%08x\n", head, temp, temp->mva); */
			list_del_init(&temp->list);

			if (list_empty(head)) {
				/* DISPMSG("after dequeue_buffer, head:0x%08x is empty\n", head); */
				/**reserve**/
			}
		} else {
			DISPMSG("list is empty, alloc new buffer\n");
			return NULL;

			size =
			    primary_display_get_width() * primary_display_get_height() *
			    primary_display_get_bpp() / 4;
			DISPMSG("size=0x%lx\n", size);
			if (ion_client == NULL)
				ion_client = ion_client_create(g_ion_device, "disp_decouple");

			buf = kzalloc(sizeof(disp_internal_buffer_info), GFP_KERNEL);
			if (buf) {
				INIT_LIST_HEAD(&buf->list);
				src_handle =
				    ion_alloc(ion_client, size, 0, ION_HEAP_MULTIMEDIA_MASK, 0);
				if (IS_ERR_OR_NULL(src_handle)) {
					DISPERR("Fatal Error, ion_alloc for size %lx failed\n",
						size);
					return NULL;
				}

				pSrc = ion_map_kernel(ion_client, src_handle);
				if (!IS_ERR_OR_NULL(pSrc)) {
					memset((void *)pSrc, 0, size);
				} else {
					DISPERR("map va failed\n");
					return NULL;
				}

				mm_data.config_buffer_param.kernel_handle = src_handle;
				/* mm_data.config_buffer_param.m4u_port= M4U_PORT_DISP_OVL0; */
				/* mm_data.config_buffer_param.prot = M4U_PROT_READ|M4U_PROT_WRITE; */
				/* mm_data.config_buffer_param.flags = M4U_FLAGS_SEQ_ACCESS; */
				mm_data.mm_cmd = ION_MM_CONFIG_BUFFER;
				if (ion_kernel_ioctl
				    (ion_client, ION_CMD_MULTIMEDIA, (unsigned long)&mm_data) < 0) {
					DISPERR("ion_test_drv: Config buffer failed.\n");
					return NULL;
				}

				ion_phys(ion_client, src_handle, (ion_phys_addr_t *) &src_mva,
					 (size_t *) &tmp_size);
				if (tmp_size == 0) {
					DISPERR("Fatal Error, get mva failed\n");
					return NULL;
				}
				buf->handle = src_handle;
				buf->mva = src_mva;
				buf->size = tmp_size;
				DISPMSG
				    ("buf:0x%p, buf->list:0x%lx, buf->mva:0x%08x, buf->size:0x%08x, buf->handle:0x%p\n",
				     buf, (unsigned long)(&buf->list), buf->mva, buf->size,
				     buf->handle);
			} else {
				DISPERR("Fatal error, kzalloc internal buffer info failed!!\n");
				return NULL;
			}

			/* ion_client_destroy(ion_client); */

			return buf;
		}
	}

	return temp;
}

disp_internal_buffer_info *find_buffer_by_mva(display_primary_path_context *ctx,
					      struct list_head *head, uint32_t mva)
{
	disp_internal_buffer_info *buf = NULL;
	disp_internal_buffer_info *temp = NULL;

	if (ctx && head && mva) {
		if (!list_empty(head)) {
			list_for_each_entry(temp, head, list) {
				/* DISPMSG("find buffer: temp=0x%08x, mva=0x%08x\n", temp, temp->mva); */
				if (mva == temp->mva) {
					buf = temp;
					break;
				}
			}
		}
		/* DISPMSG("find buffer by mva, head=0x%08x, buf=0x%08x\n", head, buf); */
	}

	return buf;
}



/*static struct task_struct *primary_path_decouple_worker_task = NULL;*/


#define pgc	_get_context()

static display_primary_path_context *_get_context(void)
{
	static int is_context_inited;
	static display_primary_path_context g_context;

	if (!is_context_inited) {
		memset((void *)&g_context, 0, sizeof(display_primary_path_context));
		is_context_inited = 1;
	}

	return &g_context;
}

static inline int _is_mirror_mode(DISP_MODE mode)
{
	if (mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE
	    || mode == DISP_SESSION_DECOUPLE_MIRROR_MODE)
		return 1;
	else
		return 0;
}

static inline int _is_decouple_mode(DISP_MODE mode)
{
	if (mode == DISP_SESSION_DECOUPLE_MODE || mode == DISP_SESSION_DECOUPLE_MIRROR_MODE)
		return 1;
	else
		return 0;
}

display_primary_path_context *primary_display_path_lock(const char *caller)
{
	dprec_logger_start(DPREC_LOGGER_PRIMARY_MUTEX, 0, 0);
	disp_sw_mutex_lock(&(pgc->lock));
	pgc->mutex_locker = caller;
	return pgc;
}

void primary_display_path_unlock(const char *caller)
{
	pgc->mutex_locker = NULL;
	disp_sw_mutex_unlock(&(pgc->lock));
	dprec_logger_done(DPREC_LOGGER_PRIMARY_MUTEX, 0, 0);
}

static void _primary_path_cmd_lock(void)
{
	mutex_lock(&(pgc->cmd_lock));
}

static void _primary_path_cmd_unlock(void)
{
	mutex_unlock(&(pgc->cmd_lock));
}

static void _primary_path_lock(const char *caller)
{
	dprec_logger_start(DPREC_LOGGER_PRIMARY_MUTEX, 0, 0);
	disp_sw_mutex_lock(&(pgc->lock));
	pgc->mutex_locker = caller;
}

static void _primary_path_unlock(const char *caller)
{
	pgc->mutex_locker = NULL;
	disp_sw_mutex_unlock(&(pgc->lock));
	dprec_logger_done(DPREC_LOGGER_PRIMARY_MUTEX, 0, 0);
}

static void _primary_path_vsync_lock(void)
{
	mutex_lock(&(pgc->vsync_lock));
}

static void _primary_path_vsync_unlock(void)
{
	mutex_unlock(&(pgc->vsync_lock));
}


/*void _cmdq_insert_wait_frame_done_token_mira(void *handle); */
static void _cmdq_flush_config_handle_mira(void *handle, int blocking);

#ifdef MTK_DISP_IDLE_LP
static atomic_t isDdp_Idle = ATOMIC_INIT(0);
static atomic_t idle_detect_flag = ATOMIC_INIT(0);
static struct mutex idle_lock;
static struct task_struct *primary_display_idle_detect_task;
#define DISP_DSI_REG_VFP 0x28

static DECLARE_WAIT_QUEUE_HEAD(idle_detect_wq);
#if 0
static void _disp_primary_idle_lock(void)
{
	mutex_lock(&idle_lock);
}

static void _disp_primary_idle_unlock(void)
{
	mutex_unlock(&idle_lock);
}
#endif
static int _disp_primary_path_idle_clock_on(unsigned int level)
{
	dpmgr_path_idle_on(pgc->dpmgr_handle, NULL, level);
	return 0;
}

static int _disp_primary_path_idle_clock_off(unsigned int level)
{
	dpmgr_path_idle_off(pgc->dpmgr_handle, NULL, level);
	return 0;
}

static int _disp_primary_path_dsi_clock_on(unsigned int level)
{
	if (!primary_display_is_video_mode())
		dpmgr_path_dsi_on(pgc->dpmgr_handle, NULL, level);
	return 0;
}

static int _disp_primary_path_dsi_clock_off(unsigned int level)
{
	if (!primary_display_is_video_mode())
		dpmgr_path_dsi_off(pgc->dpmgr_handle, NULL, level);
	return 0;
}

int _disp_primary_path_set_vfp(int enter)
{
	int ret = 0;

	if (primary_display_is_video_mode()) {
		LCM_PARAMS *lcm_param = disp_lcm_get_params(pgc->plcm);
		cmdqRecHandle cmdq_handle_vfp = NULL;

		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle_vfp);
		if (ret != 0) {
			DISPCHECK("fail to create primary cmdq handle for set vfp\n");
			return -1;
		}
		DISPCHECK("primary set vfp, handle=%p\n", cmdq_handle_vfp);
		cmdqRecReset(cmdq_handle_vfp);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_vfp);
		if (enter) {
			dpmgr_path_ioctl(pgc->dpmgr_handle, cmdq_handle_vfp, DDP_DSI_VFP_LP,
					 (unsigned long *)&(lcm_param->dsi.vertical_vfp_lp));
		} else {
			dpmgr_path_ioctl(pgc->dpmgr_handle, cmdq_handle_vfp, DDP_DSI_VFP_LP,
					 (unsigned long *)&(lcm_param->dsi.vertical_frontporch));
		}
		MMProfileLogEx(ddp_mmp_get_events()->dal_clean, MMProfileFlagPulse, 0, enter);

		_cmdq_flush_config_handle_mira(cmdq_handle_vfp, 1);
		DISPCHECK("[VFP]cmdq_handle_vfp ret=%d\n", ret);
		cmdqRecDestroy(cmdq_handle_vfp);
		cmdq_handle_vfp = NULL;
	} else {
		DISPCHECK("CMD mode don't set vfp for lows\n");
	}

	return ret;
}

/* extern void clk_stat_bug(void); */
unsigned int isIdlePowerOff = 0;
int primary_display_save_power_for_idle(int enter, unsigned int need_primary_lock)
{

	unsigned int isLowPowerMode = atomic_read(&isDdp_Idle);

	if (is_hwc_enabled == 0) {
		DISPMSG("primary_display_save_power_for_idle, hwc not enable ,just return !\n");
		return 0;
	}

	if (need_primary_lock) {
		/* if outer api has add primary lock, do not have to lock again */
		_primary_path_lock(__func__);
	}

	if (pgc->state == DISP_SLEPT) {
		DISPMSG("suspend mode can not enable low power.\n");
		goto end;
	}
	if (enter == 1 && isLowPowerMode == 1) {
		DISPMSG("already in low power mode.\n");
		goto end;
	}
	if (enter == 0 && isLowPowerMode == 0) {
		DISPMSG("already not in low power mode.\n");
		goto end;
	}
	isLowPowerMode = enter;

	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		goto end;
	}

	DISPDBG("low power in, enter=%d.\n", enter);

	if (disp_low_power_enlarge_blanking == 1) {
#if 0
		if (pgc->plcm->params) {
			vfp_for_low_power =
			    pgc->plcm->params->dsi.vertical_frontporch_for_low_power;
			vfp_original = pgc->plcm->params->dsi.vertical_frontporch;
		}

		if (enter) {
			DISP_REG_SET(pgc->cmdq_handle_config,
				     dispsys_reg[DISP_REG_DSI0] + DISP_DSI_REG_VFP,
				     vfp_for_low_power);
		} else {
			DISP_REG_SET(pgc->cmdq_handle_config,
				     dispsys_reg[DISP_REG_DSI0] + DISP_DSI_REG_VFP, vfp_original);
		}
#else
		if (primary_display_is_video_mode() == 1)
			_disp_primary_path_set_vfp(enter);
#endif
	}

	if (1 == disp_low_power_disable_ddp_clock) {
		/* open */
		static unsigned int disp_low_power_disable_ddp_clock_cnt;

		DISPMSG("MM clock, disp_low_power_disable_ddp_clock enter %d.\n",
			disp_low_power_disable_ddp_clock_cnt++);
		if ((1 == enter) && (primary_display_is_video_mode() == 0)) {
			/* only for command mode */
			if (isIdlePowerOff == 0) {
				DISPMSG("off MM clock start.\n");
				_disp_primary_path_idle_clock_off(0);	/* parameter represent level */
				DISPMSG("off MM clock end.\n");

				if (gDumpClockStatus == 1) {
					/* DISPMSG("---- start dump regs! clk_stat_bug.\n"); */
					/* clk_stat_bug(); */
					DISPMSG("start dump regs! clk_stat_check\n");
#ifdef CONFIG_MTK_CLKMGR
					clk_stat_check(SYS_DIS);
#endif
				}
			}
			isIdlePowerOff = 1;
		} else if ((0 == enter) && (primary_display_is_video_mode() == 0)) {
			/* close */
			if (isIdlePowerOff == 1) {
				DISPMSG("on MM clock start.\n");
				_disp_primary_path_idle_clock_on(0);	/* parameter represent level */
				DISPMSG("on MM clock end.\n");
			}
			isIdlePowerOff = 0;
		}
	}

	/* no need idle lock ,cause primary lock will be used inside switch_mode */
	if (disp_low_power_remove_ovl == 1) {
		if (primary_display_is_video_mode() == 1) {
			/* only for video mode */
			if (enter) {
				DISPDBG("disp_low_power_ovl:remove.\n");
				if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE) {
					primary_display_switch_mode_nolock
					    (DISP_SESSION_DECOUPLE_MODE, pgc->session_id, 1);
				}
				/* DISPMSG("disp_low_power_remove_ovl 2\n"); */
			} else {
				DISPDBG("disp_low_power_ovl:add.\n");
				if (pgc->session_mode == DISP_SESSION_DECOUPLE_MODE) {
					primary_display_switch_mode_nolock
					    (DISP_SESSION_DIRECT_LINK_MODE, pgc->session_id, 1);
				}
				/* DISPMSG("disp_low_power_remove_ovl 4\n"); */
			}
		}
	}

end:
	if (need_primary_lock)
		_primary_path_unlock(__func__);

	return 0;
}

void _disp_primary_path_exit_idle(const char *caller, unsigned int need_primary_lock)
{
	/* _disp_primary_idle_lock(); */
	if (atomic_read(&isDdp_Idle) == 1) {
		DISPDBG("[ddp_idle_on]_disp_primary_path_exit_idle (%s) &&&\n", caller);
		primary_display_save_power_for_idle(0, need_primary_lock);
		atomic_set(&isDdp_Idle, 0);
		atomic_set(&idle_detect_flag, 1);
		wake_up(&idle_detect_wq);
	}
	/* _disp_primary_idle_unlock(); */
}

static int _disp_primary_path_idle_detect_thread(void *data)
{
	int ret = 0;

	while (1) {
		msleep(1500);
		/* DISPMSG("[ddp_idle]_disp_primary_path_idle_detect start 1\n"); */

		if (gSkipIdleDetect)
			continue;

		_primary_path_lock(__func__);
		if (pgc->state == DISP_SLEPT) {
			MMProfileLogEx(ddp_mmp_get_events()->esd_check_t, MMProfileFlagPulse, 1, 0);
			DISPDBG("[ddp_idle]primary display path is slept?? -- skip ddp_idle\n");
			_primary_path_unlock(__func__);
			continue;
		}
		_primary_path_unlock(__func__);
		/* _disp_primary_idle_lock(); */
		if (((sched_clock() - last_primary_trigger_time) / 1000) > 1000000) {

			DISPDBG("[ddp_idle_off]idle for 1s, switch to low power mode!\n");
			_primary_path_lock(__func__);
			primary_display_save_power_for_idle(1, 0);
			atomic_set(&isDdp_Idle, 1);
			_primary_path_unlock(__func__);
		} else {
			/* _disp_primary_idle_unlock(); */
			continue;
		}
		/* _disp_primary_idle_unlock(); */

		ret =
		    wait_event_interruptible(idle_detect_wq, (atomic_read(&idle_detect_flag) != 0));
		atomic_set(&idle_detect_flag, 0);
		DISPDBG("[ddp_idle]ret=%d\n", ret);
		if (kthread_should_stop())
			break;
		DISPDBG("[ddp_idle]_disp_primary_path_idle_detect end\n");
	}
	return 0;
}

int primary_display_cmdq_set_reg(unsigned int addr, unsigned int val)
{
	int ret = 0;
	cmdqRecHandle handle = NULL;

	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);
	cmdqRecReset(handle);
	_cmdq_insert_wait_frame_done_token_mira(handle);

	cmdqRecWrite(handle, addr & 0x1fffffff, val, ~0);
	cmdqRecFlushAsync(handle);

	cmdqRecDestroy(handle);

	return 0;
}
#endif
static void update_frm_seq_info(unsigned int addr, unsigned int addr_offset, unsigned int seq,
				DISP_FRM_SEQ_STATE state)
{
	int i = 0;

	if (FRM_CONFIG == state) {
		frm_update_sequence[frm_update_cnt].state = state;
		frm_update_sequence[frm_update_cnt].mva = addr;
		frm_update_sequence[frm_update_cnt].max_offset = addr_offset;
		if (seq > 0)
			frm_update_sequence[frm_update_cnt].seq = seq;
		MMProfileLogEx(ddp_mmp_get_events()->primary_seq_config, MMProfileFlagPulse, addr,
			       seq);

	} else if (FRM_TRIGGER == state) {
		frm_update_sequence[frm_update_cnt].state = FRM_TRIGGER;
		MMProfileLogEx(ddp_mmp_get_events()->primary_seq_trigger, MMProfileFlagPulse,
			       frm_update_cnt, frm_update_sequence[frm_update_cnt].seq);

		dprec_logger_frame_seq_begin(pgc->session_id,
					     frm_update_sequence[frm_update_cnt].seq);

		frm_update_cnt++;
		frm_update_cnt %= FRM_UPDATE_SEQ_CACHE_NUM;


	} else if (FRM_START == state) {
		for (i = 0; i < FRM_UPDATE_SEQ_CACHE_NUM; i++) {
			if ((abs(addr - frm_update_sequence[i].mva) <=
			     frm_update_sequence[i].max_offset)
			    && (frm_update_sequence[i].state == FRM_TRIGGER)) {
				MMProfileLogEx(ddp_mmp_get_events()->primary_seq_rdma_irq,
					       MMProfileFlagPulse, frm_update_sequence[i].mva | seq,
					       frm_update_sequence[i].state);
				frm_update_sequence[i].state = FRM_START;
				/* /break; */
			}
		}
	} else if (FRM_END == state) {
		for (i = 0; i < FRM_UPDATE_SEQ_CACHE_NUM; i++) {
			if (FRM_START == frm_update_sequence[i].state) {
				frm_update_sequence[i].state = FRM_END;
				dprec_logger_frame_seq_end(pgc->session_id,
							   frm_update_sequence[i].seq);
				MMProfileLogEx(ddp_mmp_get_events()->primary_seq_release,
					       MMProfileFlagPulse, frm_update_sequence[i].mva,
					       frm_update_sequence[i].seq);

			}
		}
	}

}

static int _config_wdma_output(WDMA_CONFIG_STRUCT *wdma_config,
			       disp_path_handle disp_handle, cmdqRecHandle cmdq_handle)
{
	disp_ddp_path_config *pconfig = dpmgr_path_get_last_config(disp_handle);

	pconfig->wdma_config = *wdma_config;
	pconfig->wdma_dirty = 1;
	dpmgr_path_config(disp_handle, pconfig, cmdq_handle);
	return 0;
}

static int _config_rdma_input_data(RDMA_CONFIG_STRUCT *rdma_config,
				   disp_path_handle disp_handle, cmdqRecHandle cmdq_handle)
{
	disp_ddp_path_config *pconfig = dpmgr_path_get_last_config(disp_handle);

	pconfig->rdma_config = *rdma_config;
	pconfig->rdma_dirty = 1;
	dpmgr_path_config(disp_handle, pconfig, cmdq_handle);
	return 0;
}

#if 0
static unsigned int get_frm_seq_by_addr(unsigned int addr, DISP_FRM_SEQ_STATE state)
{
	int i = 0;

	for (i = 0; i < FRM_UPDATE_SEQ_CACHE_NUM; i++) {
		if (addr == frm_update_sequence[i].mva)
			return frm_update_sequence[i].seq;
	}
	return 0;
}

static unsigned int release_started_frm_seq(unsigned int addr, DISP_FRM_SEQ_STATE FRM_END)
{
	int i = 0;

	for (i = 0; i < FRM_UPDATE_SEQ_CACHE_NUM; i++) {
		if (FRM_END == frm_update_sequence[i].state)
			return frm_update_sequence[i].seq;
	}
	return 0;
}

#endif

#ifdef DISP_SWITCH_DST_MODE
unsigned long long last_primary_trigger_time;
bool is_switched_dst_mode = false;
static struct task_struct *primary_display_switch_dst_mode_task;
static void _primary_path_switch_dst_lock(void)
{
	mutex_lock(&(pgc->switch_dst_lock));
}

static void _primary_path_switch_dst_unlock(void)
{
	mutex_unlock(&(pgc->switch_dst_lock));
}

static int _disp_primary_path_switch_dst_mode_thread(void *data)
{
	int ret = 0;

	while (1) {
		msleep(1000);

		if (((sched_clock() - last_primary_trigger_time) / 1000) > 500000) {
			primary_display_switch_dst_mode(0);	/* switch to cmd mode */
			is_switched_dst_mode = true;
		}
		if (kthread_should_stop())
			break;
	}
	return 0;
}
#endif
/* extern int disp_od_is_enabled(void); */
int primary_display_get_debug_state(char *stringbuf, int buf_len)
{
	int len = 0;
	LCM_PARAMS *lcm_param = disp_lcm_get_params(pgc->plcm);
	LCM_DRIVER *lcm_drv = pgc->plcm->drv;

	len +=
	    scnprintf(stringbuf + len, buf_len - len,
		      "|--------------------------------------------------------------------------------------|\n");
	len +=
	    scnprintf(stringbuf + len, buf_len - len,
		      "|********Primary Display Path General Information********\n");
	len +=
	    scnprintf(stringbuf + len, buf_len - len, "|Primary Display is %s\n",
		      dpmgr_path_is_idle(pgc->dpmgr_handle) ? "idle" : "busy");

	if (mutex_trylock(&(pgc->lock))) {
		mutex_unlock(&(pgc->lock));
		len +=
		    scnprintf(stringbuf + len, buf_len - len,
			      "|primary path global mutex is free\n");
	} else {
		len +=
		    scnprintf(stringbuf + len, buf_len - len,
			      "|primary path global mutex is hold by [%s]\n", pgc->mutex_locker);
	}

	if (lcm_param && lcm_drv)
		len +=
		    scnprintf(stringbuf + len, buf_len - len,
			      "|LCM Driver=[%s]\tResolution=%dx%d,Interface:%s\n", lcm_drv->name,
			      lcm_param->width, lcm_param->height,
			      (lcm_param->type == LCM_TYPE_DSI) ? "DSI" : "Other");

	/* no OD */
	/* len += scnprintf(stringbuf+len, buf_len - len, "|OD is %s\n", disp_od_is_enabled()?"enabled":"disabled"); */
	len +=
	    scnprintf(stringbuf + len, buf_len - len, "|session_mode is 0x%08x\n",
		      pgc->session_mode);

	len +=
	    scnprintf(stringbuf + len, buf_len - len,
		      "|State=%s\tlcm_fps=%d\tmax_layer=%d\tmode:%d\tvsync_drop=%d\n",
		      pgc->state == DISP_ALIVE ? "Alive" : "Slept", pgc->lcm_fps, pgc->max_layer,
		      pgc->mode, pgc->vsync_drop);
	len +=
	    scnprintf(stringbuf + len, buf_len - len,
		      "|cmdq_handle_config=%p\tcmdq_handle_trigger=%p\tdpmgr_handle=%p\tovl2mem_path_handle=%p\n",
		      pgc->cmdq_handle_config, pgc->cmdq_handle_trigger, pgc->dpmgr_handle,
		      pgc->ovl2mem_path_handle);
	len +=
	    scnprintf(stringbuf + len, buf_len - len, "|Current display driver status=%s + %s\n",
		      primary_display_is_video_mode() ? "video mode" : "cmd mode",
		      primary_display_cmdq_enabled() ? "CMDQ Enabled" : "CMDQ Disabled");

	return len;
}

static char _dump_buffer_string[512];
static void _dump_internal_buffer_into(display_primary_path_context *ctx, char *dump_reason)
{
	int n = 0;
	int len = 0;
	disp_internal_buffer_info *buf = NULL;
	struct list_head *head = NULL;
	int i = 0;

	len = sizeof(_dump_buffer_string);
	n += scnprintf(_dump_buffer_string + n, len - n, "dump for %s\n",
		       dump_reason ? dump_reason : "unknown");

	if (ctx) {
		mutex_lock(&(pgc->dc_lock));

		head = &(pgc->dc_free_list);
		i = 0;
		n += scnprintf(_dump_buffer_string + n, len - n, "free list: ");
		if (!list_empty(head)) {
			list_for_each_entry(buf, head, list) {
				n += scnprintf(_dump_buffer_string + n, len - n, "\t0x%08x",
					       buf->mva);
			}
		}

		n += scnprintf(_dump_buffer_string + n, len - n, "\n");

#if 0
		head = &(pgc->dc_writing_list);
		i = 0;
		n += scnprintf(_dump_buffer_string + n, len - n, "writing list: ");
		if (!list_empty(head)) {
			list_for_each_entry(buf, head, list) {
				n += scnprintf(_dump_buffer_string + n, len - n, "\t0x%08x",
					       buf->mva);
			}
		}
		n += scnprintf(_dump_buffer_string + n, len - n, "\n");
#endif

		head = &(pgc->dc_reading_list);
		i = 0;
		n += scnprintf(_dump_buffer_string + n, len - n, "reading list: ");
		if (!list_empty(head)) {
			list_for_each_entry(buf, head, list) {
				n += scnprintf(_dump_buffer_string + n, len - n, "\t0x%08x",
					       buf->mva);
			}
		}
		n += scnprintf(_dump_buffer_string + n, len - n, "\n");

		DISPMSG("%s", _dump_buffer_string);
		mutex_unlock(&(pgc->dc_lock));
	}
}

int32_t decouple_rdma_worker_callback(unsigned long data)
{
	disp_internal_buffer_info *reading_buf = NULL;
	disp_internal_buffer_info *temp = NULL;
	disp_internal_buffer_info *n = NULL;
	unsigned int current_reading_addr = 0;

	mutex_lock(&(pgc->dc_lock));
	current_reading_addr = ddp_ovl_get_cur_addr(1, 0);
	DISPMSG("rdma applied, 0x%lx, 0x%08x\n", data, current_reading_addr);

	reading_buf = find_buffer_by_mva(pgc, &pgc->dc_reading_list, data);
	if (!reading_buf) {
		/* DISPERR("fatal error, we can't find related buffer info with callback data:0x%08x\n", data); */
		mutex_unlock(&(pgc->dc_lock));
		return 0;
	}

	list_for_each_entry_safe(temp, n, &(pgc->dc_reading_list), list) {
		if (temp && temp->mva != current_reading_addr) {
			DISPMSG("temp=0x%p, temp->mva=0x%08x, temp->list=0x%lx\n", temp, temp->mva,
				(unsigned long)(&temp->list));
			reset_buffer(pgc, temp);
			enqueue_buffer(pgc, &(pgc->dc_free_list), temp);
		}
	}

	mutex_unlock(&(pgc->dc_lock));

	_dump_internal_buffer_into(pgc, "interface applied");

	return 0;
}

#if 0
static int _disp_primary_path_decouple_worker_thread(void *data)
{
	struct sched_param param = {.sched_priority = 90 };

	sched_setscheduler(current, SCHED_RR, &param);
	int ret = 0;
	primary_disp_input_config input;
	disp_internal_buffer_info *writing_buf = NULL;
	disp_internal_buffer_info *reading_buf = NULL;
	uint32_t current_writing_mva = 0;
	disp_ddp_path_config *pconfig = NULL;

	while (1) {
		/* shuold wait FRAME_START here, but MUTEX1's sof always issued with MUTEX0's sof,*/
		/* root cause still unkonwn. so use FRAME_COMPLETE instead */
		dpmgr_wait_event(pgc->ovl2mem_path_handle, DISP_PATH_EVENT_FRAME_COMPLETE);
		pconfig = dpmgr_path_get_last_config(pgc->ovl2mem_path_handle);
		current_writing_mva = pconfig->wdma_config.dstAddress;


		_dump_internal_buffer_into(pgc, "ovl2mem done");
		disp_sw_mutex_lock(&(pgc->dc_lock));
		writing_buf = find_buffer_by_mva(pgc, &pgc->dc_writing_list, current_writing_mva);
		if (!writing_buf) {
			DISPERR
			    ("fatal error, we can't find related buffer info with current_writing_mva:0x%08x\n",
			     current_writing_mva);
			disp_sw_mutex_unlock(&(pgc->dc_lock));
			continue;
		} else {
			reset_buffer(pgc, writing_buf);
			enqueue_buffer(pgc, &(pgc->dc_reading_list), writing_buf);

			reading_buf = dequeue_buffer(pgc, &(pgc->dc_free_list));
			if (!reading_buf) {
				DISPERR("dc_free_list is empty!!\n");

				disp_sw_mutex_unlock(&(pgc->dc_lock));
				continue;
			} else {
				enqueue_buffer(pgc, &(pgc->dc_writing_list), reading_buf);
			}
			disp_sw_mutex_unlock(&(pgc->dc_lock));
		}

		_dump_internal_buffer_into(pgc, "trigger interface");

		pconfig->wdma_config.dstAddress = reading_buf->mva;
		pconfig->wdma_config.srcHeight = primary_display_get_height();
		pconfig->wdma_config.srcWidth = primary_display_get_width();
		pconfig->wdma_config.clipX = 0;
		pconfig->wdma_config.clipY = 0;
		pconfig->wdma_config.clipHeight = primary_display_get_height();
		pconfig->wdma_config.clipWidth = primary_display_get_width();
		pconfig->wdma_config.outputFormat = eRGB888;
		pconfig->wdma_config.useSpecifiedAlpha = 1;
		pconfig->wdma_config.alpha = 0xFF;
		pconfig->wdma_config.dstPitch =
		    primary_display_get_width() * DP_COLOR_BITS_PER_PIXEL(eRGB888) / 8;
		pconfig->wdma_dirty = 1;

		ret = dpmgr_path_config(pgc->ovl2mem_path_handle, pconfig, CMDQ_DISABLE);

		memset((void *)&input, 0, sizeof(primary_disp_input_config));

		input.addr = (unsigned int)writing_buf->mva;
		input.src_x = 0;
		input.src_y = 0;
		input.src_w = primary_display_get_width();
		input.src_h = primary_display_get_height();
		input.dst_x = 0;
		input.dst_y = 0;
		input.dst_w = primary_display_get_width();
		input.dst_h = primary_display_get_height();
		input.fmt = eRGB888;
		input.alpha = 0xFF;

		input.src_pitch = primary_display_get_width() * 3;
		input.isDirty = 1;
		MMProfileLogEx(ddp_mmp_get_events()->interface_trigger, MMProfileFlagPulse,
			       input.addr, pconfig->wdma_config.dstAddress);
		ret = primary_display_config_interface_input(&input);
		ret =
		    _trigger_display_interface(false, decouple_rdma_worker_callback,
					       ddp_ovl_get_cur_addr(1, 0));

		if (kthread_should_stop())
			break;
	}
	return 0;
}
#endif
static DISP_MODULE_ENUM _get_dst_module_by_lcm(disp_lcm_handle *plcm)
{
	if (plcm == NULL) {
		DISPERR("plcm is null\n");
		return DISP_MODULE_UNKNOWN;
	}

	if (plcm->params->type == LCM_TYPE_DSI) {
		if (plcm->lcm_if_id == LCM_INTERFACE_DSI0)
			return DISP_MODULE_DSI0;
		else if (plcm->lcm_if_id == LCM_INTERFACE_DSI1)
			return DISP_MODULE_DSI1;
		else if (plcm->lcm_if_id == LCM_INTERFACE_DSI_DUAL)
			return DISP_MODULE_DSIDUAL;
		else
			return DISP_MODULE_DSI0;
	} else if (plcm->params->type == LCM_TYPE_DPI)
		return DISP_MODULE_DPI0;
	DISPERR("can't find primary path dst module\n");
	return DISP_MODULE_UNKNOWN;
}

#define AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA

/***************************************************************
***trigger operation:  VDO+CMDQ  CMD+CMDQ VDO+CPU  CMD+CPU
*** 1.wait idle:           N         N       Y        Y
*** 2.lcm update:          N         Y       N        Y
*** 3.path start:	idle->Y      Y    idle->Y     Y
*** 4.path trigger:     idle->Y      Y    idle->Y     Y
*** 5.mutex enable:        N         N    idle->Y     Y
*** 6.set cmdq dirty:      N         Y       N        N
*** 7.flush cmdq:          Y         Y       N        N
****************************************************************/

int _should_wait_path_idle(void)
{
	/***trigger operation:  VDO+CMDQ  CMD+CMDQ VDO+CPU  CMD+CPU
	*** 1.wait idle:	          N         N        Y        Y				*/
	if (primary_display_cmdq_enabled()) {
		if (primary_display_is_video_mode())
			return 0;
		return 0;
	}
	if (primary_display_is_video_mode())
		return dpmgr_path_is_busy(pgc->dpmgr_handle);
	else
		return dpmgr_path_is_busy(pgc->dpmgr_handle);
}

int _should_update_lcm(void)
{
/***trigger operation:  VDO+CMDQ  CMD+CMDQ VDO+CPU  CMD+CPU
*** 2.lcm update:          N         Y       N        Y        **/
	if (primary_display_cmdq_enabled()) {
		if (primary_display_is_video_mode())
			return 0;
	/* TODO: lcm_update can't use cmdq now */
		return 0;
	}
	if (primary_display_is_video_mode())
		return 0;
	return 1;
}

int _should_start_path(void)
{
/***trigger operation:  VDO+CMDQ  CMD+CMDQ VDO+CPU  CMD+CPU
*** 3.path start:	idle->Y      Y    idle->Y     Y        ***/
	if (primary_display_cmdq_enabled()) {
		if (primary_display_is_video_mode()) {
			return 0;
			/* return dpmgr_path_is_idle(pgc->dpmgr_handle); */
		} else {
			return 0;
		}
	} else {
		if (primary_display_is_video_mode())
			return dpmgr_path_is_idle(pgc->dpmgr_handle);
		return 1;
	}
}

int _should_trigger_path(void)
{
/***trigger operation:  VDO+CMDQ  CMD+CMDQ VDO+CPU  CMD+CPU
*** 4.path trigger:     idle->Y      Y    idle->Y     Y
*** 5.mutex enable:        N         N    idle->Y     Y        ***/

	/* this is not a perfect design, we can't decide path trigger(ovl/rdma/dsi..) separately with mutex enable */
	/* but it's lucky because path trigger and mutex enable is the same w/o cmdq, and it's correct w/ CMDQ(Y+N). */
	if (primary_display_cmdq_enabled()) {
		if (primary_display_is_video_mode()) {
			return 0;
			/* return dpmgr_path_is_idle(pgc->dpmgr_handle); */
		} else {
			return 0;
		}
	} else {
		if (primary_display_is_video_mode())
			return dpmgr_path_is_idle(pgc->dpmgr_handle);
		return 1;
	}
}

int _should_set_cmdq_dirty(void)
{
/***trigger operation:  VDO+CMDQ  CMD+CMDQ VDO+CPU  CMD+CPU
*** 6.set cmdq dirty:	    N         Y       N        N     ***/
	if (primary_display_cmdq_enabled()) {
		if (primary_display_is_video_mode())
			return 0;
		return 1;
	}
	if (primary_display_is_video_mode())
		return 0;
	return 0;
}

int _should_flush_cmdq_config_handle(void)
{
/***trigger operation:  VDO+CMDQ  CMD+CMDQ VDO+CPU  CMD+CPU
*** 7.flush cmdq:          Y         Y       N        N        ***/
	if (primary_display_cmdq_enabled()) {
		if (primary_display_is_video_mode())
			return 1;
		return 1;
	}
	if (primary_display_is_video_mode())
		return 0;
	return 0;
}

int _should_reset_cmdq_config_handle(void)
{
	if (primary_display_cmdq_enabled()) {
		if (primary_display_is_video_mode())
			return 1;
		return 1;
	}
	if (primary_display_is_video_mode())
		return 0;
	return 0;

}

int _should_insert_wait_frame_done_token(void)
{
/***trigger operation:  VDO+CMDQ  CMD+CMDQ VDO+CPU  CMD+CPU
*** 7.flush cmdq:          Y         Y       N        N      */
	if (primary_display_cmdq_enabled()) {
		if (primary_display_is_video_mode())
			return 1;
		return 1;
	}
	if (primary_display_is_video_mode())
		return 0;
	return 0;
}

int _should_trigger_interface(void)
{
	if (pgc->mode == DECOUPLE_MODE)
		return 0;
	return 1;
}

int _should_config_ovl_input(void)
{
	/* should extend this when display path dynamic switch is ready */
	if (pgc->mode == SINGLE_LAYER_MODE || pgc->mode == DEBUG_RDMA1_DSI0_MODE)
		return 0;
	else
		return 1;

}

int _should_config_ovl_to_memory(display_primary_path_context *ctx)
{
	if (ctx == NULL) {
		DISP_FATAL_ERR("DISP", "Context is NULL!\n");
		return 0;
	}

	if (ctx->mode == DECOUPLE_MODE)
		return 1;
	else
		return 0;
}

#define OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
static unsigned long get_current_time_us(void)
{
	struct timeval t;

	do_gettimeofday(&t);
	return (t.tv_sec & 0xFFF) * 1000000 + t.tv_usec;
}

static enum hrtimer_restart _DISP_CmdModeTimer_handler(struct hrtimer *timer)
{
	DISPMSG("fake timer, wake up\n");
	dpmgr_signal_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
#if 0
	if ((get_current_time_us() - pgc->last_vsync_tick) > 16666) {
		dpmgr_signal_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
		pgc->last_vsync_tick = get_current_time_us();
	}
#endif
	return HRTIMER_RESTART;
}

int _init_vsync_fake_monitor(int fps)
{
	static struct hrtimer cmd_mode_update_timer;
	static ktime_t cmd_mode_update_timer_period;

	if (fps == 0)
		fps = 60;

	cmd_mode_update_timer_period = ktime_set(0, 1000 / fps * 1000);
	DISPMSG("[MTKFB] vsync timer_period=%d\n", 1000 / fps);
	hrtimer_init(&cmd_mode_update_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cmd_mode_update_timer.function = _DISP_CmdModeTimer_handler;

	return 0;
}

#if 0
static int _build_path_direct_link(void)
{
	int ret = 0;

	DISP_MODULE_ENUM dst_module = 0;

	DISPFUNC();
	pgc->mode = DIRECT_LINK_MODE;

	pgc->dpmgr_handle = dpmgr_create_path(DDP_SCENARIO_PRIMARY_DISP, pgc->cmdq_handle_config);
	if (pgc->dpmgr_handle) {
		DISPCHECK("dpmgr create path SUCCESS(0x%08x)\n", pgc->dpmgr_handle);
	} else {
		DISPCHECK("dpmgr create path FAIL\n");
		return -1;
	}

	dst_module = _get_dst_module_by_lcm(pgc->plcm);
	dpmgr_path_set_dst_module(pgc->dpmgr_handle, dst_module);
	DISPCHECK("dpmgr set dst module FINISHED(%s)\n", ddp_get_module_name(dst_module));
#ifndef MTKFB_NO_M4U
	{
		M4U_PORT_STRUCT sPort;

		sPort.ePortID = M4U_PORT_DISP_OVL0;
		sPort.Virtuality = primary_display_use_m4u;
		sPort.Security = 0;
		sPort.Distance = 1;
		sPort.Direction = 0;
		ret = m4u_config_port(&sPort);
		if (ret == 0) {
			DISPCHECK("config M4U Port %s to %s SUCCESS\n",
				  ddp_get_module_name(DISP_MODULE_OVL0),
				  primary_display_use_m4u ? "virtual" : "physical");
		} else {
			DISPCHECK("config M4U Port %s to %s FAIL(ret=%d)\n",
				  ddp_get_module_name(DISP_MODULE_OVL0),
				  primary_display_use_m4u ? "virtual" : "physical", ret);
			return -1;
		}
#ifdef OVL_CASCADE_SUPPORT
		sPort.ePortID = M4U_PORT_DISP_OVL1;
		ret = m4u_config_port(&sPort);
		if (ret) {
			DISPCHECK("config M4U Port %s to %s FAIL(ret=%d)\n",
				  ddp_get_module_name(DISP_MODULE_OVL1),
				  primary_display_use_m4u ? "virtual" : "physical", ret);
			return -1;
		}
#endif

	}
#endif

	dpmgr_set_lcm_utils(pgc->dpmgr_handle, pgc->plcm->drv);

	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);

	return ret;
}
#endif
void disp_set_sodi(unsigned int enable, void *cmdq_handle)
{
#ifdef DISP_ENABLE_SODI
	if (gEnableSODIControl == 1) {
		if (cmdq_handle != NULL) {
			if (enable == 1)
				cmdqRecWrite(cmdq_handle, DISP_REG_SODI_PA, 0, 1);
			else
				cmdqRecWrite(cmdq_handle, DISP_REG_SODI_PA, 1, 1);
		} else {
			if (enable == 1)
				DISP_REG_SET(0, SPM_PCM_SRC_REQ,
					     DISP_REG_GET(SPM_PCM_SRC_REQ) & (~0x1));
			else
				DISP_REG_SET(0, SPM_PCM_SRC_REQ,
					     DISP_REG_GET(SPM_PCM_SRC_REQ) | 0x1);
		}
	}
#endif
}


static void _cmdq_build_trigger_loop(void)
{
	int ret = 0;

	if (pgc->cmdq_handle_trigger == NULL) {
		cmdqRecCreate(CMDQ_SCENARIO_TRIGGER_LOOP, &(pgc->cmdq_handle_trigger));
		DISPMSG("primary path trigger thread cmd handle=%p\n", pgc->cmdq_handle_trigger);
	}
	cmdqRecReset(pgc->cmdq_handle_trigger);

	if (primary_display_is_video_mode()) {
		/* wait and clear stream_done, HW will assert mutex enable automatically in frame done reset. */
		/* todo: should let dpmanager to decide wait which mutex's eof. */
		if (gEnableMutexRisingEdge == 1)
			ret =
			    cmdqRecWait(pgc->cmdq_handle_trigger,
					dpmgr_path_get_mutex(pgc->dpmgr_handle) +
					CMDQ_EVENT_DISP_RDMA0_EOF);
		else
			ret =
			    cmdqRecWait(pgc->cmdq_handle_trigger,
					dpmgr_path_get_mutex(pgc->dpmgr_handle) +
					CMDQ_EVENT_MUTEX0_STREAM_EOF);

		/* for some module(like COLOR) to read hw register to GPR after frame done */
		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger,
				      CMDQ_AFTER_STREAM_EOF);
	} else {
		ret = cmdqRecWaitNoClear(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_CABC_EOF);
		/* DSI command mode doesn't have mutex_stream_eof, need use CMDQ token instead */
		ret = cmdqRecWait(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);

		if (_need_wait_esd_eof()) {
			/* Wait esd config thread done. */
			ret = cmdqRecWaitNoClear(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_ESD_EOF);
		}
		/* ret = cmdqRecWait(pgc->cmdq_handle_trigger, CMDQ_EVENT_MDP_DSI0_TE_SOF); */
		/* for operations before frame transfer, such as waiting for DSI TE */
		if (islcmconnected)
			dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger,
					      CMDQ_BEFORE_STREAM_SOF);

		/* cleat frame done token, now the config thread will not allowed to config registers. */
		/* remember that config thread's priority is higher than trigger thread,*/
		/* so all the config queued before will be applied then STREAM_EOF token be cleared */
		/* this is what CMDQ did as "Merge" */
		ret = cmdqRecClearEventToken(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_STREAM_EOF);

		ret =
		    cmdqRecClearEventToken(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);

		/* enable mutex, only cmd mode need this */
		/* this is what CMDQ did as "Trigger" */
		/* clear rdma EOF token before wait */
		ret = cmdqRecClearEventToken(pgc->cmdq_handle_trigger, CMDQ_EVENT_DISP_RDMA0_EOF);

		dpmgr_path_trigger(pgc->dpmgr_handle, pgc->cmdq_handle_trigger, CMDQ_ENABLE);
		/* ret = cmdqRecWrite(pgc->cmdq_handle_trigger, */
		/*(unsigned int)(DISP_REG_CONFIG_MUTEX_EN(0))&0x1fffffff, 1, ~0); */

		/* SODI is disabled in config thread, so mutex enable/dsi */
		/*start/CPU wait TE will not be blocked by SODI */
		/* should enable SODI here, */
		if (gDisableSODIForTriggerLoop == 1)
			disp_set_sodi(1, pgc->cmdq_handle_trigger);
		/* waiting for frame done, because we can't use mutex stream eof here, */
		/*so need to let dpmanager help to decide which event to wait */
		/* most time we wait rdmax frame done event. */
		ret = cmdqRecWait(pgc->cmdq_handle_trigger, CMDQ_EVENT_DISP_RDMA0_EOF);
		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger,
				      CMDQ_WAIT_STREAM_EOF_EVENT);

		/* dsi is not idle rightly after rdma frame done, so we need to polling */
		/*about 1us for dsi returns to idle */
		/* do not polling dsi idle directly which will decrease CMDQ performance */
		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger,
				      CMDQ_CHECK_IDLE_AFTER_STREAM_EOF);

		/* for some module(like COLOR) to read hw register to GPR after frame done */
		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger,
				      CMDQ_AFTER_STREAM_EOF);

		/* polling DSI idle */
		/* ret = cmdqRecPoll(pgc->cmdq_handle_trigger, 0x1401b00c, 0, 0x80000000); */
		/* polling wdma frame done */
		/* ret = cmdqRecPoll(pgc->cmdq_handle_trigger, 0x140060A0, 1, 0x1); */

		/* now frame done, config thread is allowed to config register now */
		ret = cmdqRecSetEventToken(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_STREAM_EOF);

		/* RUN forever!!!! */
		BUG_ON(ret < 0);
	}

	/* dump trigger loop instructions to check whether dpmgr_path_build_cmdq works correctly */
	DISPCHECK("primary display BUILD cmdq trigger loop finished\n");


}

static void _cmdq_start_trigger_loop(void)
{
	int ret = 0;

	cmdqRecDumpCommand(pgc->cmdq_handle_trigger);
	/* this should be called only once because trigger loop will nevet stop */
	ret = cmdqRecStartLoop(pgc->cmdq_handle_trigger);
	if (!primary_display_is_video_mode()) {
		if (_need_wait_esd_eof()) {
			/* Need set esd check eof synctoken to let trigger loop go. */
			cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_ESD_EOF);
		}
		/* need to set STREAM_EOF for the first time, otherwise we will stuck in dead loop */
		cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_STREAM_EOF);
		cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_CABC_EOF);
		dprec_event_op(DPREC_EVENT_CMDQ_SET_EVENT_ALLOW);
	} else {
#if 0
		if (dpmgr_path_is_idle(pgc->dpmgr_handle))
			cmdqCoreSetEvent(CMDQ_EVENT_MUTEX0_STREAM_EOF);
#endif
	}

	if (_is_decouple_mode(pgc->session_mode))
		cmdqCoreSetEvent(CMDQ_EVENT_DISP_WDMA0_EOF);

	DISPCHECK("primary display START cmdq trigger loop finished\n");
}

static void _cmdq_stop_trigger_loop(void)
{
	int ret = 0;

	/* this should be called only once because trigger loop will nevet stop */
	ret = cmdqRecStopLoop(pgc->cmdq_handle_trigger);

	DISPCHECK("primary display STOP cmdq trigger loop finished\n");
}


static void _cmdq_set_config_handle_dirty(void)
{
	if (!primary_display_is_video_mode()) {
		dprec_logger_start(DPREC_LOGGER_PRIMARY_CMDQ_SET_DIRTY, 0, 0);
		/* only command mode need to set dirty */
		cmdqRecSetEventToken(pgc->cmdq_handle_config, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		dprec_event_op(DPREC_EVENT_CMDQ_SET_DIRTY);
		dprec_logger_done(DPREC_LOGGER_PRIMARY_CMDQ_SET_DIRTY, 0, 0);
	}
}
static void _cmdq_handle_clear_dirty(cmdqRecHandle cmdq_handle)
{
	if (!primary_display_is_video_mode()) {
		dprec_logger_trigger(DPREC_LOGGER_PRIMARY_CMDQ_SET_DIRTY, 0, 1);
		cmdqRecClearEventToken(cmdq_handle, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
	}
}
static void _cmdq_set_config_handle_dirty_mira(void *handle)
{
	if (!primary_display_is_video_mode()) {
		dprec_logger_start(DPREC_LOGGER_PRIMARY_CMDQ_SET_DIRTY, 0, 0);
		/* only command mode need to set dirty */
		cmdqRecSetEventToken(handle, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		dprec_event_op(DPREC_EVENT_CMDQ_SET_DIRTY);
		dprec_logger_done(DPREC_LOGGER_PRIMARY_CMDQ_SET_DIRTY, 0, 0);
	}
}

static void _cmdq_reset_config_handle(void)
{
	cmdqRecReset(pgc->cmdq_handle_config);
	dprec_event_op(DPREC_EVENT_CMDQ_RESET);
}

static void _cmdq_flush_config_handle(int blocking, void *callback, unsigned int userdata)
{
	dprec_logger_start(DPREC_LOGGER_PRIMARY_CMDQ_FLUSH, blocking,
			   (unsigned int)(unsigned long)callback);
	if (blocking) {
		/* DISPERR("Should not use blocking cmdq flush, */
		/*may block primary display path for 1 frame period\n"); */
		cmdqRecFlush(pgc->cmdq_handle_config);
	} else {
		if (callback)
			cmdqRecFlushAsyncCallback(pgc->cmdq_handle_config, callback, userdata);
		else
			cmdqRecFlushAsync(pgc->cmdq_handle_config);
	}

	dprec_event_op(DPREC_EVENT_CMDQ_FLUSH);
	dprec_logger_done(DPREC_LOGGER_PRIMARY_CMDQ_FLUSH, userdata, 0);
}

static void _cmdq_flush_config_handle_mira(void *handle, int blocking)
{
	dprec_logger_start(DPREC_LOGGER_PRIMARY_CMDQ_FLUSH, 0, 0);
	if (blocking)
		cmdqRecFlush(handle);
	else
		cmdqRecFlushAsync(handle);
	dprec_event_op(DPREC_EVENT_CMDQ_FLUSH);
	dprec_logger_done(DPREC_LOGGER_PRIMARY_CMDQ_FLUSH, 0, 0);
}

static void _cmdq_insert_wait_frame_done_token(void)
{
	if (primary_display_is_video_mode()) {
		if (gEnableMutexRisingEdge == 1)
			cmdqRecWaitNoClear(pgc->cmdq_handle_config, CMDQ_EVENT_DISP_RDMA0_EOF);
		else
			cmdqRecWaitNoClear(pgc->cmdq_handle_config, CMDQ_EVENT_MUTEX0_STREAM_EOF);
	} else {
		cmdqRecWaitNoClear(pgc->cmdq_handle_config, CMDQ_SYNC_TOKEN_STREAM_EOF);
	}

	dprec_event_op(DPREC_EVENT_CMDQ_WAIT_STREAM_EOF);
}

void _cmdq_insert_wait_frame_done_token_mira(void *handle)
{
	if (primary_display_is_video_mode())
		cmdqRecWaitNoClear(handle, CMDQ_EVENT_MUTEX0_STREAM_EOF);
	else
		cmdqRecWaitNoClear(handle, CMDQ_SYNC_TOKEN_STREAM_EOF);

	dprec_event_op(DPREC_EVENT_CMDQ_WAIT_STREAM_EOF);
}

static void directlink_path_add_memory(WDMA_CONFIG_STRUCT *p_wdma)
{
	int ret = 0;
	cmdqRecHandle cmdq_handle = NULL;
	cmdqRecHandle cmdq_wait_handle = NULL;
	disp_ddp_path_config *pconfig = NULL;

	/*create config thread */
	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);
	if (ret != 0) {
		DISPCHECK("dl_to_dc capture:Fail to create cmdq handle\n");
		ret = -1;
		goto out;
	}
	cmdqRecReset(cmdq_handle);

	/*create wait thread */
	ret = cmdqRecCreate(CMDQ_SCENARIO_DISP_SCREEN_CAPTURE, &cmdq_wait_handle);
	if (ret != 0) {
		DISPCHECK("dl_to_dc capture:Fail to create cmdq wait handle\n");
		ret = -1;
		goto out;
	}
	cmdqRecReset(cmdq_wait_handle);

	/* disable SODI by CMDQ */
	if (primary_display_is_video_mode() == 1)
		disp_set_sodi(0, cmdq_handle);
	/*configure  config thread */
	_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);

	dpmgr_path_add_memout(pgc->dpmgr_handle, ENGINE_OVL0, cmdq_handle);

	pconfig = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	pconfig->wdma_config = *p_wdma;
	pconfig->wdma_dirty = 1;
	ret = dpmgr_path_config(pgc->dpmgr_handle, pconfig, cmdq_handle);

	_cmdq_set_config_handle_dirty_mira(cmdq_handle);
	_cmdq_flush_config_handle_mira(cmdq_handle, 0);
	DISPDBG("dl_to_dc capture:Flush add memout mva(0x%lx)\n", p_wdma->dstAddress);

	/*wait wdma0 sof */
	cmdqRecWait(cmdq_wait_handle, CMDQ_EVENT_DISP_WDMA0_SOF);
	cmdqRecFlush(cmdq_wait_handle);
	DISPDBG("dl_to_dc capture:Flush wait wdma sof\n");
#if 0
	cmdqRecReset(cmdq_handle);
	_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);

	dpmgr_path_remove_memout(pgc->dpmgr_handle, cmdq_handle);
	_cmdq_set_config_handle_dirty_mira(cmdq_handle);
	/* flush remove memory to cmdq */
	_cmdq_flush_config_handle_mira(cmdq_handle, 0);
	DISPMSG("dl_to_dc capture: Flush remove memout\n");

	dpmgr_path_memout_clock(pgc->dpmgr_handle, 0);
#endif
out:
	cmdqRecDestroy(cmdq_handle);
	cmdqRecDestroy(cmdq_wait_handle);

}

#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
bool _is_sec(disp_path_handle dp_handle)
{
	bool is_sec = false;
	int i = 0;
	int ovl_layer_cnt = 4;
	disp_ddp_path_config *pConfig = NULL;

	if (ovl_get_status() == DDP_OVL1_STATUS_PRIMARY ||
		ovl_get_status() == DDP_OVL1_STATUS_SUB_REQUESTING)
		ovl_layer_cnt = 8;

	pConfig = dpmgr_path_get_last_config(dp_handle);
	for (i = 0; i < ovl_layer_cnt; i++) {
		if (pConfig->ovl_config[i].layer_en
			&& (pConfig->ovl_config[i].security == DISP_SECURE_BUFFER))
			is_sec = true;
	}

	if (force_sec)
		is_sec = true;

	return is_sec;
}
#endif
static int _DL_switch_to_DC_fast(void)
{
	int ret = 0;

	RDMA_CONFIG_STRUCT rdma_config = decouple_rdma_config;
	WDMA_CONFIG_STRUCT wdma_config = decouple_wdma_config;

	disp_ddp_path_config *data_config_dl = NULL;
	disp_ddp_path_config *data_config_dc = NULL;
	unsigned int mva = pgc->dc_buf[pgc->dc_buf_id];
	#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
	bool is_sec = _is_sec(pgc->dpmgr_handle);

	if (is_sec) {
		mva = pgc->dc_buf_sec[0];
		wdma_config.security = DISP_SECURE_BUFFER;
	} else
	#endif
	{
		wdma_config.security = DISP_NORMAL_BUFFER;
	}
	wdma_config.dstAddress = mva;

	/* disable SODI by CPU */
	disp_set_sodi(0, 0);
	/* 1.save a temp frame to intermediate buffer */
	directlink_path_add_memory(&wdma_config);

	/* 2.reset primary handle */
	_cmdq_reset_config_handle();
	if (primary_display_is_video_mode() == 1)
		disp_set_sodi(0, pgc->cmdq_handle_config);
	_cmdq_insert_wait_frame_done_token();

	/* 3.modify interface path handle to new scenario(rdma->dsi) */
	if (primary_display_is_video_mode() == 1) {
		cmdqRecEnablePrefetch(pgc->cmdq_handle_config);
		disp_set_sodi(0, pgc->cmdq_handle_config);
		cmdqRecDisablePrefetch(pgc->cmdq_handle_config);
	}
	dpmgr_modify_path(pgc->dpmgr_handle, DDP_SCENARIO_PRIMARY_RDMA0_COLOR0_DISP,
			  pgc->cmdq_handle_config,
			  primary_display_is_video_mode() ? DDP_VIDEO_MODE : DDP_CMD_MODE);
	/* 4.config rdma from directlink mode to memory mode */
	#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
	if (is_sec) {
		rdma_config.security = DISP_SECURE_BUFFER;
	} else
	#endif
	{
		rdma_config.security = DISP_NORMAL_BUFFER;
	}
	rdma_config.address = mva;
	rdma_config.pitch =
	    primary_display_get_width() * DP_COLOR_BITS_PER_PIXEL(rdma_config.inputFormat) >> 3;

	data_config_dl = dpmgr_path_get_last_config(pgc->dpmgr_handle);

	data_config_dl->rdma_config = rdma_config;
	data_config_dl->rdma_dirty = 1;
	ret = dpmgr_path_config(pgc->dpmgr_handle, data_config_dl, pgc->cmdq_handle_config);

	/* 5. backup rdma address to slots */
	cmdqRecBackupUpdateSlot(pgc->cmdq_handle_config, pgc->rdma_buff_info, 0, mva);

	/* 6 .flush to cmdq */
	_cmdq_set_config_handle_dirty();
	_cmdq_flush_config_handle(1, NULL, 0);

	/* ddp_mmp_rdma_layer(&rdma_config, 0,  20, 20); */

	/* 7.reset  cmdq */
	_cmdq_reset_config_handle();
	_cmdq_insert_wait_frame_done_token();

	/* 9. create ovl2mem path handle */
	cmdqRecReset(pgc->cmdq_handle_ovl1to2_config);
	pgc->ovl2mem_path_handle =
	    dpmgr_create_path(DDP_SCENARIO_PRIMARY_OVL_MEMOUT, pgc->cmdq_handle_ovl1to2_config);
	if (pgc->ovl2mem_path_handle) {
		DISPDBG("dpmgr create ovl memout path SUCCESS(%p)\n", pgc->ovl2mem_path_handle);
	} else {
		DISPCHECK("dpmgr create path FAIL\n");
		return -1;
	}

	dpmgr_path_set_video_mode(pgc->ovl2mem_path_handle, 0);
	dpmgr_path_init(pgc->ovl2mem_path_handle, CMDQ_ENABLE);

	data_config_dc = dpmgr_path_get_last_config(pgc->ovl2mem_path_handle);

	data_config_dc->dst_w = rdma_config.width;
	data_config_dc->dst_h = rdma_config.height;
	data_config_dc->dst_dirty = 1;

	/* move ovl config info from dl to dc */
	memcpy(data_config_dc->ovl_config, data_config_dl->ovl_config,
	       sizeof(data_config_dl->ovl_config));

	ret =
	    dpmgr_path_config(pgc->ovl2mem_path_handle, data_config_dc,
			      pgc->cmdq_handle_ovl1to2_config);
	ret = dpmgr_path_start(pgc->ovl2mem_path_handle, CMDQ_ENABLE);

	/* use blocking flush to make sure all config is done. */

	/* cmdqRecDumpCommand(pgc->cmdq_handle_ovl1to2_config); */
	cmdqRecClearEventToken(pgc->cmdq_handle_ovl1to2_config, CMDQ_EVENT_DISP_WDMA0_EOF);
	_cmdq_flush_config_handle_mira(pgc->cmdq_handle_ovl1to2_config, 1);
	cmdqRecReset(pgc->cmdq_handle_ovl1to2_config);

	/* 11..enable event for new path */
/* dpmgr_enable_event(pgc->ovl2mem_path_handle, DISP_PATH_EVENT_FRAME_COMPLETE); */
/* dpmgr_map_event_to_irq(pgc->ovl2mem_path_handle, DISP_PATH_EVENT_FRAME_START, DDP_IRQ_WDMA0_FRAME_COMPLETE); */
/* dpmgr_enable_event(pgc->ovl2mem_path_handle, DISP_PATH_EVENT_FRAME_START); */

	if (primary_display_is_video_mode()) {
		dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
				       DDP_IRQ_RDMA0_DONE);
	}
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
	/* dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE); */
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);

#ifdef MTK_DISP_IDLE_LP
	atomic_set(&isDdp_Idle, 1);
#endif

	/*enable SODI after switch */
	disp_set_sodi(1, 0);
	return ret;
}

static int _DC_switch_to_DL_fast(void)
{
	int ret = 0;
	int layer = 0;
	disp_ddp_path_config *data_config_dl = NULL;
	disp_ddp_path_config *data_config_dc = NULL;
	OVL_CONFIG_STRUCT ovl_config[OVL_LAYER_NUM];
	/* 1. disable SODI */
	disp_set_sodi(0, 0);

	/* 2.enable ovl/wdma clock */

	/* 3.destroy ovl->mem path. */
	data_config_dc = dpmgr_path_get_last_config(pgc->ovl2mem_path_handle);
	/*save ovl info */;
	memcpy(ovl_config, data_config_dc->ovl_config, sizeof(ovl_config));

	dpmgr_path_deinit(pgc->ovl2mem_path_handle,
			  (int)(unsigned long)pgc->cmdq_handle_ovl1to2_config);
	dpmgr_destroy_path(pgc->ovl2mem_path_handle, pgc->cmdq_handle_ovl1to2_config);
	/*clear sof token for next dl to dc */
	cmdqRecClearEventToken(pgc->cmdq_handle_ovl1to2_config, CMDQ_EVENT_DISP_WDMA0_SOF);

	_cmdq_flush_config_handle_mira(pgc->cmdq_handle_ovl1to2_config, 1);
	cmdqRecReset(pgc->cmdq_handle_ovl1to2_config);
	pgc->ovl2mem_path_handle = NULL;

	/* release output buffer */
	layer = disp_sync_get_output_timeline_id();
	mtkfb_release_layer_fence(primary_session_id, layer);


	/* 4.modify interface path handle to new scenario(rdma->dsi) */
	_cmdq_reset_config_handle();
	_cmdq_insert_wait_frame_done_token();
	/*disable SODI, bceause ESD check may have already enable SODI */
	if (primary_display_is_video_mode() == 1) {
		cmdqRecEnablePrefetch(pgc->cmdq_handle_config);
		disp_set_sodi(0, pgc->cmdq_handle_config);
		cmdqRecDisablePrefetch(pgc->cmdq_handle_config);
	}

	dpmgr_modify_path(pgc->dpmgr_handle, DDP_SCENARIO_PRIMARY_DISP, pgc->cmdq_handle_config,
			  primary_display_is_video_mode() ? DDP_VIDEO_MODE : DDP_CMD_MODE);

	/* 5.config rdma from memory mode to directlink mode */
	data_config_dl = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	data_config_dl->rdma_config = decouple_rdma_config;
	data_config_dl->rdma_config.address = 0;
	data_config_dl->rdma_config.pitch = 0;
	data_config_dl->rdma_config.security = DISP_NORMAL_BUFFER;
	data_config_dl->rdma_dirty = 1;
	memcpy(data_config_dl->ovl_config, ovl_config, sizeof(ovl_config));
	ret = dpmgr_path_config(pgc->dpmgr_handle, data_config_dl, pgc->cmdq_handle_config);

	/* use blocking flush to make sure all config is done, then stop/start trigger loop */

	cmdqRecBackupUpdateSlot(pgc->cmdq_handle_config, pgc->rdma_buff_info, 0, 0);

	/* cmdqRecDumpCommand(pgc->cmdq_handle_config); */
	_cmdq_set_config_handle_dirty();
	_cmdq_flush_config_handle(1, NULL, 0);

	/* release output buffer */
	layer = disp_sync_get_output_interface_timeline_id();
	mtkfb_release_layer_fence(primary_session_id, layer);

	_cmdq_reset_config_handle();
	_cmdq_insert_wait_frame_done_token();

	/* 9.enable event for new path */
	if (primary_display_is_video_mode()) {
		dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
				       DDP_IRQ_RDMA0_DONE);
	}
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);

#ifdef MTK_DISP_IDLE_LP
	atomic_set(&isDdp_Idle, 0);
	atomic_set(&idle_detect_flag, 1);
	wake_up(&idle_detect_wq);
#endif
	/* 1. enable SODI */
	disp_set_sodi(1, 0);
	return ret;
}

const char *session_mode_spy(unsigned int mode)
{
	switch (mode) {
	case DISP_SESSION_DIRECT_LINK_MODE:
		return "DIRECT_LINK";
	case DISP_SESSION_DIRECT_LINK_MIRROR_MODE:
		return "DIRECT_LINK_MIRROR";
	case DISP_SESSION_DECOUPLE_MODE:
		return "DECOUPLE";
	case DISP_SESSION_DECOUPLE_MIRROR_MODE:
		return "DECOUPLE_MIRROR";
	default:
		return "UNKNOWN";
	}
}

static int config_display_m4u_port(void)
{
	int ret = 0;
#ifndef MTKFB_NO_M4U
	M4U_PORT_STRUCT sPort;

	sPort.ePortID = M4U_PORT_DISP_OVL0;
	sPort.Virtuality = primary_display_use_m4u;
	sPort.Security = 0;
	sPort.Distance = 1;
	sPort.Direction = 0;
	ret = m4u_config_port(&sPort);
	if (ret == 0) {
		DISPCHECK("config M4U Port %s to %s SUCCESS\n",
			  ddp_get_module_name(DISP_MODULE_OVL0),
			  primary_display_use_m4u ? "virtual" : "physical");
	} else {
		DISPCHECK("config M4U Port %s to %s FAIL(ret=%d)\n",
			  ddp_get_module_name(DISP_MODULE_OVL0),
			  primary_display_use_m4u ? "virtual" : "physical", ret);
		return -1;
	}
#ifdef OVL_CASCADE_SUPPORT
	sPort.ePortID = M4U_PORT_DISP_OVL1;
	ret = m4u_config_port(&sPort);
	if (ret) {
		DISPCHECK("config M4U Port %s to %s FAIL(ret=%d)\n",
			  ddp_get_module_name(DISP_MODULE_OVL1),
			  primary_display_use_m4u ? "virtual" : "physical", ret);
		return -1;
	}
#endif
	sPort.ePortID = M4U_PORT_DISP_RDMA0;
	ret = m4u_config_port(&sPort);
	if (ret) {
		DISPCHECK("config M4U Port %s to %s FAIL(ret=%d)\n",
			  ddp_get_module_name(DISP_MODULE_RDMA0),
			  primary_display_use_m4u ? "virtual" : "physical", ret);
		return -1;
	}
	sPort.ePortID = M4U_PORT_DISP_WDMA0;
	ret = m4u_config_port(&sPort);
	if (ret) {
		DISPCHECK("config M4U Port %s to %s FAIL(ret=%d)\n",
			  ddp_get_module_name(DISP_MODULE_WDMA0),
			  primary_display_use_m4u ? "virtual" : "physical", ret);
		return -1;
	}
#endif
	return ret;
}

#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
KREE_SESSION_HANDLE primary_display_secure_memory_session_handle(void)
{
	static KREE_SESSION_HANDLE disp_secure_memory_session;

	/* TODO: the race condition here is not taken into consideration. */
	if (!disp_secure_memory_session) {
		TZ_RESULT ret;

		ret = KREE_CreateSession(TZ_TA_MEM_UUID, &disp_secure_memory_session);
		if (ret != TZ_RESULT_SUCCESS) {
			DISPERR("KREE_CreateSession fail, ret=%d\n", ret);
			return 0;
		}
	}

	return disp_secure_memory_session;
}

static KREE_SECUREMEM_HANDLE allocate_decouple_sec_buffer(unsigned int buffer_size)
{
	TZ_RESULT ret;
	KREE_SECUREMEM_HANDLE mem_handle;

	/* allocate secure buffer by tz api */
	ret = KREE_AllocSecurechunkmemWithTag(primary_display_secure_memory_session_handle(),
				       &mem_handle, 0, buffer_size, "primary_disp");
	if (ret != TZ_RESULT_SUCCESS) {
		DISPERR("KREE_AllocSecurechunkmemWithTag fail, ret = %d\n", ret);
		return -1;
	}
	DISPCHECK("KREE_AllocSecurechunkmemWithTag handle=0x%x\n", mem_handle);

	return mem_handle;
}

static KREE_SECUREMEM_HANDLE free_decouple_sec_buffer(KREE_SECUREMEM_HANDLE mem_handle)
{
	TZ_RESULT ret;

	ret = KREE_UnreferenceSecurechunkmem(primary_display_secure_memory_session_handle(), mem_handle);

	if (ret != TZ_RESULT_SUCCESS)
		DISPERR("KREE_UnreferenceSecurechunkmem fail, ret=%d\n", ret);

	DISPCHECK("KREE_UnreferenceSecurechunkmem handle=0x%x\n", mem_handle);

	return ret;
}
#endif

static disp_internal_buffer_info *allocat_decouple_buffer(int size)
{
	void *buffer_va = NULL;
	unsigned int buffer_mva = 0;
	unsigned int mva_size = 0;
	struct ion_client *client = NULL;
	struct ion_handle *handle = NULL;
	disp_internal_buffer_info *buf_info = NULL;

	struct ion_mm_data mm_data;

	memset((void *)&mm_data, 0, sizeof(struct ion_mm_data));



	client = ion_client_create(g_ion_device, "disp_decouple");

	buf_info = kzalloc(sizeof(disp_internal_buffer_info), GFP_KERNEL);
	if (buf_info) {
		handle = ion_alloc(client, size, 0, ION_HEAP_MULTIMEDIA_MASK, 0);
		if (IS_ERR(handle)) {
			DISPERR("Fatal Error, ion_alloc for size %d failed\n", size);
			ion_free(client, handle);
			ion_client_destroy(client);
			kfree(buf_info);
			return NULL;
		}

		buffer_va = ion_map_kernel(client, handle);
		if (buffer_va == NULL) {
			DISPERR("ion_map_kernrl failed\n");
			ion_free(client, handle);
			ion_client_destroy(client);
			kfree(buf_info);
			return NULL;
		}
		mm_data.config_buffer_param.kernel_handle = handle;
		mm_data.mm_cmd = ION_MM_CONFIG_BUFFER;
		if (ion_kernel_ioctl(client, ION_CMD_MULTIMEDIA, (unsigned long)&mm_data) < 0) {
			DISPERR("ion_test_drv: Config buffer failed.\n");
			ion_free(client, handle);
			ion_client_destroy(client);
			kfree(buf_info);
			return NULL;
		}

		ion_phys(client, handle, (ion_phys_addr_t *) &buffer_mva, (size_t *) &mva_size);
		if (buffer_mva == 0) {
			DISPERR("Fatal Error, get mva failed\n");
			ion_free(client, handle);
			ion_client_destroy(client);
			kfree(buf_info);
			return NULL;
		}
		buf_info->handle = handle;
		buf_info->mva = buffer_mva;
		buf_info->size = mva_size;
		buf_info->va = buffer_va;
	} else {
		DISPERR("Fatal error, kzalloc internal buffer info failed!!\n");
		kfree(buf_info);
		return NULL;
	}

	return buf_info;
}

static int init_decouple_buffers(void)
{
	int i = 0;
	int height = primary_display_get_height();
	int width = primary_display_get_width();
	int bpp = 24;   /*internal picture is RGB888*/

	int buffer_size = width * height * bpp / 8;

	for (i = 0; i < DISP_INTERNAL_BUFFER_COUNT; i++) {
		if (decouple_buffer_info[i] == NULL)
			decouple_buffer_info[i] = allocat_decouple_buffer(buffer_size);

		if (decouple_buffer_info[i] != NULL) {
			pgc->dc_buf[i] = decouple_buffer_info[i]->mva;
			dc_vAddr[i] = (unsigned long)decouple_buffer_info[i]->va;

		}
	}
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
	for (i = 0; i < DISP_INTERNAL_BUFFER_COUNT; i++)
		pgc->dc_buf_sec[i] = 0;
#endif

	/*initialize rdma config */
	decouple_rdma_config.height = height;
	decouple_rdma_config.width = width;
	decouple_rdma_config.idx = 0;
	decouple_rdma_config.inputFormat = eRGB888;
	decouple_rdma_config.pitch = width * DP_COLOR_BITS_PER_PIXEL(eRGB888) / 8;
	decouple_rdma_config.security = DISP_NORMAL_BUFFER;

	/*initialize wdma config */
	decouple_wdma_config.srcHeight = height;
	decouple_wdma_config.srcWidth = width;
	decouple_wdma_config.clipX = 0;
	decouple_wdma_config.clipY = 0;
	decouple_wdma_config.clipHeight = height;
	decouple_wdma_config.clipWidth = width;
	decouple_wdma_config.outputFormat = eRGB888;
	decouple_wdma_config.useSpecifiedAlpha = 1;
	decouple_wdma_config.alpha = 0xFF;
	decouple_wdma_config.dstPitch = width * DP_COLOR_BITS_PER_PIXEL(eRGB888) / 8;
	decouple_wdma_config.security = DISP_NORMAL_BUFFER;
	return 0;
}

static int __build_path_direct_link(void)
{
	int ret = 0;

	DISP_MODULE_ENUM dst_module = 0;

	DISPFUNC();
	pgc->mode = DIRECT_LINK_MODE;

	pgc->dpmgr_handle = dpmgr_create_path(DDP_SCENARIO_PRIMARY_DISP, pgc->cmdq_handle_config);
	if (pgc->dpmgr_handle) {
		DISPCHECK("dpmgr create path SUCCESS(0x%p)\n", pgc->dpmgr_handle);
	} else {
		DISPCHECK("dpmgr create path FAIL\n");
		return -1;
	}

	dst_module = _get_dst_module_by_lcm(pgc->plcm);
	dpmgr_path_set_dst_module(pgc->dpmgr_handle, dst_module);
	DISPCHECK("dpmgr set dst module FINISHED(%s)\n", ddp_get_module_name(dst_module));
	init_decouple_buffers();
	dpmgr_set_lcm_utils(pgc->dpmgr_handle, pgc->plcm->drv);

	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);

	/* set video mode must before path_init */
	dpmgr_path_set_video_mode(pgc->dpmgr_handle, primary_display_is_video_mode());
	dpmgr_path_init(pgc->dpmgr_handle, CMDQ_ENABLE);
	config_display_m4u_port();

	return ret;
}

/* #define CONFIG_USE_CMDQ */
static int __build_path_decouple(void)
{
	int ret = 0;
	int i = 0;
	disp_ddp_path_config *pconfig = NULL;
	uint32_t writing_mva = 0;

	DISP_MODULE_ENUM dst_module = 0;

	DISPFUNC();
	pgc->mode = DECOUPLE_MODE;

	mutex_init(&(pgc->dc_lock));

	pgc->dpmgr_handle =
	    dpmgr_create_path(DDP_SCENARIO_PRIMARY_RDMA0_COLOR0_DISP, pgc->cmdq_handle_config);
	if (pgc->dpmgr_handle) {
		DISPCHECK("dpmgr create interface path SUCCESS(0x%p)\n", pgc->dpmgr_handle);
	} else {
		DISPCHECK("dpmgr create path FAIL\n");
		return -1;
	}

	dpmgr_set_lcm_utils(pgc->dpmgr_handle, pgc->plcm->drv);

	dst_module = _get_dst_module_by_lcm(pgc->plcm);
	dpmgr_path_set_dst_module(pgc->dpmgr_handle, dst_module);

	dpmgr_path_set_video_mode(pgc->dpmgr_handle, primary_display_is_video_mode());

#ifdef CONFIG_USE_CMDQ
	dpmgr_path_init(pgc->dpmgr_handle, CMDQ_ENABLE);
#else
	dpmgr_path_init(pgc->dpmgr_handle, CMDQ_DISABLE);
#endif

	DISPCHECK("dpmgr set dst module FINISHED(%s)\n", ddp_get_module_name(dst_module));

	pgc->ovl2mem_path_handle =
	    dpmgr_create_path(DDP_SCENARIO_PRIMARY_OVL_MEMOUT, pgc->cmdq_handle_config);
	if (pgc->ovl2mem_path_handle) {
		DISPCHECK("dpmgr create ovl memout path SUCCESS(0x%p)\n", pgc->ovl2mem_path_handle);
	} else {
		DISPCHECK("dpmgr create path FAIL\n");
		return -1;
	}
	dpmgr_path_set_video_mode(pgc->ovl2mem_path_handle, 0);
#ifdef CONFIG_USE_CMDQ
	dpmgr_path_init(pgc->ovl2mem_path_handle, CMDQ_ENABLE);
#else
	dpmgr_path_init(pgc->ovl2mem_path_handle, CMDQ_DISABLE);
#endif
/* dpmgr_path_power_on(pgc->ovl2mem_path_handle, CMDQ_DISABLE); */

	{
		M4U_PORT_STRUCT sPort;

		sPort.ePortID = M4U_PORT_DISP_OVL0;
		sPort.Virtuality = primary_display_use_m4u;
		sPort.Security = 0;
		sPort.Distance = 1;
		sPort.Direction = 0;
		ret = m4u_config_port(&sPort);
		if (ret != 0) {
			DISPCHECK("config M4U Port %s to %s FAIL(ret=%d)\n",
				  ddp_get_module_name(DISP_MODULE_OVL0),
				  primary_display_use_m4u ? "virtual" : "physical", ret);
			return -1;
		}

		sPort.ePortID = M4U_PORT_DISP_RDMA0;
		ret = m4u_config_port(&sPort);
		if (ret != 0) {
			DISPCHECK("config M4U Port %s to %s FAIL(ret=%d)\n",
				  ddp_get_module_name(DISP_MODULE_OVL0),
				  primary_display_use_m4u ? "virtual" : "physical", ret);
			return -1;
		}

		sPort.ePortID = M4U_PORT_DISP_WDMA0;
		ret = m4u_config_port(&sPort);
		if (ret != 0) {
			DISPCHECK("config M4U Port %s to %s FAIL(ret=%d)\n",
				  ddp_get_module_name(DISP_MODULE_OVL0),
				  primary_display_use_m4u ? "virtual" : "physical", ret);
			return -1;
		}
#ifdef OVL_CASCADE_SUPPORT
		sPort.ePortID = M4U_PORT_DISP_OVL1;
		ret = m4u_config_port(&sPort);
		if (ret) {
			DISPCHECK("config M4U Port %s to %s FAIL(ret=%d)\n",
				  ddp_get_module_name(DISP_MODULE_OVL1),
				  primary_display_use_m4u ? "virtual" : "physical", ret);
			return -1;
		}
#endif
	}

	/* VSYNC event will be provided to hwc for system vsync hw source
	 * FRAME_DONE will be used in esd/suspend/resume for path status check
	 * FRAME_START will be used in decouple-mirror mode, for post-path fence release(rdma->dsi)
	 */
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);

	dpmgr_enable_event(pgc->ovl2mem_path_handle, DISP_PATH_EVENT_FRAME_COMPLETE);
	dpmgr_enable_event(pgc->ovl2mem_path_handle, DISP_PATH_EVENT_FRAME_START);

	{
		unsigned long *pSrc;
		unsigned int src_mva;
		unsigned int size, tmp_size;


		struct ion_mm_data mm_data;
		struct ion_client *ion_client;
		struct ion_handle *src_handle;

		disp_internal_buffer_info *buf = NULL;

		INIT_LIST_HEAD(&pgc->dc_free_list);
		INIT_LIST_HEAD(&pgc->dc_writing_list);
		INIT_LIST_HEAD(&pgc->dc_reading_list);

		size =
		    primary_display_get_width() * primary_display_get_height() *
		    primary_display_get_bpp() / 8;

		ion_client = ion_client_create(g_ion_device, "disp_decouple");

		for (i = 0; i < DISP_INTERNAL_BUFFER_COUNT; i++) {
			buf = kzalloc(sizeof(disp_internal_buffer_info), GFP_KERNEL);
			if (buf) {
				DISPMSG("buf=0x%p\n", buf);
				/* list_add_tail(&buf->list, &pgc->dc_free_list); */
				src_handle =
				    ion_alloc(ion_client, size, 0, ION_HEAP_MULTIMEDIA_MASK, 0);
				if (IS_ERR(src_handle)) {
					DISPERR("Fatal Error, ion_alloc for size %d failed\n",
						size);
					return -1;
				}

				pSrc = ion_map_kernel(ion_client, src_handle);
				if (pSrc == NULL) {
					DISPERR("ion_map_kernrl failed\n");
					return -1;
				}

				memset((void *)&mm_data, 0, sizeof(struct ion_mm_data));
				mm_data.config_buffer_param.kernel_handle = src_handle;
				/* mm_data.config_buffer_param.m4u_port= M4U_PORT_DISP_OVL0; */
				/* mm_data.config_buffer_param.prot = M4U_PROT_READ|M4U_PROT_WRITE; */
				/* mm_data.config_buffer_param.flags = M4U_FLAGS_SEQ_ACCESS; */
				mm_data.mm_cmd = ION_MM_CONFIG_BUFFER;
				if (ion_kernel_ioctl
				    (ion_client, ION_CMD_MULTIMEDIA, (unsigned long)&mm_data) < 0) {
					DISPERR("ion_test_drv: Config buffer failed.\n");
					return -1;
				}

				ion_phys(ion_client, src_handle, (ion_phys_addr_t *) &src_mva,
					 (size_t *) &tmp_size);
				if (!src_mva) {
					DISPERR("Fatal Error, get mva failed\n");
					return -1;
				}
				buf->handle = src_handle;
				buf->mva = src_mva;
				buf->size = tmp_size;
				pgc->dc_buf[i] = buf->mva;
				dc_vAddr[i] = (unsigned long)pSrc;
				DISPMSG("buf:0x%p, buf->list:0x%lx, buf->mva:0x%08x, buf->size:0x%08x, ",
					     buf, (unsigned long)(&buf->list), buf->mva, buf->size);
				DISPMSG("buf->handle:0x%p,dc_vaddr = 0x%lx\n",
					buf->handle, dc_vAddr[i]);
				/* memcpy from FBMEM to DC buffer for video mode flash */
				if (0 == i) {
					unsigned int line = 0;

					for (line = 0; line < primary_display_get_height(); line++)
						memcpy((void *)(dc_vAddr[0] +
								line * primary_display_get_width() *
								primary_display_get_bpp() / 8),
						       (void *)pgc->framebuffer_va +
						       line * ALIGN_TO(primary_display_get_width(),
								       MTK_FB_ALIGNMENT) *
						       primary_display_get_bpp() / 8,
						       primary_display_get_width() *
						       primary_display_get_bpp() / 8);
				}
			} else {
				DISPERR("Fatal error, kzalloc internal buffer info failed!!\n");
				return -1;
			}
		}
	}


#if 0
	/* queue all buffer into free list */
	disp_sw_mutex_lock(&(pgc->dc_lock));

	/* for(i = 0;i<DISP_INTERNAL_BUFFER_COUNT;i++) */
	/* { */
	writing_buf = dequeue_buffer(pgc, &pgc->dc_free_list);
	if (writing_buf) {
		DISPMSG("queue buf:0x%08x into free_list\n", writing_buf);
		enqueue_buffer(pgc, &pgc->dc_reading_list, writing_buf);
	} else {
		DISPERR("dc_free_list is empty\n");
	}
	/* } */

	disp_sw_mutex_unlock(&(pgc->dc_lock));
#endif
	pconfig = dpmgr_path_get_last_config(pgc->ovl2mem_path_handle);

	writing_mva = pgc->dc_buf[pgc->dc_buf_id];
	DISPMSG("writing_mva = 0x%08x\n", writing_mva);
	pgc->dc_buf_id++;
	pgc->dc_buf_id %= DISP_INTERNAL_BUFFER_COUNT;
	pconfig->wdma_config.dstAddress = writing_mva;
	pconfig->wdma_config.srcHeight = primary_display_get_height();
	pconfig->wdma_config.srcWidth = primary_display_get_width();
	pconfig->wdma_config.clipX = 0;
	pconfig->wdma_config.clipY = 0;
	pconfig->wdma_config.clipHeight = primary_display_get_height();
	pconfig->wdma_config.clipWidth = primary_display_get_width();
	pconfig->wdma_config.outputFormat = eRGB888;
	pconfig->wdma_config.useSpecifiedAlpha = 1;
	pconfig->wdma_config.alpha = 0xFF;
	pconfig->wdma_config.dstPitch =
	    primary_display_get_width() * DP_COLOR_BITS_PER_PIXEL(eRGB888) / 8;
	pconfig->wdma_dirty = 1;

	/* ovl need dst_dirty to set background color */
	pconfig->dst_w = primary_display_get_width();
	pconfig->dst_h = primary_display_get_height();
	pconfig->dst_dirty = 1;

#ifdef CONFIG_USE_CMDQ
	ret = dpmgr_path_config(pgc->ovl2mem_path_handle, pconfig, pgc->cmdq_handle_ovl1to2_config);
	ret = dpmgr_path_start(pgc->ovl2mem_path_handle, CMDQ_ENABLE);
#else
	dpmgr_path_reset(pgc->ovl2mem_path_handle, CMDQ_DISABLE);
	ret = dpmgr_path_config(pgc->ovl2mem_path_handle, pconfig, NULL);
	ret = dpmgr_path_start(pgc->ovl2mem_path_handle, CMDQ_DISABLE);
#endif

	/* all dirty should be cleared in dpmgr_path_get_last_config() */
	pconfig = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	pconfig->rdma_config.address = (unsigned int)writing_mva;
	pconfig->rdma_config.width = primary_display_get_width();
	pconfig->rdma_config.height = primary_display_get_height();
	pconfig->rdma_config.inputFormat = eRGB888;
	pconfig->rdma_config.pitch = primary_display_get_width() * 4;
	pconfig->rdma_dirty = 1;
#ifdef CONFIG_USE_CMDQ
	ret = dpmgr_path_config(pgc->dpmgr_handle, pconfig, pgc->cmdq_handle_config);
/* ret = dpmgr_path_start(pgc->dpmgr_handle, CMDQ_ENABLE); */
#else
	ret = dpmgr_path_config(pgc->dpmgr_handle, pconfig, NULL);
/* ret = dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE); */
#endif
#if 0
	primary_path_decouple_worker_task =
	    kthread_create(_disp_primary_path_decouple_worker_thread, NULL,
			   "display_decouple_worker");
	wake_up_process(primary_path_decouple_worker_task);
#endif

	DISPCHECK("build decouple path finished\n");
	return ret;
}

static int __build_path_single_layer(void)
{
	return 0;
}

static int __build_path_debug_rdma1_dsi0(void)
{
	int ret = 0;

	DISP_MODULE_ENUM dst_module = 0;

	pgc->mode = DEBUG_RDMA1_DSI0_MODE;

	pgc->dpmgr_handle = dpmgr_create_path(DDP_SCENARIO_SUB_RDMA1_DISP, pgc->cmdq_handle_config);
	if (pgc->dpmgr_handle) {
		DISPCHECK("dpmgr create path SUCCESS(0x%p)\n", pgc->dpmgr_handle);
	} else {
		DISPCHECK("dpmgr create path FAIL\n");
		return -1;
	}

	dst_module = _get_dst_module_by_lcm(pgc->plcm);
	dpmgr_path_set_dst_module(pgc->dpmgr_handle, dst_module);
	DISPCHECK("dpmgr set dst module FINISHED(%s)\n", ddp_get_module_name(dst_module));

	{
		M4U_PORT_STRUCT sPort;

		sPort.ePortID = M4U_PORT_DISP_RDMA1;
		sPort.Virtuality = primary_display_use_m4u;
		sPort.Security = 0;
		sPort.Distance = 1;
		sPort.Direction = 0;
		ret = m4u_config_port(&sPort);
		if (ret == 0) {
			DISPCHECK("config M4U Port %s to %s SUCCESS\n",
				  ddp_get_module_name(DISP_MODULE_RDMA1),
				  primary_display_use_m4u ? "virtual" : "physical");
		} else {
			DISPCHECK("config M4U Port %s to %s FAIL(ret=%d)\n",
				  ddp_get_module_name(DISP_MODULE_RDMA1),
				  primary_display_use_m4u ? "virtual" : "physical", ret);
			return -1;
		}
	}

	dpmgr_set_lcm_utils(pgc->dpmgr_handle, pgc->plcm->drv);

	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE);

	return ret;
}


int disp_fmt_to_hw_fmt(DISP_FORMAT src_fmt, unsigned int *ovl_fmt,
		       unsigned int *Bpp, unsigned int *bpp)
{
	switch (src_fmt) {
	case DISP_FORMAT_YUV422:
		*ovl_fmt = eYUY2;
		*Bpp = 2;
		*bpp = 16;
		break;

	case DISP_FORMAT_RGB565:
		*ovl_fmt = eRGB565;
		*Bpp = 2;
		*bpp = 16;
		break;

	case DISP_FORMAT_RGB888:
		*ovl_fmt = eRGB888;
		*Bpp = 3;
		*bpp = 24;
		break;

	case DISP_FORMAT_BGR888:
		*ovl_fmt = eBGR888;
		*Bpp = 3;
		*bpp = 24;
		break;

	case DISP_FORMAT_ARGB8888:
		*ovl_fmt = eARGB8888;
		*Bpp = 4;
		*bpp = 32;
		break;

	case DISP_FORMAT_ABGR8888:
		*ovl_fmt = eABGR8888;
		*Bpp = 4;
		*bpp = 32;
		break;
	case DISP_FORMAT_RGBA8888:
		*ovl_fmt = eRGBA8888;
		*Bpp = 4;
		*bpp = 32;
		break;

	case DISP_FORMAT_BGRA8888:
		/* *ovl_fmt = eABGR8888; */
		*ovl_fmt = eBGRA8888;
		*Bpp = 4;
		*bpp = 32;
		break;
	case DISP_FORMAT_XRGB8888:
		*ovl_fmt = eARGB8888;
		*Bpp = 4;
		*bpp = 32;
		break;

	case DISP_FORMAT_XBGR8888:
		*ovl_fmt = eABGR8888;
		*Bpp = 4;
		*bpp = 32;
		break;

	case DISP_FORMAT_RGBX8888:
		*ovl_fmt = eRGBA8888;
		*Bpp = 4;
		*bpp = 32;
		break;

	case DISP_FORMAT_BGRX8888:
		*ovl_fmt = eBGRA8888;
		*Bpp = 4;
		*bpp = 32;
		break;

	case DISP_FORMAT_UYVY:
		*ovl_fmt = eUYVY;
		*Bpp = 2;
		*bpp = 16;
		break;

	case DISP_FORMAT_YV12:
		*ovl_fmt = eYV12;
		*Bpp = 1;
		*bpp = 8;
		break;
	default:
		DISPERR("Invalid color format: 0x%x\n", src_fmt);
		return -1;
	}

	return 0;
}

static int _convert_disp_input_to_ovl(OVL_CONFIG_STRUCT *dst, disp_input_config *src)
{
	int ret;
	unsigned int Bpp = 0;
	unsigned int bpp = 0;

	if (!src || !dst) {
		DISP_FATAL_ERR("display", "%s src(0x%p) or dst(0x%p) is null\n",
			       __func__, src, dst);
		return -1;
	}

	dst->layer = src->layer_id;
	dst->isDirty = 1;
	dst->buff_idx = src->next_buff_idx;
	dst->layer_en = src->layer_enable;

	/* if layer is disable, we just needs config above params. */
	if (!src->layer_enable)
		return 0;

	ret =
	    disp_fmt_to_hw_fmt(src->src_fmt, (unsigned int *)(&(dst->fmt)), (unsigned int *)(&Bpp),
			       (unsigned int *)(&bpp));

	dst->addr = (unsigned long)src->src_phy_addr;
	dst->vaddr = (unsigned long)src->src_base_addr;
	dst->src_x = src->src_offset_x;
	dst->src_y = src->src_offset_y;
	dst->src_w = src->src_width;
	dst->src_h = src->src_height;
	dst->src_pitch = src->src_pitch * Bpp;
	dst->dst_x = src->tgt_offset_x;
	dst->dst_y = src->tgt_offset_y;

	/* dst W/H should <= src W/H */
	if (src->buffer_source != DISP_BUFFER_ALPHA) {
		/* dim layer do not care for src_width */
		dst->dst_w = min(src->src_width, src->tgt_width);
		dst->dst_h = min(src->src_height, src->tgt_height);
	} else {
		dst->dst_w = src->tgt_width;
		dst->dst_h = src->tgt_height;
	}

	dst->keyEn = src->src_use_color_key;
	dst->key = src->src_color_key;


	dst->aen = src->alpha_enable;
	dst->alpha = src->alpha;
	dst->sur_aen = src->sur_aen;
	dst->src_alpha = src->src_alpha;
	dst->dst_alpha = src->dst_alpha;

#ifdef DISP_DISABLE_X_CHANNEL_ALPHA
	if (DISP_FORMAT_ARGB8888 == src->src_fmt ||
	    DISP_FORMAT_ABGR8888 == src->src_fmt ||
	    DISP_FORMAT_RGBA8888 == src->src_fmt ||
	    DISP_FORMAT_BGRA8888 == src->src_fmt || src->buffer_source == DISP_BUFFER_ALPHA) {
		/* nothing */
	} else {
		dst->aen = false;
		dst->sur_aen = false;
	}
#endif

	dst->identity = src->identity;
	dst->connected_type = src->connected_type;
	dst->security = src->security;
	dst->yuv_range = src->yuv_range;

	if (src->buffer_source == DISP_BUFFER_ALPHA) {
		dst->source = OVL_LAYER_SOURCE_RESERVED;	/* dim layer, constant alpha */
	} else if (src->buffer_source == DISP_BUFFER_ION || src->buffer_source == DISP_BUFFER_MVA) {
		dst->source = OVL_LAYER_SOURCE_MEM;	/* from memory */
	} else {
		DISPERR("unknown source = %d", src->buffer_source);
		dst->source = OVL_LAYER_SOURCE_MEM;
	}

	return ret;
}

static int _convert_disp_input_to_rdma(RDMA_CONFIG_STRUCT *dst, disp_input_config *src)
{
	int ret;
	unsigned int Bpp = 0;
	unsigned int bpp = 0;

	if (!src || !dst) {
		DISP_FATAL_ERR("display", "%s src(0x%p) or dst(0x%p) is null\n",
			       __func__, src, dst);
		return -1;
	}

	ret = disp_fmt_to_hw_fmt(src->src_fmt, &(dst->inputFormat), &Bpp, &bpp);
	dst->address = (unsigned long)src->src_phy_addr;
	dst->width = src->src_width;
	dst->height = src->src_height;
	dst->pitch = src->src_pitch * Bpp;
	dst->buf_offset = 0;

	return ret;
}


/* Video mode SODI CMDQ flow */
int _trigger_display_interface(int blocking, void *callback, unsigned int userdata)
{
	static unsigned int cnt;

	/* 4. enable SODI after config */
	if (primary_display_is_video_mode() == 1)
		disp_set_sodi(1, pgc->cmdq_handle_config);

#ifdef DISP_ENABLE_SODI_FOR_VIDEO_MODE
	if (gPrefetchControl == 1 && cnt >= 20)
		cmdqRecDisablePrefetch(pgc->cmdq_handle_config);
#endif
	if (_should_wait_path_idle())
		dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);

	if (_should_update_lcm())
		disp_lcm_update(pgc->plcm, 0, 0, pgc->plcm->params->width, pgc->plcm->params->height, 0);

	if (_should_start_path())
		dpmgr_path_start(pgc->dpmgr_handle, primary_display_cmdq_enabled());

	if (_should_trigger_path()) {
		/* trigger_loop_handle is used only for build trigger loop, */
		/*which should always be NULL for config thread */
		dpmgr_path_trigger(pgc->dpmgr_handle, NULL, primary_display_cmdq_enabled());
	}

	if (_should_set_cmdq_dirty()) {
		_cmdq_set_config_handle_dirty();
		/*disable SODI after set dirty */
		if (primary_display_is_video_mode() == 0 && gDisableSODIForTriggerLoop == 1)
			disp_set_sodi(0, pgc->cmdq_handle_config);
	}

	/*1. disable SODI by CPU before flush CMDQ by CPU */
	if (primary_display_is_video_mode() == 1)
		disp_set_sodi(0, 0);

	if (gDumpConfigCMD == 1) {
		DISPMSG("primary_display_config, dump before flush:\n");
		cmdqRecDumpCommand(pgc->cmdq_handle_config);
	}
	/* insert update ovl status slot command */
	if (primary_display_is_video_mode() == 1 && primary_display_is_decouple_mode() == 0
	    && gEnableOVLStatusCheck == 1) {
		cmdqRecBackupRegisterToSlot(pgc->cmdq_handle_config, pgc->ovl_status_info, 0,
					    DISP_REG_OVL0_STATUS_PA);
		cmdqRecBackupRegisterToSlot(pgc->cmdq_handle_config, pgc->ovl_status_info, 1,
					    DISP_REG_OVL1_STATUS_PA);
	}

	if (_should_flush_cmdq_config_handle())
		_cmdq_flush_config_handle(blocking, callback, userdata);

	if (_should_reset_cmdq_config_handle()) {
		_cmdq_reset_config_handle();
#ifdef DISP_ENABLE_SODI_FOR_VIDEO_MODE
		/* do not have to enable/disable prefetch at power on stage. */
		if (gPrefetchControl == 1 && cnt >= 20)
			cmdqRecEnablePrefetch(pgc->cmdq_handle_config);
#endif
	}
	/* TODO: _is_decouple_mode() shuold be protected by mutex!!!!!!!!when dynamic switch decouple/directlink */
	if (_should_insert_wait_frame_done_token() && (!_is_decouple_mode(pgc->session_mode))) {
		/* 2. enable SODI by CMDQ before wait */
		if (primary_display_is_video_mode() == 1)
			disp_set_sodi(1, pgc->cmdq_handle_config);

		_cmdq_insert_wait_frame_done_token();

		/* 3. disable AODI by CMDQ before config */
		if (primary_display_is_video_mode() == 1)
			disp_set_sodi(0, pgc->cmdq_handle_config);
	}

	if (cnt < 20)
		cnt++;

	return 0;
}

int _trigger_ovl_to_memory(disp_path_handle disp_handle,
			   cmdqRecHandle cmdq_handle,
			   fence_release_callback callback, unsigned int data)
{
	dpmgr_path_trigger(disp_handle, cmdq_handle, CMDQ_ENABLE);
	cmdqRecWaitNoClear(cmdq_handle, CMDQ_EVENT_DISP_WDMA0_EOF);

	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->rdma_buff_info, 0, mem_config.addr);

	cmdqRecFlushAsyncCallback(cmdq_handle, (CmdqAsyncFlushCB) callback, data);
	cmdqRecReset(cmdq_handle);
	cmdqRecWait(cmdq_handle, CMDQ_EVENT_DISP_WDMA0_EOF);
	MMProfileLogEx(ddp_mmp_get_events()->ovl_trigger, MMProfileFlagPulse, 0, data);
	return 0;
}

int _trigger_ovl_to_memory_mirror(disp_path_handle disp_handle,
				  cmdqRecHandle cmdq_handle,
				  fence_release_callback callback, unsigned int data)
{
	int layer = 0;
	unsigned int rdma_pitch_sec;

	dpmgr_path_trigger(disp_handle, cmdq_handle, CMDQ_ENABLE);

	cmdqRecWaitNoClear(cmdq_handle, CMDQ_EVENT_DISP_WDMA0_EOF);

	layer = disp_sync_get_output_timeline_id();
	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->cur_config_fence, layer, mem_config.buff_idx);

	layer = disp_sync_get_output_interface_timeline_id();
	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->cur_config_fence, layer,
				mem_config.interface_idx);

	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->rdma_buff_info, 0, (unsigned int)mem_config.addr);
	rdma_pitch_sec = mem_config.pitch | (mem_config.security << 30);
	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->rdma_buff_info, 1, rdma_pitch_sec);

	cmdqRecFlushAsyncCallback(cmdq_handle, (CmdqAsyncFlushCB) callback, data);
	cmdqRecReset(cmdq_handle);
	cmdqRecWait(cmdq_handle, CMDQ_EVENT_DISP_WDMA0_EOF);
	MMProfileLogEx(ddp_mmp_get_events()->ovl_trigger, MMProfileFlagPulse, 0, data);
	return 0;
}

#define EEEEEEEEEEEEEEE
/******************************************************************************/
/* ESD CHECK / RECOVERY ---- BEGIN                                            */
/******************************************************************************/
static struct task_struct *primary_display_esd_check_task;

static wait_queue_head_t esd_check_task_wq;	/* For Esd Check Task */
static atomic_t esd_check_task_wakeup = ATOMIC_INIT(0);	/* For Esd Check Task */
static wait_queue_head_t esd_ext_te_wq;	/* For Vdo Mode EXT TE Check */
static atomic_t esd_ext_te_event = ATOMIC_INIT(0);	/* For Vdo Mode EXT TE Check */

struct task_struct *primary_display_frame_update_task = NULL;
wait_queue_head_t primary_display_frame_update_wq;
atomic_t primary_display_frame_update_event = ATOMIC_INIT(0);

struct task_struct *decouple_fence_release_task = NULL;
wait_queue_head_t decouple_fence_release_wq;
atomic_t decouple_fence_release_event = ATOMIC_INIT(0);

static int eint_flag;	/* For DCT Setting */

unsigned int _need_do_esd_check(void)
{
	int ret = 0;
#ifdef CONFIG_OF
	if ((pgc->plcm->params->dsi.esd_check_enable == 1) && (islcmconnected == 1))
		ret = 1;
#else
	if (pgc->plcm->params->dsi.esd_check_enable == 1)
		ret = 1;

#endif
	return ret;
}


unsigned int _need_register_eint(void)
{

	int ret = 1;

	/* 1.need do esd check */
	/* 2.dsi vdo mode */
	/* 3.customization_esd_check_enable = 0 */
	if (_need_do_esd_check() == 0)
		ret = 0;
	else if (primary_display_is_video_mode() == 0)
		ret = 0;
	else if (pgc->plcm->params->dsi.customization_esd_check_enable == 1)
		ret = 0;

	return ret;

}

unsigned int _need_wait_esd_eof(void)
{
	int ret = 1;

	/* 1.need do esd check */
	/* 2.customization_esd_check_enable = 1 */
	/* 3.dsi cmd mode */
	if (_need_do_esd_check() == 0)
		ret = 0;
	else if (pgc->plcm->params->dsi.customization_esd_check_enable == 0)
		ret = 0;
	else if (primary_display_is_video_mode())
		ret = 0;

	return ret;
}

/* For Cmd Mode Read LCM Check */
/* Config cmdq_handle_config_esd */
int _esd_check_config_handle_cmd(void)
{
	int ret = 0;		/* 0:success */

	/* 1.reset */
	cmdqRecReset(pgc->cmdq_handle_config_esd);

	/* 2.write first instruction */
	/* cmd mode: wait CMDQ_SYNC_TOKEN_STREAM_EOF(wait trigger thread done) */
	cmdqRecWaitNoClear(pgc->cmdq_handle_config_esd, CMDQ_SYNC_TOKEN_STREAM_EOF);

	/* 3.clear CMDQ_SYNC_TOKEN_ESD_EOF(trigger thread need wait this sync token) */
	cmdqRecClearEventToken(pgc->cmdq_handle_config_esd, CMDQ_SYNC_TOKEN_ESD_EOF);

	/* 4.write instruction(read from lcm) */
	dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_config_esd, CMDQ_ESD_CHECK_READ);

	/* 5.set CMDQ_SYNC_TOKE_ESD_EOF(trigger thread can work now) */
	cmdqRecSetEventToken(pgc->cmdq_handle_config_esd, CMDQ_SYNC_TOKEN_ESD_EOF);

	/* 6.flush instruction */
	dprec_logger_start(DPREC_LOGGER_ESD_CMDQ, 0, 0);
	ret = cmdqRecFlush(pgc->cmdq_handle_config_esd);
	dprec_logger_done(DPREC_LOGGER_ESD_CMDQ, 0, 0);


	DISPCHECK("[ESD]_esd_check_config_handle_cmd ret=%d\n", ret);


	if (ret)
		ret = 1;
	return ret;
}

/* For Vdo Mode Read LCM Check */
/* Config cmdq_handle_config_esd */
int _esd_check_config_handle_vdo(void)
{
	int ret = 0;		/* 0:success , 1:fail */

	/* 1.reset */
	cmdqRecReset(pgc->cmdq_handle_config_esd);

	/* disable SODI by CMDQ */
	if (gESDEnableSODI == 1)
		disp_set_sodi(0, pgc->cmdq_handle_config_esd);

	/* 2.stop dsi vdo mode */
	dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_config_esd, CMDQ_STOP_VDO_MODE);

	/* 3.write instruction(read from lcm) */
	dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_config_esd, CMDQ_ESD_CHECK_READ);

	/* 4.start dsi vdo mode */
	dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_config_esd, CMDQ_START_VDO_MODE);

	/* 5. trigger path */
	cmdqRecClearEventToken(pgc->cmdq_handle_config_esd, CMDQ_EVENT_MUTEX0_STREAM_EOF);
	dpmgr_path_trigger(pgc->dpmgr_handle, pgc->cmdq_handle_config_esd, CMDQ_ENABLE);

	/* enable SODI by  CMDQ */
	if (gESDEnableSODI == 1)
		disp_set_sodi(1, pgc->cmdq_handle_config_esd);

	/* 6.flush instruction */
	dprec_logger_start(DPREC_LOGGER_ESD_CMDQ, 0, 0);
	ret = cmdqRecFlush(pgc->cmdq_handle_config_esd);
	dprec_logger_done(DPREC_LOGGER_ESD_CMDQ, 0, 0);

	DISPCHECK("[ESD]_esd_check_config_handle_vdo ret=%d\n", ret);

	if (ret)
		ret = 1;
	return ret;
}

/* For Vdo Mode EXT TE Check */
static irqreturn_t _esd_check_ext_te_irq_handler(int irq, void *data)
{
	MMProfileLogEx(ddp_mmp_get_events()->esd_vdo_eint, MMProfileFlagPulse, 0, 0);
	atomic_set(&esd_ext_te_event, 1);
	wake_up_interruptible(&esd_ext_te_wq);
	return IRQ_HANDLED;
}

int primary_display_switch_esd_mode(int mode)
{
	int ret = 0;

	DISPFUNC();


	if (pgc->plcm->params->dsi.customization_esd_check_enable != 0)
		return -1;

	DISPMSG("switch esd mode to %d\n", mode);

#ifdef GPIO_DSI_TE_PIN
	gpio_mode = mt_get_gpio_mode(GPIO_DSI_TE_PIN);
	/* DISPMSG("[ESD]gpio_mode=%d\n",gpio_mode); */
#endif
	if (mode == 1) {
		;
#ifdef GPIO_DSI_TE_PIN
/* switch to vdo mode */
		if (gpio_mode == GPIO_DSI_TE_PIN_M_DSI_TE) {
#endif
			/* if(_need_register_eint()) */
			{
				/* DISPMSG("[ESD]switch video mode\n"); */
				struct device_node *node;
				int irq;
				u32 ints[2] = { 0, 0 };
#ifdef GPIO_DSI_TE_PIN
				/* 1.set GPIO107 eint mode */
				mt_set_gpio_mode(GPIO_DSI_TE_PIN, GPIO_DSI_TE_PIN_M_GPIO);
#endif
				/* 2.register eint */
				node = of_find_compatible_node(0, 0, "mediatek, DSI_TE_1-eint");
				if (node) {
					/* DISPMSG("node 0x%x\n", node); */
					of_property_read_u32_array((void *)(unsigned long)node,
								   "debounce", ints,
								   ARRAY_SIZE(ints));
					/* mt_gpio_set_debounce(ints[0], ints[1]); */
					mt_eint_set_hw_debounce(ints[0], ints[1]);
					irq = irq_of_parse_and_map(node, 0);
					if (request_irq
					    (irq, _esd_check_ext_te_irq_handler, IRQF_TRIGGER_NONE,
					     "DSI_TE_1-eint", NULL)) {
						DISPERR("[ESD]EINT IRQ LINE NOT AVAILABLE!!\n");
					}
				} else {
					DISPERR
					    ("[ESD][%s] can't find DSI_TE_1 eint compatible node\n",
					     __func__);
				}
			}
#ifdef GPIO_DSI_TE_PIN
		}
#endif
	} else if (mode == 0) {
#ifdef GPIO_DSI_TE_PIN
		/* switch to cmd mode */
		if (gpio_mode == GPIO_DSI_TE_PIN_M_GPIO) {
#endif
			struct device_node *node;
			int irq;
			/* DISPMSG("[ESD]switch cmd mode\n"); */

			/* unregister eint */
			node = of_find_compatible_node(0, 0, "mediatek, DSI_TE_1-eint");
			/* DISPMSG("node 0x%x\n", node); */
			if (node) {
				irq = irq_of_parse_and_map(node, 0);
				free_irq(irq, NULL);
			} else {
				DISPERR("[ESD][%s] can't find DSI_TE_1 eint compatible node\n",
					__func__);
			}
#ifdef GPIO_DSI_TE_PIN
			/* set GPIO107 DSI TE mode */
			mt_set_gpio_mode(GPIO_DSI_TE_PIN, GPIO_DSI_TE_PIN_M_DSI_TE);
		}
#endif
	}
	/* DISPMSG("primary_display_switch_esd_mode end\n"); */
	return ret;
}

/* ESD CHECK FUNCTION */
/* return 1: esd check fail */
/* return 0: esd check pass */
int primary_display_esd_check(void)
{
	int ret = 0;

	dprec_logger_start(DPREC_LOGGER_ESD_CHECK, 0, 0);
	MMProfileLogEx(ddp_mmp_get_events()->esd_check_t, MMProfileFlagStart, 0, 0);
	DISPCHECK("[ESD]ESD check begin\n");
	_primary_path_lock(__func__);
	if (pgc->state == DISP_SLEPT) {
		MMProfileLogEx(ddp_mmp_get_events()->esd_check_t, MMProfileFlagPulse, 1, 0);
		DISPCHECK("[ESD]primary display path is slept?? -- skip esd check\n");
		_primary_path_unlock(__func__);
		/* goto done; */
		DISPCHECK("[ESD]ESD check end\n");
		MMProfileLogEx(ddp_mmp_get_events()->esd_check_t, MMProfileFlagEnd, 0, ret);
		dprec_logger_done(DPREC_LOGGER_ESD_CHECK, 0, 0);
		return ret;
	}
	_primary_path_unlock(__func__);
#ifdef MTK_DISP_IDLE_LP
	_disp_primary_path_dsi_clock_on(0);
#endif
	/* / Esd Check : EXT TE */
	if (pgc->plcm->params->dsi.customization_esd_check_enable == 0) {
		MMProfileLogEx(ddp_mmp_get_events()->esd_extte, MMProfileFlagStart, 0, 0);
		if (primary_display_is_video_mode()) {
			primary_display_switch_esd_mode(1);
			if (_need_register_eint()) {
				MMProfileLogEx(ddp_mmp_get_events()->esd_extte, MMProfileFlagPulse,
					       1, 1);

				if (wait_event_interruptible_timeout
				    (esd_ext_te_wq, atomic_read(&esd_ext_te_event), HZ / 2) > 0) {
					ret = 0;	/* esd check pass */
				} else {
					ret = 1;	/* esd check fail */
					DISPCHECK("esd check fail release fence fake\n");
					primary_display_release_fence_fake();
				}
				atomic_set(&esd_ext_te_event, 0);
			}
			primary_display_switch_esd_mode(0);
		} else {
			MMProfileLogEx(ddp_mmp_get_events()->esd_extte, MMProfileFlagPulse, 0, 1);
			if (dpmgr_wait_event_timeout
			    (pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, HZ / 2) > 0) {
				ret = 0;	/* esd check pass */
			} else {
				ret = 1;	/* esd check fail */
				DISPCHECK("esd check fail release fence fake\n");
				primary_display_release_fence_fake();
			}
		}
		MMProfileLogEx(ddp_mmp_get_events()->esd_extte, MMProfileFlagEnd, 0, ret);
		goto done;
	}
	/* / Esd Check : Read from lcm */
	MMProfileLogEx(ddp_mmp_get_events()->esd_rdlcm, MMProfileFlagStart, 0,
		       primary_display_cmdq_enabled());
	if (primary_display_cmdq_enabled()) {
		MMProfileLogEx(ddp_mmp_get_events()->esd_rdlcm, MMProfileFlagPulse, 0, 1);
		/* 0.create esd check cmdq */
		cmdqRecCreate(CMDQ_SCENARIO_DISP_ESD_CHECK, &(pgc->cmdq_handle_config_esd));
		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_config_esd,
				      CMDQ_ESD_ALLC_SLOT);
		MMProfileLogEx(ddp_mmp_get_events()->esd_rdlcm, MMProfileFlagPulse, 0, 2);
		DISPCHECK("[ESD]ESD config thread=%p\n", pgc->cmdq_handle_config_esd);

		/* 1.use cmdq to read from lcm */
		if (primary_display_is_video_mode())
			ret = _esd_check_config_handle_vdo();
		else
			ret = _esd_check_config_handle_cmd();
		MMProfileLogEx(ddp_mmp_get_events()->esd_rdlcm, MMProfileFlagPulse,
			       primary_display_is_video_mode(), 3);
		if (ret == 1) {
			if (_need_wait_esd_eof()) {
				/* Need set esd check eof synctoken to let trigger loop go. */
				cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_ESD_EOF);
			}
			/* do dsi reset */
			dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_config_esd,
					      CMDQ_DSI_RESET);
			DISPCHECK("esd check fail release fence fake\n");
			primary_display_release_fence_fake();
			goto destroy_cmdq;
		}

		DISPCHECK("[ESD]ESD config thread done~\n");

		/* 2.check data(*cpu check now) */
		ret =
		    dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_config_esd,
					  CMDQ_ESD_CHECK_CMP);
		MMProfileLogEx(ddp_mmp_get_events()->esd_rdlcm, MMProfileFlagPulse, 0, 4);
		if (ret) {
			ret = 1;	/* esd check fail */
			DISPCHECK("esd check fail release fence fake\n");
			primary_display_release_fence_fake();
		}

destroy_cmdq:
		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_config_esd,
				      CMDQ_ESD_FREE_SLOT);
		/* 3.destroy esd config thread */
		cmdqRecDestroy(pgc->cmdq_handle_config_esd);
		pgc->cmdq_handle_config_esd = NULL;
	} else {		/* by cpu */
		/* / 0: lock path */
		/* / 1: stop path */
		/* / 2: do esd check (!!!) */
		/* / 3: start path */
		/* / 4: unlock path */

		MMProfileLogEx(ddp_mmp_get_events()->esd_rdlcm, MMProfileFlagPulse, 0, 1);
		_primary_path_lock(__func__);

		/* / 1: stop path */
		DISPCHECK("[ESD]display cmdq trigger loop stop[begin]\n");
		_cmdq_stop_trigger_loop();
		DISPCHECK("[ESD]display cmdq trigger loop stop[end]\n");

		if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
			DISPCHECK("[ESD]primary display path is busy\n");
			ret =
			    dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE,
						     HZ * 1);
			DISPCHECK("[ESD]wait frame done ret:%d\n", ret);
		}

		DISPCHECK("[ESD]stop dpmgr path[begin]\n");
		dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);
		DISPCHECK("[ESD]stop dpmgr path[end]\n");

		if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
			DISPCHECK("[ESD]primary display path is busy after stop\n");
			dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE,
						 HZ * 1);
			DISPCHECK("[ESD]wait frame done ret:%d\n", ret);
		}

		DISPCHECK("[ESD]reset display path[begin]\n");
		dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
		DISPCHECK("[ESD]reset display path[end]\n");

		/* / 2: do esd check (!!!) */
		MMProfileLogEx(ddp_mmp_get_events()->esd_rdlcm, MMProfileFlagPulse, 0, 2);

		if (primary_display_is_video_mode()) {
			/* ret = 0; */
			ret = disp_lcm_esd_check(pgc->plcm);
		} else {
			ret = disp_lcm_esd_check(pgc->plcm);
		}

		/* / 3: start path */
		MMProfileLogEx(ddp_mmp_get_events()->esd_rdlcm, MMProfileFlagPulse,
			       primary_display_is_video_mode(), 3);

		DISPCHECK("[ESD]start dpmgr path[begin]\n");
		dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
		DISPCHECK("[ESD]start dpmgr path[end]\n");

		DISPCHECK("[ESD]start cmdq trigger loop[begin]\n");
		_cmdq_start_trigger_loop();
		DISPCHECK("[ESD]start cmdq trigger loop[end]\n");

		_primary_path_unlock(__func__);
	}
	MMProfileLogEx(ddp_mmp_get_events()->esd_rdlcm, MMProfileFlagEnd, 0, ret);

done:
#ifdef MTK_DISP_IDLE_LP
	_disp_primary_path_dsi_clock_off(0);
#endif
	DISPCHECK("[ESD]ESD check end\n");
	MMProfileLogEx(ddp_mmp_get_events()->esd_check_t, MMProfileFlagEnd, 0, ret);
	dprec_logger_done(DPREC_LOGGER_ESD_CHECK, 0, 0);
	return ret;

}


static int primary_display_esd_check_worker_kthread(void *data)
{
	int ret = 0;
	int i = 0;
	int esd_try_cnt = 5;	/* 20; */

	struct sched_param param = {.sched_priority = 87 };/*RTPM_PRIO_FB_THREAD*/

	sched_setscheduler(current, SCHED_RR, &param);


	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);

	while (1) {
#if 0
		dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);
		ret = dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ);
		if (ret <= 0) {
			DISPERR("wait frame done timeout, reset whole path now\n");
			primary_display_diagnose();
			dprec_logger_trigger(DPREC_LOGGER_ESD_RECOVERY);
			dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
		}
#else
		wait_event_interruptible(esd_check_task_wq, atomic_read(&esd_check_task_wakeup));
		msleep(2000);	/* esd check every 2s */
#ifdef DISP_SWITCH_DST_MODE
		_primary_path_switch_dst_lock();
#endif
		_primary_path_cmd_lock();
#if 0
		{
			/* let's do a mutex holder check here */
			unsigned long long period = 0;

			period = dprec_logger_get_current_hold_period(DPREC_LOGGER_PRIMARY_MUTEX);
			if (period > 2000 * 1000 * 1000) {
				DISPERR("primary display mutex is hold by %s for %dns\n",
					pgc->mutex_locker, period);
			}
		}
#endif
		ret = primary_display_esd_check();
		if (ret == 1) {
			DISPCHECK("[ESD]esd check fail, will do esd recovery %d\n", ret);
			i = esd_try_cnt;
			while (i--) {
				DISPCHECK("[ESD]esd recovery try:%d\n", i);
				primary_display_esd_recovery();
				ret = primary_display_esd_check();
				if (ret == 0) {
					DISPCHECK("[ESD]esd recovery success\n");
					break;
				}
				DISPCHECK("[ESD]after esd recovery, esd check still fail\n");
				if (i == 0) {
					DISPCHECK(
					"[ESD]after esd recovery %d times, esd check still fail, disable esd check\n",
					esd_try_cnt);
						primary_display_esd_check_enable(0);
				}
			}
		}
		_primary_path_cmd_unlock();
#ifdef DISP_SWITCH_DST_MODE
		_primary_path_switch_dst_unlock();
#endif
#endif

		if (kthread_should_stop())
			break;
	}
	return 0;
}

/* ESD RECOVERY */
int primary_display_esd_recovery(void)
{
	DISP_STATUS ret = DISP_STATUS_OK;
	LCM_PARAMS *lcm_param = NULL;

	DISPFUNC();
	dprec_logger_start(DPREC_LOGGER_ESD_RECOVERY, 0, 0);
	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagStart, 0, 0);
	DISPCHECK("[ESD]ESD recovery begin\n");

	_primary_path_lock(__func__);

	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse,
		       primary_display_is_video_mode(), 1);


	lcm_param = disp_lcm_get_params(pgc->plcm);
	if (pgc->state == DISP_SLEPT) {
		DISPCHECK("[ESD]esd recovery but primary display path is slept??\n");
		goto done;
	}

	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 2);

	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		DISPCHECK("[ESD]primary display path is busy\n");
		ret =
		    dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);
		DISPCHECK("[ESD]wait frame done ret:%d\n", ret);
	}

	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 3);

	DISPCHECK("[ESD]stop dpmgr path[begin]\n");
	dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);
	DISPCHECK("[ESD]stop dpmgr path[end]\n");

	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 4);

	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		DISPCHECK("[ESD]primary display path is busy after stop\n");
		dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);
		DISPCHECK("[ESD]wait frame done ret:%d\n", ret);
	}

	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 5);

	DISPCHECK("[ESD]display cmdq trigger loop stop[begin]\n");
	_cmdq_stop_trigger_loop();
	DISPCHECK("[ESD]display cmdq trigger loop stop[end]\n");

	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 6);

	DISPCHECK("[ESD]reset display path[begin]\n");
	dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
	DISPCHECK("[ESD]reset display path[end]\n");

	DISPCHECK("[POWER]lcm suspend[begin]\n");
	disp_lcm_suspend(pgc->plcm);
	disp_lcm_suspend_power(pgc->plcm);  //mike add
	DISPCHECK("[POWER]lcm suspend[end]\n");

	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 7);

	DISPCHECK("[ESD]lcm force init[begin]\n");
	disp_lcm_init(pgc->plcm, 1);
	DISPCHECK("[ESD]lcm force init[end]\n");

	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 8);

	DISPCHECK("[ESD]start dpmgr path[begin]\n");
	dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
	DISPCHECK("[ESD]start dpmgr path[end]\n");
	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		DISPERR("[ESD]Fatal error, we didn't trigger display path but it's already busy\n");
		ret = -1;
		/* goto done; */
	}

	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 9);

	if (primary_display_is_video_mode()) {
		/* for video mode, we need to force trigger here */
		/* for cmd mode, just set DPREC_EVENT_CMDQ_SET_EVENT_ALLOW when trigger loop start */
		dpmgr_path_trigger(pgc->dpmgr_handle, NULL, CMDQ_DISABLE);
	}

	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 10);

	DISPCHECK("[ESD]start cmdq trigger loop[begin]\n");
	_cmdq_start_trigger_loop();
	DISPCHECK("[ESD]start cmdq trigger loop[end]\n");

	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagPulse, 0, 11);

done:
	_primary_path_unlock(__func__);
	DISPCHECK("[ESD]ESD recovery end\n");
	MMProfileLogEx(ddp_mmp_get_events()->esd_recovery_t, MMProfileFlagEnd, 0, 0);
	dprec_logger_done(DPREC_LOGGER_ESD_RECOVERY, 0, 0);
	return ret;
}

void primary_display_esd_check_enable(int enable)
{
	if (_need_do_esd_check()) {
		if (_need_register_eint() && eint_flag != 2) {
			DISPCHECK("[ESD]Please check DCT setting about GPIO107/EINT107\n");
			return;
		}

		if (enable) {
			DISPCHECK("[ESD]esd check thread wakeup\n");
			atomic_set(&esd_check_task_wakeup, 1);
			wake_up_interruptible(&esd_check_task_wq);
		} else {
			DISPCHECK("[ESD]esd check thread stop\n");
			atomic_set(&esd_check_task_wakeup, 0);
		}
	}
}

/******************************************************************************/
/* ESD CHECK / RECOVERY ---- End                                              */
/******************************************************************************/
#define EEEEEEEEEEEEEEEEEEEEEEEEEE

static struct task_struct *primary_path_aal_task;
unsigned int gDecouplePQWithRDMA = 1;

static int _disp_primary_path_check_trigger(void *data)
{
	int ret = 0;

	cmdqRecHandle handle = NULL;

	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);

	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_TRIGGER);
	while (1) {
		dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_TRIGGER);
		MMProfileLogEx(ddp_mmp_get_events()->primary_display_aalod_trigger,
			       MMProfileFlagPulse, 0, 0);

		_primary_path_lock(__func__);
		if (pgc->state != DISP_SLEPT) {
#ifdef MTK_DISP_IDLE_LP
			if (gDecouplePQWithRDMA == 0) {
				last_primary_trigger_time = sched_clock();
				_disp_primary_path_exit_idle(__func__, 0);
			}
#endif
			cmdqRecReset(handle);
			_cmdq_insert_wait_frame_done_token_mira(handle);
			_cmdq_set_config_handle_dirty_mira(handle);
			_cmdq_flush_config_handle_mira(handle, 0);
		}
		_primary_path_unlock(__func__);
		if (kthread_should_stop())
			break;
	}

	cmdqRecDestroy(handle);

	return 0;
}

#define OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

/* need remove */
unsigned int cmdqDdpClockOn(uint64_t engineFlag)
{
	/* DISP_LOG_I("cmdqDdpClockOff\n"); */
	return 0;
}

unsigned int cmdqDdpClockOff(uint64_t engineFlag)
{
	/* DISP_LOG_I("cmdqDdpClockOff\n"); */
	return 0;
}

unsigned int cmdqDdpDumpInfo(uint64_t engineFlag, char *pOutBuf, unsigned int bufSize)
{
	DISPERR("cmdq timeout:%llu\n", engineFlag);
	primary_display_diagnose();
	/* DISP_LOG_I("cmdqDdpDumpInfo\n"); */

	if (primary_display_is_decouple_mode()) {
		ddp_dump_analysis(DISP_MODULE_OVL0);
		ddp_dump_analysis(DISP_MODULE_OVL1);
	}
	ddp_dump_analysis(DISP_MODULE_WDMA0);

	return 0;
}

unsigned int cmdqDdpResetEng(uint64_t engineFlag)
{
	/* DISP_LOG_I("cmdqDdpResetEng\n"); */
	return 0;
}

/* TODO: these 2 functions should be splited into another file */
unsigned int display_path_idle_cnt = 0;
static void _RDMA0_INTERNAL_IRQ_Handler(DISP_MODULE_ENUM module, unsigned int param)
{
	if (!_is_decouple_mode(pgc->session_mode) && param & 0x2) {
		/* RDMA Start */
		display_path_idle_cnt++;
#ifdef DISP_ENABLE_SODI
		spm_sodi_mempll_pwr_mode(1);
#endif
	}
	if (param & 0x4) {
		/* RDMA Done */
		display_path_idle_cnt--;
		if (display_path_idle_cnt == 0) {
#ifdef DISP_ENABLE_SODI
			spm_sodi_mempll_pwr_mode(0);
#endif
		}
	}
}

static void _WDMA0_INTERNAL_IRQ_Handler(DISP_MODULE_ENUM module, unsigned int param)
{
	if (param & 0x1) {
		/* WDMA Done */
		display_path_idle_cnt--;
		if (display_path_idle_cnt == 0) {
#ifdef DISP_ENABLE_SODI
			spm_sodi_mempll_pwr_mode(0);
#endif
		}
	}
}

static void _MUTEX_INTERNAL_IRQ_Handler(DISP_MODULE_ENUM module, unsigned int param)
{
	if (param & 0x1) {
		/* RDMA-->DSI SOF */
		display_path_idle_cnt++;
#ifdef DISP_ENABLE_SODI
		spm_sodi_mempll_pwr_mode(1);
#endif
	}
	if (param & 0x2) {
		/* OVL->WDMA SOF */
		display_path_idle_cnt++;
#ifdef DISP_ENABLE_SODI
		spm_sodi_mempll_pwr_mode(1);
#endif
	}
}

void primary_display_sodi_rule_init(void)
{
	/* if( (primary_display_mode == DECOUPLE_MODE) && primary_display_is_video_mode()) */
	if (gEnableSODIControl == 0 && primary_display_is_video_mode() == 1) {
#ifdef DISP_ENABLE_SODI
		spm_enable_sodi(0);
#endif
		DISPMSG("SODI disabled!\n");
		return;
	}

	if (primary_display_get_lcm_max_refresh_rate() == 120) {
		DISPMSG("[120HZ] SODI disabled because LCM support adjust fps\n");
		spm_enable_sodi(0);
		return;
	}

#ifdef DISP_ENABLE_SODI
	spm_enable_sodi(1);
#endif
	if (primary_display_is_video_mode()) {
		disp_unregister_module_irq_callback(DISP_MODULE_RDMA0,
			_RDMA0_INTERNAL_IRQ_Handler);
		/* if switch to video mode, should de-register callback */
		/* spm_sodi_mempll_pwr_mode(0); */
	} else {
		disp_register_module_irq_callback(DISP_MODULE_RDMA0, _RDMA0_INTERNAL_IRQ_Handler);
		if (_is_decouple_mode(pgc->session_mode)) {
			disp_register_module_irq_callback(DISP_MODULE_MUTEX,
							  _MUTEX_INTERNAL_IRQ_Handler);
			disp_register_module_irq_callback(DISP_MODULE_WDMA0,
							  _WDMA0_INTERNAL_IRQ_Handler);
		}
	}
}



int primary_display_change_lcm_resolution(unsigned int width, unsigned int height)
{
	if (pgc->plcm) {
		DISPMSG("LCM Resolution will be changed, original: %dx%d, now: %dx%d\n",
			pgc->plcm->params->width, pgc->plcm->params->height, width, height);
		/* align with 4 is the minimal check, to ensure we can boot up into kernel, */
		/*and could modify dfo setting again using meta tool */
		/* otherwise we will have a panic in lk(root cause unknown). */
		if (width > pgc->plcm->params->width || height > pgc->plcm->params->height
		    || width == 0 || height == 0 || width % 4 || height % 2) {
			DISPERR("Invalid resolution: %dx%d\n", width, height);
			return -1;
		}

		if (primary_display_is_video_mode()) {
			DISPERR("Warning!!!Video Mode can't support multiple resolution!\n");
			return -1;
		}

		pgc->plcm->params->width = width;
		pgc->plcm->params->height = height;

		return 0;
	} else {
		return -1;
	}
}

static void primary_display_frame_update_irq_callback(DISP_MODULE_ENUM module, unsigned int param)
{
	/* /if(pgc->session_mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE) */
	/* /    return; */

	if (module == DISP_MODULE_RDMA0) {
		if (param & 0x20) {
			/* rdma0 frame start */
			if (pgc->session_id > 0)
				update_frm_seq_info(ddp_ovl_get_cur_addr(1, 0), 0, 1, FRM_START);
		}

		if (param & 0x4) {
			/* rdma0 frame done */
			atomic_set(&primary_display_frame_update_event, 1);
			wake_up_interruptible(&primary_display_frame_update_wq);
		}
	}

	if ((module == DISP_MODULE_OVL0) && (_is_decouple_mode(pgc->session_mode) == 0)) {
		if (param & 0x2) {
			/* ov0 frame done */
			atomic_set(&primary_display_frame_update_event, 1);
			wake_up_interruptible(&primary_display_frame_update_wq);
		}
	}
#if defined(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT)
	/* In TEE, we have to protect WDMA registers, so we can't enable WDMA interrupt */
	/* here we use ovl frame done interrupt instead */
	if ((module == DISP_MODULE_OVL0) && (_is_decouple_mode(pgc->session_mode) == 1)) {
		if (param & 0x2) {
			atomic_set(&decouple_fence_release_event, 1);
			wake_up_interruptible(&decouple_fence_release_wq);
		}
	}
#else
	if ((module == DISP_MODULE_WDMA0) && (_is_decouple_mode(pgc->session_mode) == 1)) {
		if (param & 0x1) {
			/* wdma0 frame done */
			atomic_set(&decouple_fence_release_event, 1);
			wake_up_interruptible(&decouple_fence_release_wq);
		}
	}
#endif

}

static int primary_display_frame_update_kthread(void *data)
{
	struct sched_param param = {.sched_priority = 94 };/*RTPM_PRIO_SCRN_UPDATE*/

	sched_setscheduler(current, SCHED_RR, &param);

	for (;;) {
		wait_event_interruptible(primary_display_frame_update_wq,
					 atomic_read(&primary_display_frame_update_event));
		atomic_set(&primary_display_frame_update_event, 0);

		if (pgc->session_id > 0)
			update_frm_seq_info(0, 0, 0, FRM_END);

		if (kthread_should_stop())
			break;
	}

	return 0;
}

#if 0
static struct task_struct *if_fence_release_worker_task;

static int _if_fence_release_worker_thread(void *data)
{
	int ret = 0;
	unsigned int session_id = MAKE_DISP_SESSION(DISP_SESSION_PRIMARY, 0);
	struct sched_param param = {.sched_priority = RTPM_PRIO_SCRN_UPDATE };

	sched_setscheduler(current, SCHED_RR, &param);

	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);

	while (1) {
		dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);
		if (_is_mirror_mode(pgc->session_mode)) {
			int fence_idx, subtractor, layer;

			layer = disp_sync_get_output_interface_timeline_id();

			cmdqBackupReadSlot(pgc->cur_config_fence, layer, &fence_idx);
			cmdqBackupReadSlot(pgc->subtractor_when_free, layer, &subtractor);
			mtkfb_release_fence(session_id, layer, fence_idx - 1);
		}
		if (kthread_should_stop())
			break;
	}

	return 0;
}

static struct task_struct *ovl2mem_fence_release_worker_task;

static int _ovl2mem_fence_release_worker_thread(void *data)
{
	int ret = 0;
	unsigned int addr = 0;
	unsigned int session_id = MAKE_DISP_SESSION(DISP_SESSION_PRIMARY, 0);
	struct sched_param param = {.sched_priority = RTPM_PRIO_SCRN_UPDATE };

	sched_setscheduler(current, SCHED_RR, &param);

	dpmgr_enable_event(pgc->ovl2mem_path_handle, DISP_PATH_EVENT_FRAME_COMPLETE);

	while (1) {
		/* it's not good to use FRAME_COMPLETE here, */
		/*because when CPU read wdma addr, maybe it's already changed by next request */
		/* but luckly currently we will wait rdma frame done after wdma done(in CMDQ), so it's safe now */
		dpmgr_wait_event(pgc->ovl2mem_path_handle, DISP_PATH_EVENT_FRAME_COMPLETE);
		if (_is_mirror_mode(pgc->session_mode)) {
			int fence_idx, subtractor, layer;

			layer = disp_sync_get_output_timeline_id();

			cmdqBackupReadSlot(pgc->cur_config_fence, layer, &fence_idx);
			cmdqBackupReadSlot(pgc->subtractor_when_free, layer, &subtractor);
			mtkfb_release_fence(session_id, layer, fence_idx);
		}

		if (kthread_should_stop())
			break;
	}

	return 0;
}

static struct task_struct *fence_release_worker_task;
#endif


static void _wdma_fence_release_callback(uint32_t userdata)
{
	int fence_idx, layer;

	layer = disp_sync_get_output_timeline_id();

	cmdqBackupReadSlot(pgc->cur_config_fence, layer, &fence_idx);
	mtkfb_release_fence(primary_session_id, layer, fence_idx);
	MMProfileLogEx(ddp_mmp_get_events()->primary_wdma_fence_release, MMProfileFlagPulse, layer,
		       fence_idx);

}

static void _Interface_fence_release_callback(uint32_t userdata)
{
	int layer = disp_sync_get_output_interface_timeline_id();

	if (userdata > 0) {
		mtkfb_release_fence(primary_session_id, layer, userdata);
		MMProfileLogEx(ddp_mmp_get_events()->primary_wdma_fence_release, MMProfileFlagPulse,
			       layer, userdata);
	}


}

static void _ovl_fence_release_callback(uint32_t userdata)
{
	int i = 0;
	unsigned int addr = 0;
	int ret = 0;

	MMProfileLogEx(ddp_mmp_get_events()->session_release, MMProfileFlagStart, 1, userdata);
	/* releaes OVL1 when primary setting */
	if (ovl_get_status() == DDP_OVL1_STATUS_PRIMARY_RELEASED) {
		ovl_set_status(DDP_OVL1_STATUS_SUB);
		wake_up_interruptible(&ovl1_wait_queue);
	} else if (ovl_get_status() == DDP_OVL1_STATUS_PRIMARY_DISABLE) {
		ovl_set_status(DDP_OVL1_STATUS_IDLE);
		wake_up_interruptible(&ovl1_wait_queue);
	}

	for (i = 0; i < PRIMARY_DISPLAY_SESSION_LAYER_COUNT; i++) {
		int fence_idx = 0;
		int subtractor = 0;

		if (i == primary_display_get_option("ASSERT_LAYER") && is_DAL_Enabled()) {
			mtkfb_release_layer_fence(primary_session_id, i);
		} else {
			cmdqBackupReadSlot(pgc->cur_config_fence, i, &fence_idx);
			cmdqBackupReadSlot(pgc->subtractor_when_free, i, &subtractor);
			mtkfb_release_fence(primary_session_id, i, fence_idx - subtractor);
		}
		MMProfileLogEx(ddp_mmp_get_events()->primary_ovl_fence_release, MMProfileFlagPulse,
			       i, fence_idx - subtractor);
	}

	if (primary_display_is_video_mode() == 1 && primary_display_is_decouple_mode() == 0
	    && gEnableOVLStatusCheck == 1) {
		unsigned int ovl_status[2];

		cmdqBackupReadSlot(pgc->ovl_status_info, 0, &ovl_status[0]);
		cmdqBackupReadSlot(pgc->ovl_status_info, 1, &ovl_status[1]);
		if (((ovl_status[0] & 0x1) != OVL_STATUS_IDLE) ||
		    ((ovl_get_status() != DDP_OVL1_STATUS_SUB
		      && (ovl_status[1] & 0x1) != OVL_STATUS_IDLE))) {
			DISPERR("disp ovl status error, 0x%x, 0x%x\n", ovl_status[0],
				ovl_status[1]);
			/* dump cmdq cmd here */
		}
	}

	addr = ddp_ovl_get_cur_addr(!_should_config_ovl_input(), 0);
	if ((_is_decouple_mode(pgc->session_mode) == 0))
		update_frm_seq_info(addr, 0, 2, FRM_START);

	/* async callback,need to check if it is still decouple */
	_primary_path_lock(__func__);
	if (_is_decouple_mode(pgc->session_mode) && userdata == DISP_SESSION_DECOUPLE_MODE) {
		static cmdqRecHandle cmdq_handle;

		if (cmdq_handle == NULL)
			ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);
		if (ret == 0) {
			cmdqRecReset(cmdq_handle);
			_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);
			cmdqBackupReadSlot(pgc->rdma_buff_info, 0, &addr);
			decouple_rdma_config.address = addr;
			decouple_rdma_config.security = DISP_NORMAL_BUFFER;
			_config_rdma_input_data(&decouple_rdma_config, pgc->dpmgr_handle,
						cmdq_handle);
			_cmdq_set_config_handle_dirty_mira(cmdq_handle);
			cmdqRecFlushAsyncCallback(cmdq_handle, NULL, 0);
			MMProfileLogEx(ddp_mmp_get_events()->primary_rdma_config,
				       MMProfileFlagPulse, 0, decouple_rdma_config.address);
			/* cmdqRecDestroy(cmdq_handle); */
		} else {
			ret = -1;
			DISPERR("fail to create cmdq\n");
		}
	}
	_primary_path_unlock(__func__);


}

static int decouple_fence_release_kthread(void *data)
{
	int interface_fence = 0;
	int layer = 0;
	int ret = 0;

	struct sched_param param = {.sched_priority = 94 };/*RTPM_PRIO_SCRN_UPDATE*/

	sched_setscheduler(current, SCHED_RR, &param);



	for (;;) {
		wait_event_interruptible(decouple_fence_release_wq,
					 atomic_read(&decouple_fence_release_event));
		atomic_set(&decouple_fence_release_event, 0);

		/* async callback,need to check if it is still decouple */
		_primary_path_lock(__func__);
		if (pgc->session_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE) {
			static cmdqRecHandle cmdq_handle;
			unsigned int rdma_pitch_sec;

			if (cmdq_handle == NULL)
				ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);
			if (ret == 0) {
				RDMA_CONFIG_STRUCT tmpConfig = decouple_rdma_config;

				cmdqRecReset(cmdq_handle);
				_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);
				cmdqBackupReadSlot(pgc->rdma_buff_info, 0,
						   (uint32_t *) &(tmpConfig.address));

				/*rdma pitch only use bit[15..0], we use bit[31:30] to store secure information */
				cmdqBackupReadSlot(pgc->rdma_buff_info, 1,
						   (uint32_t *) &(rdma_pitch_sec));
				tmpConfig.pitch = rdma_pitch_sec & ~(3 << 30);
				tmpConfig.security = rdma_pitch_sec >> 30;

				_config_rdma_input_data(&tmpConfig, pgc->dpmgr_handle, cmdq_handle);

				layer = disp_sync_get_output_timeline_id();
				cmdqBackupReadSlot(pgc->cur_config_fence, layer, &interface_fence);
				_cmdq_set_config_handle_dirty_mira(cmdq_handle);
				cmdqRecFlushAsyncCallback(cmdq_handle,
							  (CmdqAsyncFlushCB)
							  _Interface_fence_release_callback,
							  interface_fence >
							  1 ? interface_fence - 1 : 0);
				MMProfileLogEx(ddp_mmp_get_events()->primary_rdma_config,
					       MMProfileFlagPulse, interface_fence,
					       decouple_rdma_config.address);

				/* dump rdma input if enabled */
				dprec_mmp_dump_rdma_layer(&tmpConfig, 0);
				/* cmdqRecDestroy(cmdq_handle); */
			} else {
				DISPERR("fail to create cmdq\n");
			}
		}
		_primary_path_unlock(__func__);

		if (kthread_should_stop())
			break;
	}

	return 0;
}

static int _olv_wdma_fence_release_callback(uint32_t userdata)
{
	int ret = 0;

	_ovl_fence_release_callback(userdata);
	_wdma_fence_release_callback(userdata);

	return ret;
}

#if 0
static int _fence_release_worker_thread(void *data)
{
	int ret = 0;
	int i = 0;
	unsigned int addr = 0;
	unsigned int session_id = MAKE_DISP_SESSION(DISP_SESSION_PRIMARY, 0);
	struct sched_param param = {.sched_priority = RTPM_PRIO_SCRN_UPDATE };

	sched_setscheduler(current, SCHED_RR, &param);

	if (!_is_decouple_mode(pgc->session_mode))
		dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);
	else
		dpmgr_enable_event(pgc->ovl2mem_path_handle, DISP_PATH_EVENT_FRAME_START);

	while (1) {
		if (!_is_decouple_mode(pgc->session_mode))
			dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);
		else
			dpmgr_wait_event(pgc->ovl2mem_path_handle, DISP_PATH_EVENT_FRAME_START);

		if (is_hwc_enabled == 0)
			continue;

		/* releaes OVL1 when primary setting */
		if (ovl_get_status() == DDP_OVL1_STATUS_PRIMARY_RELEASED) {
			ovl_set_status(DDP_OVL1_STATUS_SUB);
			wake_up_interruptible(&ovl1_wait_queue);
		} else if (ovl_get_status() == DDP_OVL1_STATUS_PRIMARY_DISABLE) {
			ovl_set_status(DDP_OVL1_STATUS_IDLE);
			wake_up_interruptible(&ovl1_wait_queue);
		}

		for (i = 0; i < PRIMARY_DISPLAY_SESSION_LAYER_COUNT; i++) {
			int fence_idx, subtractor;

			if (i == primary_display_get_option("ASSERT_LAYER") && is_DAL_Enabled()) {
				mtkfb_release_layer_fence(session_id, i);
			} else {
				cmdqBackupReadSlot(pgc->cur_config_fence, i, &fence_idx);
				cmdqBackupReadSlot(pgc->subtractor_when_free, i, &subtractor);
				mtkfb_release_fence(session_id, i, fence_idx - subtractor);
			}
		}

		addr = ddp_ovl_get_cur_addr(!_should_config_ovl_input(), 0);
		if ((_is_decouple_mode(pgc->session_mode) == 0))
			update_frm_seq_info(addr, 0, 2, FRM_START);

		MMProfileLogEx(ddp_mmp_get_events()->session_release, MMProfileFlagEnd, 0, 0);

		if (kthread_should_stop())
			break;
	}

	return 0;
}

static struct task_struct *present_fence_release_worker_task;

static int _present_fence_release_worker_thread(void *data)
{
	int ret = 0;

	cmdqRecHandle handle = NULL;

	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);

	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE);
	while (1) {
		msleep(10000000);

		if (kthread_should_stop())
			break;
	}

	cmdqRecDestroy(handle);

	return 0;
}
#endif
int primary_display_capture_framebuffer_wdma(void *data)
{
	int ret = 0;
	cmdqRecHandle cmdq_handle = NULL;
	disp_ddp_path_config *pconfig = NULL;
	m4u_client_t *m4uClient = NULL;
	unsigned int w_xres = primary_display_get_width();
	unsigned int h_yres = primary_display_get_height();
	unsigned int pixel_byte = primary_display_get_bpp() / 8;
	/* bpp is either 32 or 16, can not be other value */
	int buffer_size = h_yres * w_xres * pixel_byte;
	int buf_idx = 0;
	unsigned int mva[2] = { 0, 0 };
	void *va[2] = { NULL, NULL };
	unsigned int format = eRGBA8888;


	struct sched_param param = {.sched_priority = 87 };/*RTPM_PRIO_FB_THREAD*/

	sched_setscheduler(current, SCHED_RR, &param);

	va[0] = vmalloc(buffer_size);
	if (va[0] == NULL) {
		DISPCHECK("wdma dump:Fail to alloc vmalloc 0\n");
		ret = -1;
		goto out;
	}
	va[1] = vmalloc(buffer_size);
	if (va[1] == NULL) {
		DISPCHECK("wdma dump:Fail to alloc vmalloc 1\n");
		ret = -1;
		goto out;
	}
	m4uClient = m4u_create_client();
	if (m4uClient == NULL) {
		DISPCHECK("wdma dump:Fail to alloc  m4uClient=0x%p\n", m4uClient);
		ret = -1;
		goto out;
	}

	ret = m4u_alloc_mva(m4uClient, M4U_PORT_DISP_WDMA0, (unsigned long)va[0],
			    NULL, buffer_size, M4U_PROT_READ | M4U_PROT_WRITE,
			    0, (unsigned int *)&mva[0]);
	if (ret != 0) {
		DISPCHECK("wdma dump::Fail to allocate mva 0\n");
		ret = -1;
		goto out;
	}

	ret = m4u_alloc_mva(m4uClient, M4U_PORT_DISP_WDMA0,
			    (unsigned long)va[1], NULL, buffer_size,
			    M4U_PROT_READ | M4U_PROT_WRITE, 0, (unsigned int *)&mva[1]);
	if (ret != 0) {
		DISPCHECK("wdma dump::Fail to allocate mva 1\n");
		ret = -1;
		goto out;
	}
	if (primary_display_cmdq_enabled()) {
		/*create config thread */
		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);
		if (ret != 0) {
			DISPCHECK("wdma dump:Fail to create primary cmdq handle for capture\n");
			ret = -1;
			goto out;
		}
		dpmgr_path_memout_clock(pgc->dpmgr_handle, 1);
		dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);
		cmdqRecReset(cmdq_handle);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);
		_primary_path_lock(__func__);
		dpmgr_path_add_memout(pgc->dpmgr_handle, ENGINE_OVL0, cmdq_handle);
		_primary_path_unlock(__func__);
		while (primary_dump_wdma) {

			ret = m4u_cache_sync(m4uClient, M4U_PORT_DISP_WDMA0,
					     (unsigned long)va[buf_idx],
					     buffer_size, mva[buf_idx], M4U_CACHE_FLUSH_BY_RANGE);
			_primary_path_lock(__func__);
			pconfig = dpmgr_path_get_last_config(pgc->dpmgr_handle);
			pconfig->wdma_dirty = 1;
			pconfig->wdma_config.dstAddress = mva[buf_idx];
			pconfig->wdma_config.srcHeight = h_yres;
			pconfig->wdma_config.srcWidth = w_xres;
			pconfig->wdma_config.clipX = 0;
			pconfig->wdma_config.clipY = 0;
			pconfig->wdma_config.clipHeight = h_yres;
			pconfig->wdma_config.clipWidth = w_xres;
			pconfig->wdma_config.outputFormat = format;
			pconfig->wdma_config.useSpecifiedAlpha = 1;
			pconfig->wdma_config.alpha = 0xFF;
			pconfig->wdma_config.dstPitch =
			    w_xres * DP_COLOR_BITS_PER_PIXEL(format) / 8;

			ret = dpmgr_path_config(pgc->dpmgr_handle, pconfig, cmdq_handle);
			pconfig->wdma_dirty = 0;
			_primary_path_unlock(__func__);
			_cmdq_set_config_handle_dirty_mira(cmdq_handle);
			_cmdq_flush_config_handle_mira(cmdq_handle, 0);
			ret = dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);

			/* pconfig->wdma_config.dstAddress = va[buf_idx++]; */
			/* DISPCHECK("capture wdma\n"); */
			dprec_mmp_dump_wdma_layer(&pconfig->wdma_config, 0);
			buf_idx = buf_idx % 2;
			cmdqRecReset(cmdq_handle);
			_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);
		}
		_primary_path_lock(__func__);
		dpmgr_path_remove_memout(pgc->dpmgr_handle, cmdq_handle);
		_primary_path_unlock(__func__);

		_cmdq_set_config_handle_dirty_mira(cmdq_handle);
		/* flush remove memory to cmdq */
		_cmdq_flush_config_handle_mira(cmdq_handle, 1);
		DISPMSG("wdma dump: Flush remove memout\n");

		dpmgr_path_memout_clock(pgc->dpmgr_handle, 0);

	}

out:
	cmdqRecDestroy(cmdq_handle);
	if (mva[0] > 0)
		m4u_dealloc_mva(m4uClient, M4U_PORT_DISP_WDMA0, mva[0]);

	if (mva[1] > 0)
		m4u_dealloc_mva(m4uClient, M4U_PORT_DISP_WDMA0, mva[1]);

	if (va[0] != NULL)
		vfree(va[0]);
	if (va[1] != NULL)
		vfree(va[1]);

	if (m4uClient != 0)
		m4u_destroy_client(m4uClient);
	DISPMSG("wdma dump:end\n");

	return ret;
}


int primary_display_switch_wdma_dump(int on)
{
	if (on && (!primary_dump_wdma)) {
		primary_dump_wdma = 1;
		primary_display_wdma_out =
		    kthread_create(primary_display_capture_framebuffer_wdma, NULL,
				   "display_wdma_out");
		wake_up_process(primary_display_wdma_out);
	} else {
		primary_dump_wdma = 0;
	}
	return 0;
}

#define xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

int primary_display_set_frame_buffer_address(unsigned long va, unsigned long mva)
{

	DISPMSG("framebuffer va %lu, mva %lu\n", va, mva);
	pgc->framebuffer_va = va;
	pgc->framebuffer_mva = mva;
/*
    int frame_buffer_size = ALIGN_TO(DISP_GetScreenWidth(), MTK_FB_ALIGNMENT) *
		      ALIGN_TO(DISP_GetScreenHeight(), MTK_FB_ALIGNMENT) * 4;
    unsigned long dim_layer_va = va + 2*frame_buffer_size;
    dim_layer_mva = mva + 2*frame_buffer_size;
    memset(dim_layer_va, 0, frame_buffer_size);
*/
	return 0;
}

unsigned long primary_display_get_frame_buffer_mva_address(void)
{
	return pgc->framebuffer_mva;
}

unsigned long primary_display_get_frame_buffer_va_address(void)
{
	return pgc->framebuffer_va;
}

int is_dim_layer(unsigned int long mva)
{
	if (mva == dim_layer_mva)
		return 1;
	return 0;
}

unsigned long get_dim_layer_mva_addr(void)
{
	if (dim_layer_mva == 0) {
		int frame_buffer_size = ALIGN_TO(DISP_GetScreenWidth(), MTK_FB_ALIGNMENT) *
		    ALIGN_TO(DISP_GetScreenHeight(), MTK_FB_ALIGNMENT) * 4;
		unsigned long dim_layer_va = pgc->framebuffer_va + 1 * frame_buffer_size;

		memset_io((void *)dim_layer_va, 0, frame_buffer_size * 2);
		dim_layer_mva = pgc->framebuffer_mva + 1 * frame_buffer_size;
		DISPMSG("init dim layer mva %lu, size %d", dim_layer_mva, frame_buffer_size * 2);
	}
	return dim_layer_mva;
}

static int init_cmdq_slots(cmdqBackupSlotHandle *pSlot, int count, int init_val)
{
	int i;

	cmdqBackupAllocateSlot(pSlot, count);

	for (i = 0; i < count; i++)
		cmdqBackupWriteSlot(*pSlot, i, init_val);

	return 0;
}

int primary_display_init(char *lcm_name, unsigned int lcm_fps)
{
	disp_ddp_path_config *data_config = NULL;
	DISP_STATUS ret = DISP_STATUS_OK;
	LCM_PARAMS *lcm_param = NULL;

	DISPCHECK("primary_display_init begin\n");


	dprec_init();
	dpmgr_init();

	init_cmdq_slots(&(pgc->cur_config_fence), DISP_SESSION_TIMELINE_COUNT, 0);
	init_cmdq_slots(&(pgc->subtractor_when_free), DISP_SESSION_TIMELINE_COUNT, 0);
	init_cmdq_slots(&(pgc->rdma_buff_info), 2, 0);
	init_cmdq_slots(&(pgc->ovl_status_info), 2, 0);
	/* ret = cmdqRecCreate(CMDQ_SCENARIO_DISP_PRIMARY_DISABLE_SECURE_PATH, &(pgc->sec_switch_handle)); */
	if (ret)
		DISP_FATAL_ERR("DISP_SEC", "fail to create disable handle ret=%d\n", ret);
	mutex_init(&(pgc->capture_lock));
	mutex_init(&(pgc->lock));
	mutex_init(&(pgc->cmd_lock));
	mutex_init(&(pgc->vsync_lock));
#ifdef MTK_DISP_IDLE_LP
	mutex_init(&idle_lock);
#endif
#ifdef DISP_SWITCH_DST_MODE
	mutex_init(&(pgc->switch_dst_lock));
#endif
	_primary_path_lock(__func__);

	pgc->plcm = disp_lcm_probe(lcm_name, LCM_INTERFACE_NOTDEFINED);

	if (pgc->plcm == NULL) {
		DISPCHECK("disp_lcm_probe returns null\n");
		ret = DISP_STATUS_ERROR;
		goto done;
	} else {
		DISPCHECK("disp_lcm_probe SUCCESS\n");
	}
#ifndef MTK_FB_DFO_DISABLE
	if ((0 == dfo_query("LCM_FAKE_WIDTH", &lcm_fake_width))
	    && (0 == dfo_query("LCM_FAKE_HEIGHT", &lcm_fake_height))) {
		pr_debug("[DFO] LCM_FAKE_WIDTH=%d, LCM_FAKE_HEIGHT=%d\n", lcm_fake_width,
		       lcm_fake_height);
		if (lcm_fake_width && lcm_fake_height) {
			if (DISP_STATUS_OK !=
			    primary_display_change_lcm_resolution(lcm_fake_width,
								  lcm_fake_height)) {
				DISPMSG("[DISP\DFO]WARNING!!! Change LCM Resolution FAILED!!!\n");
			}
		}
	}
#endif
	lcm_param = disp_lcm_get_params(pgc->plcm);

	if (lcm_param == NULL) {
		DISPERR("get lcm params FAILED\n");
		ret = DISP_STATUS_ERROR;
		goto done;
	}

	if (primary_display_get_lcm_max_refresh_rate() == 120) {
		DISPMSG("[120HZ] SODI disabled because LCM support adjust fps\n");
		gEnableSODIControl = 0;
		gPrefetchControl = 0;
		gSkipIdleDetect = 1;
	}

#ifdef MTK_FB_DO_NOTHING
	return ret;
#endif

	ret =
	    cmdqCoreRegisterCB(CMDQ_GROUP_DISP, (CmdqClockOnCB) cmdqDdpClockOn,
			       (CmdqDumpInfoCB) cmdqDdpDumpInfo, (CmdqResetEngCB) cmdqDdpResetEng,
			       (CmdqClockOffCB) cmdqDdpClockOff);
	if (ret) {
		DISPERR("cmdqCoreRegisterCB failed, ret=%d\n", ret);
		ret = DISP_STATUS_ERROR;
		goto done;
	}

	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &(pgc->cmdq_handle_config));
	if (ret) {
		DISPCHECK("cmdqRecCreate FAIL, ret=%d\n", ret);
		ret = DISP_STATUS_ERROR;
		goto done;
	} else {
		DISPCHECK("cmdqRecCreate SUCCESS, g_cmdq_handle=0x%p\n", pgc->cmdq_handle_config);
	}
	/*create ovl2mem path cmdq handle */
	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_MEMOUT, &(pgc->cmdq_handle_ovl1to2_config));
	if (ret != 0) {
		DISPERR("cmdqRecCreate FAIL, ret=%d\n", ret);
		return -1;
	}
#ifndef MTK_FB_CMDQ_DISABLE
	primary_display_use_cmdq = CMDQ_ENABLE;
#else
	primary_display_use_cmdq = CMDQ_DISABLE;
#endif

	/* debug for bus hang issue (need to remove) */
	ddp_dump_analysis(DISP_MODULE_CONFIG);
	if (primary_display_mode == DIRECT_LINK_MODE) {
		__build_path_direct_link();
		pgc->session_mode = DISP_SESSION_DIRECT_LINK_MODE;
		DISPCHECK("primary display is DIRECT LINK MODE\n");
	} else if (primary_display_mode == DECOUPLE_MODE) {
		__build_path_decouple();
		pgc->session_mode = DISP_SESSION_DECOUPLE_MODE;

		DISPCHECK("primary display is DECOUPLE MODE\n");
	} else if (primary_display_mode == SINGLE_LAYER_MODE) {
		__build_path_single_layer();

		DISPCHECK("primary display is SINGLE LAYER MODE\n");
	} else if (primary_display_mode == DEBUG_RDMA1_DSI0_MODE) {
		__build_path_debug_rdma1_dsi0();

		DISPCHECK("primary display is DEBUG RDMA1 DSI0 MODE\n");
	} else {
		DISPCHECK("primary display mode is WRONG\n");
	}

/* dpmgr_path_init(pgc->dpmgr_handle, CMDQ_DISABLE); */
#if 1
	dpmgr_path_set_video_mode(pgc->dpmgr_handle, primary_display_is_video_mode());
/* dpmgr_path_init(pgc->dpmgr_handle, CMDQ_DISABLE); */

	_cmdq_build_trigger_loop();
	_cmdq_start_trigger_loop();
#endif
	if (primary_display_use_cmdq == CMDQ_ENABLE) {
		_cmdq_reset_config_handle();
		_cmdq_insert_wait_frame_done_token();
	}

	data_config = dpmgr_path_get_last_config(pgc->dpmgr_handle);

#ifdef OVL_CASCADE_SUPPORT
	if (ovl_get_status() == DDP_OVL1_STATUS_IDLE || ovl_get_status() == DDP_OVL1_STATUS_PRIMARY) {
		if (primary_display_mode == DECOUPLE_MODE)
			dpmgr_path_enable_cascade(pgc->ovl2mem_path_handle,
						  pgc->cmdq_handle_config);
		else
			dpmgr_path_enable_cascade(pgc->dpmgr_handle, pgc->cmdq_handle_config);
	}
#endif
	memcpy(&(data_config->dispif_config), lcm_param, sizeof(LCM_PARAMS));

	data_config->dst_w = lcm_param->width;
	data_config->dst_h = lcm_param->height;
	if (lcm_param->type == LCM_TYPE_DSI) {
		if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB888)
			data_config->lcm_bpp = 24;
		else if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB565)
			data_config->lcm_bpp = 16;
		else if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB666)
			data_config->lcm_bpp = 18;
	} else if (lcm_param->type == LCM_TYPE_DPI) {
		if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB888)
			data_config->lcm_bpp = 24;
		else if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB565)
			data_config->lcm_bpp = 16;
		if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB666)
			data_config->lcm_bpp = 18;
	}

	data_config->fps = lcm_fps;
	data_config->dst_dirty = 1;

	rdma_set_target_line(DISP_MODULE_RDMA0, primary_display_get_height() * 1 / 2,
			     pgc->cmdq_handle_config);
	rdma_set_target_line(DISP_MODULE_RDMA0, primary_display_get_height() * 1 / 2, NULL);

	if (primary_display_use_cmdq == CMDQ_ENABLE) {
		ret = dpmgr_path_config(pgc->dpmgr_handle, data_config, pgc->cmdq_handle_config);

		/* should we set dirty here???????? */
		_cmdq_flush_config_handle(0, NULL, 0);

		_cmdq_reset_config_handle();
		_cmdq_insert_wait_frame_done_token();
	} else {
		ret = dpmgr_path_config(pgc->dpmgr_handle, data_config, NULL);
	}

	{
#ifdef MTK_NO_DISP_IN_LK
		ret = disp_lcm_init(pgc->plcm, 1);
#else
		ret = disp_lcm_init(pgc->plcm, 0);
#endif
	}

	if (_is_decouple_mode(pgc->session_mode))
		dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
	/* this should remove? for video mode when LK has start path */

#ifdef MTK_FB_ESD_ENABLE
	primary_display_esd_check_task =
	    kthread_create(primary_display_esd_check_worker_kthread, NULL, "display_esd_check");
	init_waitqueue_head(&esd_ext_te_wq);
	init_waitqueue_head(&esd_check_task_wq);
	if (_need_do_esd_check()) {
		wake_up_process(primary_display_esd_check_task);
		/* primary_display_esd_check_enable(1); */
	}

	if (_need_register_eint()) {
		struct device_node *node;
		int irq;
		u32 ints[2] = { 0, 0 };
#ifdef GPIO_DSI_TE_PIN
		/* 1.set GPIO107 eint mode */
		mt_set_gpio_mode(GPIO_DSI_TE_PIN, GPIO_DSI_TE_PIN_M_GPIO);
		eint_flag++;
#endif

		/* 2.register eint */
		node = of_find_compatible_node(0, 0, "mediatek, DSI_TE_1-eint");
		if (node) {
			of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
			/* FIXME: find definatition of  mt_gpio_set_debounce */
			/* mt_gpio_set_debounce(ints[0],ints[1]); */
			mt_eint_set_hw_debounce(ints[0], ints[1]);
			irq = irq_of_parse_and_map(node, 0);
			if (request_irq
			    (irq, _esd_check_ext_te_irq_handler, IRQF_TRIGGER_NONE, "DSI_TE_1-eint",
			     NULL)) {
				DISPCHECK("[ESD]EINT IRQ LINE NOT AVAILABLE!!\n");
			} else {
				eint_flag++;
			}
		} else {
			DISPCHECK("[ESD][%s] can't find DSI_TE_1 eint compatible node\n", __func__);
		}
	}


	if (_need_do_esd_check())
		primary_display_esd_check_enable(1);

#endif
#ifdef DISP_SWITCH_DST_MODE
	primary_display_switch_dst_mode_task =
	    kthread_create(_disp_primary_path_switch_dst_mode_thread, NULL,
			   "display_switch_dst_mode");
	wake_up_process(primary_display_switch_dst_mode_task);
#endif

	primary_path_aal_task =
	    kthread_create(_disp_primary_path_check_trigger, NULL, "display_check_aal");
	wake_up_process(primary_path_aal_task);
#if 0				/* Zaikuo: disable it when HWC not enable to reduce error log */
	fence_release_worker_task =
	    kthread_create(_fence_release_worker_thread, NULL, "fence_worker");
	wake_up_process(fence_release_worker_task);
	if (_is_decouple_mode(pgc->session_mode)) {
		if_fence_release_worker_task =
		    kthread_create(_if_fence_release_worker_thread, NULL, "if_fence_worker");
		wake_up_process(if_fence_release_worker_task);

		ovl2mem_fence_release_worker_task =
		    kthread_create(_ovl2mem_fence_release_worker_thread, NULL,
				   "ovl2mem_fence_worker");
		wake_up_process(ovl2mem_fence_release_worker_task);
	}
	present_fence_release_worker_task =
	    kthread_create(_present_fence_release_worker_thread, NULL, "present_fence_worker");
	wake_up_process(present_fence_release_worker_task);
#endif

	if (primary_display_frame_update_task == NULL) {
		init_waitqueue_head(&primary_display_frame_update_wq);
		disp_register_module_irq_callback(DISP_MODULE_RDMA0,
						  primary_display_frame_update_irq_callback);
		disp_register_module_irq_callback(DISP_MODULE_OVL0,
						  primary_display_frame_update_irq_callback);
		disp_register_module_irq_callback(DISP_MODULE_WDMA0,
						  primary_display_frame_update_irq_callback);
		primary_display_frame_update_task =
		    kthread_create(primary_display_frame_update_kthread, NULL,
				   "frame_update_worker");
		wake_up_process(primary_display_frame_update_task);
		init_waitqueue_head(&decouple_fence_release_wq);
		decouple_fence_release_task =
		    kthread_create(decouple_fence_release_kthread, NULL,
				   "decouple_fence_release_worker");
		wake_up_process(decouple_fence_release_task);
	}
	/* primary_display_use_cmdq = CMDQ_ENABLE; */

	/* this will be set to always enable cmdq later */
	if (primary_display_is_video_mode()) {
#ifdef DISP_SWITCH_DST_MODE
		primary_display_cur_dst_mode = 1;	/* video mode */
		primary_display_def_dst_mode = 1;	/* default mode is video mode */
#endif
		dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
				       DDP_IRQ_RDMA0_DONE);
	} else {

	}

#ifndef MTKFB_NO_M4U
	{
		M4U_PORT_STRUCT sPort;

		sPort.ePortID = M4U_PORT_DISP_WDMA0;
		sPort.Virtuality = primary_display_use_m4u;
		sPort.Security = 0;
		sPort.Distance = 1;
		sPort.Direction = 0;
		ret = m4u_config_port(&sPort);
		if (ret != 0) {
			DISPCHECK("config M4U Port %s to %s FAIL(ret=%d)\n",
				  ddp_get_module_name(DISP_MODULE_WDMA0),
				  primary_display_use_m4u ? "virtual" : "physical", ret);
			return -1;
		}
	}
#endif

	pgc->lcm_refresh_rate = lcm_fps/100;
	pgc->lcm_fps = lcm_fps;
	if (lcm_fps > 6000) {
		/* FIXME: if fps bigger than 60, support 8 layer? */
		pgc->max_layer = OVL_LAYER_NUM;
	} else
		pgc->max_layer = OVL_LAYER_NUM;
	pgc->state = DISP_ALIVE;

	primary_display_sodi_rule_init();

#ifdef MTK_DISP_IDLE_LP
	init_waitqueue_head(&idle_detect_wq);
	primary_display_idle_detect_task =
	    kthread_create(_disp_primary_path_idle_detect_thread, NULL, "display_idle_detect");
	wake_up_process(primary_display_idle_detect_task);
#endif
done:

	_primary_path_unlock(__func__);
	DISPCHECK("primary_display_init end\n");
	return ret;
}

int primary_display_deinit(void)
{
	int i;

	_primary_path_lock(__func__);

	_cmdq_stop_trigger_loop();
	dpmgr_path_deinit(pgc->dpmgr_handle, CMDQ_DISABLE);

	for (i = 0; i < DISP_INTERNAL_BUFFER_COUNT; i++) {
		if (pgc->dc_buf_sec[i]) {
			pgc->dc_buf_sec[i] = free_decouple_sec_buffer(pgc->dc_buf_sec[i]);
			pgc->dc_buf_sec[i] = 0;
		}
	}
	_primary_path_unlock(__func__);
	return 0;
}

/* register rdma done event */
int primary_display_wait_for_idle(void)
{
	DISP_STATUS ret = DISP_STATUS_OK;

	DISPFUNC();

	_primary_path_lock(__func__);


	_primary_path_unlock(__func__);
	return ret;
}

int _display_set_lcm_refresh_rate(int fps)
{
	int ret = 0;
	static cmdqRecHandle cmdq_handle;

	if (pgc->state == DISP_SLEPT) {
		DISPCHECK("Sleep State set lcm rate\n");
		return -1;
	}

	if (primary_display_get_lcm_max_refresh_rate() <= 60) {
		DISPCHECK("not support set lcm rate\n");
		return 0;
	}

	if (fps == pgc->lcm_refresh_rate)
		return 0;

	if (cmdq_handle == NULL) {
		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);
		if (ret != 0) {
			DISPCHECK("fail to create primary display cmdq handle for adjust fps\n");
			return -1;
		}
	}
	DISPCHECK("primary_display_set_lcm_refresh_rate:fps(%d)\n", fps);
	cmdqRecReset(cmdq_handle);
	_cmdq_handle_clear_dirty(cmdq_handle);
	_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);

	disp_lcm_adjust_fps(cmdq_handle, pgc->plcm, fps);
	if (fps == 120) {
		pgc->plcm->params->dsi.PLL_CLOCK = 480;
		dpmgr_path_ioctl(pgc->dpmgr_handle, cmdq_handle,
			DDP_PHY_CLK_CHANGE, (unsigned long *)&(pgc->plcm->params->dsi.PLL_CLOCK));

	} else {
		pgc->plcm->params->dsi.PLL_CLOCK = 225;
		dpmgr_path_ioctl(pgc->dpmgr_handle, cmdq_handle,
			DDP_PHY_CLK_CHANGE, (unsigned long *)&(pgc->plcm->params->dsi.PLL_CLOCK));
	}
	_cmdq_set_config_handle_dirty_mira(cmdq_handle);
	_cmdq_flush_config_handle_mira(cmdq_handle, 0);
	pgc->lcm_refresh_rate = fps;
	pgc->lcm_fps = fps*100;

	return 0;
}

int primary_display_set_lcm_refresh_rate(int fps)
{
	int ret = 0;

	_primary_path_lock(__func__);
	ret = _display_set_lcm_refresh_rate(fps);
	_primary_path_unlock(__func__);
	return ret;
}

int primary_display_get_lcm_max_refresh_rate(void)
{
	if (disp_lcm_is_support_adjust_fps(pgc->plcm) != 0)
		return 120;
	return 60;
}

int primary_display_get_lcm_refresh_rate(void)
{
	int fps = 0;

	_primary_path_lock(__func__);
	fps = pgc->lcm_refresh_rate;
	_primary_path_unlock(__func__);
	return fps;
}

int primary_display_get_hwc_refresh_rate(void)
{
	int fps = 0;
	int time_interval =  0;

	_primary_path_lock(__func__);
	time_interval = get_current_time_us() - pgc->hwc_refresh_time;

	if (time_interval > 0)
		fps = (pgc->hwc_refresh_cnt * 1000000)/time_interval;

	pgc->hwc_refresh_cnt = 0;
	pgc->hwc_refresh_time = 0;
	_primary_path_unlock(__func__);
	return fps;
}

int primary_display_wait_for_dump(void)
{
	return 0;
}

int primary_display_release_fence_fake(void)
{
	unsigned int layer_en = 0;
	unsigned int addr = 0;
	unsigned int fence_idx = -1;
	unsigned int session_id = MAKE_DISP_SESSION(DISP_SESSION_PRIMARY, 0);
	int i = 0;

	DISPFUNC();

	for (i = 0; i < PRIMARY_DISPLAY_SESSION_LAYER_COUNT; i++) {
		if (i == primary_display_get_option("ASSERT_LAYER") && is_DAL_Enabled()) {
			mtkfb_release_layer_fence(session_id, 3);
		} else {
			disp_sync_get_cached_layer_info(session_id, i, &layer_en,
							(unsigned long *)&addr, &fence_idx);
			if (fence_idx < 0) {
				if (fence_idx == -1) {
					DISPPR_ERROR
					    ("find fence idx for layer %d,addr 0x%08x fail, unregistered addr%d\n",
					     i, 0, fence_idx);
				} else if (fence_idx == -2) {
					;
				} else {
					DISPPR_ERROR
					    ("find fence idx for layer %d,addr 0x%08x fail,reason unknown%d\n",
					     i, 0, fence_idx);
				}
			} else {
				if (layer_en)
					mtkfb_release_fence(session_id, i, fence_idx - 1);
				else
					mtkfb_release_fence(session_id, i, fence_idx);
			}
		}
	}
	return 0;

}

int primary_display_wait_for_vsync(void *config)
{
	int ret = 0;
	unsigned long long ts = 0ULL;
	disp_session_vsync_config *c = (disp_session_vsync_config *) config;

	_primary_path_vsync_lock();
	if (pgc->state == DISP_SLEPT) {
		DISPDBG("VSYNC DISP_SLEPT Return\n");
		_primary_path_vsync_unlock();
		return 0;
	}
#ifdef MTK_DISP_IDLE_LP
	_disp_primary_path_dsi_clock_on(0);
#endif
	if (!islcmconnected) {
		DISPCHECK("lcm not connect, use fake vsync\n");
		/* msleep(16); */
		usleep_range(16000, 17000);
		ret = -1;
		goto done;
	}

	ret = dpmgr_wait_event_ts(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, &ts);

	if (pgc->vsync_drop)
		ret = dpmgr_wait_event_ts(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, &ts);

	if (ret != 0)
		DISPCHECK("vsync signaled by unknown signal ret=%d\n", ret);


	c->vsync_ts = ts;
	c->vsync_cnt++;
	c->lcm_fps = pgc->lcm_refresh_rate;

done:
#ifdef MTK_DISP_IDLE_LP
	_disp_primary_path_dsi_clock_off(0);
#endif
	_primary_path_vsync_unlock();

	return ret;

}

unsigned int primary_display_get_ticket(void)
{
	return dprec_get_vsync_count();
}

int primary_suspend_release_fence(void)
{
	unsigned int session = (unsigned int)((DISP_SESSION_PRIMARY) << 16 | (0));
	unsigned int i = 0;

	for (i = 0; i < HW_OVERLAY_COUNT; i++) {
		DISPDBG("mtkfb_release_layer_fence  session=0x%x,layerid=%d\n", session, i);
		mtkfb_release_layer_fence(session, i);
	}
	return 0;
}

int primary_display_suspend(void)
{
	DISP_STATUS ret = DISP_STATUS_OK;

	DISPCHECK("primary_display_suspend begin\n");
	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagStart, 0, 0);
#ifdef DISP_SWITCH_DST_MODE
	primary_display_switch_dst_mode(primary_display_def_dst_mode);
#endif
	disp_sw_mutex_lock(&(pgc->capture_lock));
	_primary_path_lock(__func__);
	_primary_path_vsync_lock();
	if (pgc->state == DISP_SLEPT) {
		DISPCHECK("primary display path is already slept, skip\n");
		goto done;
	}
#ifdef MTK_DISP_IDLE_LP
	_disp_primary_path_exit_idle(__func__, 0);
#endif

	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 0, 1);
	/* msleep(16); */ /* wait last frame done */
	usleep_range(16000, 17000);

	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		int event_ret;

		MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 1, 2);
		event_ret =
		    dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);
		MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 2, 2);
		DISPCHECK
		    ("[POWER]primary display path is busy now, wait frame done, event_ret=%d\n",
		     event_ret);
		if (event_ret <= 0) {
			DISPERR("wait frame done in suspend timeout\n");
			MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 3,
				       2);
			primary_display_diagnose();
			ret = -1;
		}
	}
	/* for decouple mode */
	if (_is_decouple_mode(pgc->session_mode)) {
		if (dpmgr_path_is_busy(pgc->ovl2mem_path_handle)) {
			dpmgr_wait_event_timeout(pgc->ovl2mem_path_handle,
						 DISP_PATH_EVENT_FRAME_COMPLETE, HZ);
		}
		/* xuecheng, BAD WROKAROUND for decouple mode */
		msleep(30);
	}

	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 0, 2);

	DISPDBG("[POWER]display cmdq trigger loop stop[begin]\n");
	_cmdq_stop_trigger_loop();
	DISPCHECK("[POWER]display cmdq trigger loop stop[end]\n");
	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 0, 3);

#ifdef CONFIG_LCM_SEND_CMD_IN_VIDEO
	DISPDBG("[POWER]lcm suspend[begin]\n");
	disp_lcm_suspend(pgc->plcm);
	DISPCHECK("[POWER]lcm suspend[end]\n");
#endif

	DISPDBG("[POWER]primary display path stop[begin]\n");
	dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);
	DISPCHECK("[POWER]primary display path stop[end]\n");
	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 0, 4);

	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 1, 4);
		DISPERR("[POWER]stop display path failed, still busy\n");
		dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
		ret = -1;
		/* even path is busy(stop fail), we still need to continue power off other module/devices */
		/* goto done; */
	}
	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 0, 5);

	/* remove OVL1 from display if there is more than one session, */
	/* because the other session may use OVL1 in suspend mode */
#if 0
	{
		if (ovl_get_status() != DDP_OVL1_STATUS_SUB && disp_get_session_number() > 1) {
			unsigned int dp_handle;
			unsigned int cmdq_handle;

			DISPMSG("disable cascade before suspend!\n");
			dpmgr_path_get_handle(&dp_handle, &cmdq_handle);
			dpmgr_path_disable_cascade(dp_handle, CMDQ_DISABLE);
			if (ovl_get_status() == DDP_OVL1_STATUS_SUB_REQUESTING) {
				ovl_set_status(DDP_OVL1_STATUS_SUB);
				wake_up_interruptible(&ovl1_wait_queue);
			} else {
				ovl_set_status(DDP_OVL1_STATUS_IDLE);
			}
			_cmdq_build_trigger_loop();
		}
	}
#else
	DISPDBG("disable cascade before suspend!\n");
	if (_is_decouple_mode(pgc->session_mode) == 0)
		dpmgr_path_disable_cascade(pgc->dpmgr_handle, CMDQ_DISABLE);
	else
		dpmgr_path_disable_cascade(pgc->ovl2mem_path_handle, CMDQ_DISABLE);

	if (ovl_get_status() == DDP_OVL1_STATUS_SUB_REQUESTING) {
		ovl_set_status(DDP_OVL1_STATUS_SUB);
		wake_up_interruptible(&ovl1_wait_queue);
	} else if (ovl_get_status() != DDP_OVL1_STATUS_SUB) {
		ovl_set_status(DDP_OVL1_STATUS_IDLE);
	}
#endif

#ifndef CONFIG_LCM_SEND_CMD_IN_VIDEO
	DISPDBG("[POWER]lcm suspend[begin]\n");
	disp_lcm_suspend(pgc->plcm);
	DISPCHECK("[POWER]lcm suspend[end]\n");
#endif
	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 0, 6);
	DISPDBG("[POWER]primary display path Release Fence[begin]\n");
	primary_suspend_release_fence();
	DISPCHECK("[POWER]primary display path Release Fence[end]\n");
	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 0, 7);

	DISPDBG("[POWER]dpmanager path power off[begin]\n");
	dpmgr_path_power_off(pgc->dpmgr_handle, CMDQ_DISABLE);
	DISPCHECK("[POWER]dpmanager path power off[end]\n");

	if (_is_decouple_mode(pgc->session_mode))
		dpmgr_path_power_off(pgc->ovl2mem_path_handle, CMDQ_DISABLE);

	//mike add
	DISPDBG("[POWER]lcm suspend power[begin]\n");
	disp_lcm_suspend_power(pgc->plcm);
	DISPCHECK("[POWER]lcm suspend[end]\n");

	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 0, 8);
	display_path_idle_cnt = 0;

	pgc->state = DISP_SLEPT;
done:
	_primary_path_vsync_unlock();
	_primary_path_unlock(__func__);
	disp_sw_mutex_unlock(&(pgc->capture_lock));
	aee_kernel_wdt_kick_Powkey_api("mtkfb_early_suspend", WDT_SETBY_Display);
	primary_trigger_cnt = 0;
	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagEnd, 0, 0);
	DISPCHECK("primary_display_suspend end\n");
	return ret;
}

int primary_display_get_lcm_index(void)
{
	int index = 0;

	DISPFUNC();
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return 0;
	}
	index = pgc->plcm->index;
	DISPMSG("lcm index = %d\n", index);
	return index;
}

int primary_display_resume(void)
{
	DISP_STATUS ret = DISP_STATUS_OK;
	LCM_PARAMS *lcm_param;
	disp_ddp_path_config *data_config;

	DISPFUNC();
	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagStart, 0, 0);

	_primary_path_lock(__func__);
	if (pgc->state == DISP_ALIVE) {
		DISPCHECK("primary display path is already resume, skip\n");
		goto done;
	}
	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 1);

	//mike add
	DISPDBG("[POWER]lcm resume power[begin]\n");
	disp_lcm_resume_power(pgc->plcm);
	DISPCHECK("[POWER]lcm resume power[end]\n");

	DISPCHECK("dpmanager path power on[begin]\n");
	dpmgr_path_power_on(pgc->dpmgr_handle, CMDQ_DISABLE);
	DISPDBG("dpmanager path power on[end]\n");

	if (_is_decouple_mode(pgc->session_mode))
		dpmgr_path_power_on(pgc->ovl2mem_path_handle, CMDQ_DISABLE);

	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 2);
	if (is_ipoh_bootup) {
		DISPCHECK("[primary display path] leave primary_display_resume -- IPOH\n");
		is_ipoh_bootup = false;
		DISPCHECK("[POWER]start cmdq[begin]--IPOH\n");
		_cmdq_start_trigger_loop();
		DISPCHECK("[POWER]start cmdq[end]--IPOH\n");
		if (_is_decouple_mode(pgc->session_mode) == 0)
			dpmgr_path_enable_cascade(pgc->dpmgr_handle, CMDQ_DISABLE);
		else
			dpmgr_path_enable_cascade(pgc->ovl2mem_path_handle, CMDQ_DISABLE);
		pgc->state = DISP_ALIVE;
		goto done;
	}
	DISPDBG("[POWER]dpmanager re-init[begin]\n");

	{
		dpmgr_path_connect(pgc->dpmgr_handle, CMDQ_DISABLE);
		if (_is_decouple_mode(pgc->session_mode))
			dpmgr_path_connect(pgc->ovl2mem_path_handle, CMDQ_DISABLE);

		MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 1, 2);
		lcm_param = disp_lcm_get_params(pgc->plcm);

		data_config = dpmgr_path_get_last_config(pgc->dpmgr_handle);

		data_config->dst_w = lcm_param->width;
		data_config->dst_h = lcm_param->height;
		if (lcm_param->type == LCM_TYPE_DSI) {
			if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB888)
				data_config->lcm_bpp = 24;
			else if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB565)
				data_config->lcm_bpp = 16;
			else if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB666)
				data_config->lcm_bpp = 18;
		} else if (lcm_param->type == LCM_TYPE_DPI) {
			if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB888)
				data_config->lcm_bpp = 24;
			else if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB565)
				data_config->lcm_bpp = 16;
			if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB666)
				data_config->lcm_bpp = 18;
		}

		data_config->fps = pgc->lcm_fps;
		data_config->dst_dirty = 1;

		ret = dpmgr_path_config(pgc->dpmgr_handle, data_config, NULL);
		MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 2, 2);

		if (_is_decouple_mode(pgc->session_mode)) {
			data_config = dpmgr_path_get_last_config(pgc->ovl2mem_path_handle);

			data_config->fps = pgc->lcm_fps;
			data_config->dst_dirty = 1;
			data_config->wdma_dirty = 1;
			ret = dpmgr_path_config(pgc->ovl2mem_path_handle, data_config, NULL);
			MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 2,
				       2);
		}
		data_config->dst_dirty = 0;
	}
	DISPCHECK("[POWER]dpmanager re-init[end]\n");
	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 3);

	DISPDBG("[POWER]lcm resume[begin]\n");
	disp_lcm_resume(pgc->plcm);
	DISPCHECK("[POWER]lcm resume[end]\n");

	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 4);
	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 1, 4);
		DISPERR("[POWER]Fatal error, we didn't start display path but it's already busy\n");
		ret = -1;
		/* goto done; */
	}

	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 5);
	DISPDBG("[POWER]dpmgr path start[begin]\n");
	dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
	if (_is_decouple_mode(pgc->session_mode))
		dpmgr_path_start(pgc->ovl2mem_path_handle, CMDQ_DISABLE);

	DISPCHECK("[POWER]dpmgr path start[end]\n");

	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 6);
	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 1, 6);
		DISPERR
		    ("[POWER]Fatal error, we didn't trigger display path but it's already busy\n");
		ret = -1;
		/* goto done; */
	}

	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 7);
	if (primary_display_is_video_mode()) {
		MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 1, 7);
		/* for video mode, we need to force trigger here */
		/* for cmd mode, just set DPREC_EVENT_CMDQ_SET_EVENT_ALLOW when trigger loop start */
		dpmgr_path_trigger(pgc->dpmgr_handle, NULL, CMDQ_DISABLE);

		/* insert a wait token to make sure first config after resume will config to HW when HW idle */
		_cmdq_insert_wait_frame_done_token();
	}
	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 8);

	DISPDBG("[POWER]start cmdq[begin]\n");
	_cmdq_start_trigger_loop();
	DISPCHECK("[POWER]start cmdq[end]\n");
	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 9);

	if (!primary_display_is_video_mode()) {
		/*refresh black picture of ovl bg */
		DISPCHECK("[POWER]triggger cmdq[begin]\n");
		_trigger_display_interface(0, NULL, 0);
		DISPCHECK("[POWER]triggger cmdq[end]\n");
	}
#if 0
	DISPCHECK("[POWER]wakeup aal/od trigger process[begin]\n");
	wake_up_process(primary_path_aal_task);
	DISPCHECK("[POWER]wakeup aal/od trigger process[end]\n");
#endif
	pgc->state = DISP_ALIVE;

done:
	_primary_path_unlock(__func__);
	last_primary_trigger_time = sched_clock();
	/* primary_display_diagnose(); */
	aee_kernel_wdt_kick_Powkey_api("mtkfb_late_resume", WDT_SETBY_Display);
	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagEnd, 0, 0);

	/* ddp_dump_analysis(DISP_MODULE_OVL0); */
	/* ddp_dump_analysis(DISP_MODULE_OVL1); */
	/* ddp_dump_analysis(DISP_MODULE_RDMA0); */

	return 0;
}

int primary_display_ipoh_restore(void)
{
	DISPMSG("primary_display_ipoh_restore In\n");
	if (NULL != pgc->cmdq_handle_trigger) {
		struct TaskStruct *pTask = pgc->cmdq_handle_trigger->pRunningTask;

		if (NULL != pTask) {
			DISPCHECK("[Primary_display]display cmdq trigger loop stop[begin]\n");
			_cmdq_stop_trigger_loop();
			DISPCHECK("[Primary_display]display cmdq trigger loop stop[end]\n");
		}
	}
	DISPMSG("primary_display_ipoh_restore Out\n");
	return 0;
}

int primary_display_ipoh_recover(void)
{
	DISPMSG("%s In\n", __func__);
	_cmdq_start_trigger_loop();
	DISPMSG("%s Out\n", __func__);
	return 0;
}

int primary_display_start(void)
{
	DISP_STATUS ret = DISP_STATUS_OK;

	DISPFUNC();

	_primary_path_lock(__func__);
	dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);

	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		DISPCHECK("Fatal error, we didn't trigger display path but it's already busy\n");
		ret = -1;
		goto done;
	}

done:
	_primary_path_unlock(__func__);
	return ret;
}

int primary_display_stop(void)
{
	DISP_STATUS ret = DISP_STATUS_OK;

	DISPFUNC();
	_primary_path_lock(__func__);

	if (dpmgr_path_is_busy(pgc->dpmgr_handle))
		dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);

	dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);

	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		DISPCHECK("stop display path failed, still busy\n");
		ret = -1;
		goto done;
	}

done:
	_primary_path_unlock(__func__);
	return ret;
}

static int primary_display_remove_output(void *callback, unsigned int userdata)
{
	int ret = 0;
	static cmdqRecHandle cmdq_handle;
	static cmdqRecHandle cmdq_wait_handle;

	if (pgc->need_trigger_ovl1to2 == 0) {
		DISPPR_ERROR("There is no output config when directlink mirror!!\n");
		return 0;
	}
	/*create config thread */
	if (cmdq_handle == NULL)
		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);

	if (ret == 0) {
		/* capture thread wait wdma sof */
		if (cmdq_wait_handle == NULL)
			ret = cmdqRecCreate(CMDQ_SCENARIO_DISP_SCREEN_CAPTURE, &cmdq_wait_handle);

		if (ret == 0) {
			cmdqRecReset(cmdq_wait_handle);
			cmdqRecWait(cmdq_wait_handle, CMDQ_EVENT_DISP_WDMA0_SOF);
			cmdqRecFlush(cmdq_wait_handle);
			/* cmdqRecDestroy(cmdq_wait_handle); */
		} else {
			DISPERR("fail to create  wait handle\n");
		}
		cmdqRecReset(cmdq_handle);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);

		/* update output fence */
		cmdqRecBackupUpdateSlot(cmdq_handle, pgc->cur_config_fence,
					disp_sync_get_output_timeline_id(), mem_config.buff_idx);

		dpmgr_path_remove_memout(pgc->dpmgr_handle, cmdq_handle);

		cmdqRecClearEventToken(cmdq_handle, CMDQ_EVENT_DISP_WDMA0_SOF);
		_cmdq_set_config_handle_dirty_mira(cmdq_handle);
		cmdqRecFlushAsyncCallback(cmdq_handle, callback, 0);
		pgc->need_trigger_ovl1to2 = 0;
		/* cmdqRecDestroy(cmdq_handle); */
	} else {
		ret = -1;
		DISPERR("fail to remove memout out\n");
	}
	return ret;
}

int primary_display_trigger(int blocking, void *callback, unsigned int userdata)
{
	int ret = 0;
#ifdef MTK_FB_DO_NOTHING
	return ret;
#endif
	last_primary_trigger_time = sched_clock();
#ifdef DISP_SWITCH_DST_MODE

	if (is_switched_dst_mode) {
		primary_display_switch_dst_mode(1);	/* swith to vdo mode if trigger disp */
		is_switched_dst_mode = false;
	}
#endif

	primary_trigger_cnt++;

	_primary_path_lock(__func__);

	if (pgc->state == DISP_SLEPT) {
		DISPMSG("%s, skip because primary dipslay is slept\n", __func__);
		goto done;
	}

	if (blocking)
		DISPMSG("%s, change blocking to non blocking trigger\n", __func__);

#ifdef MTK_DISP_IDLE_LP
	_disp_primary_path_exit_idle(__func__, 0);
#endif
	dprec_logger_start(DPREC_LOGGER_PRIMARY_TRIGGER, 0, 0);

	if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE) {
		_trigger_display_interface(blocking, _ovl_fence_release_callback,
					   DISP_SESSION_DIRECT_LINK_MODE);
	} else if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE) {
		_trigger_display_interface(0, _ovl_fence_release_callback,
					   DISP_SESSION_DIRECT_LINK_MIRROR_MODE);
		primary_display_remove_output(_wdma_fence_release_callback,
					      DISP_SESSION_DIRECT_LINK_MIRROR_MODE);
	} else if (pgc->session_mode == DISP_SESSION_DECOUPLE_MODE) {
		_trigger_ovl_to_memory(pgc->ovl2mem_path_handle, pgc->cmdq_handle_ovl1to2_config,
				       _ovl_fence_release_callback, DISP_SESSION_DECOUPLE_MODE);
	} else if (pgc->session_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE) {
		if (pgc->need_trigger_dcMirror_out == 0) {
			DISPPR_ERROR("There is no output config when decouple mirror!!\n");
		} else {
			pgc->need_trigger_dcMirror_out = 0;
			_trigger_ovl_to_memory_mirror(pgc->ovl2mem_path_handle,
						      pgc->cmdq_handle_ovl1to2_config,
						      (fence_release_callback)
						      _olv_wdma_fence_release_callback,
						      DISP_SESSION_DECOUPLE_MIRROR_MODE);
		}
	}
	if (primary_display_use_cmdq == CMDQ_DISABLE)
		dpmgr_path_flush(pgc->dpmgr_handle, primary_display_cmdq_enabled());

	dprec_logger_done(DPREC_LOGGER_PRIMARY_TRIGGER, 0, 0);

done:
	_primary_path_unlock(__func__);
	/* FIXME: find aee_kernel_Powerkey_is_press definitation */
	if ((primary_trigger_cnt > PRIMARY_DISPLAY_TRIGGER_CNT) && aee_kernel_Powerkey_is_press()) {
		aee_kernel_wdt_kick_Powkey_api("primary_display_trigger", WDT_SETBY_Display);
		primary_trigger_cnt = 0;
	}

	if (pgc->session_id > 0)
		update_frm_seq_info(0, 0, 0, FRM_TRIGGER);
	return ret;
}

static int primary_display_ovl2mem_callback(unsigned int userdata)
{
	unsigned int session_id = MAKE_DISP_SESSION(DISP_SESSION_PRIMARY, 0);
	int fence_idx = userdata;
	disp_ddp_path_config *data_config;

	WDMA0_FRAME_START_FLAG = 0;
	data_config = dpmgr_path_get_last_config(pgc->dpmgr_handle);

	if (data_config) {
		WDMA_CONFIG_STRUCT wdma_layer;

		wdma_layer.dstAddress = mtkfb_query_buf_mva(session_id, 4, fence_idx);
		wdma_layer.outputFormat = data_config->wdma_config.outputFormat;
		wdma_layer.srcWidth = primary_display_get_width();
		wdma_layer.srcHeight = primary_display_get_height();
		wdma_layer.dstPitch = data_config->wdma_config.dstPitch;
		dprec_mmp_dump_wdma_layer(&wdma_layer, 0);
	}

	if (fence_idx > 0)
		mtkfb_release_fence(session_id, EXTERNAL_DISPLAY_SESSION_LAYER_COUNT, fence_idx);

#ifdef OVL_CASCADE_SUPPORT
	if (fence_idx < NEW_BUF_IDX && ALL_LAYER_DISABLE_STEP == 1) {
		DISPMSG("primary and memout does not match!!\n");
		cmdqCoreSetEvent(CMDQ_EVENT_DISP_WDMA0_SOF);
		if (_should_set_cmdq_dirty())
			_cmdq_set_config_handle_dirty_mira(pgc->cmdq_handle_ovl1to2_config);

		if ((fence_idx + 1) == NEW_BUF_IDX)
			ALL_LAYER_DISABLE_STEP = 0;
	}
#endif
	DISPMSG("mem_out release fence idx:0x%x\n", fence_idx);

	return 0;
}


int primary_display_mem_out_trigger(int blocking, void *callback, unsigned int userdata)
{
	int ret = 0;
	/* DISPFUNC(); */
	if (pgc->state == DISP_SLEPT || !_is_mirror_mode(pgc->session_mode)) {
		DISPMSG("mem out trigger is already slept or is not mirror mode(%d)\n",
			pgc->session_mode);
		return 0;
	}
	/* /dprec_logger_start(DPREC_LOGGER_PRIMARY_TRIGGER, 0, 0); */

	/* if(blocking) */
	{
		_primary_path_lock(__func__);
	}

	if (pgc->need_trigger_ovl1to2 == 0)
		goto done;


	NEW_BUF_IDX = userdata;
	ALL_LAYER_DISABLE_STEP = 0;

	if (_should_wait_path_idle()) {
		dpmgr_wait_event_timeout(pgc->ovl2mem_path_handle, DISP_PATH_EVENT_FRAME_DONE,
					 HZ * 1);
	}

	if (_should_trigger_path())
		/* /dpmgr_path_trigger(pgc->dpmgr_handle, NULL, primary_display_cmdq_enabled()); */

	if (_should_set_cmdq_dirty())
		_cmdq_set_config_handle_dirty_mira(pgc->cmdq_handle_ovl1to2_config);

	if (gDumpMemoutCmdq == 1) {
		DISPMSG("primary_display_mem_out_trigger, dump before flush 1:\n");
		cmdqRecDumpCommand(pgc->cmdq_handle_ovl1to2_config);
	}

	if (_should_flush_cmdq_config_handle())
		_cmdq_flush_config_handle_mira(pgc->cmdq_handle_ovl1to2_config, 0);

	if (_should_reset_cmdq_config_handle())
		cmdqRecReset(pgc->cmdq_handle_ovl1to2_config);

	cmdqRecWait(pgc->cmdq_handle_ovl1to2_config, CMDQ_EVENT_DISP_WDMA0_SOF);
	WDMA0_FRAME_START_FLAG = 1;
	_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_ovl1to2_config);
	dpmgr_path_remove_memout(pgc->ovl2mem_path_handle, pgc->cmdq_handle_ovl1to2_config);

	if (gDumpMemoutCmdq == 1) {
		DISPMSG("primary_display_mem_out_trigger, dump before flush 2:\n");
		cmdqRecDumpCommand(pgc->cmdq_handle_ovl1to2_config);
	}

	if (_should_flush_cmdq_config_handle())
		/* /_cmdq_flush_config_handle_mira(pgc->cmdq_handle_ovl1to2_config, 0); */
		cmdqRecFlushAsyncCallback(pgc->cmdq_handle_ovl1to2_config,
					  (CmdqAsyncFlushCB) primary_display_ovl2mem_callback,
					  userdata);

	if (_should_reset_cmdq_config_handle())
		cmdqRecReset(pgc->cmdq_handle_ovl1to2_config);

	/* /_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_ovl1to2_config); */

done:

	pgc->need_trigger_ovl1to2 = 0;

	_primary_path_unlock(__func__);

	/* /dprec_logger_done(DPREC_LOGGER_PRIMARY_TRIGGER, 0, 0); */

	return ret;
}


int primary_display_config_output(disp_mem_output_config *output)
{
	int ret = 0;
	disp_ddp_path_config *pconfig = NULL;
	void *cmdq_handle = NULL;

	DISPFUNC();
	_primary_path_lock(__func__);

	if (pgc->state == DISP_SLEPT) {
		DISPMSG("mem out is already slept or mode wrong(%d)\n", pgc->session_mode);
		goto done;
	}

	if (!_is_mirror_mode(pgc->session_mode)) {
		DISPERR("should not config output if not mirror mode!!\n");
		goto done;
	}

	if (_is_decouple_mode(pgc->session_mode)) {

		DISPMSG("primary display config output for decoule mirror\n");
		/*decouple mirror mode */
		ASSERT(output != NULL);
		/* config ovl to mem mode path */
		cmdq_handle = pgc->cmdq_handle_ovl1to2_config;

		pconfig = dpmgr_path_get_last_config(pgc->ovl2mem_path_handle);
		pconfig->wdma_config.dstAddress = output->addr;
		pconfig->wdma_config.srcHeight = output->h;
		pconfig->wdma_config.srcWidth = output->w;
		pconfig->wdma_config.clipX = output->x;
		pconfig->wdma_config.clipY = output->y;
		pconfig->wdma_config.clipHeight = output->h;
		pconfig->wdma_config.clipWidth = output->w;
		pconfig->wdma_config.outputFormat = output->fmt;
		pconfig->wdma_config.useSpecifiedAlpha = 1;
		pconfig->wdma_config.alpha = 0xFF;
		pconfig->wdma_config.dstPitch = output->pitch;
		pconfig->wdma_config.security = output->security;
		pconfig->wdma_dirty = 1;

		if ((pgc->session_id > 0) && _is_decouple_mode(pgc->session_mode))
			update_frm_seq_info(pconfig->wdma_config.dstAddress, 0,
					    mtkfb_query_frm_seq_by_addr(pgc->session_id, 0, 0),
					    FRM_CONFIG);

		ret = dpmgr_path_config(pgc->ovl2mem_path_handle, pconfig, cmdq_handle);

		mem_config = *output;

		MMProfileLogEx(ddp_mmp_get_events()->primary_wdma_config, MMProfileFlagPulse,
			       output->buff_idx, (unsigned int)output->addr);
		pgc->need_trigger_dcMirror_out = 1;

	} else {
		/*direct link  mirror mode */
		cmdq_handle = pgc->cmdq_handle_config;
		dpmgr_path_add_memout(pgc->dpmgr_handle, ENGINE_OVL0, cmdq_handle);

		pconfig = dpmgr_path_get_last_config(pgc->dpmgr_handle);
		pconfig->wdma_config.dstAddress = output->addr;
		pconfig->wdma_config.srcHeight = output->h;
		pconfig->wdma_config.srcWidth = output->w;
		pconfig->wdma_config.clipX = output->x;
		pconfig->wdma_config.clipY = output->y;
		pconfig->wdma_config.clipHeight = output->h;
		pconfig->wdma_config.clipWidth = output->w;
		pconfig->wdma_config.outputFormat = output->fmt;
		pconfig->wdma_config.useSpecifiedAlpha = 1;
		pconfig->wdma_config.alpha = 0xFF;
		pconfig->wdma_config.dstPitch = output->pitch;
		pconfig->wdma_dirty = 1;

		ret = dpmgr_path_config(pgc->dpmgr_handle, pconfig, cmdq_handle);

		mem_config = *output;
		pgc->need_trigger_ovl1to2 = 1;
	}

done:
	_primary_path_unlock(__func__);

	/* /dprec_logger_done(DPREC_LOGGER_PRIMARY_CONFIG, output->src_x, output->src_y); */
	return ret;

}

#if 0
static int _config_interface_input(primary_disp_input_config *input)
{
	int ret = 0;
	int i = 0;
	int layer = 0;
	void *cmdq_handle = NULL;
	disp_ddp_path_config *data_config;

	/* all dirty should be cleared in dpmgr_path_get_last_config() */
	data_config = dpmgr_path_get_last_config(pgc->dpmgr_handle);

	dprec_logger_start(DPREC_LOGGER_PRIMARY_CONFIG, input->layer | (input->layer_en << 16),
			   input->addr);

	ret = _convert_disp_input_to_rdma(&(data_config->rdma_config), input);
	data_config->rdma_dirty = 1;

	if (_should_wait_path_idle()) {
		if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
			dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE,
						 HZ * 1);
		}
	}

	if (primary_display_cmdq_enabled())
		cmdq_handle = pgc->cmdq_handle_config;

	ret = dpmgr_path_config(pgc->dpmgr_handle, data_config, cmdq_handle);


	dprec_logger_done(DPREC_LOGGER_PRIMARY_CONFIG, input->src_x, input->src_y);

	return ret;
}
#endif
static void update_debug_fps_meter(disp_ddp_path_config *data_config)
{
	int i, dst_id = 0;

	for (i = 0; i < HW_OVERLAY_COUNT; i++) {
		if (data_config->ovl_config[i].layer_en
		    && data_config->ovl_config[i].dst_x == 0
		    && data_config->ovl_config[i].dst_y == 0)
			dst_id = i;
	}
	_debug_fps_meter(data_config->ovl_config[dst_id].addr,
			 data_config->ovl_config[dst_id].vaddr,
			 data_config->ovl_config[dst_id].dst_w,
			 data_config->ovl_config[dst_id].dst_h,
			 data_config->ovl_config[dst_id].src_pitch, 0x00000000, dst_id,
			 data_config->ovl_config[dst_id].buff_idx);
}

static int _config_ovl_input(disp_session_input_config *session_input)
{
	int ret = 0;
	int i = 0;
	int layer = 0;
	disp_ddp_path_config *data_config = NULL;
	int max_layer_id_configed = 0;
	int force_disable_ovl1 = 0;
	disp_path_handle disp_handle;
	cmdqRecHandle cmdq_handle;

	if (_is_decouple_mode(pgc->session_mode))
		disp_handle = pgc->ovl2mem_path_handle;
	else
		disp_handle = pgc->dpmgr_handle;

 /*=== create new data_config for ovl input ===*/
	data_config = dpmgr_path_get_last_config(disp_handle);
	for (i = 0; i < session_input->config_layer_num; i++) {
		disp_input_config *input_cfg = &session_input->config[i];
		OVL_CONFIG_STRUCT *ovl_cfg;

		layer = input_cfg->layer_id;
		ovl_cfg = &(data_config->ovl_config[layer]);
		if (session_input->setter != SESSION_USER_AEE) {
			if (isAEEEnabled && layer == primary_display_get_option("ASSERT_LAYER")) {
				DISPMSG("skip AEE layer %d\n", layer);
				continue;
			}
		} else {
			DISPMSG("set AEE layer %d\n", layer);
		}
		_convert_disp_input_to_ovl(ovl_cfg, input_cfg);

		if (ovl_cfg->layer_en)
			_debug_pattern(ovl_cfg->addr, ovl_cfg->vaddr, ovl_cfg->dst_w,
				       ovl_cfg->dst_h, ovl_cfg->src_pitch, 0x00000000,
				       ovl_cfg->layer, ovl_cfg->buff_idx);

		dprec_logger_start(DPREC_LOGGER_PRIMARY_CONFIG,
				   ovl_cfg->layer | (ovl_cfg->layer_en << 16), ovl_cfg->addr);
		dprec_logger_done(DPREC_LOGGER_PRIMARY_CONFIG, ovl_cfg->src_x, ovl_cfg->src_y);

		dprec_mmp_dump_ovl_layer(ovl_cfg, layer, 1);

		if ((ovl_cfg->layer == 0) && (!_is_decouple_mode(pgc->session_mode)))
			update_frm_seq_info(ovl_cfg->addr,
					    ovl_cfg->src_x * 4 +
					    ovl_cfg->src_y * ovl_cfg->src_pitch,
					    mtkfb_query_frm_seq_by_addr(pgc->session_id, 0, 0),
					    FRM_CONFIG);

		if (max_layer_id_configed < layer)
			max_layer_id_configed = layer;

		data_config->ovl_dirty = 1;
	}

#ifdef OVL_CASCADE_SUPPORT
	if (ovl_get_status() == DDP_OVL1_STATUS_SUB_REQUESTING) {
		/* disable ovl layer 4~8 to free ovl1 */
		if (max_layer_id_configed < OVL_LAYER_NUM_PER_OVL - isAEEEnabled) {
			for (i = OVL_LAYER_NUM_PER_OVL; i < OVL_LAYER_NUM; i++)
				data_config->ovl_config[i].layer_en = 0;

			force_disable_ovl1 = 1;
			DISPMSG("cascade: HWC set %d layers, force disable OVL1 layers\n",
				max_layer_id_configed);
		} else {
			DISPMSG("cascade: try to split ovl1 fail: HWC set %d layers\n",
				max_layer_id_configed);
		}
	}
	if (ovl_get_status() == DDP_OVL1_STATUS_SUB) {
		/*disable ovl layer 4~8 to free ovl1 */
		if (OVL_LAYER_NUM_PER_OVL - isAEEEnabled <= max_layer_id_configed) {
			for (i = OVL_LAYER_NUM_PER_OVL; i < OVL_LAYER_NUM; i++)
				data_config->ovl_config[i].layer_en = 0;

			force_disable_ovl1 = 1;
			DISPMSG
			    ("cascade: HWC set %d layers, but ovl1 is %d state, force disable OVL1 layers\n",
			     max_layer_id_configed, ovl_get_status());
		}
	}
#endif
	if (_is_decouple_mode(pgc->session_mode))
		cmdq_handle = pgc->cmdq_handle_ovl1to2_config;
	else
		cmdq_handle = pgc->cmdq_handle_config;

	if (_should_wait_path_idle())
		dpmgr_wait_event_timeout(disp_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);

	/*should we use cmdq_hand_config ? need to check */
	ret =
	    dpmgr_path_config(disp_handle, data_config,
			      primary_display_cmdq_enabled() ? cmdq_handle : NULL);

	/* write fence_id/enable to DRAM using cmdq
	 * it will be used when release fence (put these after config registers done)*/
	for (i = 0; i < session_input->config_layer_num; i++) {
		unsigned int last_fence, cur_fence;
		disp_input_config *input_cfg = &session_input->config[i];

		layer = input_cfg->layer_id;

		cmdqBackupReadSlot(pgc->cur_config_fence, layer, &last_fence);
		cur_fence = input_cfg->next_buff_idx;

		if (cur_fence != -1 && cur_fence > last_fence)
			cmdqRecBackupUpdateSlot(cmdq_handle, pgc->cur_config_fence, layer,
						cur_fence);

		/* for dim_layer/disable_layer/no_fence_layer, just release all fences configured */
		/* for other layers, release current_fence-1 */
		if (input_cfg->buffer_source == DISP_BUFFER_ALPHA
		    || input_cfg->layer_enable == 0 || cur_fence == -1)
			cmdqRecBackupUpdateSlot(cmdq_handle, pgc->subtractor_when_free, layer, 0);
		else
			cmdqRecBackupUpdateSlot(cmdq_handle, pgc->subtractor_when_free, layer, 1);
	}

	if (force_disable_ovl1) {
		for (layer = OVL_LAYER_NUM_PER_OVL; layer < OVL_LAYER_NUM; layer++) {
			/* will release all fences */
			cmdqRecBackupUpdateSlot(cmdq_handle, pgc->subtractor_when_free, layer, 0);
		}
	}

	update_debug_fps_meter(data_config);

	return ret;
}

#if 0
static int _config_ovl_output(disp_mem_output_config *output)
{
	int ret = 0;
	disp_ddp_path_config *data_config = NULL;
	disp_path_handle *handle = NULL;

	data_config = dpmgr_path_get_last_config(pgc->ovl2mem_path_handle);

	data_config->wdma_dirty = 1;

	if (_should_wait_path_idle())
		dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);

	handle = pgc->ovl2mem_path_handle;

	ret = dpmgr_path_config(pgc->ovl2mem_path_handle, data_config, pgc->cmdq_handle_config);

	return ret;
}
#endif
static int _config_rdma_input(disp_session_input_config *session_input, disp_path_handle *handle)
{
	int ret;
	disp_ddp_path_config *data_config = NULL;

	/* all dirty should be cleared in dpmgr_path_get_last_config() */
	data_config = dpmgr_path_get_last_config(handle);
	data_config->dst_dirty = 0;
	data_config->ovl_dirty = 0;
	data_config->rdma_dirty = 0;
	data_config->wdma_dirty = 0;

	ret =
	    _convert_disp_input_to_rdma(&(data_config->rdma_config),
					(disp_input_config *) session_input);
	data_config->rdma_dirty = 1;

	if (_should_wait_path_idle())
		dpmgr_wait_event_timeout(handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);

	ret =
	    dpmgr_path_config(handle, data_config,
			      primary_display_cmdq_enabled() ? pgc->cmdq_handle_config : NULL);
	return ret;
}

int primary_display_config_input_multiple(disp_session_input_config *session_input)
{
	int ret = 0;
	unsigned int wdma_mva = 0;
	disp_path_handle disp_handle = NULL;
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
	bool is_sec = false;
	int i = 0;
	int height = primary_display_get_height();
	int width = primary_display_get_width();
	int bpp = 24;   /*internal picture is RGB888*/

	int buffer_size = width * height * bpp / 8;
#endif
	_primary_path_lock(__func__);

	if (pgc->state == DISP_SLEPT) {
		DISPMSG("%s, skip because primary dipslay is slept\n", __func__);
		goto done;
	}
#ifdef MTK_DISP_IDLE_LP
	/* call this in trigger is enough, do not have to call this in config */
	_disp_primary_path_exit_idle(__func__, 0);
#endif

	if (_should_config_ovl_input())
		_config_ovl_input(session_input);
	else
		_config_rdma_input(session_input, disp_handle);

#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
	if (_is_decouple_mode(pgc->session_mode))
		disp_handle = pgc->ovl2mem_path_handle;
	else
		disp_handle = pgc->dpmgr_handle;

	is_sec = _is_sec(disp_handle);
	if (is_sec && (pgc->dc_buf_sec[0] == 0)) {
		for (i = 0; i < DISP_INTERNAL_BUFFER_COUNT; i++)
			pgc->dc_buf_sec[i] = allocate_decouple_sec_buffer(buffer_size);
	}
#endif
	if (_is_decouple_mode(pgc->session_mode) && !_is_mirror_mode(pgc->session_mode)) {
		pgc->dc_buf_id++;
		pgc->dc_buf_id %= DISP_INTERNAL_BUFFER_COUNT;
		wdma_mva = pgc->dc_buf[pgc->dc_buf_id];
	#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
		if (is_sec) {
			wdma_mva = pgc->dc_buf_sec[pgc->dc_buf_id];
			decouple_wdma_config.security = DISP_SECURE_BUFFER;
		} else {
			decouple_wdma_config.security = DISP_NORMAL_BUFFER;
		}
	#endif
		decouple_wdma_config.dstAddress = wdma_mva;
		_config_wdma_output(&decouple_wdma_config, pgc->ovl2mem_path_handle,
				    pgc->cmdq_handle_ovl1to2_config);
		mem_config.addr = wdma_mva;
		MMProfileLogEx(ddp_mmp_get_events()->primary_wdma_config, MMProfileFlagPulse,
			       pgc->dc_buf_id, wdma_mva);
	}

	pgc->hwc_refresh_cnt++;
	pgc->hwc_refresh_time = get_current_time_us();

done:
	_primary_path_unlock(__func__);
	return ret;
}

int primary_display_config_interface_input(primary_disp_input_config *input)
{
	int ret = 0;
	void *cmdq_handle = NULL;
	disp_ddp_path_config *data_config;

	/* all dirty should be cleared in dpmgr_path_get_last_config() */
	data_config = dpmgr_path_get_last_config(pgc->dpmgr_handle);

	dprec_logger_start(DPREC_LOGGER_PRIMARY_CONFIG, input->layer | (input->layer_en << 16),
			   input->addr);

	_primary_path_lock(__func__);

	if (pgc->state == DISP_SLEPT) {
		DISPMSG("%s, skip because primary dipslay is slept\n", __func__);
		goto done;
	}

	ret = _convert_disp_input_to_rdma(&(data_config->rdma_config), (disp_input_config *) input);
	data_config->rdma_dirty = 1;

	if (_should_wait_path_idle()) {
		if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
			dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE,
						 HZ * 1);
		}
	}

	if (primary_display_cmdq_enabled())
		cmdq_handle = pgc->cmdq_handle_config;

	ret = dpmgr_path_config(pgc->dpmgr_handle, data_config, cmdq_handle);

	/* this is used for decouple mode, to indicate whether we need to trigger ovl */
	pgc->need_trigger_overlay = 1;

	dprec_logger_done(DPREC_LOGGER_PRIMARY_CONFIG, input->src_x, input->src_y);

done:
	_primary_path_unlock(__func__);

	return ret;
}

static int Panel_Master_primary_display_config_dsi(const char *name, uint32_t config_value)
{
	int ret = 0;
	disp_ddp_path_config *data_config;
	/* all dirty should be cleared in dpmgr_path_get_last_config() */
	data_config = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	/* modify below for config dsi */
	if (!strcmp(name, "PM_CLK")) {
		pr_debug("Pmaster_config_dsi: PM_CLK:%d\n", config_value);
		data_config->dispif_config.dsi.PLL_CLOCK = config_value;
	} else if (!strcmp(name, "PM_SSC")) {
		data_config->dispif_config.dsi.ssc_range = config_value;
	}
	pr_debug("Pmaster_config_dsi: will Run path_config()\n");
	ret = dpmgr_path_config(pgc->dpmgr_handle, data_config, NULL);

	return ret;
}

int primary_display_user_cmd(unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	cmdqRecHandle handle = NULL;
	int cmdqsize = 0;

	MMProfileLogEx(ddp_mmp_get_events()->primary_display_cmd, MMProfileFlagStart,
		       (unsigned long)handle, 0);

	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);
	cmdqRecReset(handle);
	_cmdq_insert_wait_frame_done_token_mira(handle);
	cmdqsize = cmdqRecGetInstructionCount(handle);

	_primary_path_lock(__func__);
	if (pgc->state == DISP_SLEPT) {
		cmdqRecDestroy(handle);
		handle = NULL;
	}
	_primary_path_unlock(__func__);

#ifdef MTK_DISP_IDLE_LP
	/* will write register in dpmgr_path_user_cmd, need to exit idle */
	if (gDecouplePQWithRDMA == 0) {
		last_primary_trigger_time = sched_clock();
		_disp_primary_path_exit_idle(__func__, 1);
	}
#endif
	ret = dpmgr_path_user_cmd(pgc->dpmgr_handle, cmd, arg, handle);

	if (handle) {
		if (cmdqRecGetInstructionCount(handle) > cmdqsize) {
			_primary_path_lock(__func__);
			if (pgc->state == DISP_ALIVE) {
				_cmdq_set_config_handle_dirty_mira(handle);
				/* use non-blocking flush here to avoid primary path is locked for too long */
				if (_should_flush_cmdq_config_handle())
					_cmdq_flush_config_handle_mira(handle, 0);
			}
			_primary_path_unlock(__func__);
		}

		cmdqRecDestroy(handle);
	}

	MMProfileLogEx(ddp_mmp_get_events()->primary_display_cmd, MMProfileFlagEnd,
		       (unsigned long)handle, cmdqsize);

	return ret;
}

int primary_display_switch_mode(int sess_mode, unsigned int session, int force)
{
	DISPDBG("primary_display_switch_mode sess_mode %d, session 0x%x\n", sess_mode, session);

	_primary_path_lock(__func__);

	MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagStart,
		       pgc->session_mode, sess_mode);

	if (pgc->session_mode == sess_mode)
		goto done;

	if (pgc->state == DISP_SLEPT) {
		DISPMSG("primary display switch from %s to %s in suspend state!!!\n",
			session_mode_spy(pgc->session_mode), session_mode_spy(sess_mode));
		pgc->session_mode = sess_mode;
		goto done;
	}
	DISPMSG("primary display will switch from %s to %s\n", session_mode_spy(pgc->session_mode),
		session_mode_spy(sess_mode));

	if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE
	    && sess_mode == DISP_SESSION_DECOUPLE_MODE) {
		/* dl to dc */
		_DL_switch_to_DC_fast();
		pgc->session_mode = sess_mode;
		DISPMSG("primary display is %s mode now\n", session_mode_spy(pgc->session_mode));
		MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse,
			       pgc->session_mode, sess_mode);
		/* primary_display_diagnose(); */
	} else if (pgc->session_mode == DISP_SESSION_DECOUPLE_MODE
		   && sess_mode == DISP_SESSION_DIRECT_LINK_MODE) {
		/* dc to dl */
		_DC_switch_to_DL_fast();
		pgc->session_mode = sess_mode;
		DISPMSG("primary display is %s mode now\n", session_mode_spy(pgc->session_mode));
		MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse,
			       pgc->session_mode, sess_mode);
		/* primary_display_diagnose(); */
	} else if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE
		   && sess_mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE) {
		/* dl to dl mirror */
		/* cmdqRecCreate(CMDQ_SCENARIO_DISP_SCREEN_CAPTURE, &pgc->cmdq_handle_dl_mirror); */
		/* cmdqRecReset(pgc->cmdq_handle_dl_mirror); */
		pgc->session_mode = sess_mode;
		DISPMSG("primary display is %s mode now\n", session_mode_spy(pgc->session_mode));
		MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse,
			       pgc->session_mode, sess_mode);
	} else if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE
		   && sess_mode == DISP_SESSION_DIRECT_LINK_MODE) {
		/*dl mirror to dl */
		/* cmdqRecDestroy(pgc->cmdq_handle_dl_mirror); */
		/* pgc->cmdq_handle_dl_mirror = NULL; */
		pgc->session_mode = sess_mode;
		DISPMSG("primary display is %s mode now\n", session_mode_spy(pgc->session_mode));
		MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse,
			       pgc->session_mode, sess_mode);
	} else if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE
		   && sess_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE) {
		/* dl to dc mirror  mirror */
		_DL_switch_to_DC_fast();
		pgc->session_mode = sess_mode;
		DISPMSG("primary display is %s mode now\n", session_mode_spy(pgc->session_mode));
		MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse,
			       pgc->session_mode, sess_mode);
		/* primary_display_diagnose(); */
	} else if (pgc->session_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE
		   && sess_mode == DISP_SESSION_DIRECT_LINK_MODE) {
		/*dc mirror  to dl */
		_DC_switch_to_DL_fast();
		pgc->session_mode = sess_mode;
		DISPMSG("primary display is %s mode now\n", session_mode_spy(pgc->session_mode));
		MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse,
			       pgc->session_mode, sess_mode);
		/* primary_display_diagnose(); */
	} else {
		DISPERR("invalid mode switch from %s to %s\n", session_mode_spy(pgc->session_mode),
			session_mode_spy(sess_mode));
	}
done:
	_primary_path_unlock(__func__);
	pgc->session_id = session;

	MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagEnd,
		       pgc->session_mode, sess_mode);
	return 0;

}

#ifdef MTK_DISP_IDLE_LP
int primary_display_switch_mode_nolock(int sess_mode, unsigned int session, int force)
{
	DISPDBG("[ddp_idle]primary_display_switch_mode sess_mode %d, session 0x%x\n", sess_mode, session);

	/* if(!force && _is_decouple_mode(pgc->session_mode)) */
	/* return 0; */

	/* _primary_path_lock(__func__); */

	MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagStart,
		       pgc->session_mode, sess_mode);

	if (pgc->session_mode == sess_mode)
		goto done;

	DISPMSG("[ddp_idle]primary display will switch from %s to %s\n", session_mode_spy(pgc->session_mode),
		session_mode_spy(sess_mode));

	if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE
	    && sess_mode == DISP_SESSION_DECOUPLE_MODE) {
		/* signal frame start event to switch logic in fence_release_worker_thread */
		/* dpmgr_signal_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START); */
		_DL_switch_to_DC_fast();
		pgc->session_mode = sess_mode;
		DISPMSG("[ddp_idle]primary display is %s mode now\n", session_mode_spy(pgc->session_mode));
		MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse,
			       pgc->session_mode, sess_mode);
		/* primary_display_diagnose(); */
	} else if (pgc->session_mode == DISP_SESSION_DECOUPLE_MODE
		   && sess_mode == DISP_SESSION_DIRECT_LINK_MODE) {
		/* signal frame start event to switch logic in fence_release_worker_thread */
		/* dpmgr_signal_event(pgc->ovl2mem_path_handle, DISP_PATH_EVENT_FRAME_START); */
		_DC_switch_to_DL_fast();
		pgc->session_mode = sess_mode;
		DISPMSG("[ddp_idle]primary display is %s mode now\n", session_mode_spy(pgc->session_mode));
		MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse,
			       pgc->session_mode, sess_mode);
		/* primary_display_diagnose(); */
	} else if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE
		   && sess_mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE) {
		/*need delay switch to output */
		pgc->session_mode = sess_mode;
	} else if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE
		   && sess_mode == DISP_SESSION_DIRECT_LINK_MODE) {
		/* xxx */
		pgc->session_mode = sess_mode;
	} else if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE
		   && sess_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE) {
		/*need delay switch to output */
		_DL_switch_to_DC_fast();
		pgc->session_mode = sess_mode;
		DISPMSG("[ddp_idle]primary display is %s mode now\n", session_mode_spy(pgc->session_mode));
		MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse,
			       pgc->session_mode, sess_mode);
		/* primary_display_diagnose(); */
		/* pgc->session_delay_mode = sess_mode; */
	} else if (pgc->session_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE
		   && sess_mode == DISP_SESSION_DIRECT_LINK_MODE) {
		_DC_switch_to_DL_fast();
		pgc->session_mode = sess_mode;
		DISPMSG("[ddp_idle]primary display is %s mode now\n", session_mode_spy(pgc->session_mode));
		MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse,
			       pgc->session_mode, sess_mode);
		/* primary_display_diagnose(); */
	} else {
		DISPERR("invalid mode switch from %s to %s\n", session_mode_spy(pgc->session_mode),
			session_mode_spy(sess_mode));
	}
done:
	/* _primary_path_unlock(__func__); */
	pgc->session_id = session;

	MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagEnd,
		       pgc->session_mode, sess_mode);
	return 0;

}
#endif

int primary_display_is_alive(void)
{
	unsigned int temp = 0;
	/* DISPFUNC(); */
	_primary_path_lock(__func__);

	if (pgc->state == DISP_ALIVE)
		temp = 1;

	_primary_path_unlock(__func__);

	return temp;
}

int primary_display_is_sleepd(void)
{
	unsigned int temp = 0;
	/* DISPFUNC(); */
	_primary_path_lock(__func__);

	if (pgc->state == DISP_SLEPT)
		temp = 1;

	_primary_path_unlock(__func__);

	return temp;
}



int primary_display_get_width(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return 0;
	}

	if (pgc->plcm->params)
		return pgc->plcm->params->width;
	DISPERR("lcm_params is null!\n");
	return 0;
}

int primary_display_get_height(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return 0;
	}

	if (pgc->plcm->params)
		return pgc->plcm->params->height;
	DISPERR("lcm_params is null!\n");
	return 0;
}


int primary_display_get_original_width(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return 0;
	}

	if (pgc->plcm->params)
		return pgc->plcm->lcm_original_width;
	DISPERR("lcm_params is null!\n");
	return 0;
}

int primary_display_get_original_height(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return 0;
	}

	if (pgc->plcm->params)
		return pgc->plcm->lcm_original_height;
	DISPERR("lcm_params is null!\n");
	return 0;
}

int primary_display_get_bpp(void)
{
	return 32;
}

void primary_display_set_max_layer(int maxlayer)
{
	pgc->max_layer = maxlayer;
}

int primary_display_get_info(void *info)
{
#if 1
	/* DISPFUNC(); */
	disp_session_info *dispif_info = (disp_session_info *) info;

	LCM_PARAMS *lcm_param = disp_lcm_get_params(pgc->plcm);

	if (lcm_param == NULL) {
		DISPCHECK("lcm_param is null\n");
		return -1;
	}

	memset((void *)dispif_info, 0, sizeof(disp_session_info));

#ifdef OVL_CASCADE_SUPPORT
	if (is_DAL_Enabled() && pgc->max_layer == OVL_LAYER_NUM) {
		if (ovl_get_status() == DDP_OVL1_STATUS_PRIMARY ||
			ovl_get_status() == DDP_OVL1_STATUS_IDLE) {
			/* OVL1 is used by mem session */
			dispif_info->maxLayerNum = pgc->max_layer - 1;
		} else
			dispif_info->maxLayerNum = pgc->max_layer - 1 - 4;

	} else {
		if (ovl_get_status() == DDP_OVL1_STATUS_PRIMARY ||
			ovl_get_status() == DDP_OVL1_STATUS_IDLE) {
			/* OVL1 is used by mem session */
			dispif_info->maxLayerNum = pgc->max_layer;
		} else
			dispif_info->maxLayerNum = pgc->max_layer - 4;

	}
#else
	if (is_DAL_Enabled() && pgc->max_layer == OVL_LAYER_NUM)
		dispif_info->maxLayerNum = pgc->max_layer - 1;
	else
		dispif_info->maxLayerNum = pgc->max_layer;
#endif
	/* DISPDBG("available layer num=%d\n", dispif_info->maxLayerNum); */

	switch (lcm_param->type) {
	case LCM_TYPE_DBI:
		{
			dispif_info->displayType = DISP_IF_TYPE_DBI;
			dispif_info->displayMode = DISP_IF_MODE_COMMAND;
			dispif_info->isHwVsyncAvailable = 1;
			/* DISPMSG("DISP Info: DBI, CMD Mode, HW Vsync enable\n"); */
			break;
		}
	case LCM_TYPE_DPI:
		{
			dispif_info->displayType = DISP_IF_TYPE_DPI;
			dispif_info->displayMode = DISP_IF_MODE_VIDEO;
			dispif_info->isHwVsyncAvailable = 1;
			/* DISPMSG("DISP Info: DPI, VDO Mode, HW Vsync enable\n"); */
			break;
		}
	case LCM_TYPE_DSI:
		{
			dispif_info->displayType = DISP_IF_TYPE_DSI0;
			if (lcm_param->dsi.mode == CMD_MODE) {
				dispif_info->displayMode = DISP_IF_MODE_COMMAND;
				dispif_info->isHwVsyncAvailable = 1;
				/* DISPMSG("DISP Info: DSI, CMD Mode, HW Vsync enable\n"); */
			} else {
				dispif_info->displayMode = DISP_IF_MODE_VIDEO;
				dispif_info->isHwVsyncAvailable = 1;
				/* DISPMSG("DISP Info: DSI, VDO Mode, HW Vsync enable\n"); */
			}

			break;
		}
	default:
		break;
	}


	dispif_info->displayFormat = DISP_IF_FORMAT_RGB888;

	dispif_info->displayWidth = primary_display_get_width();
	dispif_info->displayHeight = primary_display_get_height();

	dispif_info->physicalWidth = DISP_GetActiveWidth();
	dispif_info->physicalHeight = DISP_GetActiveHeight();


	dispif_info->vsyncFPS = pgc->lcm_fps;
	dispif_info->isConnected = 1;

#if 0
#error
	{
		LCM_PARAMS lcm_params_temp;

		memset((void *)&lcm_params_temp, 0, sizeof(lcm_params_temp));
		if (lcm_drv) {
			lcm_drv->get_params(&lcm_params_temp);
			dispif_info->lcmOriginalWidth = lcm_params_temp.width;
			dispif_info->lcmOriginalHeight = lcm_params_temp.height;
			DISPMSG("DISP Info: LCM Panel Original Resolution(For DFO Only): %d x %d\n",
				dispif_info->lcmOriginalWidth, dispif_info->lcmOriginalHeight);
		} else {
			DISPMSG("DISP Info: Fatal Error!!, lcm_drv is null\n");
		}
	}
#endif

#endif
	return 0;
}

int primary_display_get_pages(void)
{
	return 3;
}


int primary_display_is_video_mode(void)
{
	/* TODO: we should store the video/cmd mode in runtime, because we will support cmd/vdo dynamic switch */
	return disp_lcm_is_video_mode(pgc->plcm);
}

int primary_display_is_decouple_mode(void)
{
	return _is_decouple_mode(pgc->session_mode);
}

int primary_display_is_mirror_mode(void)
{
	return _is_mirror_mode(pgc->session_mode);
}

int primary_display_diagnose(void)
{
	int ret = 0;
#ifdef MTK_FB_DO_NOTHING
	return ret;
#endif
	dpmgr_check_status(pgc->dpmgr_handle);
	if (_is_decouple_mode(pgc->session_mode) && (pgc->ovl2mem_path_handle != NULL))
		dpmgr_check_status(pgc->ovl2mem_path_handle);
	primary_display_check_path(NULL, 0);

	return ret;
}

CMDQ_SWITCH primary_display_cmdq_enabled(void)
{
	return primary_display_use_cmdq;
}

int primary_display_switch_cmdq_cpu(CMDQ_SWITCH use_cmdq)
{
	_primary_path_lock(__func__);

	primary_display_use_cmdq = use_cmdq;
	DISPCHECK("display driver use %s to config register now\n",
		  (use_cmdq == CMDQ_ENABLE) ? "CMDQ" : "CPU");

	_primary_path_unlock(__func__);
	return primary_display_use_cmdq;
}

int primary_display_manual_lock(void)
{
	_primary_path_lock(__func__);
	return 0;
}

int primary_display_manual_unlock(void)
{
	_primary_path_unlock(__func__);
	return 0;
}

void primary_display_reset(void)
{
	dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
}

unsigned int primary_display_get_fps(void)
{
	unsigned int fps = 0;

	_primary_path_lock(__func__);
	fps = pgc->lcm_fps;
	_primary_path_unlock(__func__);

	return fps;
}

int primary_display_force_set_vsync_fps(unsigned int fps)
{
	int ret = 0;

	DISPMSG("force set fps to %d\n", fps);
	_primary_path_lock(__func__);

	if (fps == pgc->lcm_fps) {
		pgc->vsync_drop = 0;
		ret = 0;
	} else if (fps == 30) {
		pgc->vsync_drop = 1;
		ret = 0;
	} else {
		ret = -1;
	}

	_primary_path_unlock(__func__);

	return ret;
}

int primary_display_enable_path_cg(int enable)
{
	int ret = 0;
#ifdef ENABLE_CLK_MGR
	DISPMSG("%s primary display's path cg\n", enable ? "enable" : "disable");
	_primary_path_lock(__func__);

	if (enable) {
#ifdef CONFIG_MTK_CLKMGR
		ret += disable_clock(MT_CG_DISP1_DSI_ENGINE, "DSI0");
		ret += disable_clock(MT_CG_DISP1_DSI_DIGITAL, "DSI0");
		ret += disable_clock(MT_CG_DISP0_DISP_OVL0, "DDP");
		ret += disable_clock(MT_CG_DISP0_DISP_COLOR, "DDP");
		ret += disable_clock(MT_CG_DISP0_DISP_CCORR, "DDP");
		ret += disable_clock(MT_CG_DISP0_DISP_AAL, "DDP");
		ret += disable_clock(MT_CG_DISP0_DISP_GAMMA, "DDP");
		ret += disable_clock(MT_CG_DISP0_DISP_DITHER, "DDP");
		ret += disable_clock(MT_CG_DISP0_DISP_RDMA0, "DDP");
		ret += disable_clock(MT_CG_DISP0_DISP_UFOE, "DDP");

		ret += disable_clock(MT_CG_DISP1_DISP_PWM_26M, "PWM");
		ret += disable_clock(MT_CG_DISP1_DISP_PWM_MM, "PWM");

		/* ret += disable_clock(MT_CG_DISP0_MUTEX_32K     , "Debug"); */
		ret += disable_clock(MT_CG_DISP0_SMI_LARB0, "Debug");
		ret += disable_clock(MT_CG_DISP0_SMI_COMMON, "Debug");

		/* ret += disable_clock(MT_CG_DISP0_MUTEX_32K   , "Debug2"); */
		ret += disable_clock(MT_CG_DISP0_SMI_LARB0, "Debug2");
		ret += disable_clock(MT_CG_DISP0_SMI_COMMON, "Debug2");
#else
#endif
		ret += ddp_module_clock_enable(MM_CLK_DSI_ENGINE, false);
		ret += ddp_module_clock_enable(MM_CLK_DSI_DIGITAL, false);
		ret += ddp_module_clock_enable(MM_CLK_DISP_OVL0, false);
		ret += ddp_module_clock_enable(MM_CLK_DISP_COLOR, false);
		ret += ddp_module_clock_enable(MM_CLK_DISP_CCORR, false);
		ret += ddp_module_clock_enable(MM_CLK_DISP_AAL, false);
		ret += ddp_module_clock_enable(MM_CLK_DISP_GAMMA, false);
		ret += ddp_module_clock_enable(MM_CLK_DISP_DITHER, false);
		ret += ddp_module_clock_enable(MM_CLK_DISP_RDMA0, false);
		ret += ddp_module_clock_enable(MM_CLK_DISP_UFOE, false);
		ret += ddp_module_clock_enable(MM_CLK_DISP_PWM_26M, false);
		ret += ddp_module_clock_enable(MM_CLK_DISP_PWM, false);
		ret += mtk_smi_larb_clock_on(0, true);

	} else {
#ifdef CONFIG_MTK_CLKMGR
		ret += enable_clock(MT_CG_DISP1_DSI_ENGINE, "DSI0");
		ret += enable_clock(MT_CG_DISP1_DSI_DIGITAL, "DSI0");
		ret += enable_clock(MT_CG_DISP0_DISP_OVL0, "DDP");
		ret += enable_clock(MT_CG_DISP0_DISP_COLOR, "DDP");
		ret += enable_clock(MT_CG_DISP0_DISP_CCORR, "DDP");
		ret += enable_clock(MT_CG_DISP0_DISP_AAL, "DDP");
		ret += enable_clock(MT_CG_DISP0_DISP_GAMMA, "DDP");
		ret += enable_clock(MT_CG_DISP0_DISP_DITHER, "DDP");
		ret += enable_clock(MT_CG_DISP0_DISP_RDMA0, "DDP");
		ret += enable_clock(MT_CG_DISP0_DISP_UFOE, "DDP");

		ret += enable_clock(MT_CG_DISP1_DISP_PWM_26M, "PWM");
		ret += enable_clock(MT_CG_DISP1_DISP_PWM_MM, "PWM");

		/* ret += enable_clock(MT_CG_DISP0_MUTEX_32K      , "Debug"); */
		ret += enable_clock(MT_CG_DISP0_SMI_LARB0, "Debug");
		ret += enable_clock(MT_CG_DISP0_SMI_COMMON, "Debug");

		/* ret += enable_clock(MT_CG_DISP0_MUTEX_32K   , "Debug2"); */
		ret += enable_clock(MT_CG_DISP0_SMI_LARB0, "Debug2");
		ret += enable_clock(MT_CG_DISP0_SMI_COMMON, "Debug2");
#else
#endif
		ret += ddp_module_clock_enable(MM_CLK_DSI_ENGINE, true);
		ret += ddp_module_clock_enable(MM_CLK_DSI_DIGITAL, true);
		ret += ddp_module_clock_enable(MM_CLK_DISP_OVL0, true);
		ret += ddp_module_clock_enable(MM_CLK_DISP_COLOR, true);
		ret += ddp_module_clock_enable(MM_CLK_DISP_CCORR, true);
		ret += ddp_module_clock_enable(MM_CLK_DISP_AAL, true);
		ret += ddp_module_clock_enable(MM_CLK_DISP_GAMMA, true);
		ret += ddp_module_clock_enable(MM_CLK_DISP_DITHER, true);
		ret += ddp_module_clock_enable(MM_CLK_DISP_RDMA0, true);
		ret += ddp_module_clock_enable(MM_CLK_DISP_UFOE, true);
		ret += ddp_module_clock_enable(MM_CLK_DISP_PWM_26M, true);
		ret += ddp_module_clock_enable(MM_CLK_DISP_PWM, true);
		mtk_smi_larb_clock_off(0, true);

	}

	_primary_path_unlock(__func__);
#endif

	return ret;
}

int _set_backlight_by_cmdq(unsigned int level)
{
	int ret = 0;
	cmdqRecHandle cmdq_handle_backlight = NULL;

	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 1);
	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle_backlight);
	DISPCHECK("primary backlight, handle=%p\n", cmdq_handle_backlight);
	if (ret != 0) {
		DISPCHECK("fail to create primary cmdq handle for backlight\n");
		return -1;
	}

	if (primary_display_is_video_mode()) {
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 2);
		cmdqRecReset(cmdq_handle_backlight);
		dpmgr_path_ioctl(pgc->dpmgr_handle, cmdq_handle_backlight, DDP_BACK_LIGHT,
				 (unsigned long *)&level);
		_cmdq_flush_config_handle_mira(cmdq_handle_backlight, 1);
		DISPCHECK("[BL]_set_backlight_by_cmdq ret=%d\n", ret);
	} else {
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 3);
		cmdqRecReset(cmdq_handle_backlight);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_backlight);
		cmdqRecClearEventToken(cmdq_handle_backlight, CMDQ_SYNC_TOKEN_CABC_EOF);
		dpmgr_path_ioctl(pgc->dpmgr_handle, cmdq_handle_backlight, DDP_BACK_LIGHT,
				 (unsigned long *)&level);
		cmdqRecSetEventToken(cmdq_handle_backlight, CMDQ_SYNC_TOKEN_CABC_EOF);
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 4);
		_cmdq_flush_config_handle_mira(cmdq_handle_backlight, 1);
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 6);
		DISPCHECK("[BL]_set_backlight_by_cmdq ret=%d\n", ret);
	}
	cmdqRecDestroy(cmdq_handle_backlight);
	cmdq_handle_backlight = NULL;
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 5);
	return ret;
}

int _set_backlight_by_cpu(unsigned int level)
{
	int ret = 0;

	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 0, 1);
	if (primary_display_is_video_mode()) {
		disp_lcm_set_backlight(pgc->plcm, level);
	} else {
		DISPCHECK("[BL]display cmdq trigger loop stop[begin]\n");
		_cmdq_stop_trigger_loop();
		DISPCHECK("[BL]display cmdq trigger loop stop[end]\n");

		if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
			DISPCHECK("[BL]primary display path is busy\n");
			ret =
			    dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE,
						     HZ * 1);
			DISPCHECK("[BL]wait frame done ret:%d\n", ret);
		}

		DISPCHECK("[BL]stop dpmgr path[begin]\n");
		dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);
		DISPCHECK("[BL]stop dpmgr path[end]\n");
		if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
			DISPCHECK("[BL]primary display path is busy after stop\n");
			dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE,
						 HZ * 1);
			DISPCHECK("[BL]wait frame done ret:%d\n", ret);
		}
		DISPCHECK("[BL]reset display path[begin]\n");
		dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
		DISPCHECK("[BL]reset display path[end]\n");

		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 0, 2);

		disp_lcm_set_backlight(pgc->plcm, level);

		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 0, 3);

		DISPCHECK("[BL]start dpmgr path[begin]\n");
		dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
		DISPCHECK("[BL]start dpmgr path[end]\n");

		DISPCHECK("[BL]start cmdq trigger loop[begin]\n");
		_cmdq_start_trigger_loop();
		DISPCHECK("[BL]start cmdq trigger loop[end]\n");
	}
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 0, 7);
	return ret;
}

int primary_display_setbacklight(unsigned int level)
{
	int ret = 0;

	DISPFUNC();

	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagStart, 0, 0);
#ifdef DISP_SWITCH_DST_MODE
	_primary_path_switch_dst_lock();
#endif
	_primary_path_cmd_lock();
	_primary_path_lock(__func__);
	if (pgc->state == DISP_SLEPT) {
		DISPCHECK("Slept State set backlight invald\n");
	} else {
		if (primary_display_cmdq_enabled()) {
			if (primary_display_is_video_mode()) {
				MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl,
					       MMProfileFlagPulse, 0, 7);
				disp_lcm_set_backlight(pgc->plcm, level);
			} else {
				_set_backlight_by_cmdq(level);
			}
		} else {
			_set_backlight_by_cpu(level);
		}
	}
#ifdef GPIO_LCM_LED_EN
	if (0 == level)
		mt_set_gpio_out(GPIO_LCM_LED_EN, GPIO_OUT_ZERO);
	else
		mt_set_gpio_out(GPIO_LCM_LED_EN, GPIO_OUT_ONE);
#endif
#if 0
	{

		uint8_t buffer[2];

		if (primary_display_is_video_mode())
			dpmgr_path_ioctl(pgc->dpmgr_handle, NULL, DDP_STOP_VIDEO_MODE, NULL);

		DSI_dcs_read_lcm_reg_v2(DISP_MODULE_DSI0, NULL, 0x51, buffer, 1);
		pr_debug("[CABC check result 0x51 = 0x%x,0x%x]\n", buffer[0], buffer[1]);
		dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
		if (primary_display_is_video_mode()) {
			/* for video mode, we need to force trigger here */
			/* for cmd mode, just set DPREC_EVENT_CMDQ_SET_EVENT_ALLOW when trigger loop start */
			dpmgr_path_trigger(pgc->dpmgr_handle, NULL, CMDQ_DISABLE);
		}
	}
#endif
	_primary_path_unlock(__func__);
	_primary_path_cmd_unlock();
#ifdef DISP_SWITCH_DST_MODE
	_primary_path_switch_dst_unlock();
#endif
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagEnd, 0, 0);
	return ret;
}

int primary_display_set_cmd(int *lcm_cmd, unsigned int cmd_num)
{
	int ret = 0;

	DISPFUNC();


	_primary_path_cmd_lock();
	_primary_path_lock(__func__);
	if (pgc->state == DISP_SLEPT) {
		DISPCHECK("Slept State set cmd invald\n");
	} else {
		if (primary_display_cmdq_enabled()) {	/* cmdq */
			int ret = 0;
			cmdqRecHandle cmdq_handle_cmd = NULL;

			ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle_cmd);
			DISPCHECK("primary set cmd, handle=%p\n", cmdq_handle_cmd);
			if (ret != 0) {
				DISPCHECK("fail to create primary cmdq handle for cmd\n");
				return -1;
			}
			if (primary_display_is_video_mode()) {
				/* video mode */
				cmdqRecReset(cmdq_handle_cmd);
				disp_lcm_set_cmd(pgc->plcm, (void *)cmdq_handle_cmd, lcm_cmd,
						 cmd_num);
				_cmdq_flush_config_handle_mira(cmdq_handle_cmd, 1);
				DISPCHECK("[Display]_set_cmd_by_cmdq ret=%d\n", ret);

			} else {
				/* cmd mode */
				cmdqRecReset(cmdq_handle_cmd);
				_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_cmd);
				cmdqRecClearEventToken(cmdq_handle_cmd, CMDQ_SYNC_TOKEN_CABC_EOF);
				disp_lcm_set_cmd(pgc->plcm, (void *)cmdq_handle_cmd, lcm_cmd,
						 cmd_num);
				cmdqRecSetEventToken(cmdq_handle_cmd, CMDQ_SYNC_TOKEN_CABC_EOF);
				_cmdq_flush_config_handle_mira(cmdq_handle_cmd, 1);
				DISPCHECK("[Display]_set_cmd_by_cmdq ret=%d\n", ret);
			}
			cmdqRecDestroy(cmdq_handle_cmd);
			cmdq_handle_cmd = NULL;
		}
	}
	_primary_path_unlock(__func__);
	_primary_path_cmd_unlock();
	return ret;

}

#define LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL

/***********************/
/*****Legacy DISP API*****/
/***********************/
uint32_t DISP_GetScreenWidth(void)
{
	return primary_display_get_width();
}

uint32_t DISP_GetScreenHeight(void)
{
	return primary_display_get_height();
}

uint32_t DISP_GetActiveHeight(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return 0;
	}

	if (pgc->plcm->params)
		return pgc->plcm->params->physical_height;
	DISPERR("lcm_params is null!\n");
	return 0;
}


uint32_t DISP_GetActiveWidth(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return 0;
	}

	if (pgc->plcm->params)
		return pgc->plcm->params->physical_width;
	DISPERR("lcm_params is null!\n");
	return 0;
}

LCM_PARAMS *DISP_GetLcmPara(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return NULL;
	}

	if (pgc->plcm->params)
		return pgc->plcm->params;
	return NULL;
}

LCM_DRIVER *DISP_GetLcmDrv(void)
{

	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return NULL;
	}

	if (pgc->plcm->drv)
		return pgc->plcm->drv;
	return NULL;
}

int primary_display_capture_framebuffer_decouple(unsigned long pbuf, unsigned int format)
{
	unsigned int i = 0;
	int ret = 0;
	disp_ddp_path_config *pconfig = NULL;
	m4u_client_t *m4uClient = NULL;
	unsigned int mva = 0;
	unsigned long va = 0;
	unsigned int mapped_size = 0;
	unsigned int w_xres = primary_display_get_width();
	unsigned int h_yres = primary_display_get_height();
	unsigned int pixel_byte = primary_display_get_bpp() / 8;
	/* bpp is either 32 or 16, can not be other value */
	unsigned int pitch = 0;
	int buffer_size = h_yres * w_xres * pixel_byte;

	m4uClient = m4u_create_client();
	if (m4uClient == NULL) {
		DISPCHECK("primary capture:Fail to alloc  m4uClient=0x%p\n", m4uClient);
		ret = -1;
		goto out;
	}

	_primary_path_lock(__func__);

/* mva = pgc->dc_buf[pgc->dc_buf_id]; */
	pconfig = dpmgr_path_get_last_config(pgc->dpmgr_handle);

	w_xres = pconfig->rdma_config.width;
	h_yres = pconfig->rdma_config.height;
	pitch = pconfig->rdma_config.pitch;
	mva = pconfig->rdma_config.address;
	buffer_size = h_yres * pitch;
	ASSERT((pitch / 4) >= w_xres);
/* dpmgr_get_input_address(pgc->dpmgr_handle,&mva); */
	_primary_path_unlock(__func__);
	m4u_mva_map_kernel(mva, buffer_size, &va, &mapped_size);
	if (!va) {
		DISPERR("map mva 0x%08x failed\n", mva);
		goto out;
	}

	DISPMSG("map 0x%08x with %d bytes to 0x%08lx with %d bytes\n", mva, buffer_size, va,
		mapped_size);

	ret =
	    m4u_cache_sync(m4uClient, M4U_PORT_DISP_WDMA0, va, buffer_size, mva,
			   M4U_CACHE_FLUSH_ALL);
#if 1
	{
		unsigned int j = 0;
		unsigned char tem_va[4];
		unsigned long src_va = va;

		for (i = 0; i < h_yres; i++) {
			for (j = 0; j < w_xres; j++) {
				memcpy((void *)tem_va, (void *)src_va, 4);
				*(unsigned long *)(pbuf + (i * w_xres + j) * 4) =
				    0xFF000000 | (tem_va[0] << 16) | (tem_va[1] << 8) | (tem_va[2]);
				src_va += 4;
			}
			src_va += (pitch - w_xres * 4);
		}
	}
#else
	memcpy(pbuf, va, mapped_size);
#endif
out:
	if (mapped_size)
		m4u_mva_unmap_kernel(mva, mapped_size, va);

	if (m4uClient != NULL)
		m4u_destroy_client(m4uClient);

	DISPMSG("primary capture: end\n");

	return ret;
}

int primary_display_capture_framebuffer_ovl(unsigned long pbuf, unsigned int format)
{
	int ret = 0;
	cmdqRecHandle cmdq_handle = NULL;
	cmdqRecHandle cmdq_wait_handle = NULL;
	disp_ddp_path_config *pconfig = NULL;
	m4u_client_t *m4uClient = NULL;
	unsigned int mva = 0;
	unsigned int w_xres = primary_display_get_width();
	unsigned int h_yres = primary_display_get_height();
	unsigned int pixel_byte = primary_display_get_bpp() / 8;
	/* bpp is either 32 or 16, can not be other value */
	int buffer_size = h_yres * w_xres * pixel_byte;

	DISPMSG("primary capture: begin\n");

	disp_sw_mutex_lock(&(pgc->capture_lock));

	if (primary_display_is_sleepd() || !primary_display_cmdq_enabled()) {
		memset((void *)pbuf, 0, buffer_size);
		DISPMSG("primary capture:fill black\n");
		goto out;
	}

	if (_is_decouple_mode(pgc->session_mode) || _is_mirror_mode(pgc->session_mode)) {
		/* primary_display_capture_framebuffer_decouple(pbuf, format); */
		memset((void *)pbuf, 0, buffer_size);
		DISPMSG("primary capture: fill black for decouple & mirror mode End\n");
		goto out;
	}

	m4uClient = m4u_create_client();
	if (m4uClient == NULL) {
		DISPCHECK("primary capture:Fail to alloc  m4uClient=0x%p\n", m4uClient);
		ret = -1;
		goto out;
	}

	ret =
	    m4u_alloc_mva(m4uClient, M4U_PORT_DISP_WDMA0, pbuf, NULL, buffer_size,
			  M4U_PROT_READ | M4U_PROT_WRITE, 0, &mva);
	if (ret != 0) {
		DISPCHECK("primary capture:Fail to allocate mva\n");
		ret = -1;
		goto out;
	}

	ret =
	    m4u_cache_sync(m4uClient, M4U_PORT_DISP_WDMA0, pbuf, buffer_size, mva,
			   M4U_CACHE_FLUSH_BY_RANGE);
	if (ret != 0) {
		DISPCHECK("primary capture:Fail to cach sync\n");
		ret = -1;
		goto out;
	}

	if (primary_display_cmdq_enabled()) {
		/*create config thread */
		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);
		if (ret != 0) {
			DISPCHECK
			    ("primary capture:Fail to create primary cmdq handle for capture\n");
			ret = -1;
			goto out;
		}
		cmdqRecReset(cmdq_handle);

		/*create wait thread */
		ret = cmdqRecCreate(CMDQ_SCENARIO_DISP_SCREEN_CAPTURE, &cmdq_wait_handle);
		if (ret != 0) {
			DISPCHECK
			    ("primary capture:Fail to create primary cmdq wait handle for capture\n");
			ret = -1;
			goto out;
		}
		cmdqRecReset(cmdq_wait_handle);

		/*configure  config thread */
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);
		dpmgr_path_memout_clock(pgc->dpmgr_handle, 1);

		_primary_path_lock(__func__);
		pconfig = dpmgr_path_get_last_config(pgc->dpmgr_handle);
		pconfig->wdma_dirty = 1;
		pconfig->wdma_config.dstAddress = mva;
		pconfig->wdma_config.srcHeight = h_yres;
		pconfig->wdma_config.srcWidth = w_xres;
		pconfig->wdma_config.clipX = 0;
		pconfig->wdma_config.clipY = 0;
		pconfig->wdma_config.clipHeight = h_yres;
		pconfig->wdma_config.clipWidth = w_xres;
		pconfig->wdma_config.outputFormat = format;
		pconfig->wdma_config.useSpecifiedAlpha = 1;
		pconfig->wdma_config.alpha = 0xFF;
		pconfig->wdma_config.dstPitch = w_xres * DP_COLOR_BITS_PER_PIXEL(format) / 8;
		dpmgr_path_add_memout(pgc->dpmgr_handle, ENGINE_OVL0, cmdq_handle);

#if 0
		if (ovl_get_status() == DDP_OVL1_STATUS_IDLE
		    || ovl_get_status() == DDP_OVL1_STATUS_PRIMARY) {
			DISPMSG("dpmgr_path_enable_cascade called!\n");
			dpmgr_path_enable_cascade(pgc->dpmgr_handle, cmdq_handle);

			if (ovl_get_status() != DDP_OVL1_STATUS_PRIMARY)
				pconfig->ovl_dirty = 1;
		}
#endif

		ret = dpmgr_path_config(pgc->dpmgr_handle, pconfig, cmdq_handle);
		pconfig->wdma_dirty = 0;
		_primary_path_unlock(__func__);
		_cmdq_set_config_handle_dirty_mira(cmdq_handle);
		_cmdq_flush_config_handle_mira(cmdq_handle, 0);
		DISPMSG("primary capture:Flush add memout mva(0x%x)\n", mva);

		/* primary_display_diagnose(); */
		/*wait wdma0 sof */
		cmdqRecWait(cmdq_wait_handle, CMDQ_EVENT_DISP_WDMA0_SOF);
		cmdqRecFlush(cmdq_wait_handle);
		DISPMSG("primary capture:Flush wait wdma sof\n");

		cmdqRecReset(cmdq_handle);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);
		_primary_path_lock(__func__);
		dpmgr_path_remove_memout(pgc->dpmgr_handle, cmdq_handle);
		cmdqRecClearEventToken(cmdq_handle, CMDQ_EVENT_DISP_WDMA0_SOF);
		_primary_path_unlock(__func__);
		_cmdq_set_config_handle_dirty_mira(cmdq_handle);
		/* flush remove memory to cmdq */
		_cmdq_flush_config_handle_mira(cmdq_handle, 1);
		DISPMSG("primary capture: Flush remove memout\n");

		dpmgr_path_memout_clock(pgc->dpmgr_handle, 0);
	}

out:

	cmdqRecDestroy(cmdq_handle);
	cmdqRecDestroy(cmdq_wait_handle);
	if (mva > 0)
		m4u_dealloc_mva(m4uClient, M4U_PORT_DISP_WDMA0, mva);

	if (m4uClient != 0)
		m4u_destroy_client(m4uClient);

	disp_sw_mutex_unlock(&(pgc->capture_lock));
	DISPMSG("primary capture: end\n");

	return ret;
}

int primary_display_capture_framebuffer(unsigned long pbuf)
{
#if 1
	unsigned int i;
	unsigned long ttt;
	unsigned int fb_layer_id = primary_display_get_option("FB_LAYER");
	unsigned int w_xres = primary_display_get_width();
	unsigned int h_yres = primary_display_get_height();
	unsigned int pixel_bpp = primary_display_get_bpp() / 8;	/* bpp is either 32 or 16, can not be other value */
	unsigned int w_fb = ALIGN_TO(w_xres, MTK_FB_ALIGNMENT);
	unsigned int fbsize = w_fb * h_yres * pixel_bpp;	/* frame buffer size */
	unsigned long fbaddress =
	    dpmgr_path_get_last_config(pgc->dpmgr_handle)->ovl_config[fb_layer_id].addr;

	unsigned long fbv = 0;

	DISPMSG("w_res=%d, h_yres=%d, pixel_bpp=%d, w_fb=%d, fbsize=%d, fbaddress=0x%lx\n", w_xres,
		h_yres, pixel_bpp, w_fb, fbsize, fbaddress);
	fbv = (unsigned long)ioremap(fbaddress, fbsize);
	DISPMSG
	    ("w_xres = %d, h_yres = %d, w_fb = %d, pixel_bpp = %d, fbsize = %d, fbaddress = 0x%lx\n",
	     w_xres, h_yres, w_fb, pixel_bpp, fbsize, fbaddress);
	if (!fbv) {
		DISPMSG
		    ("[FB Driver], Unable to allocate memory for frame buffer: address=0x%lx, size=0x%08x\n",
		     fbaddress, fbsize);
		return -1;
	}

	ttt = get_current_time_us();
	for (i = 0; i < h_yres; i++) {

		memcpy((void *)(pbuf + i * w_xres * pixel_bpp),
		       (void *)(fbv + i * w_fb * pixel_bpp), w_xres * pixel_bpp);
	}
	DISPMSG("capture framebuffer cost %ld us\n", get_current_time_us() - ttt);
	iounmap((void *)fbv);
#endif
	return -1;
}

uint32_t DISP_GetPanelBPP(void)
{
#if 0
	PANEL_COLOR_FORMAT fmt;

	disp_drv_init_context();

	if (disp_if_drv->get_panel_color_format == NULL)
		return DISP_STATUS_NOT_IMPLEMENTED;


	fmt = disp_if_drv->get_panel_color_format();
	switch (fmt) {
	case PANEL_COLOR_FORMAT_RGB332:
		return 8;
	case PANEL_COLOR_FORMAT_RGB444:
		return 12;
	case PANEL_COLOR_FORMAT_RGB565:
		return 16;
	case PANEL_COLOR_FORMAT_RGB666:
		return 18;
	case PANEL_COLOR_FORMAT_RGB888:
		return 24;
	default:
		return 0;
	}
#endif
	return 0;
}

static uint32_t disp_fb_bpp = 32;
static uint32_t disp_fb_pages = 3;

uint32_t DISP_GetScreenBpp(void)
{
	return disp_fb_bpp;
}

uint32_t DISP_GetPages(void)
{
	return disp_fb_pages;
}

uint32_t DISP_GetFBRamSize(void)
{
	return ALIGN_TO(DISP_GetScreenWidth(), MTK_FB_ALIGNMENT) *
	    ALIGN_TO(DISP_GetScreenHeight(), MTK_FB_ALIGNMENT) *
	    ((DISP_GetScreenBpp() + 7) >> 3) * DISP_GetPages();
}


uint32_t DISP_GetVRamSize(void)
{
#if 0
	/* Use a local static variable to cache the calculated vram size */
	/*  */
	static uint32_t vramSize;

	if (0 == vramSize) {
		disp_drv_init_context();

		/* /get framebuffer size */
		vramSize = DISP_GetFBRamSize();

		/* /get DXI working buffer size */
		vramSize += disp_if_drv->get_working_buffer_size();

		/* get assertion layer buffer size */
		vramSize += DAL_GetLayerSize();

		/* Align vramSize to 1MB */
		/*  */
		vramSize = ALIGN_TO_POW_OF_2(vramSize, 0x100000);

		DISP_LOG("DISP_GetVRamSize: %u bytes\n", vramSize);
	}

	return vramSize;
#endif
	return 0;
}



uint32_t DISP_GetVRamSizeBoot(char *cmdline)
{
#ifdef CONFIG_OF
	_parse_tag_videolfb();
	if (vramsize == 0)
		vramsize = 0x3000000;
	DISPCHECK("[DT]display vram size = 0x%08x|%d\n", vramsize, vramsize);
	return vramsize;
#else
	int ret = 0;
	char *p = NULL;
	uint32_t vramSize = 0;

	DISPMSG("%s, cmdline=%s\n", __func__, cmdline);
	p = strstr(cmdline, "vram=");
	if (p == NULL) {
		vramSize = 0x3000000;
		DISPERR("[FB driver]can not get vram size from lk\n");
	} else {
		p += 5;
		ret = kstrtol(p, 10, &vramSize);
		if (ret)
			pr_err("DISP/%s: errno %d\n", __func__, ret);

		if (0 == vramSize)
			vramSize = 0x3000000;
	}

	DISPCHECK("display vram size = 0x%08x|%d\n", vramSize, vramSize);
	return vramSize;
#endif
}


struct sg_table table;

int disp_hal_allocate_framebuffer(phys_addr_t pa_start, phys_addr_t pa_end, unsigned long *va,
				  unsigned long *mva)
{
	int ret = 0;
	*va = (unsigned long)ioremap_nocache(pa_start, pa_end - pa_start + 1);
	pr_debug("disphal_allocate_fb, pa=%p, va=0x%lx\n", &pa_start, *va);

/* if (_get_init_setting("M4U")) */
	/* xuecheng, m4u not enabled now */
#ifndef MTKFB_NO_M4U
	if (1) {
		m4u_client_t *client;

		struct sg_table *sg_table = &table;

		sg_alloc_table(sg_table, 1, GFP_KERNEL);

		sg_dma_address(sg_table->sgl) = pa_start;
		sg_dma_len(sg_table->sgl) = (pa_end - pa_start + 1);
		client = m4u_create_client();
		if (IS_ERR_OR_NULL(client))
			DISPMSG("create client fail!\n");

		*mva = pa_start & 0xffffffffULL;
		ret = m4u_alloc_mva(client, M4U_PORT_DISP_OVL0, 0, sg_table,
				    (pa_end - pa_start + 1), M4U_PROT_READ | M4U_PROT_WRITE,
				    M4U_FLAGS_FIX_MVA, (unsigned int *)mva);
		/* m4u_alloc_mva(M4U_PORT_DISP_OVL0, pa_start, (pa_end - pa_start + 1), 0, 0, mva); */
		if (ret)
			DISPMSG("m4u_alloc_mva returns fail: %d\n", ret);

		pr_debug("[DISPHAL] FB MVA is 0x%lx PA is %pa\n", *mva, &pa_start);

	} else
#endif
	{
		*mva = pa_start & 0xffffffffULL;
	}

	return 0;
}

int primary_display_remap_irq_event_map(void)
{
	return 0;
}

unsigned int primary_display_get_option(const char *option)
{
	if (!strcmp(option, "FB_LAYER"))
		return 0;
	if (!strcmp(option, "ASSERT_LAYER")) {
#ifdef OVL_CASCADE_SUPPORT
		if (ovl_get_status() == DDP_OVL1_STATUS_PRIMARY
		    || ovl_get_status() == DDP_OVL1_STATUS_IDLE)
			return (OVL_LAYER_NUM - 1);
		else
			return (OVL_LAYER_NUM - 4 - 1);
#else
		return (OVL_LAYER_NUM - 1);
#endif
	}
	if (!strcmp(option, "M4U_ENABLE"))
		return 1;
	ASSERT(0);
	return 0;
}

int primary_display_get_debug_info(char *buf)
{
	/* resolution */
	/* cmd/video mode */
	/* display path */
	/* dsi data rate/lane number/state */
	/* primary path trigger count */
	/* frame done count */
	/* suspend/resume count */
	/* current fps 10s/5s/1s */
	/* error count and message */
	/* current state of each module on the path */
	return 0;
}

#include "ddp_reg.h"

#define IS_READY(x)	((x)?"READY\t":"Not READY")
#define IS_VALID(x)	((x)?"VALID\t":"Not VALID")
#define READY_BIT0(x) ((DISP_REG_GET(DISPSYS_CONFIG_BASE + 0x8a0) & (1 << x)))
#define VALID_BIT0(x) ((DISP_REG_GET(DISPSYS_CONFIG_BASE + 0x8a4) & (1 << x)))
#define READY_BIT1(x) ((DISP_REG_GET(DISPSYS_CONFIG_BASE + 0x8a8) & (1 << x)))
#define VALID_BIT1(x) ((DISP_REG_GET(DISPSYS_CONFIG_BASE + 0x8ac) & (1 << x)))

int primary_display_check_path(char *stringbuf, int buf_len)
{
	int len = 0;

	return 0;		/* status will print in config dump. */

	DISPMSG("primary_display_check_path() check signal status:\n");
	if (stringbuf) {
		len +=
		    scnprintf(stringbuf + len, buf_len - len,
			      "|--------------------------------------------------------------------------------------|\n");
		len +=
		    scnprintf(stringbuf + len, buf_len - len,
			      "READY0=0x%08x, READY1=0x%08x, VALID0=0x%08x, VALID1=0x%08x\n",
			      DISP_REG_GET(DISPSYS_CONFIG_BASE + 0x8a0),
			      DISP_REG_GET(DISPSYS_CONFIG_BASE + 0x8a4),
			      DISP_REG_GET(DISPSYS_CONFIG_BASE + 0x8a8),
			      DISP_REG_GET(DISPSYS_CONFIG_BASE + 0x8ac));
		len +=
		    scnprintf(stringbuf + len, buf_len - len, "OVL0\t\t\t%s\t%s\n",
			      IS_READY(READY_BIT0(DDP_SIGNAL_OVL0__OVL0_MOUT)),
			      IS_VALID(READY_BIT0(DDP_SIGNAL_OVL0__OVL0_MOUT)));
		len +=
		    scnprintf(stringbuf + len, buf_len - len, "OVL0_MOUT:\t\t%s\t%s\n",
			      IS_READY(READY_BIT1(DDP_SIGNAL_OVL0_MOUT0__COLOR_SIN1)),
			      IS_VALID(READY_BIT1(DDP_SIGNAL_OVL0_MOUT0__COLOR_SIN1)));
		len +=
		    scnprintf(stringbuf + len, buf_len - len, "COLOR0_SEL:\t\t%s\t%s\n",
			      IS_READY(READY_BIT0(DDP_SIGNAL_COLOR_SEL__COLOR)),
			      IS_VALID(READY_BIT0(DDP_SIGNAL_COLOR_SEL__COLOR)));
		len +=
		    scnprintf(stringbuf + len, buf_len - len, "COLOR0:\t\t\t%s\t%s\n",
			      IS_READY(READY_BIT0(DDP_SIGNAL_COLOR__CCORR)),
			      IS_VALID(READY_BIT0(DDP_SIGNAL_COLOR__CCORR)));
		len +=
		    scnprintf(stringbuf + len, buf_len - len, "CCORR:\t\t%s\t%s\n",
			      IS_READY(READY_BIT0(DDP_SIGNAL_CCORR__AAL)),
			      IS_VALID(READY_BIT0(DDP_SIGNAL_CCORR__AAL)));
		len +=
		    scnprintf(stringbuf + len, buf_len - len, "AAL0:\t\t\t%s\t%s\n",
			      IS_READY(READY_BIT0(DDP_SIGNAL_AAL__GAMMA)),
			      IS_VALID(READY_BIT0(DDP_SIGNAL_AAL__GAMMA)));
		len +=
		    scnprintf(stringbuf + len, buf_len - len, "GAMMA:\t\t\t%s\t%s\n",
			      IS_READY(READY_BIT1(DDP_SIGNAL_GAMMA__DITHER)),
			      IS_VALID(READY_BIT1(DDP_SIGNAL_GAMMA__DITHER)));
		len +=
		    scnprintf(stringbuf + len, buf_len - len, "DITHER:\t\t\t%s\t%s\n",
			      IS_READY(READY_BIT1(DDP_SIGNAL_DITHER__DITHER_MOUT)),
			      IS_VALID(READY_BIT1(DDP_SIGNAL_DITHER__DITHER_MOUT)));
		len +=
		    scnprintf(stringbuf + len, buf_len - len, "DITHER_MOUT:\t\t%s\t%s\n",
			      IS_READY(READY_BIT1(DDP_SIGNAL_DITHER_MOUT0__RDMA0)),
			      IS_VALID(READY_BIT1(DDP_SIGNAL_DITHER_MOUT0__RDMA0)));
		len +=
		    scnprintf(stringbuf + len, buf_len - len, "RDMA0:\t\t\t%s\t%s\n",
			      IS_READY(READY_BIT1(DDP_SIGNAL_RDMA0__RDMA0_SOUT)),
			      IS_VALID(READY_BIT1(DDP_SIGNAL_RDMA0__RDMA0_SOUT)));
		len +=
		    scnprintf(stringbuf + len, buf_len - len, "RDMA0_SOUT:\t\t%s\t%s\n",
			      IS_READY(READY_BIT1(DDP_SIGNAL_RDMA0_SOUT0__UFOE_SIN0)),
			      IS_VALID(READY_BIT1(DDP_SIGNAL_RDMA0_SOUT0__UFOE_SIN0)));
		len +=
		    scnprintf(stringbuf + len, buf_len - len, "UFOE_SEL:\t\t%s\t%s\n",
			      IS_READY(READY_BIT0(DDP_SIGNAL_UFOE_SEL__UFOE)),
			      IS_VALID(READY_BIT0(DDP_SIGNAL_UFOE_SEL__UFOE)));
		len +=
		    scnprintf(stringbuf + len, buf_len - len, "UFOE:\t\t\t%s\t%s\n",
			      IS_READY(READY_BIT0(DDP_SIGNAL_UFOE__UFOE_MOUT)),
			      IS_VALID(READY_BIT0(DDP_SIGNAL_UFOE__UFOE_MOUT)));
		len +=
		    scnprintf(stringbuf + len, buf_len - len, "UFOE_MOUT:\t\t%s\t%s\n",
			      IS_READY(READY_BIT0(DDP_SIGNAL_UFOE_MOUT0__DSI0_SIN0)),
			      IS_VALID(READY_BIT0(DDP_SIGNAL_UFOE_MOUT0__DSI0_SIN0)));
		len +=
		    scnprintf(stringbuf + len, buf_len - len, "DSI0_SEL:\t\t%s\t%s\n",
			      IS_READY(READY_BIT1(DDP_SIGNAL_DIS0_SEL__DSI0)),
			      IS_VALID(READY_BIT1(DDP_SIGNAL_DIS0_SEL__DSI0)));
	} else {
		DISPMSG
		    ("|--------------------------------------------------------------------------------------|\n");
		DISPMSG("READY0=0x%08x, READY1=0x%08x, VALID0=0x%08x, VALID1=0x%08x\n",
			DISP_REG_GET(DISPSYS_CONFIG_BASE + 0x8a0),
			DISP_REG_GET(DISPSYS_CONFIG_BASE + 0x8a4),
			DISP_REG_GET(DISPSYS_CONFIG_BASE + 0x8a8),
			DISP_REG_GET(DISPSYS_CONFIG_BASE + 0x8ac));
		DISPMSG("OVL0\t\t\t%s\t%s\n", IS_READY(READY_BIT0(DDP_SIGNAL_OVL0__OVL0_MOUT)),
			IS_VALID(READY_BIT0(DDP_SIGNAL_OVL0__OVL0_MOUT)));
		DISPMSG("OVL0_MOUT:\t\t%s\t%s\n",
			IS_READY(READY_BIT1(DDP_SIGNAL_OVL0_MOUT0__COLOR_SIN1)),
			IS_VALID(READY_BIT1(DDP_SIGNAL_OVL0_MOUT0__COLOR_SIN1)));
		DISPMSG("COLOR0_SEL:\t\t%s\t%s\n",
			IS_READY(READY_BIT0(DDP_SIGNAL_COLOR_SEL__COLOR)),
			IS_VALID(READY_BIT0(DDP_SIGNAL_COLOR_SEL__COLOR)));
		DISPMSG("COLOR0:\t\t\t%s\t%s\n", IS_READY(READY_BIT0(DDP_SIGNAL_COLOR__CCORR)),
			IS_VALID(READY_BIT0(DDP_SIGNAL_COLOR__CCORR)));
		DISPMSG("CCORR:\t\t%s\t%s\n", IS_READY(READY_BIT0(DDP_SIGNAL_CCORR__AAL)),
			IS_VALID(READY_BIT0(DDP_SIGNAL_CCORR__AAL)));
		DISPMSG("AAL0:\t\t\t%s\t%s\n", IS_READY(READY_BIT0(DDP_SIGNAL_AAL__GAMMA)),
			IS_VALID(READY_BIT0(DDP_SIGNAL_AAL__GAMMA)));
		DISPMSG("GAMMA:\t\t\t%s\t%s\n", IS_READY(READY_BIT1(DDP_SIGNAL_GAMMA__DITHER)),
			IS_VALID(READY_BIT1(DDP_SIGNAL_GAMMA__DITHER)));
		DISPMSG("DITHER:\t\t\t%s\t%s\n",
			IS_READY(READY_BIT1(DDP_SIGNAL_DITHER__DITHER_MOUT)),
			IS_VALID(READY_BIT1(DDP_SIGNAL_DITHER__DITHER_MOUT)));
		DISPMSG("DITHER_MOUT:\t\t%s\t%s\n",
			IS_READY(READY_BIT1(DDP_SIGNAL_DITHER_MOUT0__RDMA0)),
			IS_VALID(READY_BIT1(DDP_SIGNAL_DITHER_MOUT0__RDMA0)));
		DISPMSG("RDMA0:\t\t\t%s\t%s\n", IS_READY(READY_BIT1(DDP_SIGNAL_RDMA0__RDMA0_SOUT)),
			IS_VALID(READY_BIT1(DDP_SIGNAL_RDMA0__RDMA0_SOUT)));
		DISPMSG("RDMA0_SOUT:\t\t%s\t%s\n",
			IS_READY(READY_BIT1(DDP_SIGNAL_RDMA0_SOUT0__UFOE_SIN0)),
			IS_VALID(READY_BIT1(DDP_SIGNAL_RDMA0_SOUT0__UFOE_SIN0)));
		DISPMSG("UFOE_SEL:\t\t%s\t%s\n", IS_READY(READY_BIT0(DDP_SIGNAL_UFOE_SEL__UFOE)),
			IS_VALID(READY_BIT0(DDP_SIGNAL_UFOE_SEL__UFOE)));
		DISPMSG("UFOE:\t\t\t%s\t%s\n", IS_READY(READY_BIT0(DDP_SIGNAL_UFOE__UFOE_MOUT)),
			IS_VALID(READY_BIT0(DDP_SIGNAL_UFOE__UFOE_MOUT)));
		DISPMSG("UFOE_MOUT:\t\t%s\t%s\n",
			IS_READY(READY_BIT0(DDP_SIGNAL_UFOE_MOUT0__DSI0_SIN0)),
			IS_VALID(READY_BIT0(DDP_SIGNAL_UFOE_MOUT0__DSI0_SIN0)));
		DISPMSG("DSI0_SEL:\t\t%s\t%s\n", IS_READY(READY_BIT1(DDP_SIGNAL_DIS0_SEL__DSI0)),
			IS_VALID(READY_BIT1(DDP_SIGNAL_DIS0_SEL__DSI0)));
	}

	return len;
}

int primary_display_lcm_ATA(void)
{
	DISP_STATUS ret = DISP_STATUS_OK;

	DISPFUNC();
	_primary_path_lock(__func__);
	if (pgc->state == 0) {
		DISPCHECK("ATA_LCM, primary display path is already slept, skip\n");
		goto done;
	}

	DISPCHECK("[ATA_LCM]primary display path stop[begin]\n");
	if (primary_display_is_video_mode())
		dpmgr_path_ioctl(pgc->dpmgr_handle, NULL, DDP_STOP_VIDEO_MODE, NULL);

	DISPCHECK("[ATA_LCM]primary display path stop[end]\n");
	ret = disp_lcm_ATA(pgc->plcm);
	dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
	if (primary_display_is_video_mode()) {
		/* for video mode, we need to force trigger here */
		/* for cmd mode, just set DPREC_EVENT_CMDQ_SET_EVENT_ALLOW when trigger loop start */
		dpmgr_path_trigger(pgc->dpmgr_handle, NULL, CMDQ_DISABLE);
	}
done:
	_primary_path_unlock(__func__);
	return ret;
}

int fbconfig_get_esd_check_test(uint32_t dsi_id, uint32_t cmd, uint8_t *buffer, uint32_t num)
{
	int ret = 0;

	_primary_path_lock(__func__);

	if (pgc->state == DISP_SLEPT) {
		DISPCHECK("[ESD]primary display path is slept?? -- skip esd check\n");
		_primary_path_unlock(__func__);
		goto done;
	}
	/* / 1: stop path */
	_cmdq_stop_trigger_loop();
	if (dpmgr_path_is_busy(pgc->dpmgr_handle))
		DISPCHECK("[ESD]wait frame done ret:%d\n", ret);

	dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);
	DISPCHECK("[ESD]stop dpmgr path[end]\n");
	if (dpmgr_path_is_busy(pgc->dpmgr_handle))
		DISPCHECK("[ESD]wait frame done ret:%d\n", ret);

	dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);

	ret = fbconfig_get_esd_check(dsi_id, cmd, buffer, num);
	dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
	DISPCHECK("[ESD]start dpmgr path[end]\n");
	if (primary_display_is_video_mode()) {
		/* for video mode, we need to force trigger here */
		/* for cmd mode, just set DPREC_EVENT_CMDQ_SET_EVENT_ALLOW when trigger loop start */
		dpmgr_path_trigger(pgc->dpmgr_handle, NULL, CMDQ_DISABLE);
	}
	_cmdq_start_trigger_loop();
	DISPCHECK("[ESD]start cmdq trigger loop[end]\n");
	_primary_path_unlock(__func__);

done:
	return ret;
}

int Panel_Master_dsi_config_entry(const char *name, void *config_value)
{
	int ret = 0;
	int force_trigger_path = 0;
	LCM_PARAMS *lcm_param = NULL;
	uint32_t *config_dsi = (uint32_t *) config_value;
	LCM_DRIVER *pLcm_drv = DISP_GetLcmDrv();
	int esd_check_backup = atomic_read(&esd_check_task_wakeup);

	DISPFUNC();

	if (!strcmp(name, "DRIVER_IC_RESET") || !strcmp(name, "PM_DDIC_CONFIG")) {
		primary_display_esd_check_enable(0);
		msleep(2500);
	}
	_primary_path_lock(__func__);

	lcm_param = disp_lcm_get_params(pgc->plcm);
	if (pgc->state == DISP_SLEPT) {
		DISPERR("[Pmaster]Panel_Master: primary display path is slept??\n");
		goto done;
	}
	/* / Esd Check : Read from lcm */
	/* / the following code is to */
	/* / 0: lock path */
	/* / 1: stop path */
	/* / 2: do esd check (!!!) */
	/* / 3: start path */
	/* / 4: unlock path */
	/* / 1: stop path */
	_cmdq_stop_trigger_loop();

	if (dpmgr_path_is_busy(pgc->dpmgr_handle))
		DISPCHECK("[ESD]wait frame done ret:%d\n", ret);

	dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);
	DISPCHECK("[ESD]stop dpmgr path[end]\n");

	if (dpmgr_path_is_busy(pgc->dpmgr_handle))
		DISPCHECK("[ESD]wait frame done ret:%d\n", ret);

	dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
	if ((!strcmp(name, "PM_CLK")) || (!strcmp(name, "PM_SSC")))
		Panel_Master_primary_display_config_dsi(name, *config_dsi);
	else if (!strcmp(name, "PM_DDIC_CONFIG")) {
		Panel_Master_DDIC_config();
		force_trigger_path = 1;
	} else if (!strcmp(name, "DRIVER_IC_RESET")) {
		if (pLcm_drv && pLcm_drv->init_power)
			pLcm_drv->init_power();
		if (pLcm_drv)
			pLcm_drv->init();
		else
			ret = -1;
		force_trigger_path = 1;
	}
	dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
	if (primary_display_is_video_mode()) {
		/* for video mode, we need to force trigger here */
		/* for cmd mode, just set DPREC_EVENT_CMDQ_SET_EVENT_ALLOW when trigger loop start */
		dpmgr_path_trigger(pgc->dpmgr_handle, NULL, CMDQ_DISABLE);
		force_trigger_path = 0;
	}
	_cmdq_start_trigger_loop();
	DISPCHECK("[Pmaster]start cmdq trigger loop\n");
done:
	_primary_path_unlock(__func__);

	if (force_trigger_path)	{
		/* command mode only */
		primary_display_trigger(0, NULL, 0);
		DISPCHECK("[Pmaster]force trigger display path\r\n");
	}
	atomic_set(&esd_check_task_wakeup, esd_check_backup);

	return ret;
}

/*
mode: 0, switch to cmd mode; 1, switch to vdo mode
*/
int primary_display_switch_dst_mode(int mode)
{
	DISP_STATUS ret = DISP_STATUS_ERROR;
#ifdef DISP_SWITCH_DST_MODE
	void *lcm_cmd = NULL;

	DISPFUNC();
	_primary_path_switch_dst_lock();
	disp_sw_mutex_lock(&(pgc->capture_lock));
	if (pgc->plcm->params->type != LCM_TYPE_DSI) {
		pr_debug("[primary_display_switch_dst_mode] Error, only support DSI IF\n");
		goto done;
	}
	if (pgc->state == DISP_SLEPT) {
		DISPCHECK
		    ("[primary_display_switch_dst_mode], primary display path is already slept, skip\n");
		goto done;
	}

	if (mode == primary_display_cur_dst_mode) {
		DISPCHECK
		    ("[primary_display_switch_dst_mode]not need switch,cur_mode:%d, switch_mode:%d\n",
		     primary_display_cur_dst_mode, mode);
		goto done;
	}

	lcm_cmd = disp_lcm_switch_mode(pgc->plcm, mode);
	if (lcm_cmd == NULL) {
		DISPCHECK("[primary_display_switch_dst_mode]get lcm cmd fail\n",
			  primary_display_cur_dst_mode, mode);
		goto done;
	} else {
		int temp_mode = 0;

		if (0 !=
		    dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_config,
				     DDP_SWITCH_LCM_MODE, lcm_cmd)) {
			pr_debug("switch lcm mode fail, return directly\n");
			goto done;
		}
		_primary_path_lock(__func__);
		temp_mode = (int)(pgc->plcm->params->dsi.mode);
		pgc->plcm->params->dsi.mode = pgc->plcm->params->dsi.switch_mode;
		pgc->plcm->params->dsi.switch_mode = temp_mode;
		dpmgr_path_set_video_mode(pgc->dpmgr_handle, primary_display_is_video_mode());
		if (0 !=
		    dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_config,
				     DDP_SWITCH_DSI_MODE, lcm_cmd)) {
			pr_debug("switch dsi mode fail, return directly\n");
			_primary_path_unlock(__func__);
			goto done;
		}
	}
	primary_display_sodi_rule_init();
	_cmdq_stop_trigger_loop();
	_cmdq_build_trigger_loop();
	_cmdq_start_trigger_loop();
	_cmdq_reset_config_handle();	/* must do this */
	_cmdq_insert_wait_frame_done_token();

	primary_display_cur_dst_mode = mode;

	if (primary_display_is_video_mode()) {
		dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
				       DDP_IRQ_RDMA0_DONE);
	} else {
		dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
				       DDP_IRQ_DSI0_EXT_TE);
	}
	_primary_path_unlock(__func__);
	ret = DISP_STATUS_OK;
done:
/* dprec_handle_option(0x0); */
	disp_sw_mutex_unlock(&(pgc->capture_lock));
	_primary_path_switch_dst_unlock();
#else
	pr_debug("[ERROR: primary_display_switch_dst_mode]this function not enable in disp driver\n");
#endif
	return ret;
}

#ifdef CONFIG_MTK_SEGMENT_TEST
LCM_PARAMS *lcm_param2 = NULL;
disp_ddp_path_config data_config2;

int primary_display_te_test(void)
{
	int ret = 0;

	if (primary_display_is_video_mode()) {
		pr_debug("Video Mode No TE\n");
		return ret;
	}

	ret = DSI_test_force_te(DISP_MODULE_DSI0);

	if (ret >= 0)
		pr_debug("[display_test_result]==>Force On TE Open!\n");
	else
		pr_debug("[display_test_result]==>Force On TE Closed!\n");

	return ret;

}

int primary_display_rdma_res_test(void)
{
	int ret = 0;

	ret = rdma_resolution_test(DISP_MODULE_RDMA1, 720, 1280);
	if (ret < 0)
		pr_debug("[display_test_result]==>RDMA RESOLUTION SETTING !\n");
	else
		pr_debug("[display_test_result]==>RDMA RESOLUTION SETTING ONCE!\n");
	return ret;
}

int primary_display_fps_test(void)
{
	int ret = 0;
	unsigned int w_backup = 0;
	unsigned int h_backup = 0;
	LCM_DSI_MODE_CON dsi_mode_backup = primary_display_is_video_mode();

	memset((void *)&data_config2, 0, sizeof(data_config2));
	lcm_param2 = NULL;
	memcpy((void *)&data_config2, (void *)dpmgr_path_get_last_config(pgc->dpmgr_handle),
	       sizeof(disp_ddp_path_config));
	w_backup = data_config2.dst_w;
	h_backup = data_config2.dst_h;
	DISPCHECK("[display_test]w_backup %d h_backup %d dsi_mode_backup %d\n", w_backup, h_backup,
		  dsi_mode_backup);

	/* for dsi config */
	DSI_ForceConfig(1);

	DISPCHECK("[display_test]FPS config[begin]\n");

	lcm_param2 = disp_lcm_get_params(pgc->plcm);
	lcm_param2->dsi.mode = SYNC_PULSE_VDO_MODE;
	lcm_param2->dsi.vertical_active_line = 1280;
	lcm_param2->dsi.horizontal_active_pixel = 360;

	data_config2.dst_w = 360;
	data_config2.dst_h = 1280;
	data_config2.dispif_config.dsi.vertical_active_line = 1280;
	data_config2.dispif_config.dsi.horizontal_active_pixel = 360;
	data_config2.dispif_config.dsi.mode = SYNC_PULSE_VDO_MODE;
	data_config2.dst_dirty = 1;

	dpmgr_path_set_video_mode(pgc->dpmgr_handle, primary_display_is_video_mode());
	DISPCHECK("[display_test]==>FPS set vdo mode done, is_vdo_mode:%d\n",
		  primary_display_is_video_mode());
	dpmgr_path_connect(pgc->dpmgr_handle, CMDQ_DISABLE);
	dpmgr_path_config(pgc->dpmgr_handle, &data_config2, CMDQ_DISABLE);
	data_config2.dst_dirty = 0;
	DISPCHECK("[display_test]FPS config[end]\n");

	DISPCHECK("[display_test]Start dpmgr path[begin]\n");
	dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		DISPERR
		    ("[display_test]==>Fatal error, we didn't trigger display path but it's already busy\n");
	}
	DISPCHECK("[display_test]Start dpmgr path[end]\n");

	DISPCHECK("[display_test]Trigger dpmgr path[begin]\n");
	dpmgr_path_trigger(pgc->dpmgr_handle, NULL, CMDQ_DISABLE);
	DISPCHECK("[display_test]Trigger dpmgr path[end]\n");

	/* check fps bonding: rdma frame end interval < 12ms */
	disp_record_rdma_end_interval(1);

	/* loop 50 times to get max rdma end interval */
	int i = 50;

	while (i--)
		dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);

	unsigned long long min_interval = disp_get_rdma_min_interval();
	unsigned long long max_interval = disp_get_rdma_max_interval();

	disp_record_rdma_end_interval(0);

	DISPCHECK("[display_test]Check RDMA frame end interval:%lld[end]\n", min_interval);
	if (min_interval < 12 * 1000000) {
		DISPCHECK("[display_test_result]=>0.No limit\n");
	} else {
		DISPCHECK("[display_test_result]=>1.limit max_interval %lld\n", max_interval);
		if (max_interval < 13 * 1000000)
			DISPCHECK("[display_test_result]=>2.naughty enable\n");

	}

	DISPCHECK("[display_test]Stop dpmgr path[begin]\n");
	dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);
	DISPCHECK("[display_test]Stop dpmgr path[end]\n");

	DISPCHECK("[display_test]Restore path config[begin]\n");
	lcm_param2 = disp_lcm_get_params(pgc->plcm);
	lcm_param2->dsi.mode = dsi_mode_backup;
	lcm_param2->dsi.vertical_active_line = h_backup;
	lcm_param2->dsi.horizontal_active_pixel = w_backup;
	data_config2.dispif_config.dsi.vertical_active_line = h_backup;
	data_config2.dispif_config.dsi.horizontal_active_pixel = w_backup;
	data_config2.dispif_config.dsi.mode = dsi_mode_backup;
	data_config2.dst_w = w_backup;
	data_config2.dst_h = h_backup;
	data_config2.dst_dirty = 1;

	dpmgr_path_set_video_mode(pgc->dpmgr_handle, primary_display_is_video_mode());
	DISPCHECK("[display_test]==>Restore mode done, is_vdo_mode:%d\n",
		  primary_display_is_video_mode());
	DISPCHECK("[display_test]==>Restore resolution done, w=%d, h=%d\n", w_backup, h_backup);
	dpmgr_path_connect(pgc->dpmgr_handle, CMDQ_DISABLE);
	dpmgr_path_config(pgc->dpmgr_handle, &data_config2, CMDQ_DISABLE);
	data_config2.dst_dirty = 0;
	DSI_ForceConfig(0);
	DISPCHECK("[display_test]Restore path config[end]\n");
	return ret;
}

int primary_display_roi_test(int x, int y)
{
	int ret = 0;
	int location[3];

	if (x < 0 || y < 0)
		return -1;

	location[0] = x;
	location[1] = y;
	/* dpmgr_path_ioctl(pgc->dpmgr_handle, NULL, DDP_SET_ROI, location); */
	DSI_test_roi(location[0], location[1]);
	return ret;
}

int primary_display_check_test(void)
{
	int ret = 0;
	int esd_backup = 0;

	DISPCHECK("[display_test]Display test[Start]\n");
	_primary_path_lock(__func__);
	/* disable esd check */
	if (atomic_read(&esd_check_task_wakeup)) {
		esd_backup = 1;
		primary_display_esd_check_enable(0);
		msleep(2000);
		DISPCHECK("[display_test]Disable esd check end\n");
	}
	/* if suspend => return */
	if (pgc->state == DISP_SLEPT) {
		DISPCHECK("[display_test_result]======================================\n");
		DISPCHECK("[display_test_result]==>Test Fail : primary display path is slept\n");
		DISPCHECK("[display_test_result]======================================\n");
		goto done;
	}
	/* stop trigger loop */
	DISPCHECK("[display_test]Stop trigger loop[begin]\n");
	_cmdq_stop_trigger_loop();
	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		DISPCHECK("[display_test]==>primary display path is busy\n");
		ret =
		    dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);
		DISPCHECK("[display_test]==>wait frame done ret:%d\n", ret);
	}
	DISPCHECK("[display_test]Stop trigger loop[end]\n");
	/* test rdma res after reset */
	primary_display_rdma_res_test();
	/* test roi */
	primary_display_roi_test(10, 10);
	/* test force te */
	primary_display_te_test();
	/* mutex fps */
	primary_display_fps_test();

	DISPCHECK("[display_test]start dpmgr path[begin]\n");
	dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		DISPERR
		    ("[display_test]==>Fatal error, we didn't trigger display path but it's already busy\n");
	}
	DISPCHECK("[display_test]start dpmgr path[end]\n");

	DISPCHECK("[display_test]Start trigger loop[begin]\n");
	_cmdq_start_trigger_loop();
	DISPCHECK("[display_test]Start trigger loop[end]\n");
done:

	/* restore esd */
	if (esd_backup == 1) {
		primary_display_esd_check_enable(1);
		DISPCHECK("[display_test]Restore esd check\n");
	}
	/* unlock path */
	_primary_path_unlock(__func__);
	DISPCHECK("[display_test]Display test[End]\n");
	return ret;
}
#endif
