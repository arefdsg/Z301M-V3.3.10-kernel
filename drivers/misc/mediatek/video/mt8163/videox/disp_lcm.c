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

#include <linux/slab.h>

#include "disp_drv_log.h"
#include "lcm_drv.h"
#include "disp_drv_platform.h"
#include "ddp_manager.h"
#include "disp_lcm.h"

/* This macro and arrya is designed for multiple LCM support */
/* for multiple LCM, we should assign I/F Port id in lcm driver, such as DPI0, DSI0/1 */
/*static disp_lcm_handle _disp_lcm_driver[MAX_LCM_NUMBER] = {{0}};*/

char *lk_lcmname;
EXPORT_SYMBOL(lk_lcmname);

int _lcm_count(void)
{
	return lcm_count;
}

int _is_lcm_inited(disp_lcm_handle *plcm)
{
	if (plcm) {
		if (plcm->params && plcm->drv)
			return 1;
	} else {
		DISPERR("WARNING, invalid lcm handle: %p\n", plcm);
		return 0;
	}
	return 0;
}

LCM_PARAMS *_get_lcm_params_by_handle(disp_lcm_handle *plcm)
{
	if (plcm)
		return plcm->params;
	DISPERR("WARNING, invalid lcm handle:%p\n", plcm);
	return NULL;
}

LCM_PARAMS *_get_lcm_driver_by_handle(disp_lcm_handle *plcm)
{
	if (plcm)
		return (LCM_PARAMS *) plcm->drv;
	DISPERR("WARNING, invalid lcm handle:%p\n", plcm);
	return NULL;
}

void _dump_lcm_info(disp_lcm_handle *plcm)
{

	LCM_DRIVER *l = NULL;
	LCM_PARAMS *p = NULL;

	if (plcm == NULL) {
		DISPERR("plcm is null\n");
		return;
	}

	l = plcm->drv;
	p = plcm->params;

	if (l && p) {
		DISPCHECK("[LCM], name: %s\n", l->name);

		DISPCHECK("[LCM] resolution: %d x %d\n", p->width, p->height);
		DISPCHECK("[LCM] physical size: %d x %d\n", p->physical_width, p->physical_height);
		DISPCHECK("[LCM] physical size: %d x %d\n", p->physical_width, p->physical_height);

		switch (p->lcm_if) {
		case LCM_INTERFACE_DSI0:
			DISPCHECK("[LCM] interface: DSI0\n");
			break;
		case LCM_INTERFACE_DSI1:
			DISPCHECK("[LCM] interface: DSI1\n");
			break;
		case LCM_INTERFACE_DPI0:
			DISPCHECK("[LCM] interface: DPI0\n");
			break;
		case LCM_INTERFACE_DPI1:
			DISPCHECK("[LCM] interface: DPI1\n");
			break;
		case LCM_INTERFACE_DBI0:
			DISPCHECK("[LCM] interface: DBI0\n");
			break;
		default:
			DISPCHECK("[LCM] interface: unknown\n");
			break;
		}

		switch (p->type) {
		case LCM_TYPE_DBI:
			DISPCHECK("[LCM] Type: DBI\n");
			break;
		case LCM_TYPE_DSI:
			DISPCHECK("[LCM] Type: DSI\n");

			break;
		case LCM_TYPE_DPI:
			DISPCHECK("[LCM] Type: DPI\n");
			break;
		default:
			DISPCHECK("[LCM] TYPE: unknown\n");
			break;
		}

		if (p->type == LCM_TYPE_DSI) {
			switch (p->dsi.mode) {
			case CMD_MODE:
				DISPCHECK("[LCM] DSI Mode: CMD_MODE\n");
				break;
			case SYNC_PULSE_VDO_MODE:
				DISPCHECK("[LCM] DSI Mode: SYNC_PULSE_VDO_MODE\n");
				break;
			case SYNC_EVENT_VDO_MODE:
				DISPCHECK("[LCM] DSI Mode: SYNC_EVENT_VDO_MODE\n");
				break;
			case BURST_VDO_MODE:
				DISPCHECK("[LCM] DSI Mode: BURST_VDO_MODE\n");
				break;
			default:
				DISPCHECK("[LCM] DSI Mode: Unknown\n");
				break;
			}
		}

		if (p->type == LCM_TYPE_DSI) {
			DISPCHECK("[LCM] LANE_NUM: %d,data_format\n", (int)p->dsi.LANE_NUM);
#ifdef MT_TODO
#error
#endif
			DISPCHECK
				("[LCM] vact: %d, vbp: %d, vfp: %d, vact_line: %d\n",
				p->dsi.vertical_sync_active, p->dsi.vertical_backporch,
				p->dsi.vertical_frontporch, p->dsi.vertical_active_line);
			DISPCHECK
				("[LCM] hact: %d, hbp: %d, hfp: %d, hblank: %d\n",
				p->dsi.horizontal_sync_active, p->dsi.horizontal_backporch,
				p->dsi.horizontal_frontporch, p->dsi.horizontal_blanking_pixel);
			DISPCHECK(
			"[LCM] pll_select: %d, pll_div1: %d, pll_div2: %d, fbk_div: %d,fbk_sel: %d, rg_bir: %d\n",
				p->dsi.pll_select, p->dsi.pll_div1, p->dsi.pll_div2, p->dsi.fbk_div,
				p->dsi.fbk_sel, p->dsi.rg_bir);
			DISPCHECK
				("[LCM] rg_bic: %d, rg_bp: %d, PLL_CLOCK: %d, dsi_clock: %d, ssc_range: %d\n",
				p->dsi.rg_bic, p->dsi.rg_bp, p->dsi.PLL_CLOCK, p->dsi.dsi_clock,
				p->dsi.ssc_range);
			DISPCHECK
				("[LCM] ssc_disable: %d, compatibility_for_nvk: %d, cont_clock: %d\n",
				p->dsi.ssc_disable, p->dsi.compatibility_for_nvk, p->dsi.cont_clock);
			DISPCHECK
				("[LCM] lcm_ext_te_enable: %d, noncont_clock: %d, noncont_clock_period: %d\n",
				p->dsi.lcm_ext_te_enable, p->dsi.noncont_clock,
				p->dsi.noncont_clock_period);

		}
	}

}


disp_lcm_handle *disp_lcm_probe(char *plcm_name, LCM_INTERFACE_ID lcm_id)
{
	int lcmindex = 0;
	bool isLCMFound = false;
	bool isLCMInited = false;

	LCM_DRIVER *lcm_drv = NULL;
	LCM_PARAMS *lcm_param = NULL;
	disp_lcm_handle *plcm = NULL;

	DISPFUNC();

	DISPCHECK("plcm_name=%s\n", plcm_name);
	lk_lcmname = plcm_name; //mike add for lcm_init
	if (_lcm_count() == 0) {
		DISPERR("no lcm driver defined in linux kernel driver\n");
		return NULL;
	} else if (_lcm_count() == 1) {
		if (plcm_name == NULL) {
			lcm_drv = lcm_driver_list[0];

			isLCMFound = true;
			isLCMInited = false;
		} else {
			lcm_drv = lcm_driver_list[0];
			if (strcmp(lcm_drv->name, plcm_name)) {
				DISPERR
				    ("FATAL ERROR!!!LCM Driver defined in kernel is different with LK\n");
				return NULL;
			}

			isLCMInited = true;
			isLCMFound = true;
		}
		lcmindex = 0;
	} else {
		if (plcm_name == NULL) {
			/* TODO: we need to detect all the lcm driver */
		} else {
			int i = 0;

			for (i = 0; i < _lcm_count(); i++) {
				lcm_drv = lcm_driver_list[i];
				if (!strcmp(lcm_drv->name, plcm_name)) {
					isLCMFound = true;
					isLCMInited = true;
					lcmindex = i;
					break;
				}
			}

			DISPERR("FATAL ERROR: can't found lcm driver:%s in linux kernel driver\n",
				plcm_name);
		}
		/* TODO: */
	}

	if (isLCMFound == false) {
		DISPERR("FATAL ERROR!!!No LCM Driver defined\n");
		return NULL;
	}

	plcm = kzalloc(sizeof(uint8_t *) * sizeof(disp_lcm_handle), GFP_KERNEL);
	lcm_param = kzalloc(sizeof(uint8_t *) * sizeof(LCM_PARAMS), GFP_KERNEL);
	if (plcm && lcm_param) {
		plcm->params = lcm_param;
		plcm->drv = lcm_drv;
		plcm->is_inited = isLCMInited;
		plcm->index = lcmindex;
	} else {
		DISPERR("FATAL ERROR!!!kzalloc plcm and plcm->params failed\n");
		goto FAIL;
	}

	{
		plcm->drv->get_params(plcm->params);
		plcm->lcm_if_id = plcm->params->lcm_if;

		/* below code is for lcm driver forward compatible */
		if (plcm->params->type == LCM_TYPE_DSI
		    && plcm->params->lcm_if == LCM_INTERFACE_NOTDEFINED)
			plcm->lcm_if_id = LCM_INTERFACE_DSI0;
		if (plcm->params->type == LCM_TYPE_DPI
		    && plcm->params->lcm_if == LCM_INTERFACE_NOTDEFINED)
			plcm->lcm_if_id = LCM_INTERFACE_DPI0;
		if (plcm->params->type == LCM_TYPE_DBI
		    && plcm->params->lcm_if == LCM_INTERFACE_NOTDEFINED)
			plcm->lcm_if_id = LCM_INTERFACE_DBI0;

		if ((lcm_id == LCM_INTERFACE_NOTDEFINED) || lcm_id == plcm->lcm_if_id) {
			plcm->lcm_original_width = plcm->params->width;
			plcm->lcm_original_height = plcm->params->height;
			_dump_lcm_info(plcm);
			return plcm;
		}
		DISPERR("the specific LCM Interface [%d] didn't define any lcm driver\n",
			lcm_id);
		goto FAIL;
	}

FAIL:

	kfree(plcm);
	kfree(lcm_param);
	return NULL;
}


int disp_lcm_init(disp_lcm_handle *plcm, int force)
{
	LCM_DRIVER *lcm_drv = NULL;

	DISPFUNC();



	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;

		if (lcm_drv->init_power) {
			if (!disp_lcm_is_inited(plcm) || force)
				lcm_drv->init_power();
		}

		if (lcm_drv->init) {
			if (!disp_lcm_is_inited(plcm) || force)
				lcm_drv->init();
		} else {
			DISPERR("FATAL ERROR, lcm_drv->init is null\n");
			return -1;
		}

		return 0;
	}
	DISPERR("plcm is null\n");
	return -1;
}

LCM_PARAMS *disp_lcm_get_params(disp_lcm_handle *plcm)
{
	/* DISPFUNC(); */

	if (_is_lcm_inited(plcm))
		return plcm->params;
	else
		return NULL;
}

LCM_INTERFACE_ID disp_lcm_get_interface_id(disp_lcm_handle *plcm)
{
	DISPFUNC();

	if (_is_lcm_inited(plcm))
		return plcm->lcm_if_id;
	else
		return LCM_INTERFACE_NOTDEFINED;
}

int disp_lcm_update(disp_lcm_handle *plcm, int x, int y, int w, int h, int force)
{
	LCM_DRIVER *lcm_drv = NULL;

	DISPDBGFUNC();


	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->update) {
			lcm_drv->update(x, y, w, h);
		} else {
			if (disp_lcm_is_video_mode(plcm)) {
				;
				/* do nothing */
			} else {
				DISPERR("FATAL ERROR, lcm is cmd mode lcm_drv->update is null\n");
			}
			return -1;
		}

		return 0;
	}
	DISPERR("lcm_drv is null\n");
	return -1;
}

/* return 1: esd check fail */
/* return 0: esd check pass */
int disp_lcm_esd_check(disp_lcm_handle *plcm)
{
	LCM_DRIVER *lcm_drv = NULL;

	DISPFUNC();


	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->esd_check)
			return lcm_drv->esd_check();
		DISPERR("FATAL ERROR, lcm_drv->esd_check is null\n");
		return 0;
	}
	DISPERR("lcm_drv is null\n");
	return 0;
}



int disp_lcm_esd_recover(disp_lcm_handle *plcm)
{
	LCM_DRIVER *lcm_drv = NULL;

	DISPFUNC();


	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->esd_recover) {
			lcm_drv->esd_recover();
		} else {
			DISPERR("FATAL ERROR, lcm_drv->esd_check is null\n");
			return -1;
		}

		return 0;
	}
	DISPERR("lcm_drv is null\n");
	return -1;
}




int disp_lcm_suspend(disp_lcm_handle *plcm)
{
	LCM_DRIVER *lcm_drv = NULL;

	DISPFUNC();


	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->suspend) {
			lcm_drv->suspend();
		} else {
			DISPERR("FATAL ERROR, lcm_drv->suspend is null\n");
			return -1;
		}

		return 0;
	}
	DISPERR("lcm_drv is null\n");
	return -1;
}

int disp_lcm_suspend_power(disp_lcm_handle *plcm)
{
	LCM_DRIVER *lcm_drv = NULL;

	DISPFUNC();

	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;

		if (lcm_drv->suspend_power)
			lcm_drv->suspend_power();

		return 0;
	}
	DISPERR("lcm_drv is null\n");
	return -1;
}

int disp_lcm_resume_power(disp_lcm_handle *plcm)
{
	LCM_DRIVER *lcm_drv = NULL;

	DISPFUNC();

	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;

		if (lcm_drv->resume_power)
			lcm_drv->resume_power();

		return 0;
	}
	DISPERR("lcm_drv is null\n");
	return -1;
}

int disp_lcm_resume(disp_lcm_handle *plcm)
{
	LCM_DRIVER *lcm_drv = NULL;

	DISPFUNC();


	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;

		if (lcm_drv->resume)
			lcm_drv->resume();
		else {
			DISPERR("FATAL ERROR, lcm_drv->resume is null\n");
			return -1;
		}

		return 0;
	}
	DISPERR("lcm_drv is null\n");
	return -1;

}

int disp_lcm_is_support_adjust_fps(disp_lcm_handle *plcm)
{
	LCM_DRIVER *lcm_drv = NULL;

	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->adjust_fps)
			return 1;
		else
			return 0;
	}
	DISPERR("lcm not initialied\n");
	return 0;
}

int disp_lcm_adjust_fps(void *cmdq, disp_lcm_handle *plcm, int fps)
{
	LCM_DRIVER *lcm_drv = NULL;

	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->adjust_fps) {
			lcm_drv->adjust_fps(cmdq, fps, plcm->params);
			return 0;
		}
	}
	DISPERR("lcm not initialied\n");
	return -1;
}

#ifdef MT_TODO
#error "maybe CABC can be moved into lcm_ioctl??"
#endif
int disp_lcm_set_backlight(disp_lcm_handle *plcm, int level)
{
	LCM_DRIVER *lcm_drv = NULL;

	DISPFUNC();


	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->set_backlight) {
			lcm_drv->set_backlight(level);
		} else {
			DISPERR("FATAL ERROR, lcm_drv->set_backlight is null\n");
			return -1;
		}

		return 0;
	}
	DISPERR("lcm_drv is null\n");
	return -1;
}




int disp_lcm_ioctl(disp_lcm_handle *plcm, LCM_IOCTL ioctl, unsigned int arg)
{
	return 0;
}

int disp_lcm_is_inited(disp_lcm_handle *plcm)
{
	if (_is_lcm_inited(plcm))
		return plcm->is_inited;
	else
		return 0;
}

unsigned int disp_lcm_ATA(disp_lcm_handle *plcm)
{
	unsigned int ret = 0;
	LCM_DRIVER *lcm_drv = NULL;

	DISPFUNC();


	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->ata_check) {

			ret = lcm_drv->ata_check(NULL);
		} else {
			DISPERR("FATAL ERROR, lcm_drv->ata_check is null\n");
			return 0;
		}

		return ret;
	}
	DISPERR("lcm_drv is null\n");
	return 0;

}

void *disp_lcm_switch_mode(disp_lcm_handle *plcm, int mode)
{

/* DISPFUNC(); */
	LCM_DRIVER *lcm_drv = NULL;
	LCM_DSI_MODE_SWITCH_CMD *lcm_cmd = NULL;

	if (_is_lcm_inited(plcm)) {
		if (plcm->params->dsi.switch_mode_enable == 0) {
			DISPERR(" ERROR, Not enable switch in lcm_get_params function\n");
			return NULL;
		}
		lcm_drv = plcm->drv;
		if (lcm_drv->switch_mode) {
			lcm_cmd = (LCM_DSI_MODE_SWITCH_CMD *) lcm_drv->switch_mode(mode);
			lcm_cmd->cmd_if = (unsigned int)(plcm->params->lcm_cmd_if);
		} else {
			DISPERR("FATAL ERROR, lcm_drv->switch_mode is null\n");
			return NULL;
		}

		return (void *)(lcm_cmd);
	}
	DISPERR("lcm_drv is null\n");
	return NULL;


}

int disp_lcm_is_video_mode(disp_lcm_handle *plcm)
{
	/* DISPFUNC(); */
	LCM_PARAMS *lcm_param = NULL;


	if (_is_lcm_inited(plcm))
		lcm_param = plcm->params;
	else
		BUG();

	switch (lcm_param->type) {
	case LCM_TYPE_DBI:
		return false;
	case LCM_TYPE_DSI:
		break;
	case LCM_TYPE_DPI:
		return true;
	default:
		DISPMSG("[LCM] TYPE: unknown\n");
		break;
	}

	if (lcm_param->type == LCM_TYPE_DSI) {
		switch (lcm_param->dsi.mode) {
		case CMD_MODE:
			return false;
		case SYNC_PULSE_VDO_MODE:
		case SYNC_EVENT_VDO_MODE:
		case BURST_VDO_MODE:
			return true;
		default:
			DISPMSG("[LCM] DSI Mode: Unknown\n");
			break;
		}
	}

	BUG();
	return 0;
}

int disp_lcm_set_cmd(disp_lcm_handle *plcm, void *handle, int *lcm_cmd, unsigned int cmd_num)
{
	LCM_DRIVER *lcm_drv = NULL;

	DISPFUNC();


	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->set_cmd) {
			lcm_drv->set_cmd(handle, lcm_cmd, cmd_num);
		} else {
			DISPERR("FATAL ERROR, lcm_drv->set_cmd is null\n");
			return -1;
		}

		return 0;
	}
	DISPERR("lcm_drv is null\n");
	return -1;

}
