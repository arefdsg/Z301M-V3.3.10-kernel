/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "slimport_custom_declare.h"



#define SLIMPORT_DRV_DEBUG

#ifndef XTAL_CLK_DEF
#define XTAL_CLK_DEF XTAL_27M
#endif

#define XTAL_CLK_M10 (pXTAL_data[XTAL_CLK_DEF].xtal_clk_m10)
#define XTAL_CLK (pXTAL_data[XTAL_CLK_DEF].xtal_clk)

static unsigned char sp_tx_test_bw;
static bool sp_tx_test_lt;
static bool sp_tx_test_edid;
static unsigned char rx_hdcp_err_count;
static unsigned char ds_vid_stb_cntr;
static unsigned char HDCP_fail_count;
static unsigned char audio_stable_count;
static unsigned char external_block_en_bak;
/* static unsigned char ext_int_index; */

static unsigned char g_changed_bandwidth;
static unsigned char g_hdmi_dvi_status;

static unsigned char g_need_clean_status;

/* HDCP control enable/disable from AP  */
/* external_block_en = 1: enable, 0: disable*/
extern int external_block_en;

#ifdef ENABLE_READ_EDID
unsigned char g_edid_break;
unsigned char g_edid_checksum;
unsigned char edid_blocks[256];
static unsigned char g_read_edid_flag;
static unsigned char sleepdelay_count;

#endif

#define HDCP_REPEATER_MODE (__i2c_read_byte(RX_P1, 0x2A) & REPEATER_M)

static struct Packet_AVI sp_tx_packet_avi;
static struct Packet_SPD sp_tx_packet_spd;
static struct Packet_MPEG sp_tx_packet_mpeg;
static struct AudiInfoframe sp_tx_audioinfoframe;

enum SP_TX_System_State sp_tx_system_state;

enum CHARGING_STATUS downstream_charging_status;
enum AUDIO_OUTPUT_STATUS sp_tx_ao_state;
enum VIDEO_OUTPUT_STATUS sp_tx_vo_state;
enum SINK_CONNECTION_STATUS sp_tx_sc_state;
enum SP_TX_LT_STATUS sp_tx_LT_state;
enum SP_TX_System_State sp_tx_system_state_bak;
enum HDCP_STATUS HDCP_state;

unsigned int co3_chipid_list[CO3_NUMS] = {
	0x7818,
	0x7816,
	0x7812,
	0x7810,
	0x7806,
	0x7802
};

struct COMMON_INT common_int_status;
struct HDMI_RX_INT hdmi_rx_int_status;

#define COMMON_INT1 (common_int_status.common_int[0])
#define COMMON_INT2 (common_int_status.common_int[1])
#define COMMON_INT3 (common_int_status.common_int[2])
#define COMMON_INT4 (common_int_status.common_int[3])
#define COMMON_INT5 (common_int_status.common_int[4])
#define COMMON_INT_CHANGED (common_int_status.change_flag)
#define HDMI_RX_INT1 (hdmi_rx_int_status.hdmi_rx_int[0])
#define HDMI_RX_INT2 (hdmi_rx_int_status.hdmi_rx_int[1])
#define HDMI_RX_INT3 (hdmi_rx_int_status.hdmi_rx_int[2])
#define HDMI_RX_INT4 (hdmi_rx_int_status.hdmi_rx_int[3])
#define HDMI_RX_INT5 (hdmi_rx_int_status.hdmi_rx_int[4])
#define HDMI_RX_INT6 (hdmi_rx_int_status.hdmi_rx_int[5])
#define HDMI_RX_INT7 (hdmi_rx_int_status.hdmi_rx_int[6])
#define HDMI_RX_INT_CHANGED (hdmi_rx_int_status.change_flag)
static unsigned char down_sample_en;

static unsigned char assisted_HDCP_repeater;
#define HDCP_ERROR 2
#define HDCP_DOING 1
#define HDCP_DONE 0

static unsigned char __i2c_read_byte(unsigned char dev, unsigned char offset);
static void hardware_power_ctl(unsigned char enable);
static void hdmi_rx_new_vsi_int(void);

#define sp_tx_aux_polling_enable() \
	sp_write_reg_or(TX_P0, TX_DEBUG1, POLLING_EN)
#define sp_tx_aux_polling_disable() \
	sp_write_reg_and(TX_P0, TX_DEBUG1, ~POLLING_EN)

#define reg_bit_ctl(addr, offset, data, enable) \
	do { \
		unsigned char c; \
		sp_read_reg(addr, offset, &c); \
		if (enable) { \
			if ((c & data) != data) { \
				c |= data; \
				sp_write_reg(addr, offset, c); \
			} \
		} else { \
			if ((c & data) == data) { \
				c &= ~data; \
				sp_write_reg(addr, offset, c); \
			} \
		} \
	} while (0)

#define sp_tx_video_mute(enable) \
	reg_bit_ctl(TX_P2, VID_CTRL1, VIDEO_MUTE, enable)
#define hdmi_rx_mute_audio(enable) \
	reg_bit_ctl(RX_P0, RX_MUTE_CTRL, AUD_MUTE, enable)
#define hdmi_rx_mute_video(enable) \
	reg_bit_ctl(RX_P0, RX_MUTE_CTRL, VID_MUTE, enable)
#define sp_tx_addronly_set(enable) \
	reg_bit_ctl(TX_P0, AUX_CTRL2, ADDR_ONLY_BIT, enable)

#define sp_tx_set_link_bw(bw) \
	sp_write_reg(TX_P0, SP_TX_LINK_BW_SET_REG, bw);
#define sp_tx_get_link_bw() \
	__i2c_read_byte(TX_P0, SP_TX_LINK_BW_SET_REG)

#define sp_tx_get_pll_lock_status() \
	((__i2c_read_byte(TX_P0, TX_DEBUG1) & DEBUG_PLL_LOCK) != 0 ? 1 : 0)

#define gen_M_clk_with_downspeading() \
	sp_write_reg_or(TX_P0, SP_TX_M_CALCU_CTRL, M_GEN_CLK_SEL)
#define gen_M_clk_without_downspeading \
	sp_write_reg_and(TX_P0, SP_TX_M_CALCU_CTRL, (~M_GEN_CLK_SEL))

#define hdmi_rx_set_hpd(enable) \
	do { \
		if ((bool)enable) \
			sp_write_reg_or(TX_P2, SP_TX_VID_CTRL3_REG, HPD_OUT); \
		else \
			sp_write_reg_and(TX_P2, SP_TX_VID_CTRL3_REG, ~HPD_OUT); \
	} while (0)

#define hdmi_rx_set_termination(enable) \
	do { \
		if ((bool)enable) \
			sp_write_reg_and(RX_P0, HDMI_RX_TMDS_CTRL_REG7, ~TERM_PD); \
		else \
			sp_write_reg_or(RX_P0, HDMI_RX_TMDS_CTRL_REG7, TERM_PD); \
	} while (0)


#define sp_tx_clean_hdcp_status() do { \
		sp_write_reg(TX_P0, TX_HDCP_CTRL0, 0x03); \
		sp_write_reg_or(TX_P0, TX_HDCP_CTRL0, RE_AUTH); \
		msleep(2); \
		pr_info("%s %s : clean_hdcp_status\n", LOG_TAG, __func__); \
	} while (0)
#define reg_hardware_reset() do { \
		sp_write_reg_or(TX_P2, SP_TX_RST_CTRL_REG, HW_RST); \
		sp_tx_clean_state_machine(); \
		sp_tx_set_sys_state(STATE_SP_INITIALIZED); \
		msleep(500); \
	} while (0)

#define write_dpcd_addr(addrh, addrm, addrl) \
	do { \
		unsigned char temp; \
		if (__i2c_read_byte(TX_P0, AUX_ADDR_7_0) != (unsigned char)addrl) \
			sp_write_reg(TX_P0, AUX_ADDR_7_0, (unsigned char)addrl); \
		if (__i2c_read_byte(TX_P0, AUX_ADDR_15_8) != (unsigned char)addrm) \
			sp_write_reg(TX_P0, AUX_ADDR_15_8, (unsigned char)addrm); \
		sp_read_reg(TX_P0, AUX_ADDR_19_16, &temp); \
		if ((unsigned char)(temp & 0x0F)  != ((unsigned char)addrh & 0x0F)) \
			sp_write_reg(TX_P0, AUX_ADDR_19_16, (temp  & 0xF0) | ((unsigned char)addrh)); \
	} while (0)

#define sp_tx_set_sys_state(ss) \
	do { \
		pr_info("%s %s : set: clean_status: %x,\n ", LOG_TAG, __func__, (unsigned int)g_need_clean_status); \
		if ((sp_tx_system_state >= STATE_LINK_TRAINING) && (ss < STATE_LINK_TRAINING)) \
			sp_write_reg_or(TX_P0, SP_TX_ANALOG_PD_REG, CH0_PD); \
		sp_tx_system_state_bak = sp_tx_system_state; \
		sp_tx_system_state = (unsigned char)ss; \
		g_need_clean_status = 1; \
		print_sys_state(sp_tx_system_state); \
	} while (0)

#define goto_next_system_state() \
	do { \
		pr_info("%s %s : next: clean_status: %x,\n ", LOG_TAG, __func__, (unsigned int)g_need_clean_status); \
		sp_tx_system_state_bak = sp_tx_system_state; \
		sp_tx_system_state++;\
		print_sys_state(sp_tx_system_state); \
	} while (0)

#define redo_cur_system_state() \
	do { \
		pr_info("%s %s : redo: clean_status: %x,\n ", LOG_TAG, __func__, (unsigned int)g_need_clean_status); \
		g_need_clean_status = 1; \
		sp_tx_system_state_bak = sp_tx_system_state; \
		print_sys_state(sp_tx_system_state); \
	} while (0)

#define system_state_change_with_case(status) \
	do { \
		if (sp_tx_system_state >= status) { \
			pr_info("%s %s : change_case: clean_status: %xm,\n ", LOG_TAG, __func__, (unsigned int)g_need_clean_status); \
			if ((sp_tx_system_state >= STATE_LINK_TRAINING) && (status < STATE_LINK_TRAINING)) \
				sp_write_reg_or(TX_P0, SP_TX_ANALOG_PD_REG, CH0_PD); \
			g_need_clean_status = 1; \
			sp_tx_system_state_bak = sp_tx_system_state; \
			sp_tx_system_state = (unsigned char)status; \
			print_sys_state(sp_tx_system_state); \
		} \
	} while (0)

#define sp_write_reg_or(address, offset, mask) \
	(sp_write_reg(address, offset, __i2c_read_byte(address, offset) | (mask)))
#define sp_write_reg_and(address, offset, mask) \
	(sp_write_reg(address, offset, __i2c_read_byte(address, offset) & (mask)))

#define sp_write_reg_and_or(address, offset, and_mask, or_mask) \
	(sp_write_reg(address, offset, ((__i2c_read_byte(address, offset)) & and_mask) | (or_mask)))
#define sp_write_reg_or_and(address, offset, or_mask, and_mask) \
	(sp_write_reg(address, offset, ((__i2c_read_byte(address, offset)) | or_mask) & (and_mask)))

static unsigned char __i2c_read_byte(unsigned char dev, unsigned char offset)
{
	unsigned char temp;
	sp_read_reg(dev, offset, &temp);
	return temp;
}

void hardware_power_ctl(unsigned char enable)
{
	/* static unsigned char supply_power = 0; */

	/* if(supply_power != enable) */

	/*      supply_power = enable; */

	/* else */

	/*      return; */

	if (enable == 0)
		sp_tx_hardware_powerdown();
	else
		sp_tx_hardware_poweron();
}

void wait_aux_op_finish(unsigned char *err_flag)
{
	unsigned char cnt;
	unsigned char c;

	*err_flag = 0;
	cnt = 150;

	while (__i2c_read_byte(TX_P0, AUX_CTRL2) & AUX_OP_EN) {
		mdelay(2);

		if ((cnt--) == 0) {
			pr_info("%s %s :aux operate failed!\n", LOG_TAG,
				__func__);
			*err_flag = 1;
			break;
		}
	}

	sp_read_reg(TX_P0, SP_TX_AUX_STATUS, &c);

	if (c & 0x0F) {
		pr_info("%s %s : wait aux operation status %.2x\n", LOG_TAG,
			__func__, (unsigned int) c);
		*err_flag = 1;
	}
}


void print_sys_state(unsigned char ss)
{
	switch (ss) {
	case STATE_WAITTING_CABLE_PLUG:
		pr_info("%s %s : -STATE_WAITTING_CABLE_PLUG-\n", LOG_TAG,
			__func__);
		break;
	case STATE_SP_INITIALIZED:
		pr_info("%s %s : -STATE_SP_INITIALIZED-\n", LOG_TAG, __func__);
		break;
	case STATE_SINK_CONNECTION:
		pr_info("%s %s : -STATE_SINK_CONNECTION-\n", LOG_TAG,
			__func__);
		break;
#ifdef ENABLE_READ_EDID
	case STATE_PARSE_EDID:
		pr_info("%s %s : -STATE_PARSE_EDID-\n", LOG_TAG, __func__);
		break;
#endif
	case STATE_LINK_TRAINING:
		pr_info("%s %s : -STATE_LINK_TRAINING-\n", LOG_TAG, __func__);
		break;
	case STATE_VIDEO_OUTPUT:
		pr_info("%s %s : -STATE_VIDEO_OUTPUT-\n", LOG_TAG, __func__);
		break;
	case STATE_HDCP_AUTH:
		pr_info("%s %s : -STATE_HDCP_AUTH-\n", LOG_TAG, __func__);
		break;
	case STATE_AUDIO_OUTPUT:
		pr_info("%s %s : -STATE_AUDIO_OUTPUT-\n", LOG_TAG, __func__);
		break;
	case STATE_PLAY_BACK:
		pr_info("%s %s : -STATE_PLAY_BACK-\n", LOG_TAG, __func__);
		break;
	default:
		pr_err("%s %s : system state is error1\n", LOG_TAG, __func__);
		break;
	}
}

/* DPCD */

void sp_tx_rst_aux(void)
{
	/*sp_tx_aux_polling_disable();*/
	sp_write_reg_or(TX_P2, RST_CTRL2, AUX_RST);
	sp_write_reg_and(TX_P2, RST_CTRL2, ~AUX_RST);
	/*sp_tx_aux_polling_enable();*/
}

unsigned char sp_tx_aux_dpcdread_bytes(unsigned char addrh, unsigned char addrm, unsigned char addrl,
				unsigned char cCount, unsigned char *pBuf)
{
	unsigned char c, c1, i;
	unsigned char bOK;
	/* clear buffer */
	sp_write_reg(TX_P0, BUF_DATA_COUNT, 0x80);
	/* command and length */

	c = ((cCount - 1) << 4) | 0x09;
	sp_write_reg(TX_P0, AUX_CTRL, c);
	write_dpcd_addr(addrh, addrm, addrl);
	sp_write_reg_or(TX_P0, AUX_CTRL2, AUX_OP_EN);
	mdelay(2);

	wait_aux_op_finish(&bOK);

	if (bOK == AUX_ERR) {
		pr_err("%s %s : aux read failed\n", LOG_TAG, __func__);
		/* add by span 20130217.   */

		sp_read_reg(TX_P2, SP_TX_INT_STATUS1, &c);
		sp_read_reg(TX_P0, TX_DEBUG1, &c1);
		/* if polling is enabled, wait polling error interrupt */

		if (c1 & POLLING_EN) {
			if (c & POLLING_ERR)
				sp_tx_rst_aux();
		} else
			sp_tx_rst_aux();

		return AUX_ERR;
	}

	for (i = 0; i < cCount; i++) {
		sp_read_reg(TX_P0, BUF_DATA_0 + i, &c);
		*(pBuf + i) = c;

		if (i >= MAX_BUF_CNT)
			break;
	}

	return AUX_OK;
}

unsigned char sp_tx_aux_dpcdwrite_bytes(unsigned char addrh, unsigned char addrm, unsigned char addrl,
				 unsigned char cCount, unsigned char *pBuf)
{
	unsigned char c, i, ret;
	c = ((cCount - 1) << 4) | 0x08;
	sp_write_reg(TX_P0, AUX_CTRL, c);
	write_dpcd_addr(addrh, addrm, addrl);

	for (i = 0; i < cCount; i++) {
		c = *pBuf;
		pBuf++;
		sp_write_reg(TX_P0, BUF_DATA_0 + i, c);

		if (i >= 15)
			break;
	}

	sp_write_reg_or(TX_P0, AUX_CTRL2, AUX_OP_EN);
	wait_aux_op_finish(&ret);
	return ret;
}

unsigned char sp_tx_aux_dpcdwrite_byte(unsigned char addrh, unsigned char addrm, unsigned char addrl,
				unsigned char data1)
{
	unsigned char ret;
	/* one byte write. */
	sp_write_reg(TX_P0, AUX_CTRL, 0x08);
	write_dpcd_addr(addrh, addrm, addrl);
	sp_write_reg(TX_P0, BUF_DATA_0, data1);
	sp_write_reg_or(TX_P0, AUX_CTRL2, AUX_OP_EN);
	wait_aux_op_finish(&ret);
	return ret;
}

/* ========initialized system */

void slimport_block_power_ctrl(enum SP_TX_POWER_BLOCK sp_tx_pd_block,
			       unsigned char power)
{
	if (power == SP_POWER_ON)
		sp_write_reg_and(TX_P2, SP_POWERD_CTRL_REG, (~sp_tx_pd_block));
	else
		sp_write_reg_or(TX_P2, SP_POWERD_CTRL_REG, (sp_tx_pd_block));

	pr_info("%s %s : sp_tx_power_on: %.2x\n", LOG_TAG, __func__,
		(unsigned int) sp_tx_pd_block);
}

void vbus_power_ctrl(unsigned char ON)
{
	unsigned char i;

	if (ON == 0) {
		/* sp_tx_aux_polling_disable(); */

		sp_write_reg_and(TX_P2, TX_PLL_FILTER, ~V33_SWITCH_ON);
		sp_write_reg_or(TX_P2, TX_PLL_FILTER5,
				P5V_PROTECT_PD | SHORT_PROTECT_PD);
		pr_info("%s %s : 3.3V output disabled\n", LOG_TAG, __func__);
	} else {
		for (i = 0; i < 5; i++) {
			sp_write_reg_and(TX_P2, TX_PLL_FILTER5,
					 (~P5V_PROTECT_PD & ~SHORT_PROTECT_PD));
			sp_write_reg_or(TX_P2, TX_PLL_FILTER, V33_SWITCH_ON);

			if (!
			    ((unsigned char) __i2c_read_byte(TX_P2, TX_PLL_FILTER5) &
			     0xc0)) {
				pr_info("%s %s : 3.3V output enabled\n",
					LOG_TAG, __func__);
				break;
			} else {
				pr_info
				    ("%s %s : VBUS power can not be supplied\n",
				     LOG_TAG, __func__);
			}
		}
	}
}

void system_power_ctrl(unsigned char enable)
{
	vbus_power_ctrl(enable);
	slimport_block_power_ctrl(SP_TX_PWR_REG, enable);
	slimport_block_power_ctrl(SP_TX_PWR_TOTAL, enable);
	hardware_power_ctl(enable);
	sp_tx_clean_state_machine();
	sp_tx_set_sys_state(STATE_WAITTING_CABLE_PLUG);
}

void sp_tx_clean_state_machine(void)
{
	sp_tx_system_state = STATE_WAITTING_CABLE_PLUG;
	sp_tx_system_state_bak = STATE_WAITTING_CABLE_PLUG;
	sp_tx_sc_state = SC_INIT;
	sp_tx_LT_state = LT_INIT;
	HDCP_state = HDCP_CAPABLE_CHECK;
	sp_tx_vo_state = VO_WAIT_VIDEO_STABLE;
	sp_tx_ao_state = AO_INIT;
}

unsigned char sp_tx_cur_states(void)
{
	return sp_tx_system_state;
}

unsigned char sp_tx_cur_bw(void)
{
	return g_changed_bandwidth;
}

void sp_tx_set_bw(unsigned char bw)
{
	g_changed_bandwidth = bw;
}

void sp_tx_variable_init(void)
{
	unsigned int i;

	sp_tx_system_state = STATE_WAITTING_CABLE_PLUG;
	sp_tx_system_state_bak = STATE_WAITTING_CABLE_PLUG;
#ifdef ENABLE_READ_EDID
	g_edid_break = 0;
	g_read_edid_flag = 0;
	g_edid_checksum = 0;

	for (i = 0; i < 256; i++)
		edid_blocks[i] = 0;

#endif
	sp_tx_LT_state = LT_INIT;
	HDCP_state = HDCP_CAPABLE_CHECK;
	g_need_clean_status = 0;
	sp_tx_sc_state = SC_INIT;
	sp_tx_vo_state = VO_WAIT_VIDEO_STABLE;
	sp_tx_ao_state = AO_INIT;
	g_changed_bandwidth = LINK_5P4G;
	g_hdmi_dvi_status = HDMI_MODE;

	sp_tx_test_lt = 0;
	sp_tx_test_bw = 0;
	sp_tx_test_edid = 0;

	assisted_HDCP_repeater = HDCP_DONE;
	rx_hdcp_err_count = 0;
	ds_vid_stb_cntr = 0;
	HDCP_fail_count = 0;
	audio_stable_count = 0;
	external_block_en_bak = 0;
}

static void hdmi_rx_tmds_phy_initialization(void)
{
	sp_write_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG2, 0xa9);
	/* sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG4, 0x28); */

	/* sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG5, 0xe3); */

	sp_write_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG7, 0x80);
	/* sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG19, 0x00); */

	/* sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG21, 0x04); */

	/* sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG22, 0x38); */

	/* HDMI RX PHY */

	sp_write_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG1, 0x90);
	sp_write_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG6, 0x92);
	sp_write_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG20, 0xf2);

	//Paddy add for DDC bus
	sp_write_reg(TX_P0, TX_EXTRA_ADDR, 0x50); /* enable DDC stretch */
}

void hdmi_rx_initialization(void)
{
#ifdef CHANGE_TX_P0_ADDR
	/*Change 0x70 I2C address to 0x78 */
	sp_write_reg(TX_P2, SP_TX_DP_ADDR_REG1, 0xBC);
#endif
	sp_write_reg(RX_P0, RX_MUTE_CTRL, AUD_MUTE | VID_MUTE);
	sp_write_reg_or(RX_P0, RX_CHIP_CTRL,
			MAN_HDMI5V_DET | PLLLOCK_CKDT_EN | DIGITAL_CKDT_EN);
	/* sp_write_reg_or(RX_P0, RX_AEC_CTRL, AVC_OE);          */

	sp_write_reg_or(RX_P0, RX_SRST,
			HDCP_MAN_RST | SW_MAN_RST | TMDS_RST | VIDEO_RST);
	sp_write_reg_and(RX_P0, RX_SRST,
			 (~HDCP_MAN_RST) & (~SW_MAN_RST) & (~TMDS_RST) &
			 (~VIDEO_RST));

	/* Sync detect change , GP set mute */
	sp_write_reg_or(RX_P0, RX_AEC_EN0, AEC_EN06 | AEC_EN05);
	sp_write_reg_or(RX_P0, RX_AEC_EN2, AEC_EN21);
	sp_write_reg_or(RX_P0, RX_AEC_CTRL, AVC_EN | AAC_OE | AAC_EN);

	sp_write_reg_and(RX_P0, RX_SYS_PWDN1, ~PWDN_CTRL);

	/*

	sp_write_reg(RX_P0, HDMI_RX_INT_MASK1_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK2_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK3_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK4_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK5_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK6_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK7_REG, 0x00);
	*/
	sp_write_reg_or(RX_P0, RX_VID_DATA_RNG, R2Y_INPUT_LIMIT);
	/* add10, dvdd10 and avdd18 setting */

	sp_write_reg(RX_P0, 0x65, 0xc4);
	sp_write_reg(RX_P0, 0x66, 0x18);

	hdmi_rx_tmds_phy_initialization();
	hdmi_rx_set_hpd(0);
	hdmi_rx_set_termination(0);
	pr_info("%s %s : HDMI Rx is initialized...\n", LOG_TAG, __func__);
}

struct clock_Data const pXTAL_data[XTAL_CLK_NUM] = {
	{19, 192},
	{24, 240},
	{25, 250},
	{26, 260},
	{27, 270},
	{38, 384},
	{52, 520},
	{27, 270},
};

void xtal_clk_sel(void)
{
	pr_info("%s %s : define XTAL_CLK:  %x\n ", LOG_TAG, __func__,
		(unsigned int) XTAL_CLK_DEF);
	sp_write_reg_and_or(TX_P2, TX_ANALOG_DEBUG2, (~0x3c),
			    0x3c & (XTAL_CLK_DEF << 2));
	sp_write_reg(TX_P0, 0xEC, (unsigned char) (((unsigned int) XTAL_CLK_M10)));
	sp_write_reg(TX_P0, 0xED,
		     (unsigned char) (((unsigned int) XTAL_CLK_M10 & 0xFF00) >> 2) | XTAL_CLK);

	sp_write_reg(TX_P0, I2C_GEN_10US_TIMER0,
		     (unsigned char) (((unsigned int) XTAL_CLK_M10)));
	sp_write_reg(TX_P0, I2C_GEN_10US_TIMER1,
		     (unsigned char) (((unsigned int) XTAL_CLK_M10 & 0xFF00) >> 8));
	sp_write_reg(TX_P0, 0xBF, (unsigned char) (((unsigned int) XTAL_CLK - 1)));

	/* CEC function need to change the value. */

	sp_write_reg_and_or(RX_P0, 0x49, 0x07,
			    (unsigned char) (((((unsigned int) XTAL_CLK) >> 1) - 2) << 3));
	/* sp_write_reg(RX_P0, 0x49, 0x5b);//cec test */

}

void sp_tx_initialization(void)
{
	/* set terminal reistor to 50ohm */
	sp_write_reg(TX_P0, AUX_CTRL2, 0x30);
	/* add by jiaocx@20140917, enable aux double diff output */
	sp_write_reg_or(TX_P0, AUX_CTRL2, 0x08);

	if (!HDCP_REPEATER_MODE) {
		sp_write_reg_and(TX_P0, TX_HDCP_CTRL,
				 (~AUTO_EN) & (~AUTO_START));
		sp_write_reg(TX_P0, OTP_KEY_PROTECT1, OTP_PSW1);
		sp_write_reg(TX_P0, OTP_KEY_PROTECT2, OTP_PSW2);
		sp_write_reg(TX_P0, OTP_KEY_PROTECT3, OTP_PSW3);
		sp_write_reg_or(TX_P0, HDCP_KEY_CMD, DISABLE_SYNC_HDCP);
	} else
		sp_write_reg_or(TX_P0, 0x1D, 0xC0);

	sp_write_reg(TX_P2, SP_TX_VID_CTRL8_REG, VID_VRES_TH);

	sp_write_reg(TX_P0, HDCP_AUTO_TIMER, HDCP_AUTO_TIMER_VAL);
	sp_write_reg_or(TX_P0, TX_HDCP_CTRL, LINK_POLLING);

	/* sp_write_reg_or(TX_P0, 0x65 , 0x30); */

	sp_write_reg_or(TX_P0, TX_LINK_DEBUG, M_VID_DEBUG);
	/* sp_write_reg_or(TX_P0, TX_DEBUG1, FORCE_HPD); */
	/* sp_tx_aux_polling_disable(); */

	/*
	   sp_write_reg_or(TX_P2, TX_PLL_FILTER, AUX_TERM_50OHM);
	   sp_write_reg_and(TX_P2, TX_PLL_FILTER5,
	   (~P5V_PROTECT_PD) & (~SHORT_PROTECT_PD));
	 */
	sp_write_reg_or(TX_P2, TX_ANALOG_DEBUG2, POWERON_TIME_1P5MS);
	/*
	   sp_write_reg_or(TX_P0, TX_HDCP_CTRL0,
	   BKSV_SRM_PASS |KSVLIST_VLD);
	 */
	/* sp_write_reg(TX_P2, TX_ANALOG_CTRL, 0xC5); */

	xtal_clk_sel();
	sp_write_reg(TX_P0, AUX_DEFER_CTRL, 0x8C);

	sp_write_reg_or(TX_P0, TX_DP_POLLING, AUTO_POLLING_DISABLE);
	/*Short the link intergrity check timer to speed up bstatus
	   polling for HDCP CTS item 1A-07 */
	sp_write_reg(TX_P0, SP_TX_LINK_CHK_TIMER, 0x1d);
	sp_write_reg_or(TX_P0, TX_MISC, EQ_TRAINING_LOOP);

	/* power down main link by default */

	sp_write_reg_or(TX_P0, SP_TX_ANALOG_PD_REG, CH0_PD);

	/*
	   sp_write_reg(TX_P2, SP_COMMON_INT_MASK1, 0X00);
	   sp_write_reg(TX_P2, SP_COMMON_INT_MASK2, 0X00);
	   sp_write_reg(TX_P2, SP_COMMON_INT_MASK3, 0X00);
	   sp_write_reg(TX_P2, SP_COMMON_INT_MASK4, 0X00);
	 */
	/* sp_write_reg(TX_P2, SP_INT_MASK, 0X90); */

	sp_write_reg(TX_P2, SP_TX_INT_CTRL_REG, 0X01);
	/*disable HDCP mismatch function for VGA dongle */
	/* sp_write_reg(TX_P0, SP_TX_LT_SET_REG, 0); */

	sp_tx_link_phy_initialization();
	gen_M_clk_with_downspeading();

	down_sample_en = 0;
}

void slimport_chip_initial(void)
{
	sp_tx_variable_init();
	/* vbus_power_ctrl(0);  */

	/* hardware_power_ctl(0); */
	/* sp_tx_set_sys_state(STATE_CABLE_PLUG); */

}

bool slimport_chip_detect(void)
{
	unsigned int c;
	unsigned char i;
	bool big_endian;
	unsigned char *ptemp;
	/*check whether CPU is big endian */
	c = 0x1122;
	ptemp = (unsigned char *) &c;

	if (*ptemp == 0x11 && *(ptemp + 1) == 0x22)
		big_endian = 1;
	else
		big_endian = 0;

	hardware_power_ctl(1);
	c = 0;

	/*check chip id */
	if (big_endian) {
		sp_read_reg(TX_P2, SP_TX_DEV_IDL_REG, (unsigned char *) (&c) + 1);
		sp_read_reg(TX_P2, SP_TX_DEV_IDH_REG, (unsigned char *) (&c));
	} else {
		sp_read_reg(TX_P2, SP_TX_DEV_IDL_REG, (unsigned char *) (&c));
		sp_read_reg(TX_P2, SP_TX_DEV_IDH_REG, (unsigned char *) (&c) + 1);
	}

	pr_info("%s %s : CHIPID: ANX%x\n", LOG_TAG, __func__, c & 0x0000FFFF);

	for (i = 0; i < CO3_NUMS; i++) {
		if (c == co3_chipid_list[i])
			return 1;
	}

	return 0;
}
#ifndef USING_HPD_FOR_POWER_MANAGEMENT
unsigned char DP_alt_mode_entered(void)
{
	/*if system enter DP alt_mode, return 1; */
	/*if not, return 0;	*/
	return 1;
	
}
#else
/* ========cable plug */
unsigned char is_cable_detected(void)
{
	return sink_is_connected();
}
#endif

void slimport_waitting_cable_plug_process(void)
{
#ifdef USING_HPD_FOR_POWER_MANAGEMENT
	if (is_cable_detected()) {
#else
	if (DP_alt_mode_entered()) {
#endif
		sp_tx_variable_init();
		hardware_power_ctl(1);
		goto_next_system_state();
	} else
		hardware_power_ctl(0);
}

unsigned char is_anx_dongle(void);
void sp_tx_get_rx_bw(unsigned char *bw)
{
	unsigned char data_buf[4];
	/*When ANX dongle connected, if CHIP_ID = 0x7750, bandwidth is 6.75G,
	because ANX7750 DPCD 0x052x was not available.
	*/
	if(is_anx_dongle()){
		sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x03, 0x04, data_buf);
		if ((data_buf[0] == 0x37) && (data_buf[1] == 0x37)
			&& (data_buf[2] == 0x35) && (data_buf[3] == 0x30))			
			*bw = LINK_6P75G;
		else
			sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x21, 1, bw);	
	}else
		sp_tx_aux_dpcdread_bytes(0x00, 0x00, DPCD_MAX_LINK_RATE, 1, bw);

}

static unsigned char sp_tx_get_cable_type(enum CABLE_TYPE_STATUS det_cable_type_state)
{
	unsigned char ds_port_preset;
	unsigned char aux_status;
	unsigned char data_buf[16];
	unsigned char cur_cable_type;

	ds_port_preset = 0;
	cur_cable_type = DWN_STRM_IS_NULL;

	aux_status =
	    sp_tx_aux_dpcdread_bytes(0x00, 0x00, 0x05, 1, &ds_port_preset);
	pr_info("%s %s : DPCD 0x005: %x\n", LOG_TAG, __func__,
		(int)ds_port_preset);

	switch (det_cable_type_state) {
	case CHECK_AUXCH:

		if (AUX_OK == aux_status) {
			sp_tx_aux_dpcdread_bytes(0x00, 0x00, 0, 0x0c, data_buf);
			det_cable_type_state = GETTED_CABLE_TYPE;
		} else {
			mdelay(50);
			pr_err("%s %s :  AUX access error\n", LOG_TAG,
			       __func__);
			break;
		}

	case GETTED_CABLE_TYPE:

		switch ((ds_port_preset & (_BIT1 | _BIT2)) >> 1) {
		case 0x00:
			cur_cable_type = DWN_STRM_IS_DIGITAL;
			pr_info("%s %s : Downstream is DP dongle.\n", LOG_TAG,
				__func__);
			break;
		case 0x01:
		case 0x03:

			cur_cable_type = DWN_STRM_IS_ANALOG;

			pr_info("%s %s : Downstream is VGA dongle.\n",
			     LOG_TAG, __func__);

			break;
		case 0x02:
			cur_cable_type = DWN_STRM_IS_HDMI;
			pr_info("%s %s : Downstream is HDMI dongle.\n", LOG_TAG,
				__func__);
			break;
		default:
			cur_cable_type = DWN_STRM_IS_NULL;
			pr_err("%s %s : Downstream can not recognized.\n",
			       LOG_TAG, __func__);
			break;
		}

	default:
		break;
	}

	return cur_cable_type;
}


unsigned char sp_tx_get_dp_connection(void)
{
	unsigned char c;

	if (AUX_OK !=
	    sp_tx_aux_dpcdread_bytes(0x00, 0x02, DPCD_SINK_COUNT, 1, &c))
		return 0;

	if (c & 0x1f) {
		sp_tx_aux_dpcdread_bytes(0x00, 0x00, 0x04, 1, &c);

		if (c & 0x20) {
			sp_tx_aux_dpcdread_bytes(0x00, 0x06, 0x00, 1, &c);
			/*Bit 5 = SET_DN_DEVICE_DP_PWR_5V
			   Bit 6 = SET_DN_DEVICE_DP_PWR_12V
			   Bit 7 = SET_DN_DEVICE_DP_PWR_18V */
			c = c & 0x1F;
			sp_tx_aux_dpcdwrite_byte(0x00, 0x06, 0x00, c | 0x20);
		}

		return 1;
	} else
		return 0;
}

/********************************************/
/* Check if it is ANALOGIX dongle.
*/
/********************************************/
unsigned char ANX_OUI[3] = { 0x00, 0x22, 0xB9 };

unsigned char is_anx_dongle(void)
{
	unsigned char buf[3];
	sp_tx_aux_dpcdread_bytes(0x00, 0x04, 0x00, 3, buf);

	if (buf[0] == ANX_OUI[0] && buf[1] == ANX_OUI[1]
	    && buf[2] == ANX_OUI[2]) {
		return 1;
		/* debug_puts("DPCD 400 show ANX-dongle"); */

	}

	/*  just for ANX7730 */
	sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x00, 3, buf);

	if (buf[0] == ANX_OUI[0] && buf[1] == ANX_OUI[1]
	    && buf[2] == ANX_OUI[2]) {
		return 1;
		/* debug_puts("DPCD 500 show ANX-dongle"); */

	}

	return 0;
}


unsigned char sp_tx_get_downstream_connection(void)
{
	return sp_tx_get_dp_connection();
}

void slimport_sink_connection(void)
{
	/*
	if (check_cable_det_pin() == 0) {
		sp_tx_set_sys_state(STATE_WAITTING_CABLE_PLUG);
		return;
	}*/

	switch (sp_tx_sc_state) {
	case SC_INIT:
		sp_tx_sc_state++;
	case SC_CHECK_CABLE_TYPE:
	case SC_WAITTING_CABLE_TYPE:
	default:
		if (sp_tx_get_cable_type(CHECK_AUXCH) == DWN_STRM_IS_NULL) {
			sp_tx_sc_state++;

			if (sp_tx_sc_state >= SC_WAITTING_CABLE_TYPE) {
				sp_tx_sc_state = SC_NOT_CABLE;
				pr_info("%s %s : Can not get cable type!\n",
					LOG_TAG, __func__);
			}

			break;
		}
					
		sp_tx_sc_state = SC_SINK_CONNECTED;
	case SC_SINK_CONNECTED:

		if (sp_tx_get_downstream_connection()) {
			goto_next_system_state();
		}

		break;
	case SC_NOT_CABLE:
		/* vbus_power_ctrl(0); */
		reg_hardware_reset();
		break;
	}
}

/******************start EDID process********************/
void sp_tx_enable_video_input(unsigned char enable)
{
	unsigned char c;
	sp_read_reg(TX_P2, VID_CTRL1, &c);

	if (enable) {
		if ((c & VIDEO_EN) != VIDEO_EN) {
			c = (c & 0xf7) | VIDEO_EN;
			sp_write_reg(TX_P2, VID_CTRL1, c);
			pr_info("%s %s : Slimport Video is enabled!\n", LOG_TAG,
				__func__);
		}
	} else {
		if ((c & VIDEO_EN) == VIDEO_EN) {
			c &= ~VIDEO_EN;
			sp_write_reg(TX_P2, VID_CTRL1, c);
			pr_info("%s %s : Slimport Video is disabled!\n",
				LOG_TAG, __func__);
		}
	}
}

static unsigned char read_dvi_hdmi_mode(void)
{
	return 1;
}

#ifdef ENABLE_READ_EDID
static unsigned char get_edid_detail(unsigned char *data_buf)
{
	unsigned int pixclock_edid;

	pixclock_edid =
	    ((((unsigned int) data_buf[1] << 8)) | ((unsigned int) data_buf[0] & 0xFF));

	if (pixclock_edid <= 5300)
		return LINK_1P62G;
	else if ((5300 < pixclock_edid) && (pixclock_edid <= 8900))
		return LINK_2P7G;
	else if ((8900 < pixclock_edid) && (pixclock_edid <= 18000))
		return LINK_5P4G;
	else
		return LINK_6P75G;

}

static unsigned char parse_edid_to_get_bandwidth(void)
{
	unsigned char desc_offset = 0;
	unsigned char i, bandwidth, temp;
	bandwidth = LINK_1P62G;
	temp = LINK_1P62G;
	i = 0;

	while (4 > i && 0 != edid_blocks[0x36 + desc_offset]) {
		temp = get_edid_detail(edid_blocks + 0x36 + desc_offset);
		pr_info("%s %s : bandwidth via EDID : %x\n", LOG_TAG, __func__,
			(unsigned int) temp);

		if (bandwidth < temp)
			bandwidth = temp;

		if (bandwidth > LINK_5P4G)
			break;

		desc_offset += 0x12;
		++i;
	}

	return bandwidth;

}

static void sp_tx_aux_wr(unsigned char offset)
{
	sp_write_reg(TX_P0, BUF_DATA_0, offset);
	sp_write_reg(TX_P0, AUX_CTRL, 0x04);
	sp_write_reg_or(TX_P0, AUX_CTRL2, AUX_OP_EN);
	wait_aux_op_finish(&g_edid_break);
}

static void sp_tx_aux_rd(unsigned char len_cmd)
{
	sp_write_reg(TX_P0, AUX_CTRL, len_cmd);
	sp_write_reg_or(TX_P0, AUX_CTRL2, AUX_OP_EN);
	wait_aux_op_finish(&g_edid_break);
}

unsigned char sp_tx_get_edid_block(void)
{
	unsigned char c;

	sp_tx_aux_wr(0x7e);
	sp_tx_aux_rd(0x01);
	sp_read_reg(TX_P0, BUF_DATA_0, &c);
	pr_info("%s %s : EDID Block = %d\n", LOG_TAG, __func__, (int)(c + 1));

	if (c > 3)
		c = 1;

	/* g_edid_break = 1; */

	return c;
}

void edid_read(unsigned char offset, unsigned char *pblock_buf)
{
	unsigned char data_cnt, cnt;
	unsigned char c;
	sp_tx_aux_wr(offset);
	/* set I2C read com 0x05 mot = 1 and read 16 bytes          */
	sp_tx_aux_rd(0xf5);
	data_cnt = 0;
	cnt = 0;

	while ((data_cnt) < 16) {
		sp_read_reg(TX_P0, BUF_DATA_COUNT, &c);

		if ((c & 0x1f) != 0) {
			data_cnt = data_cnt + c;

			do {
				sp_read_reg(TX_P0, BUF_DATA_0 + c - 1,
					    &(pblock_buf[c - 1]));
				if (c == 1)
					break;
			} while (c--);

		} else {
			/* pr_info("%s %s : edid read : length: 0\n",
					LOG_TAG, __func__); */

			if (cnt++ <= 2) {
				sp_tx_rst_aux();
				c = 0x05 | ((0x0f - data_cnt) << 4);
				sp_tx_aux_rd(c);
			} else {
				g_edid_break = 1;
				break;
			}
		}
	}

	/* issue a stop every 16 bytes read */

	sp_write_reg(TX_P0, AUX_CTRL, 0x01);
	sp_write_reg_or(TX_P0, AUX_CTRL2, ADDR_ONLY_BIT | AUX_OP_EN);
	wait_aux_op_finish(&g_edid_break);
	sp_tx_addronly_set(0);

}

void sp_tx_edid_read_initial(void)
{
	sp_write_reg(TX_P0, AUX_ADDR_7_0, 0x50);
	sp_write_reg(TX_P0, AUX_ADDR_15_8, 0);
	sp_write_reg_and(TX_P0, AUX_ADDR_19_16, 0xf0);
}

static void segments_edid_read(unsigned char segment, unsigned char offset)
{
	unsigned char c, cnt;
	int i;

	sp_write_reg(TX_P0, AUX_CTRL, 0x04);

	sp_write_reg(TX_P0, AUX_ADDR_7_0, 0x30);

	sp_write_reg_or(TX_P0, AUX_CTRL2, ADDR_ONLY_BIT | AUX_OP_EN);
	/* sp_tx_addronly_set(0);    */

	sp_read_reg(TX_P0, AUX_CTRL2, &c);

	wait_aux_op_finish(&g_edid_break);
	sp_read_reg(TX_P0, AUX_CTRL, &c);

	sp_write_reg(TX_P0, BUF_DATA_0, segment);

	/* set I2C write com 0x04 mot = 1 */

	sp_write_reg(TX_P0, AUX_CTRL, 0x04);

	sp_write_reg_and_or(TX_P0, AUX_CTRL2, ~ADDR_ONLY_BIT, AUX_OP_EN);
	cnt = 0;
	sp_read_reg(TX_P0, AUX_CTRL2, &c);

	while (c & AUX_OP_EN) {
		mdelay(1);
		cnt++;

		if (cnt == 10) {
			pr_info("%s %s : write break", LOG_TAG, __func__);
			sp_tx_rst_aux();
			cnt = 0;
			g_edid_break = 1;
			/*  bReturn; */
			return;
		}

		sp_read_reg(TX_P0, AUX_CTRL2, &c);

	}

	sp_write_reg(TX_P0, AUX_ADDR_7_0, 0x50);

	sp_tx_aux_wr(offset);

	sp_tx_aux_rd(0xf5);
	cnt = 0;

	for (i = 0; i < 16; i++) {
		sp_read_reg(TX_P0, BUF_DATA_COUNT, &c);

		while ((c & 0x1f) == 0) {
			mdelay(2);
			cnt++;
			sp_read_reg(TX_P0, BUF_DATA_COUNT, &c);

			if (cnt == 10) {
				pr_info("%s %s : read break", LOG_TAG,
					__func__);
				sp_tx_rst_aux();
				g_edid_break = 1;
				return;
			}
		}

		sp_read_reg(TX_P0, BUF_DATA_0 + i, &c);
	}

	sp_write_reg(TX_P0, AUX_CTRL, 0x01);
	sp_write_reg_or(TX_P0, AUX_CTRL2, ADDR_ONLY_BIT | AUX_OP_EN);
	sp_write_reg_and(TX_P0, AUX_CTRL2, ~ADDR_ONLY_BIT);
	sp_read_reg(TX_P0, AUX_CTRL2, &c);

	while (c & AUX_OP_EN)
		sp_read_reg(TX_P0, AUX_CTRL2, &c);

	/*      sp_tx_addronly_set(0);   */

}

static bool edid_checksum_result(unsigned char *pBuf)
{
	unsigned char cnt, checksum;
	checksum = 0;

	for (cnt = 0; cnt < 0x80; cnt++)
		checksum = checksum + pBuf[cnt];

	g_edid_checksum = checksum - pBuf[0x7f];
	g_edid_checksum = ~g_edid_checksum + 1;
	/*pr_info("%s %s : g_edid_checksum%x\n",
			LOG_TAG, __func__, (unsigned int)g_edid_checksum); */
	return checksum == 0 ? 1 : 0;
}

static void edid_header_result(unsigned char *pBuf)
{
	if ((pBuf[0] == 0) && (pBuf[7] == 0) && (pBuf[1] == 0xff)
	    && (pBuf[2] == 0xff) && (pBuf[3] == 0xff)
	    && (pBuf[4] == 0xff) && (pBuf[5] == 0xff) && (pBuf[6] == 0xff))
		pr_info("%s %s : Good EDID header!\n", LOG_TAG, __func__);
	else
		pr_err("%s %s : Bad EDID header!\n", LOG_TAG, __func__);

}

void check_edid_data(unsigned char *pblock_buf)
{
	unsigned char i;
	edid_header_result(pblock_buf);

	for (i = 0; i <= ((pblock_buf[0x7e] > 1) ? 1 : pblock_buf[0x7e]); i++) {
		if (!edid_checksum_result(pblock_buf + i * 128))
			pr_err("%s %s : Block %x edid checksum error\n",
			       LOG_TAG, __func__, (unsigned int) i);
		else
			pr_info("%s %s : Block %x edid checksum OK\n", LOG_TAG,
				__func__, (unsigned int) i);
	}
}

bool sp_tx_edid_read(unsigned char *pedid_blocks_buf)
{
	unsigned char offset = 0;
	unsigned char count, blocks_num;
	unsigned char pblock_buf[16];
	unsigned char i, j, c;
	g_edid_break = 0;
	sp_tx_edid_read_initial();
	sp_write_reg(TX_P0, AUX_CTRL, 0x04);
	sp_write_reg_or(TX_P0, AUX_CTRL2, 0x03);
	wait_aux_op_finish(&g_edid_break);
	sp_tx_addronly_set(0);

	blocks_num = sp_tx_get_edid_block();

	count = 0;

	do {
		switch (count) {
		case 0:
		case 1:

			for (i = 0; i < 8; i++) {
				offset = (i + count * 8) * 16;
				edid_read(offset, pblock_buf);

				if (g_edid_break == 1)
					break;

				for (j = 0; j < 16; j++) {
					pedid_blocks_buf[offset + j] =
					    pblock_buf[j];
				}
			}

			break;
		case 2:
			offset = 0x00;

			for (j = 0; j < 8; j++) {
				if (g_edid_break == 1)
					break;

				segments_edid_read(count / 2, offset);
				offset = offset + 0x10;
			}

			break;
		case 3:
			offset = 0x80;

			for (j = 0; j < 8; j++) {
				if (g_edid_break == 1)
					break;

				segments_edid_read(count / 2, offset);
				offset = offset + 0x10;
			}

			break;
		default:
			break;
		}

		count++;

		if (g_edid_break == 1)
			break;
	} while (blocks_num >= count);

	sp_tx_rst_aux();
	/* check edid data */

	if (g_read_edid_flag == 0) {
		check_edid_data(pedid_blocks_buf);
		g_read_edid_flag = 1;
	}

	/*  test edid << */

	sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x18, 1, &c);

	if (c & 0x04) {
		pr_info("%s %s : check sum = %.2x\n", LOG_TAG, __func__,
			(unsigned int) g_edid_checksum);
		c = g_edid_checksum;
		sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, 0x61, 1, &c);
		sp_tx_test_edid = 1;
		c = 0x04;
		sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, 0x60, 1, &c);
		pr_info("%s %s : Test EDID done\n", LOG_TAG, __func__);

	}

	/*  test edid  >> */

	return 0;
}

static bool check_with_pre_edid(unsigned char *org_buf)
{
	unsigned char i;
	unsigned char temp_buf[16];
	bool return_flag;
	return_flag = 0;
	/* check checksum and blocks number */

	g_edid_break = 0;
	sp_tx_edid_read_initial();
	sp_write_reg(TX_P0, AUX_CTRL, 0x04);
	sp_write_reg_or(TX_P0, AUX_CTRL2, 0x03);
	wait_aux_op_finish(&g_edid_break);
	sp_tx_addronly_set(0);

	edid_read(0x70, temp_buf);

	if (g_edid_break == 0) {

		for (i = 0; i < 16; i++) {
			if (org_buf[0x70 + i] != temp_buf[i]) {
				pr_info
				    ("%s %s : different checksum and blocks num\n",
				     LOG_TAG, __func__);
				/* need re-read edid */
				return_flag = 1;
				break;
			}
		}
	} else
		return_flag = 1;

	if (return_flag)
		goto return_point;

	/* check edid information */

	edid_read(0x08, temp_buf);

	if (g_edid_break == 0) {
		for (i = 0; i < 16; i++) {
			if (org_buf[i + 8] != temp_buf[i]) {
				pr_info("%s %s : different edid information\n",
					LOG_TAG, __func__);
				return_flag = 1;
				break;
			}
		}
	} else
		return_flag = 1;

 return_point:

	sp_tx_rst_aux();
	return return_flag;
}

void slimport_edid_process(void)
{
	unsigned char temp_value, temp_value1;
	unsigned char i;

	pr_info("%s %s : edid_process\n", LOG_TAG, __func__);

	if (g_read_edid_flag == 1) {
		if (check_with_pre_edid(edid_blocks))
			g_read_edid_flag = 0;
		else
			pr_info("%s %s : Don`t need to read edid!\n", LOG_TAG,
				__func__);
	}

	if (g_read_edid_flag == 0) {
		sp_tx_edid_read(edid_blocks);

		if (g_edid_break)
			pr_err("%s %s : ERR:EDID corruption!\n", LOG_TAG,
			       __func__);
	}

	/*Release the HPD after the OTP loaddown */
	i = 10;

	do {
		if ((__i2c_read_byte(TX_P0, HDCP_KEY_STATUS) & 0x01))
			break;
		else {
			pr_info("%s %s : waiting HDCP KEY loaddown\n", LOG_TAG,
				__func__);
			mdelay(1);
		}
	} while (--i);

	sp_write_reg(RX_P0, HDMI_RX_INT_MASK1_REG, 0xe2);

	if (!HDCP_REPEATER_MODE) {
		hdmi_rx_set_hpd(1);
		pr_info("%s %s : hdmi_rx_set_hpd 1 !\n", LOG_TAG, __func__);
		hdmi_rx_set_termination(1);
	}

	sp_tx_get_rx_bw(&temp_value);
	temp_value1 = parse_edid_to_get_bandwidth();

	if (temp_value <= temp_value1)
		temp_value1 = temp_value;

	pr_info("%s %s : set link bw in edid %x\n", LOG_TAG, __func__,
		(unsigned int) temp_value1);
	/* sp_tx_set_link_bw(temp_value1); */

	g_changed_bandwidth = temp_value1;
	/*
	   sp_tx_send_message(
	   (g_hdmi_dvi_status == HDMI_MODE) ? MSG_INPUT_HDMI : MSG_INPUT_DVI);
	 */
	goto_next_system_state();
}
#endif
/******************End EDID process********************/
/******************start Link training process********************/

static void sp_tx_lvttl_bit_mapping(void)
{
	unsigned char c, colorspace;
	unsigned char vid_bit;

	vid_bit = 0;
	sp_read_reg(RX_P1, HDMI_RX_AVI_DATA00_REG, &colorspace);
	colorspace &= 0x60;

	switch (((__i2c_read_byte(RX_P0, HDMI_RX_VIDEO_STATUS_REG1) &
		  COLOR_DEPTH) >> 4)) {
	default:
	case Hdmi_legacy:
		c = IN_BPC_8BIT;
		vid_bit = 0;
		break;
	case Hdmi_24bit:
		c = IN_BPC_8BIT;

		if (colorspace == 0x20)
			vid_bit = 5;
		else
			vid_bit = 1;

		break;
	case Hdmi_30bit:
		c = IN_BPC_10BIT;

		if (colorspace == 0x20)
			vid_bit = 6;
		else
			vid_bit = 2;

		break;
	case Hdmi_36bit:
		c = IN_BPC_12BIT;

		if (colorspace == 0x20)
			vid_bit = 6;
		else
			vid_bit = 3;

		break;

	}

	/* For down sample video (12bit, 10bit ---> 8bit), this register don`t change */
	if (down_sample_en == 0)
		sp_write_reg_and_or(TX_P2, SP_TX_VID_CTRL2_REG, 0x8c,
				    colorspace >> 5 | c);

	/* mdelay(500); */

	/*Patch: for 10bit video must be set this value to 12bit by someone */
	if (down_sample_en == 1 && c == IN_BPC_10BIT)
		vid_bit = 3;

	sp_write_reg_and_or(TX_P2, BIT_CTRL_SPECIFIC, 0x00,
			    ENABLE_BIT_CTRL | vid_bit << 1);

	if (sp_tx_test_edid) {
		/* set color depth to 18-bit for link cts */

		sp_write_reg_and(TX_P2, SP_TX_VID_CTRL2_REG, 0x8f);
		sp_tx_test_edid = 0;
		pr_info("%s %s : ***color space is set to 18bit***\n", LOG_TAG,
			__func__);
	}

	if (colorspace) {
		sp_write_reg(TX_P0, SP_TX_VID_BLANK_SET1, 0x80);
		sp_write_reg(TX_P0, SP_TX_VID_BLANK_SET2, 0x00);
		sp_write_reg(TX_P0, SP_TX_VID_BLANK_SET3, 0x80);
	} else {
		sp_write_reg(TX_P0, SP_TX_VID_BLANK_SET1, 0x0);
		sp_write_reg(TX_P0, SP_TX_VID_BLANK_SET2, 0x0);
		sp_write_reg(TX_P0, SP_TX_VID_BLANK_SET3, 0x0);
	}

}

unsigned long sp_tx_pclk_calc(void)
{
	unsigned long str_plck;
	unsigned int vid_counter;
	unsigned char c;
	sp_read_reg(RX_P0, 0x8d, &c);
	vid_counter = c;
	vid_counter = vid_counter << 8;
	sp_read_reg(RX_P0, 0x8c, &c);
	vid_counter |= c;
	str_plck = ((unsigned long) vid_counter * XTAL_CLK_M10) >> 12;
	pr_info("%s %s : PCLK = %d.%d\n", LOG_TAG, __func__,
		(((unsigned int) (str_plck)) / 10),
		((unsigned int) str_plck - (((unsigned int) str_plck / 10) * 10)));
	return str_plck;
}

static unsigned char sp_tx_bw_lc_sel(unsigned long pclk)
{
	unsigned long pixel_clk;
	unsigned char c1;

	switch (((__i2c_read_byte(RX_P0, HDMI_RX_VIDEO_STATUS_REG1) &
		  COLOR_DEPTH) >> 4)) {
	case Hdmi_legacy:
	case Hdmi_24bit:
	default:
		pixel_clk = pclk;
		break;
	case Hdmi_30bit:
		pixel_clk = (pclk * 5) >> 2;
		break;
	case Hdmi_36bit:
		pixel_clk = (pclk * 3) >> 1;
		break;
	}

	pr_info("%s %s : pixel_clk = %d.%d\n", LOG_TAG, __func__,
		(((unsigned int) (pixel_clk)) / 10),
		((unsigned int) pixel_clk - (((unsigned int) pixel_clk / 10) * 10)));

	down_sample_en = 0;

	if (pixel_clk <= 530)
		c1 = LINK_1P62G;
	else if ((530 < pixel_clk) && (pixel_clk <= 890))
		c1 = LINK_2P7G;
	else if ((890 < pixel_clk) && (pixel_clk <= 1800))
		c1 = LINK_5P4G;
	else {
		c1 = LINK_6P75G;

		if (pixel_clk > 2240)
			down_sample_en = 1;
	}

	if (sp_tx_get_link_bw() != c1) {
		g_changed_bandwidth = c1;
		pr_info
		    ("%s %s : Different BW between sink Max. and video!%.2x\n",
		     LOG_TAG, __func__, (unsigned int) c1);
		return 1;
	}

	return 0;
}

void sp_tx_spread_enable(unsigned char benable)
{
	unsigned char c;

	sp_read_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, &c);

	if (benable) {
		c |= SP_TX_SSC_DWSPREAD;
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, c);

		sp_tx_aux_dpcdread_bytes(0x00, 0x01, DPCD_DOWNSPREAD_CTRL, 1,
					 &c);
		c |= SPREAD_AMPLITUDE;
		sp_tx_aux_dpcdwrite_byte(0x00, 0x01, DPCD_DOWNSPREAD_CTRL, c);

	} else {
		c &= ~SP_TX_SSC_DISABLE;
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, c);

		sp_tx_aux_dpcdread_bytes(0x00, 0x01, DPCD_DOWNSPREAD_CTRL, 1,
					 &c);
		c &= ~SPREAD_AMPLITUDE;
		sp_tx_aux_dpcdwrite_byte(0x00, 0x01, DPCD_DOWNSPREAD_CTRL, c);
	}

}

void sp_tx_config_ssc(enum SP_SSC_DEP sscdep)
{
	/* clear register */
	sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, 0x0);
	sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, sscdep);
	sp_tx_spread_enable(1);
}

void sp_tx_enhancemode_set(void)
{
	unsigned char c;
	sp_tx_aux_dpcdread_bytes(0x00, 0x00, DPCD_MAX_LANE_COUNT, 1, &c);

	if (c & ENHANCED_FRAME_CAP) {

		sp_write_reg_or(TX_P0, SP_TX_SYS_CTRL4_REG, ENHANCED_MODE);

		sp_tx_aux_dpcdread_bytes(0x00, 0x01, DPCD_LANE_COUNT_SET, 1,
					 &c);
		c |= ENHANCED_FRAME_EN;
		sp_tx_aux_dpcdwrite_byte(0x00, 0x01, DPCD_LANE_COUNT_SET, c);

		pr_info("%s %s : Enhance mode enabled\n", LOG_TAG, __func__);
	} else {

		sp_write_reg_and(TX_P0, SP_TX_SYS_CTRL4_REG, ~ENHANCED_MODE);

		sp_tx_aux_dpcdread_bytes(0x00, 0x01, DPCD_LANE_COUNT_SET, 1,
					 &c);
		c &= ~ENHANCED_FRAME_EN;
		sp_tx_aux_dpcdwrite_byte(0x00, 0x01, DPCD_LANE_COUNT_SET, c);

		pr_info("%s %s : Enhance mode disabled\n", LOG_TAG, __func__);
	}
}

unsigned int sp_tx_link_err_check(void)
{
	unsigned int errl = 0, errh = 0;
	unsigned char bytebuf[2];

	sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x10, 2, bytebuf);
	mdelay(5);
	sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x10, 2, bytebuf);
	errh = bytebuf[1];

	if (errh & 0x80) {
		errl = bytebuf[0];
		errh = (errh & 0x7f) << 8;
		errl = errh + errl;
	}

	pr_err("%s %s :  Err of Lane = %d\n", LOG_TAG, __func__, errl);
	return errl;
}

static void serdes_fifo_reset(void)
{
	sp_write_reg_or(TX_P2, RST_CTRL2, SERDES_FIFO_RST);
	msleep(20);
	sp_write_reg_and(TX_P2, RST_CTRL2, (~SERDES_FIFO_RST));
}

void slimport_link_training(void)
{
	unsigned char temp_value, return_value, c;
	return_value = 1;
	pr_info("%s %s : sp_tx_LT_state : %x\n", LOG_TAG, __func__,
		(int)sp_tx_LT_state);

	switch (sp_tx_LT_state) {
	case LT_INIT:
		slimport_block_power_ctrl(SP_TX_PWR_VIDEO, SP_POWER_ON);
		sp_tx_video_mute(1);
		sp_tx_enable_video_input(0);
		sp_tx_LT_state++;

			
	case LT_WAIT_PLL_LOCK:

		if (!sp_tx_get_pll_lock_status()) {
			/* pll reset when plll not lock.  */

			sp_read_reg(TX_P0, SP_TX_PLL_CTRL_REG, &temp_value);
			temp_value |= PLL_RST;
			sp_write_reg(TX_P0, SP_TX_PLL_CTRL_REG, temp_value);
			temp_value &= ~PLL_RST;
			sp_write_reg(TX_P0, SP_TX_PLL_CTRL_REG, temp_value);
			pr_info("%s %s : PLL not lock!\n", LOG_TAG, __func__);
		} else
			sp_tx_LT_state = LT_CHECK_LINK_BW;

		SP_BREAK(LT_WAIT_PLL_LOCK, sp_tx_LT_state);
	case LT_CHECK_LINK_BW:
		sp_tx_get_rx_bw(&temp_value);

		if (temp_value < g_changed_bandwidth) {
			pr_info("%s %s : ****Over bandwidth****\n", LOG_TAG,
				__func__);
			g_changed_bandwidth = temp_value;
		} else
			sp_tx_LT_state++;

	case LT_START:

		if (sp_tx_test_lt) {
			/* sp_tx_test_lt = 0;  */

			g_changed_bandwidth = sp_tx_test_bw;
			sp_write_reg_and(TX_P2, SP_TX_VID_CTRL2_REG, 0x8f);
		} else {
#ifdef DEMO_4K_2K

			if (down_sample_en)
				sp_write_reg(TX_P0, SP_TX_LT_SET_REG, 0x09);
			else
#endif
				sp_write_reg(TX_P0, SP_TX_LT_SET_REG, 0x0);
		}

		/* power on main link before link training */

		sp_write_reg_and(TX_P0, SP_TX_ANALOG_PD_REG, ~CH0_PD);

		sp_tx_config_ssc(SSC_DEP_4000PPM);
		sp_tx_set_link_bw(g_changed_bandwidth);
		sp_tx_enhancemode_set();

		/*  judge downstream DP version  and set downstream sink to D0 (normal operation mode) */

		sp_tx_aux_dpcdread_bytes(0x00, 0x00, 0x00, 0x01, &c);
		sp_tx_aux_dpcdread_bytes(0x00, 0x06, 0x00, 0x01, &temp_value);

		/* DP 1.2 and above  */
		if (c >= 0x12) {
			temp_value &= 0xf8;
		}
		/* DP 1.1 or 1.0 */
		else {
			temp_value &= 0xfc;
		}

		temp_value |= 0x01;
		sp_tx_aux_dpcdwrite_byte(0x00, 0x06, 0x00, temp_value);

		/* sp_write_reg_or(TX_P2, SP_INT_MASK, 0X20); //hardware interrupt mask enable */

		sp_write_reg(TX_P0, LT_CTRL, SP_TX_LT_EN);
		sp_tx_LT_state = LT_WAITTING_FINISH;
		/* There is no break; */

	case LT_WAITTING_FINISH:
		/*here : waitting interrupt to change training state. */
		break;
	case LT_ERROR:
		serdes_fifo_reset();
		pr_info("LT ERROR Status: SERDES FIFO reset.");
		redo_cur_system_state();
		sp_tx_LT_state = LT_INIT;
		break;
	case LT_FINISH:
		sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x02, 1, &temp_value);

		/*  for one lane case. */
		if ((temp_value & 0x07) == 0x07) {
			/* if there is link error, adjust pre-emphsis to check error again.
			   If there is no error,keep the setting, otherwise use 400mv0db */
			if (!sp_tx_test_lt) {
				if (sp_tx_link_err_check()) {
					sp_read_reg(TX_P0, SP_TX_LT_SET_REG,
						    &temp_value);

					if (!(temp_value & MAX_PRE_REACH)) {
						/* increase one pre-level */
						sp_write_reg(TX_P0,
							     SP_TX_LT_SET_REG,
							     (temp_value +
							      0x08));
						/* if error still exist, return to the link traing value */

						if (sp_tx_link_err_check())
							sp_write_reg(TX_P0,
							SP_TX_LT_SET_REG,
							temp_value);
					}
				}

				sp_read_reg(TX_P0, SP_TX_LINK_BW_SET_REG,
					    &temp_value);

				if (temp_value == g_changed_bandwidth) {
					pr_info("%s %s : LT succeed, bw: %.2x ",
						LOG_TAG, __func__,
						(unsigned int) temp_value);
					pr_info("Lane0 Set: %.2x\n",
						(unsigned int) __i2c_read_byte(TX_P0,
						       SP_TX_LT_SET_REG));
					sp_tx_LT_state = LT_INIT;

					if (HDCP_REPEATER_MODE) {
						hdmi_rx_set_hpd(1);
						pr_info
						    ("%s %s : hdmi_rx_set_hpd 1 !\n",
						     LOG_TAG, __func__);
						hdmi_rx_set_termination(1);
					}

					/******A Patch for JIRA CLD-142******
					Description: Under low voltage(DVD10 = 0.97V),
					some chip(4/46) can not output video,
					link down interrupt always happen.
					*************************************/
					if (sp_tx_link_err_check() > 200) {
						pr_info
						    ("%s %s :Need to reset Serdes FIFO.\n",
						     LOG_TAG, __func__);
						sp_tx_LT_state = LT_ERROR;
					} else
						goto_next_system_state();

					/***********Patch END**********/
				} else {
					pr_info
					    ("%s %s : bw cur:%.2x, per:%.2x\n",
					     LOG_TAG, __func__,
					     (unsigned int) temp_value,
					     (unsigned int) g_changed_bandwidth);
					sp_tx_LT_state = LT_ERROR;
				}
			} else {
				/*sp_tx_test_lt = 0;*/
				sp_tx_LT_state = LT_INIT;
				goto_next_system_state();
			}
		} else {
			pr_info("%s %s : LANE0 Status error: %.2x\n", LOG_TAG,
				__func__, (unsigned int) (temp_value & 0x07));
			sp_tx_LT_state = LT_ERROR;
		}

		break;
	default:
		break;
	}

}

/******************End Link training process********************/
/******************Start Output video process********************/
void sp_tx_set_colorspace(void)
{
	unsigned char color_space;
	unsigned char c;

	if (down_sample_en) {
		sp_read_reg(RX_P1, HDMI_RX_AVI_DATA00_REG, &color_space);
		color_space &= 0x60;

		if (color_space == 0x20) {
			pr_info("%s %s : YCbCr4:2:2 ---> PASS THROUGH.\n",
				LOG_TAG, __func__);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL6_REG, 0x00);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL5_REG, 0x00);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL2_REG, 0x11);
			/* sp_write_reg_and_or(TX_P2, SP_TX_VID_CTRL2_REG, 0xfc,0x01); */

		} else if (color_space == 0x40) {
			pr_info("%s %s : YCbCr4:4:4 ---> YCbCr4:2:2\n", LOG_TAG,
				__func__);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL6_REG, 0x41);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL5_REG, 0x00);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL2_REG, 0x12);
			/* sp_write_reg_and_or(TX_P2, SP_TX_VID_CTRL2_REG, 0xfc,0x02); */

		} else if (color_space == 0x00) {
			pr_info("%s %s : RGB4:4:4 ---> YCbCr4:2:2\n", LOG_TAG,
				__func__);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL6_REG, 0x41);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL5_REG, 0x83);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL2_REG, 0x10);
			/* sp_write_reg_and(TX_P2, SP_TX_VID_CTRL2_REG, 0xfc); */

		}
	} else {
		sp_read_reg(TX_P2, SP_TX_VID_CTRL2_REG, &color_space);

		/* YCBCR422 */
		if ((color_space & 0x03) == 0x01) {

			/* YUV->RGB */
			sp_write_reg_or(TX_P2, SP_TX_VID_CTRL5_REG,
					RANGE_Y2R | CSPACE_Y2R);

			sp_read_reg(RX_P1, (HDMI_RX_AVI_DATA00_REG + 3),
				    &c);
			/* vic for BT709.  */

			if ((c == 0x04) || (c == 0x05) || (c == 0x10)
			    || (c == 0x13) || (c == 0x14) || (c == 0x1F)
			    || (c == 0x20) || (c == 0x21) || (c == 0x22)
			    || (c == 0x27) || (c == 0x28) || (c == 0x29)
			    || (c == 0x2E) || (c == 0x2F) || (c == 0x3C)
			    || (c == 0x3D) || (c == 0x3E) || (c == 0x3F)
			    || (c == 0x40))
				sp_write_reg_or(TX_P2,
						SP_TX_VID_CTRL5_REG,
						CSC_STD_SEL);
			else
				sp_write_reg_and(TX_P2,
						 SP_TX_VID_CTRL5_REG,
						 ~CSC_STD_SEL);

			/* up sample */
			sp_write_reg_or(TX_P2, SP_TX_VID_CTRL6_REG,
					VIDEO_PROCESS_EN | UP_SAMPLE);
			/* YCBCR444 */
		} else if ((color_space & 0x03) == 0x02) {

			/* YUV->RGB */
			sp_write_reg_or(TX_P2, SP_TX_VID_CTRL5_REG,
					RANGE_Y2R | CSPACE_Y2R);

			sp_read_reg(RX_P1, (HDMI_RX_AVI_DATA00_REG + 3),
				    &c);
			/* vic for BT709.  */

			if ((c == 0x04) || (c == 0x05) || (c == 0x10)
			    || (c == 0x13) || (c == 0x14) || (c == 0x1F)
			    || (c == 0x20) || (c == 0x21) || (c == 0x22)
			    || (c == 0x27) || (c == 0x28) || (c == 0x29)
			    || (c == 0x2E) || (c == 0x2F) || (c == 0x3C)
			    || (c == 0x3D) || (c == 0x3E) || (c == 0x3F)
			    || (c == 0x40))
				sp_write_reg_or(TX_P2,
						SP_TX_VID_CTRL5_REG,
						CSC_STD_SEL);
			else
				sp_write_reg_and(TX_P2,
						 SP_TX_VID_CTRL5_REG,
						 ~CSC_STD_SEL);

			sp_write_reg_or_and(TX_P2, SP_TX_VID_CTRL6_REG,
					    VIDEO_PROCESS_EN,
					    ~UP_SAMPLE);
			/* RGB */
		} else if ((color_space & 0x03) == 0x00) {
			sp_write_reg_and(TX_P2, SP_TX_VID_CTRL5_REG,
					 (~RANGE_Y2R) & (~CSPACE_Y2R) &
					 (~CSC_STD_SEL));
			sp_write_reg_and(TX_P2, SP_TX_VID_CTRL6_REG,
					 (~VIDEO_PROCESS_EN) &
					 (~UP_SAMPLE));
		}
	}
}

void sp_tx_avi_setup(void)
{
	unsigned char c;
	int i;

	for (i = 0; i < 13; i++) {
		sp_read_reg(RX_P1, (HDMI_RX_AVI_DATA00_REG + i), &c);
		sp_tx_packet_avi.AVI_data[i] = c;
	}	

	sp_tx_packet_avi.AVI_data[0] &= ~0x60;
}

static void sp_tx_load_packet(enum PACKETS_TYPE type)
{
	int i;
	unsigned char c;

	switch (type) {
	case AVI_PACKETS:
		sp_write_reg(TX_P2, SP_TX_AVI_TYPE, 0x82);
		sp_write_reg(TX_P2, SP_TX_AVI_VER, 0x02);
		sp_write_reg(TX_P2, SP_TX_AVI_LEN, 0x0d);

		for (i = 0; i < 13; i++) {
			sp_write_reg(TX_P2, SP_TX_AVI_DB0 + i,
				     sp_tx_packet_avi.AVI_data[i]);
		}

		break;

	case SPD_PACKETS:
		sp_write_reg(TX_P2, SP_TX_SPD_TYPE, 0x83);
		sp_write_reg(TX_P2, SP_TX_SPD_VER, 0x01);
		sp_write_reg(TX_P2, SP_TX_SPD_LEN, 0x19);

		for (i = 0; i < 25; i++) {
			sp_write_reg(TX_P2, SP_TX_SPD_DB0 + i,
				     sp_tx_packet_spd.SPD_data[i]);
		}

		break;

	case VSI_PACKETS:
		sp_write_reg(TX_P2, SP_TX_MPEG_TYPE, 0x81);
		sp_write_reg(TX_P2, SP_TX_MPEG_VER, 0x01);
		sp_read_reg(RX_P1, HDMI_RX_MPEG_LEN_REG, &c);
		sp_write_reg(TX_P2, SP_TX_MPEG_LEN, c);

		for (i = 0; i < 10; i++) {
			sp_write_reg(TX_P2, SP_TX_MPEG_DB0 + i,
				     sp_tx_packet_mpeg.MPEG_data[i]);
		}

		break;
	case MPEG_PACKETS:
		sp_write_reg(TX_P2, SP_TX_MPEG_TYPE, 0x85);
		sp_write_reg(TX_P2, SP_TX_MPEG_VER, 0x01);
		sp_write_reg(TX_P2, SP_TX_MPEG_LEN, 0x0d);

		for (i = 0; i < 10; i++) {
			sp_write_reg(TX_P2, SP_TX_MPEG_DB0 + i,
				     sp_tx_packet_mpeg.MPEG_data[i]);
		}

		break;
	case AUDIF_PACKETS:
		sp_write_reg(TX_P2, SP_TX_AUD_TYPE, 0x84);
		sp_write_reg(TX_P2, SP_TX_AUD_VER, 0x01);
		sp_write_reg(TX_P2, SP_TX_AUD_LEN, 0x0a);

		for (i = 0; i < 10; i++) {
			sp_write_reg(TX_P2, SP_TX_AUD_DB0 + i,
				     sp_tx_audioinfoframe.pb_byte[i]);
		}

		break;

	default:
		break;
	}
}

void sp_tx_config_packets(enum PACKETS_TYPE bType)
{
	unsigned char c;

	switch (bType) {
	case AVI_PACKETS:

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c &= ~AVI_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);
		sp_tx_load_packet(AVI_PACKETS);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= AVI_IF_UD;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= AVI_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		break;

	case SPD_PACKETS:
		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c &= ~SPD_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);
		sp_tx_load_packet(SPD_PACKETS);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= SPD_IF_UD;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= SPD_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		break;

	case VSI_PACKETS:
		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c &= ~MPEG_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		sp_tx_load_packet(VSI_PACKETS);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= MPEG_IF_UD;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= MPEG_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		break;
	case MPEG_PACKETS:
		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c &= ~MPEG_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		sp_tx_load_packet(MPEG_PACKETS);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= MPEG_IF_UD;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= MPEG_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		break;
	case AUDIF_PACKETS:
		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c &= ~AUD_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		sp_tx_load_packet(AUDIF_PACKETS);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= AUD_IF_UP;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= AUD_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		break;

	default:
		break;
	}

}

void slimport_config_video_output(void)
{
	unsigned char temp_value;

	switch (sp_tx_vo_state) {
	default:
	case VO_WAIT_VIDEO_STABLE:
		/* HDMI RX video judge */

		sp_read_reg(RX_P0, HDMI_RX_SYS_STATUS_REG, &temp_value);

		/* input video stable */
		if ((temp_value & (TMDS_DE_DET | TMDS_CLOCK_DET)) == 0x03) {
			sp_tx_bw_lc_sel(sp_tx_pclk_calc());
			sp_tx_enable_video_input(0);
			sp_tx_avi_setup();
			sp_tx_config_packets(AVI_PACKETS);
			sp_tx_set_colorspace();
			sp_tx_lvttl_bit_mapping();

			/* VSI packet received */
			if (__i2c_read_byte(RX_P0, RX_PACKET_REV_STA) &
			    VSI_RCVD)
				hdmi_rx_new_vsi_int();

			sp_tx_enable_video_input(1);
			sp_tx_vo_state = VO_WAIT_TX_VIDEO_STABLE;
		} else {
			pr_info("%s %s :HDMI input video not stable!\n",
				LOG_TAG, __func__);
		}

		SP_BREAK(VO_WAIT_VIDEO_STABLE, sp_tx_vo_state);

	case VO_WAIT_TX_VIDEO_STABLE:
		/* DP TX video judge */

		sp_read_reg(TX_P0, SP_TX_SYS_CTRL2_REG, &temp_value);
		sp_write_reg(TX_P0, SP_TX_SYS_CTRL2_REG, temp_value);
		sp_read_reg(TX_P0, SP_TX_SYS_CTRL2_REG, &temp_value);

		if (temp_value & CHA_STA) {
			pr_info("%s %s : Stream clock not stable!\n", LOG_TAG,
				__func__);
		} else {
			sp_read_reg(TX_P0, SP_TX_SYS_CTRL3_REG, &temp_value);
			sp_write_reg(TX_P0, SP_TX_SYS_CTRL3_REG, temp_value);
			sp_read_reg(TX_P0, SP_TX_SYS_CTRL3_REG, &temp_value);

			if (!(temp_value & STRM_VALID)) {
				pr_err("%s %s : video stream not valid!\n",
				       LOG_TAG, __func__);
			} else {
#ifdef DEMO_4K_2K

				if (down_sample_en)
					sp_tx_vo_state = VO_FINISH;
				else
#endif
					{
						if(sp_tx_test_lt) {
							sp_tx_test_lt=0;
							sp_tx_vo_state = VO_FINISH;
						} else
							//sp_tx_vo_state = VO_CHECK_VIDEO_INFO;
							sp_tx_vo_state = VO_FINISH;
					}
			}
		}

		SP_BREAK(VO_WAIT_TX_VIDEO_STABLE, sp_tx_vo_state);
		/*
	case VO_WAIT_PLL_LOCK:
		if (!sp_tx_get_pll_lock_status()) {

			sp_read_reg(TX_P0, SP_TX_PLL_CTRL_REG, &temp_value);
			temp_value |= PLL_RST;
			sp_write_reg(TX_P0, SP_TX_PLL_CTRL_REG, temp_value);
			temp_value &= ~PLL_RST;
			sp_write_reg(TX_P0, SP_TX_PLL_CTRL_REG, temp_value);
			pr_info("%s %s : PLL not lock!\n", LOG_TAG, __func__);
		}else
			sp_tx_vo_state = VO_CHECK_BW;

	SP_BREAK(VO_WAIT_PLL_LOCK, sp_tx_vo_state);
*/
	case VO_CHECK_VIDEO_INFO:
			//temp_value = __i2c_read_byte(RX_P0, HDMI_STATUS) & HDMI_MODE;
			///g_hdmi_dvi_status = read_dvi_hdmi_mode();
			if (!sp_tx_bw_lc_sel(sp_tx_pclk_calc())) 
				sp_tx_vo_state++;
			else{
			sp_tx_set_sys_state(STATE_LINK_TRAINING);
		}

		SP_BREAK(VO_CHECK_VIDEO_INFO, sp_tx_vo_state);
	case VO_FINISH:

		slimport_block_power_ctrl(SP_TX_PWR_AUDIO, SP_POWER_DOWN);
		hdmi_rx_mute_video(0);
		/* sp_tx_lvttl_bit_mapping(); */

		sp_tx_video_mute(0);
		/* sp_tx_set_colorspace(); */

		/* sp_tx_avi_setup(); */

		/* sp_tx_config_packets(AVI_PACKETS);  */

		/* sp_tx_enable_video_input(1); */

		sp_tx_show_infomation();
		goto_next_system_state();
		break;
	}

}

/******************End Output video process********************/
/******************Start HDCP process********************/
static void sp_tx_hdcp_encryption_disable(void)
{
	sp_write_reg_and(TX_P0, TX_HDCP_CTRL0, ~ENC_EN);
}

static void sp_tx_hdcp_encryption_enable(void)
{
	sp_write_reg_or(TX_P0, TX_HDCP_CTRL0, ENC_EN);
}

static void sp_tx_hw_hdcp_enable(void)
{
	unsigned char c;

	sp_write_reg_and(TX_P0, TX_HDCP_CTRL0, (~ENC_EN) & (~HARD_AUTH_EN));
	sp_write_reg_or(TX_P0, TX_HDCP_CTRL0,
			HARD_AUTH_EN | BKSV_SRM_PASS | KSVLIST_VLD | ENC_EN);

	sp_read_reg(TX_P0, TX_HDCP_CTRL0, &c);
	pr_info("%s %s : TX_HDCP_CTRL0 = %.2x\n", LOG_TAG, __func__, (unsigned int) c);
	sp_write_reg(TX_P0, SP_TX_WAIT_R0_TIME, 0xb0);
	sp_write_reg(TX_P0, SP_TX_WAIT_KSVR_TIME, 0xc8);

	pr_info("%s %s : Hardware HDCP is enabled.\n", LOG_TAG, __func__);

}

void slimport_hdcp_process(void)
{

	/* unsigned char c; */
	switch (HDCP_state) {
	case HDCP_CAPABLE_CHECK:
		ds_vid_stb_cntr = 0;
		HDCP_fail_count = 0;
		if(is_anx_dongle())
			HDCP_state = HDCP_WAITTING_VID_STB;
		else
			HDCP_state = HDCP_HW_ENABLE;		
		if (external_block_en == 0) {
			if (slimport_hdcp_cap_check() == 0)
				HDCP_state = HDCP_NOT_SUPPORT;
		}
		/******************************************
		Just for debug, pin: P2-2
		There is a switch to disable/enable HDCP.
		******************************************/
		/* if(HDCP_EN == 0)
			HDCP_state = HDCP_NOT_SUPPORT; */
		/*****************************************/
		SP_BREAK(HDCP_CAPABLE_CHECK, HDCP_state);

	case HDCP_WAITTING_VID_STB:
		msleep(100);
		HDCP_state = HDCP_HW_ENABLE;
		SP_BREAK(HDCP_WAITTING_VID_STB, HDCP_state);
	case HDCP_HW_ENABLE:
		sp_tx_video_mute(1);
		sp_tx_clean_hdcp_status();
		slimport_block_power_ctrl(SP_TX_PWR_HDCP, SP_POWER_DOWN);
		msleep(20);
		slimport_block_power_ctrl(SP_TX_PWR_HDCP, SP_POWER_ON);
		sp_write_reg(TX_P2, SP_COMMON_INT_MASK2, 0x01);
		/* sp_tx_video_mute(0); */

		/* sp_tx_aux_polling_disable(); */

		/* sp_tx_clean_hdcp_status(); */

		mdelay(50);
		/* disable auto polling during hdcp. */

		sp_tx_hw_hdcp_enable();
		HDCP_state = HDCP_WAITTING_FINISH;
	case HDCP_WAITTING_FINISH:
		break;
	case HDCP_FINISH:
		sp_tx_hdcp_encryption_enable();
		hdmi_rx_mute_video(0);
		sp_tx_video_mute(0);
		/* enable auto polling after hdcp. */

		/* sp_tx_aux_polling_enable(); */

		goto_next_system_state();
		HDCP_state = HDCP_CAPABLE_CHECK;
		pr_info("%s %s : @@@@@@@hdcp_auth_pass@@@@@@\n", LOG_TAG,
			__func__);
		break;
	case HDCP_FAILE:

		if (HDCP_fail_count > 5) {
			/* vbus_power_ctrl(0); */
			reg_hardware_reset();
			HDCP_state = HDCP_CAPABLE_CHECK;
			HDCP_fail_count = 0;
			pr_info("%s %s : *********hdcp_auth_failed*********\n",
				LOG_TAG, __func__);
		} else {
			HDCP_fail_count++;
			HDCP_state = HDCP_WAITTING_VID_STB;
		}

		break;
	default:
	case HDCP_NOT_SUPPORT:
		pr_info("%s %s : Sink is not capable HDCP\n", LOG_TAG,
			__func__);
		slimport_block_power_ctrl(SP_TX_PWR_HDCP, SP_POWER_DOWN);
		sp_tx_video_mute(0);
		/* sp_tx_aux_polling_enable(); */

		goto_next_system_state();
		HDCP_state = HDCP_CAPABLE_CHECK;
		break;
	}
}
/******************End HDCP process********************/
/******************Start Audio process********************/
static void sp_tx_audioinfoframe_setup(void)
{
	int i;
	unsigned char c;
	sp_read_reg(RX_P1, HDMI_RX_AUDIO_TYPE_REG, &c);
	sp_tx_audioinfoframe.type = c;
	sp_read_reg(RX_P1, HDMI_RX_AUDIO_VER_REG, &c);
	sp_tx_audioinfoframe.version = c;
	sp_read_reg(RX_P1, HDMI_RX_AUDIO_LEN_REG, &c);
	sp_tx_audioinfoframe.length = c;

	for (i = 0; i < 11; i++) {
		sp_read_reg(RX_P1, (HDMI_RX_AUDIO_DATA00_REG + i), &c);
		sp_tx_audioinfoframe.pb_byte[i] = c;
	}

}

static void sp_tx_enable_audio_output(unsigned char benable)
{
	unsigned char c;

	sp_read_reg(TX_P0, SP_TX_AUD_CTRL, &c);

	if (benable) {

		/* if it has been enabled, disable first. */
		if (c & AUD_EN) {
			c &= ~AUD_EN;
			sp_write_reg(TX_P0, SP_TX_AUD_CTRL, c);
		}

		sp_tx_audioinfoframe_setup();
		sp_tx_config_packets(AUDIF_PACKETS);

		sp_read_reg(RX_P0, HDMI_STATUS, &c);
		/*patch for JIRA issue CLD-200*/
		if (c & HDMI_AUD_LAYOUT) {
			sp_read_reg(RX_P1, HDMI_RX_AUDIO_DATA00_REG, &c);

			sp_write_reg(TX_P2, SP_TX_AUD_CH_NUM_REG5,  (c&0x07)<<5 |AUD_LAYOUT);
		} else {
			sp_write_reg(TX_P2, SP_TX_AUD_CH_NUM_REG5, 0x20 & (~AUD_LAYOUT));
		}

		c |= AUD_EN;
		sp_write_reg(TX_P0, SP_TX_AUD_CTRL, c);

	} else {
		c &= ~AUD_EN;
		sp_write_reg(TX_P0, SP_TX_AUD_CTRL, c);
		sp_write_reg_and(TX_P0, SP_TX_PKT_EN_REG, ~AUD_IF_EN);
	}

}

static void sp_tx_config_audio(void)
{
	unsigned char c;
	int i;
	unsigned long M_AUD, LS_Clk = 0;
	unsigned long AUD_Freq = 0;
	pr_info("%s %s : **Config audio **\n", LOG_TAG, __func__);
	slimport_block_power_ctrl(SP_TX_PWR_AUDIO, SP_POWER_ON);
	sp_read_reg(RX_P0, 0xCA, &c);

	switch (c & 0x0f) {
	case 0x00:
		AUD_Freq = 44.1;
		break;
	case 0x02:
		AUD_Freq = 48;
		break;
	case 0x03:
		AUD_Freq = 32;
		break;
	case 0x08:
		AUD_Freq = 88.2;
		break;
	case 0x0a:
		AUD_Freq = 96;
		break;
	case 0x0c:
		AUD_Freq = 176.4;
		break;
	case 0x0e:
		AUD_Freq = 192;
		break;
	default:
		break;
	}

	switch (sp_tx_get_link_bw()) {
	case LINK_1P62G:
		LS_Clk = 162000;
		break;
	case LINK_2P7G:
		LS_Clk = 270000;
		break;
	case LINK_5P4G:
		LS_Clk = 540000;
		break;
	case LINK_6P75G:
		LS_Clk = 675000;
		break;
	default:
		break;
	}

	pr_info("%s %s : AUD_Freq = %ld , LS_CLK = %ld\n", LOG_TAG, __func__,
		AUD_Freq, LS_Clk);

	M_AUD = ((512 * AUD_Freq) / LS_Clk) * 32768;
	M_AUD = M_AUD + 0x05;
	sp_write_reg(TX_P1, SP_TX_AUD_INTERFACE_CTRL4, (M_AUD & 0xff));
	M_AUD = M_AUD >> 8;
	sp_write_reg(TX_P1, SP_TX_AUD_INTERFACE_CTRL5, (M_AUD & 0xff));
	sp_write_reg(TX_P1, SP_TX_AUD_INTERFACE_CTRL6, 0x00);

	sp_write_reg_and(TX_P1, SP_TX_AUD_INTERFACE_CTRL0,
			 ~AUD_INTERFACE_DISABLE);

	sp_write_reg_or(TX_P1, SP_TX_AUD_INTERFACE_CTRL2, M_AUD_ADJUST_ST);


	/* transfer audio chaneel status from HDMI Rx to Slinmport Tx */
	for (i = 0; i < 5; i++) {
		sp_read_reg(RX_P0, (HDMI_RX_AUD_IN_CH_STATUS1_REG + i), &c);
		sp_write_reg(TX_P2, (SP_TX_AUD_CH_STATUS_REG1 + i), c);
	}

	/* enable audio */
	sp_tx_enable_audio_output(1);
	/*
	   sp_read_reg(TX_P2, SP_COMMON_INT_MASK1, &c);
	   c |= 0x04;
	   sp_write_reg(TX_P2, SP_COMMON_INT_MASK1, c);
	 */

}

void slimport_config_audio_output(void)
{
	switch (sp_tx_ao_state) {
	default:
	case AO_INIT:
	case AO_CTS_RCV_INT:
	case AO_AUDIO_RCV_INT:

		if (!(__i2c_read_byte(RX_P0, HDMI_STATUS) & HDMI_MODE)) {
			sp_tx_ao_state = AO_INIT;
			goto_next_system_state();
		}

		break;
	case AO_RCV_INT_FINISH:

		if (audio_stable_count++ > 2)
			sp_tx_ao_state = AO_OUTPUT;
		else
			sp_tx_ao_state = AO_INIT;

		SP_BREAK(AO_INIT, sp_tx_ao_state);
	case AO_OUTPUT:
		audio_stable_count = 0;
		sp_tx_ao_state = AO_INIT;
		sp_tx_video_mute(0);
		hdmi_rx_mute_audio(0);
		sp_tx_config_audio();
		goto_next_system_state();
		break;
	}

}
/******************End Audio process********************/
void slimport_initialization(void)
{
	/*Waitting Hot plug event!
	if (!(COMMON_INT4 & PLUG)){
		return;
	}*/
	#ifdef ENABLE_READ_EDID
	g_read_edid_flag = 0;
	#endif
	/*Power on all modules*/
	sp_write_reg(TX_P2, SP_POWERD_CTRL_REG, 0x00);
	/*Driver Version*/
	sp_write_reg(TX_P1, FW_VER_REG, FW_VERSION);
	/*vbus_power_ctrl(1);*/
	hdmi_rx_initialization();
	sp_tx_initialization();
	msleep(200);
	/* sp_tx_aux_polling_enable(); */
	goto_next_system_state();
}

void hdcp_external_ctrl_flag_monitor(void)
{
	if (external_block_en != external_block_en_bak) {
		external_block_en_bak = external_block_en;
		system_state_change_with_case(STATE_HDCP_AUTH);
	}
}

/*
 * Apple VGA dongle
 * [  431.720698]  (2)[2109:kworker/u8:0]asus_check_sink_status: DPCD 000001h = 0xa (ret=0)
 * [  431.725792]  (2)[2109:kworker/u8:0]asus_check_sink_status: DPCD 000002h = 0x82 (ret=0)
 * [  431.730671]  (2)[2109:kworker/u8:0]asus_check_sink_status: DPCD 000005h = 0x3 (ret=0)
 *
 * ASUS HDMI dongle
 * [  461.479196]  (0)[2155:kworker/u8:2]asus_check_sink_status: DPCD 000001h = 0x14 (ret=0)
 * [  461.484846]  (0)[2155:kworker/u8:2]asus_check_sink_status: DPCD 000002h = 0xc4 (ret=0)
 * [  461.489763]  (0)[2155:kworker/u8:2]asus_check_sink_status: DPCD 000005h = 0x15 (ret=0)
 *
 */

int asus_check_link_rate = 0;
int asus_check_lane_count = 0;
int asus_check_port_type = 0;

bool asus_dongle_support_1080p(void){
	bool result = false;
	if((asus_check_link_rate <= 0xa) && (asus_check_lane_count <= 0x2)){
		result = false;
	} else {
		result = true;
	}

	printk("asus_dongle_support_1080p: result = %d\n", result);
	return result;
}
EXPORT_SYMBOL(asus_dongle_support_1080p);

void asus_check_sink_status(void){
	unsigned char data_buf;
	unsigned char aux_status = 0;

	// MAX_LINK_RATE: Bits 7:0
	aux_status = sp_tx_aux_dpcdread_bytes(0x00, 0x00, DPCD_MAX_LINK_RATE, 1, &data_buf);
	if(!aux_status){
		asus_check_link_rate = data_buf & 0xFF;
		printk("asus_check_sink_status: asus_check_link_rate = 0x%x\n", asus_check_link_rate);
	}
	printk("asus_check_sink_status: DPCD 000001h = 0x%x (ret=%d)\n", data_buf, aux_status);

	// MAX_LANE_COUNT: Bits 4:0
	aux_status = sp_tx_aux_dpcdread_bytes(0x00, 0x00, DPCD_MAX_LANE_COUNT, 1, &data_buf);
	if(!aux_status){
		asus_check_lane_count = data_buf & 0x1F;
		printk("asus_check_sink_status: asus_check_lane_count = 0x%x\n", asus_check_lane_count);
	}
	printk("asus_check_sink_status: DPCD 000002h = 0x%x (ret=%d)\n", data_buf, aux_status);

	// DWN_STRM_PORT_TYPE: Bits 2:1
	aux_status = sp_tx_aux_dpcdread_bytes(0x00, 0x00, DPCD_DSPORT_PRESENT, 1, &data_buf);
	if(!aux_status){
		asus_check_port_type = (data_buf>>1) & 0x3;
		printk("asus_check_sink_status: asus_check_port_type = 0x%x\n", asus_check_port_type);

		if(asus_check_port_type == 0x1){
			external_block_en = 0;
		} else {
			external_block_en = 1;
		}
	}
	printk("asus_check_sink_status: DPCD 000005h = 0x%x (ret=%d)\n", data_buf, aux_status);

	return;
}

void slimport_state_process(void)
{

	switch (sp_tx_system_state) {
	case STATE_WAITTING_CABLE_PLUG:
		slimport_waitting_cable_plug_process();
		SP_BREAK(STATE_WAITTING_CABLE_PLUG, sp_tx_system_state);
	case STATE_SP_INITIALIZED:
		slimport_initialization();
		SP_BREAK(STATE_SP_INITIALIZED, sp_tx_system_state);
	case STATE_SINK_CONNECTION:
		slimport_sink_connection();
		SP_BREAK(STATE_SINK_CONNECTION, sp_tx_system_state);
#ifdef ENABLE_READ_EDID
	case STATE_PARSE_EDID:
		asus_check_sink_status();
		slimport_edid_process();
		SP_BREAK(STATE_PARSE_EDID, sp_tx_system_state);
#endif
	case STATE_LINK_TRAINING:
		slimport_link_training();
		SP_BREAK(STATE_LINK_TRAINING, sp_tx_system_state);
	case STATE_VIDEO_OUTPUT:
		slimport_config_video_output();
		SP_BREAK(STATE_VIDEO_OUTPUT, sp_tx_system_state);
	case STATE_HDCP_AUTH:

		if (!HDCP_REPEATER_MODE) {
			slimport_hdcp_process();
			SP_BREAK(STATE_HDCP_AUTH, sp_tx_system_state);
		} else {
			goto_next_system_state();
		}

	case STATE_AUDIO_OUTPUT:
		slimport_config_audio_output();
		SP_BREAK(STATE_AUDIO_OUTPUT, sp_tx_system_state);
	case STATE_PLAY_BACK:
		/* slimport_playback_process();  */

		SP_BREAK(STATE_PLAY_BACK, sp_tx_system_state);
	default:
		break;
	}

}

/******************Start INT process********************/
void sp_tx_int_rec(void)
{
	sp_read_reg(TX_P2, SP_COMMON_INT_STATUS1, &COMMON_INT1);
	sp_write_reg(TX_P2, SP_COMMON_INT_STATUS1, COMMON_INT1);

	sp_read_reg(TX_P2, SP_COMMON_INT_STATUS1 + 1, &COMMON_INT2);
	sp_write_reg(TX_P2, SP_COMMON_INT_STATUS1 + 1, COMMON_INT2);

	sp_read_reg(TX_P2, SP_COMMON_INT_STATUS1 + 2, &COMMON_INT3);
	sp_write_reg(TX_P2, SP_COMMON_INT_STATUS1 + 2, COMMON_INT3);

	sp_read_reg(TX_P2, SP_COMMON_INT_STATUS1 + 3, &COMMON_INT4);
	sp_write_reg(TX_P2, SP_COMMON_INT_STATUS1 + 3, COMMON_INT4);

	sp_read_reg(TX_P2, SP_COMMON_INT_STATUS1 + 6, &COMMON_INT5);
	sp_write_reg(TX_P2, SP_COMMON_INT_STATUS1 + 6, COMMON_INT5);

}

void hdmi_rx_int_rec(void)
{
	/*       if((hdmi_system_state < HDMI_CLOCK_DET)||(sp_tx_system_state <STATE_CONFIG_HDMI)) */

	/*    return;    */

	sp_read_reg(RX_P0, HDMI_RX_INT_STATUS1_REG, &HDMI_RX_INT1);
	sp_write_reg(RX_P0, HDMI_RX_INT_STATUS1_REG, HDMI_RX_INT1);
	sp_read_reg(RX_P0, HDMI_RX_INT_STATUS2_REG, &HDMI_RX_INT2);
	sp_write_reg(RX_P0, HDMI_RX_INT_STATUS2_REG, HDMI_RX_INT2);
	sp_read_reg(RX_P0, HDMI_RX_INT_STATUS3_REG, &HDMI_RX_INT3);
	sp_write_reg(RX_P0, HDMI_RX_INT_STATUS3_REG, HDMI_RX_INT3);
	sp_read_reg(RX_P0, HDMI_RX_INT_STATUS4_REG, &HDMI_RX_INT4);
	sp_write_reg(RX_P0, HDMI_RX_INT_STATUS4_REG, HDMI_RX_INT4);
	sp_read_reg(RX_P0, HDMI_RX_INT_STATUS5_REG, &HDMI_RX_INT5);
	sp_write_reg(RX_P0, HDMI_RX_INT_STATUS5_REG, HDMI_RX_INT5);
	sp_read_reg(RX_P0, HDMI_RX_INT_STATUS6_REG, &HDMI_RX_INT6);
	sp_write_reg(RX_P0, HDMI_RX_INT_STATUS6_REG, HDMI_RX_INT6);
	sp_read_reg(RX_P0, HDMI_RX_INT_STATUS7_REG, &HDMI_RX_INT7);
	sp_write_reg(RX_P0, HDMI_RX_INT_STATUS7_REG, HDMI_RX_INT7);
}

void slimport_int_rec(void)
{
	sp_tx_int_rec();
	hdmi_rx_int_rec();
	/*
	   if(!sp_tx_pd_mode ){
	   sp_tx_int_irq_handler();
	   hdmi_rx_int_irq_handler();
	   }
	 */
}

/******************End INT process********************/
/******************Start task process********************/

static void sp_tx_pll_changed_int_handler(void)
{
	if (sp_tx_system_state >= STATE_LINK_TRAINING) {
		if (!sp_tx_get_pll_lock_status()) {
			pr_info("%s %s : PLL:PLL not lock!\n", LOG_TAG,
				__func__);
			sp_tx_set_sys_state(STATE_LINK_TRAINING);
		}
	}
}

static void sp_tx_hdcp_link_chk_fail_handler(void)
{
	if (!HDCP_REPEATER_MODE)
		//system_state_change_with_case(STATE_HDCP_AUTH);
	system_state_change_with_case(STATE_LINK_TRAINING);

	pr_info("%s %s : hdcp_link_chk_fail:HDCP Sync lost!\n", LOG_TAG,
		__func__);
}

void link_down_check(void)
{
	unsigned char data_buf[6];
	pr_info("%s %s : link_down_check\n", LOG_TAG, __func__);
	sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x00, 6, data_buf);
	if ((sp_tx_system_state > STATE_LINK_TRAINING)) {
		if(!(data_buf[4] & 0x01) || ( (data_buf[2] & (0x01 | 0x04)) != 0x05)){ //link down flag
			if (sp_tx_get_downstream_connection()) {
				sp_tx_set_sys_state(STATE_LINK_TRAINING);
				pr_info("%s %s : INT:re-LT request!\n", LOG_TAG,
					__func__);
			} else {
				/* vbus_power_ctrl(0); */
				reg_hardware_reset();
			}
		} else {
			pr_info("%s %s : Lane align %x\n", LOG_TAG, __func__, (unsigned int)data_buf[4]);
			pr_info("%s %s : Lane clock recovery %x\n", LOG_TAG, __func__, (unsigned int)data_buf[2]);
		}
	}
}

static void sp_tx_phy_auto_test(void)
{
	unsigned char b_sw;
	unsigned char c1;
	unsigned char bytebuf[16];
	unsigned char link_bw;

	/*DPCD 0x219 TEST_LINK_RATE */
	sp_tx_aux_dpcdread_bytes(0x0, 0x02, 0x19, 1, bytebuf);
	pr_info("%s %s : DPCD:0x00219 = %.2x\n", LOG_TAG, __func__,
		(unsigned int) bytebuf[0]);

	switch (bytebuf[0]) {
	case 0x06:
	case 0x0A:
	case 0x14:
	case 0x19:
		sp_tx_set_link_bw(bytebuf[0]);
		sp_tx_test_bw = bytebuf[0];
		break;
	default:
		sp_tx_set_link_bw(0x19);
		sp_tx_test_bw = 0x19;
		break;
	}

	/*DPCD 0x248 PHY_TEST_PATTERN */
	sp_tx_aux_dpcdread_bytes(0x0, 0x02, 0x48, 1, bytebuf);
	pr_info("%s %s : DPCD:0x00248 = %.2x\n", LOG_TAG, __func__,
		(unsigned int) bytebuf[0]);

	switch (bytebuf[0]) {
	case 0:
		/* pr_info("%s %s : No test pattern selected\n", LOG_TAG, __func__);  */

		break;
	case 1:
		sp_write_reg(TX_P0, SP_TX_TRAINING_PTN_SET_REG, 0x04);
		/* pr_info("%s %s : D10.2 Pattern\n", LOG_TAG, __func__);  */

		break;
	case 2:
		sp_write_reg(TX_P0, SP_TX_TRAINING_PTN_SET_REG, 0x08);
		/* pr_info("%s %s : Symbol Error Measurement Count\n", LOG_TAG, __func__);  */

		break;
	case 3:
		sp_write_reg(TX_P0, SP_TX_TRAINING_PTN_SET_REG, 0x0c);
		/* pr_info("%s %s : PRBS7 Pattern\n", LOG_TAG, __func__);  */

		break;
	case 4:
		sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x50, 0xa, bytebuf);
		sp_write_reg(TX_P1, 0x80, bytebuf[0]);
		sp_write_reg(TX_P1, 0x81, bytebuf[1]);
		sp_write_reg(TX_P1, 0x82, bytebuf[2]);
		sp_write_reg(TX_P1, 0x83, bytebuf[3]);
		sp_write_reg(TX_P1, 0x84, bytebuf[4]);
		sp_write_reg(TX_P1, 0x85, bytebuf[5]);
		sp_write_reg(TX_P1, 0x86, bytebuf[6]);
		sp_write_reg(TX_P1, 0x87, bytebuf[7]);
		sp_write_reg(TX_P1, 0x88, bytebuf[8]);
		sp_write_reg(TX_P1, 0x89, bytebuf[9]);
		sp_write_reg(TX_P0, SP_TX_TRAINING_PTN_SET_REG, 0x30);
		/* pr_info("%s %s : 80bit custom pattern transmitted\n", LOG_TAG, __func__);  */

		break;
	case 5:
		sp_write_reg(TX_P0, 0xA9, 0x00);
		sp_write_reg(TX_P0, 0xAA, 0x01);
		sp_write_reg(TX_P0, SP_TX_TRAINING_PTN_SET_REG, 0x14);
		/* pr_info("%s %s : HBR2 Compliance Eye Pattern\n", LOG_TAG, __func__);  */

		break;
	}

	sp_tx_aux_dpcdread_bytes(0x00, 0x00, 0x03, 1, bytebuf);
	pr_info("%s %s : DPCD:0x00003 = %.2x\n", LOG_TAG, __func__,
		(unsigned int) bytebuf[0]);

	switch (bytebuf[0] & 0x01) {
	case 0:
		sp_tx_spread_enable(0);
		/* pr_info("%s %s : SSC OFF\n", LOG_TAG, __func__);  */

		break;
	case 1:
		sp_read_reg(TX_P0, SP_TX_LINK_BW_SET_REG, &c1);

		switch (c1) {
		case 0x06:
			link_bw = 0x06;
			break;
		case 0x0a:
			link_bw = 0x0a;
			break;
		case 0x14:
			link_bw = 0x14;
			break;
		case 0x19:
			link_bw = 0x19;
			break;
		default:
			link_bw = 0x00;
			break;
		}

		sp_tx_config_ssc(SSC_DEP_4000PPM);
		/* pr_info("%s %s : SSC ON\n", LOG_TAG, __func__);  */

		break;
	}

	/*get swing and emphasis adjust request */
	sp_read_reg(TX_P0, 0xA3, &b_sw);

	sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x06, 1, bytebuf);
	pr_info("%s %s : DPCD:0x00206 = %.2x\n", LOG_TAG, __func__,
		(unsigned int) bytebuf[0]);
	c1 = bytebuf[0] & 0x0f;

	switch (c1) {
	case 0x00:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x00);
		/* pr_info("%s %s : lane0,Swing0, emp 0db.\n", LOG_TAG, __func__);  */

		break;
	case 0x01:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x01);
		/* pr_info("%s %s : lane0,Swing1, emp 0db.\n", LOG_TAG, __func__);  */

		break;
	case 0x02:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x02);
		/* pr_info("%s %s : lane0,Swing2, emp 0db.\n", LOG_TAG, __func__);  */

		break;
	case 0x03:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x03);
		/* pr_info("%s %s : lane0,Swing3, emp 0db.\n", LOG_TAG, __func__);  */

		break;
	case 0x04:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x08);
		/* pr_info("%s %s : lane0,Swing0, emp 3.5db.\n", LOG_TAG, __func__);  */

		break;
	case 0x05:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x09);
		/* pr_info("%s %s : lane0,Swing1, emp 3.5db.\n", LOG_TAG, __func__);  */

		break;
	case 0x06:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x0a);
		/* pr_info("%s %s : lane0,Swing2, emp 3.5db.\n", LOG_TAG, __func__);  */

		break;
	case 0x08:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x10);
		/* pr_info("%s %s : lane0,Swing0, emp 6db.\n", LOG_TAG, __func__);  */

		break;
	case 0x09:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x11);
		/* pr_info("%s %s : lane0,Swing1, emp 6db.\n", LOG_TAG, __func__);  */

		break;
	case 0x0c:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x18);
		/* pr_info("%s %s : lane0,Swing0, emp 9.5db.\n", LOG_TAG, __func__);  */

		break;
	default:
		break;
	}
}

static void hpd_irq_process(void)
{
	unsigned char c, c1;
	unsigned char test_vector;
	unsigned char data_buf[6];
	sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x00, 6, data_buf);
	pr_info("+++++++++++++Get HPD IRQ %x\n", (int)data_buf[1]);
	
	if(data_buf[1] != 0)
		sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, DPCD_SERVICE_IRQ_VECTOR, 1, &(data_buf[1]));

	/* HDCP IRQ */	
	if ((data_buf[1] & CP_IRQ)
	    && ((HDCP_state > HDCP_WAITTING_FINISH)
		|| (sp_tx_system_state >= STATE_HDCP_AUTH))) {
		sp_tx_aux_dpcdread_bytes(0x06, 0x80, 0x29, 1, &c1);

		if (c1 & 0x04) {
			if (!HDCP_REPEATER_MODE) {
				system_state_change_with_case(STATE_HDCP_AUTH);
				sp_tx_clean_hdcp_status();
			} else
				assisted_HDCP_repeater = HDCP_ERROR;

			pr_info("%s %s :CP_IRQ, HDCP sync lost.\n", LOG_TAG,
				__func__);
		}
	}

	/* AUTOMATED TEST IRQ */
	if (data_buf[1] & TEST_IRQ) {
		sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x18, 1, &test_vector);

		/* test link training */
		if (test_vector & 0x01) {
			sp_tx_test_lt = 1;

			sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x19, 1, &c);

			switch (c) {
			case 0x06:
			case 0x0A:
			case 0x14:
			case 0x19:
				sp_tx_set_link_bw(c);
				sp_tx_test_bw = c;
				break;
			default:
				sp_tx_set_link_bw(0x19);
				sp_tx_test_bw = 0x19;
				break;
			}

			pr_info("%s %s :  test_bw = %.2x\n", LOG_TAG, __func__,
				(unsigned int) sp_tx_test_bw);

			sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x60, 1, &c);
			c = c | TEST_ACK;
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, 0x60, 1, &c);

			pr_info("%s %s : Set TEST_ACK!\n", LOG_TAG, __func__);

			if (sp_tx_system_state >= STATE_LINK_TRAINING) {
				sp_tx_LT_state = LT_INIT;
				sp_tx_set_sys_state(STATE_LINK_TRAINING);
			}

			pr_info("%s %s : IRQ:test-LT request!\n", LOG_TAG,
				__func__);
		}

		if (test_vector & 0x02) {
			/* pr_info("-------------------test pattern\n"); */

			sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x60, 1, &c);
			c = c | TEST_ACK;
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, 0x60, 1, &c);
		}

		/* test edid */
		if (test_vector & 0x04) {
			if (sp_tx_system_state > STATE_PARSE_EDID)
				sp_tx_set_sys_state(STATE_PARSE_EDID);

			sp_tx_test_edid = 1;
			pr_info("%s %s : Test EDID Requested!\n", LOG_TAG,
				__func__);
		}

		/* phy test pattern */
		if (test_vector & 0x08) {
			sp_tx_test_lt = 1;

			sp_tx_phy_auto_test();

			sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x60, 1, &c);
			c = c | 0x01;
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, 0x60, 1, &c);
			/*
			   sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x60, 1, &c);
			   while((c & 0x03) == 0){
			   c = c | 0x01;
			   sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x60, 1, &c);
			   sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x60, 1, &c);
			   } */
		}
	}
		
	if((sp_tx_system_state > STATE_LINK_TRAINING)) {
		if (((sp_tx_system_state == STATE_HDCP_AUTH)&&(data_buf[1] & CP_IRQ)))
			{pr_info("%s %s : CP_IRQ!\n", LOG_TAG, __func__);}
		else
		if(!(data_buf[4] & 0x01) || ( (data_buf[2] & (0x01 | 0x04)) != 0x05)){ //link down flag
			sp_tx_set_sys_state(STATE_LINK_TRAINING);
			pr_info("%s %s : INT:re-LT request!\n", LOG_TAG, __func__);
			return;
		} else {
			pr_info("%s %s : Lane align %x\n", LOG_TAG, __func__, (unsigned int)data_buf[4]);
			pr_info("%s %s : Lane clock recovery %x\n", LOG_TAG, __func__, (unsigned int)data_buf[2]);
		}
	}
}
static void sp_tx_vsi_setup(void)
{
	unsigned char c;
	int i;

	for (i = 0; i < 10; i++) {
		sp_read_reg(RX_P1, (HDMI_RX_MPEG_DATA00_REG + i), &c);
		sp_tx_packet_mpeg.MPEG_data[i] = c;
	}
}

static void sp_tx_mpeg_setup(void)
{
	unsigned char c;
	int i;

	for (i = 0; i < 10; i++) {
		sp_read_reg(RX_P1, (HDMI_RX_MPEG_DATA00_REG + i), &c);
		sp_tx_packet_mpeg.MPEG_data[i] = c;
	}
}

static void sp_tx_auth_done_int_handler(void)
{
	unsigned char bytebuf[2];

	if (HDCP_REPEATER_MODE) {
		if ((__i2c_read_byte(TX_P0, SP_TX_HDCP_STATUS) &
		     SP_TX_HDCP_AUTH_PASS)
		    && (assisted_HDCP_repeater == HDCP_DOING))
			assisted_HDCP_repeater = HDCP_DONE;
		else
			assisted_HDCP_repeater = HDCP_ERROR;

		return;
	}

	if (HDCP_state > HDCP_HW_ENABLE
	    && sp_tx_system_state == STATE_HDCP_AUTH) {
		/*patch for JIRA issue CLD-221*/
		sp_read_reg(TX_P0, 0x1b, &bytebuf[0]);
		sp_read_reg(TX_P0, 0x1c, &bytebuf[1]);
		/* max cascade exceeded or max devs exceed, disable encryption */
		if ((bytebuf[1] & 0x08)||(bytebuf[0]&0x80)){
				pr_info("%s %s : max cascade/devs exceeded!\n", LOG_TAG, __func__);
			/* sp_tx_video_mute(1); */
			/* sp_tx_clean_hdcp_status(); */
			sp_tx_hdcp_encryption_disable();

			HDCP_state = HDCP_FINISH;
			/* HDCP_state = HDCP_NOT_SUPPORT; */
		} else {
		sp_read_reg(TX_P0, SP_TX_HDCP_STATUS, bytebuf);

		if (bytebuf[0] & SP_TX_HDCP_AUTH_PASS) {
			sp_tx_aux_dpcdread_bytes(0x06, 0x80, 0x2A, 2, bytebuf);
			/* max cascade exceeded or max devs exceed,
			 disable encryption */

			if ((bytebuf[1] & 0x08) || (bytebuf[0] & 0x80)) {
				pr_info("%s %s : max cascade/devs exceeded!\n",
					LOG_TAG, __func__);
				sp_tx_hdcp_encryption_disable();
			} else
				pr_info
				    ("%s %s : Auth pass in Auth_Done\n",
				     LOG_TAG, __func__);

			HDCP_state = HDCP_FINISH;

		} else {
			pr_err("%s %s : Authentication failed in AUTH_done\n",
			       LOG_TAG, __func__);
			sp_tx_video_mute(1);
			sp_tx_clean_hdcp_status();
			HDCP_state = HDCP_FAILE;
		}
		}
	}
	pr_info("%s %s : sp_tx_auth_done_int_handler\n", LOG_TAG, __func__);

}

static void sp_tx_lt_done_int_handler(void)
{
	unsigned char c;

	if (sp_tx_LT_state == LT_WAITTING_FINISH
	    && sp_tx_system_state == STATE_LINK_TRAINING) {
		sp_read_reg(TX_P0, LT_CTRL, &c);

		if (c & 0x70) {
			c = (c & 0x70) >> 4;
			pr_info
			    ("%s %s :LT failed in interrupt, ERR code = %.2x\n",
			     LOG_TAG, __func__, (unsigned int) c);
			sp_tx_LT_state = LT_ERROR;
		} else {
			pr_info("%s %s : lt_done: LT Finish\n", LOG_TAG,
				__func__);
			sp_tx_LT_state = LT_FINISH;
		}
	}

}

static void hdmi_rx_clk_det_int(void)
{
	pr_info("%s %s : *HDMI_RX Interrupt: Pixel Clock Change.\n", LOG_TAG,
		__func__);

	if (sp_tx_system_state > STATE_VIDEO_OUTPUT) {
		sp_tx_video_mute(1);
		sp_tx_enable_audio_output(0);
		sp_tx_set_sys_state(STATE_VIDEO_OUTPUT);
	}
}

static void hdmi_rx_sync_det_int(void)
{
	pr_info("%s %s : *HDMI_RX Interrupt: Sync Detect.\n", LOG_TAG,
		__func__);
}

static void hdmi_rx_hdmi_dvi_int(void)
{
	unsigned char c;
	pr_info("%s %s : hdmi_rx_hdmi_dvi_int.\n", LOG_TAG, __func__);
	sp_read_reg(RX_P0, HDMI_STATUS, &c);
	g_hdmi_dvi_status = read_dvi_hdmi_mode();

	if ((c & _BIT0) != (g_hdmi_dvi_status & _BIT0)) {
		pr_info("%s %s : hdmi_dvi_int: Is HDMI MODE: %x.\n", LOG_TAG,
			__func__, (unsigned int) (c & HDMI_MODE));
		g_hdmi_dvi_status = (c & _BIT0);
		hdmi_rx_mute_audio(1);
		system_state_change_with_case(STATE_LINK_TRAINING);
	}
}

static void hdmi_rx_new_avi_int(void)
{
	pr_info("%s %s : *HDMI_RX Interrupt: New AVI Packet.\n", LOG_TAG,
		__func__);
	sp_tx_lvttl_bit_mapping();
	sp_tx_set_colorspace();
	sp_tx_avi_setup();
	sp_tx_config_packets(AVI_PACKETS);
}

static void hdmi_rx_new_vsi_int(void)
{
	/* unsigned char c; */

	unsigned char hdmi_video_format, v3d_structure;
	pr_info("%s %s : *HDMI_RX Interrupt: NEW VSI packet.\n", LOG_TAG,
		__func__);

	sp_write_reg_and(TX_P0, SP_TX_3D_VSC_CTRL, ~INFO_FRAME_VSC_EN);

	/*VSI package header */
	if ((__i2c_read_byte(RX_P1, HDMI_RX_MPEG_TYPE_REG) != 0x81)
	    || (__i2c_read_byte(RX_P1, HDMI_RX_MPEG_VER_REG) != 0x01))
		return;

	pr_info("%s %s : Setup VSI package!\n", LOG_TAG, __func__);

	/* use mpeg packet as mail box to send vsi packet */
	sp_tx_vsi_setup();
	sp_tx_config_packets(VSI_PACKETS);

	sp_read_reg(RX_P1, HDMI_RX_MPEG_DATA03_REG, &hdmi_video_format);

	if ((hdmi_video_format & 0xe0) == 0x40) {
		pr_info
		    ("%s %s : 3D VSI packet is detected. Config VSC packet\n",
		     LOG_TAG, __func__);
		sp_read_reg(RX_P1, HDMI_RX_MPEG_DATA05_REG, &v3d_structure);

		switch (v3d_structure & 0xf0) {
			/* frame packing */
		case 0x00:
			v3d_structure = 0x02;
			break;
			/* Line alternative */
		case 0x20:
			v3d_structure = 0x03;
			break;
			/* Side-by-side(full) */
		case 0x30:
			v3d_structure = 0x04;
			break;
		default:
			v3d_structure = 0x00;
			pr_info("%s %s : 3D structure is not supported\n",
				LOG_TAG, __func__);
			break;
		}

		sp_write_reg(TX_P0, SP_TX_VSC_DB1, v3d_structure);
	}

	sp_write_reg_or(TX_P0, SP_TX_3D_VSC_CTRL, INFO_FRAME_VSC_EN);
	sp_write_reg_and(TX_P0, SP_TX_PKT_EN_REG, ~SPD_IF_EN);
	sp_write_reg_or(TX_P0, SP_TX_PKT_EN_REG, SPD_IF_UD);
	sp_write_reg_or(TX_P0, SP_TX_PKT_EN_REG, SPD_IF_EN);
}

static void hdmi_rx_no_vsi_int(void)
{

	unsigned char c;

	sp_read_reg(TX_P0, SP_TX_3D_VSC_CTRL, &c);

	if (c & INFO_FRAME_VSC_EN) {
		pr_info("%s %s : No new VSI is received, disable  VSC packet\n",
			LOG_TAG, __func__);
		c &= ~INFO_FRAME_VSC_EN;
		sp_write_reg(TX_P0, SP_TX_3D_VSC_CTRL, c);
		sp_tx_mpeg_setup();
		sp_tx_config_packets(MPEG_PACKETS);
	}

}

static void hdmi_rx_restart_audio_chk(void)
{
	pr_info("%s %s : WAIT_AUDIO: hdmi_rx_restart_audio_chk.\n", LOG_TAG,
		__func__);
	system_state_change_with_case(STATE_AUDIO_OUTPUT);
}

static void hdmi_rx_cts_rcv_int(void)
{
	if (sp_tx_ao_state == AO_INIT)
		sp_tx_ao_state = AO_CTS_RCV_INT;
	else if (sp_tx_ao_state == AO_AUDIO_RCV_INT)
		sp_tx_ao_state = AO_RCV_INT_FINISH;

	/* pr_info("%s %s : cts_rcv\n", LOG_TAG, __func__); */

	return;
}

static void hdmi_rx_audio_rcv_int(void)
{
	if (sp_tx_ao_state == AO_INIT)
		sp_tx_ao_state = AO_AUDIO_RCV_INT;
	else if (sp_tx_ao_state == AO_CTS_RCV_INT)
		sp_tx_ao_state = AO_RCV_INT_FINISH;

	/* pr_info("%s %s : *hdmi_rx_audio_rcv_int\n", LOG_TAG, __func__); */

	return;
}

static void hdmi_rx_audio_samplechg_int(void)
{
	unsigned int i;
	unsigned char c;

	/* transfer audio chaneel status from HDMI Rx to Slinmport Tx */
	for (i = 0; i < 5; i++) {
		sp_read_reg(RX_P0, (HDMI_RX_AUD_IN_CH_STATUS1_REG + i), &c);
		sp_write_reg(TX_P2, (SP_TX_AUD_CH_STATUS_REG1 + i), c);
	}

	/*  pr_info("%s %s :audio samplechg\n", LOG_TAG, __func__); */

	return;
}

static void hdmi_rx_hdcp_error_int(void)
{
	pr_info("%s %s : *HDMI_RX Interrupt: hdcp error.\n", LOG_TAG, __func__);

	if (rx_hdcp_err_count >= 40) {
		rx_hdcp_err_count = 0;
		pr_info("%s %s : Lots of hdcp error occured ...\n", LOG_TAG,
			__func__);
		hdmi_rx_mute_audio(1);
		hdmi_rx_mute_video(1);
		hdmi_rx_set_hpd(0);
		mdelay(10);
		hdmi_rx_set_hpd(1);
	} else
		rx_hdcp_err_count++;
}

static void hdmi_rx_new_gcp_int(void)
{
	unsigned char c;
	/* pr_info("%s %s :New GCP Packet.\n", LOG_TAG, __func__); */

	sp_read_reg(RX_P1, HDMI_RX_GENERAL_CTRL, &c);

	if (c & SET_AVMUTE) {
		hdmi_rx_mute_video(1);
		hdmi_rx_mute_audio(1);

	} else if (c & CLEAR_AVMUTE) {
		/* if ((g_video_muted) && */

		/*      (hdmi_system_state >HDMI_VIDEO_CONFIG)) */

		hdmi_rx_mute_video(0);
		/* if ((g_audio_muted) && */

		/*      (hdmi_system_state >HDMI_AUDIO_CONFIG)) */

		hdmi_rx_mute_audio(0);
	}
}

static void sp_tx_hpd_int_handler(unsigned char hpd_source)
{
	pr_info("%s %s : sp_tx_hpd_int_handler \n", LOG_TAG, __func__);

	switch (hpd_source) {
	case HPD_LOST:		
		pr_info("HPD:____________HPD LOST!\n");
		msleep(2);
		if (__i2c_read_byte(TX_P0, SP_TX_SYS_CTRL3_REG) & HPD_STATUS) {
			return;
		}else {
			sleepdelay_count = 10;
		}
		break;
	case HPD_CHANGE:
		pr_info("HPD:____________HPD changed!\n");
		/*Double confirm it is not IRQ*/
		msleep(2);
		sleepdelay_count = 10;
		if(COMMON_INT4 & HPD_IRQ) {
			hpd_irq_process();
			break;
		}

		if (__i2c_read_byte(TX_P0, SP_TX_SYS_CTRL3_REG) & HPD_STATUS) {
			/* IRQ */
			if (COMMON_INT4 & HPD_IRQ) {
				hpd_irq_process();
			//	break;
			}
			/*if (slimport_hdcp_cap_check() == 0) {
				sp_tx_set_sys_state(STATE_LINK_TRAINING);
			}
			*/
		} else {
			if (__i2c_read_byte(TX_P0, SP_TX_SYS_CTRL3_REG) & HPD_STATUS){
				hdmi_rx_set_hpd(0);
				sp_tx_set_sys_state(STATE_WAITTING_CABLE_PLUG);
			} else {
				g_read_edid_flag = 0;
				hdmi_rx_set_hpd(0);
				sleepdelay_count = 10;
				sp_tx_set_sys_state(STATE_SP_INITIALIZED/*STATE_SINK_CONNECTION*/);
			}
		}
		break;	
	case PLUG:
		pr_info("HPD:____________HPD PLUG!\n");
		sleepdelay_count = 0;
		if(sp_tx_system_state < STATE_SP_INITIALIZED)
			sp_tx_set_sys_state(STATE_SP_INITIALIZED);
		break;
	default:
		break;
	}

/*
	if (sleepdelay_count > 0) {
		sleepdelay_count--;
		msleep(5);
	}

	if (sleepdelay_count == 1) {
		sleepdelay_count = 0;
		sp_tx_set_sys_state(STATE_WAITTING_CABLE_PLUG);
	}
	*/
}

void system_isr_handler(void)
{
	/* if(sp_tx_system_state < STATE_PLAY_BACK) */
	/* pr_info("%s %s : =========system_isr==========\n",
		LOG_TAG, __func__); */
	/* Standard DP HPD status */

	if (sp_tx_system_state == STATE_WAITTING_CABLE_PLUG) {
		if (COMMON_INT4 & PLUG) /* PLUG, hpd changed */
			sp_tx_hpd_int_handler(PLUG);
	} else {
		if (COMMON_INT4 & HPD_CHANGE)
			sp_tx_hpd_int_handler(HPD_CHANGE);
		else if (COMMON_INT4 & HPD_LOST)
			sp_tx_hpd_int_handler(HPD_LOST);
	}
	/*if(COMMON_INT4 & HPD_IRQ)
		pr_info("++++++++++++++++========HDCP_IRQ interrupt\n");*/
	if (COMMON_INT1 & PLL_LOCK_CHG)
		sp_tx_pll_changed_int_handler();

	if (COMMON_INT2 & HDCP_AUTH_DONE)
		sp_tx_auth_done_int_handler();

	if (COMMON_INT3 & HDCP_LINK_CHECK_FAIL)
		sp_tx_hdcp_link_chk_fail_handler();
	/*
	if (sp_tx_system_state > STATE_SINK_CONNECTION) {
		if (COMMON_INT5 & DPCD_IRQ_REQUEST)
			sp_tx_sink_irq_int_handler();
	}
	#ifdef CO2_CABLE_DET_MODE
	if (sp_tx_system_state > STATE_SP_INITIALIZED) {
		if (COMMON_INT5 & POLLING_ERR)
			sp_tx_polling_err_int_handler();
	}
	#endif
	*/
	if (COMMON_INT5 & TRAINING_Finish)
		sp_tx_lt_done_int_handler();

	/*
	if (COMMON_INT5 & LINK_CHANGE)
		sp_tx_link_change_int_handler();
	*/
	if (sp_tx_system_state > STATE_SINK_CONNECTION) {
		if (HDMI_RX_INT6 & NEW_AVI)
			hdmi_rx_new_avi_int();
	}

	if (sp_tx_system_state > STATE_VIDEO_OUTPUT) {

		if (HDMI_RX_INT7 & NEW_VS) {
			/* clear no_vsi interrupt flag */
			HDMI_RX_INT7 &= ~NO_VSI;
			hdmi_rx_new_vsi_int();
		}

		if (HDMI_RX_INT7 & NO_VSI)
			hdmi_rx_no_vsi_int();

	}

	if (sp_tx_system_state >= STATE_VIDEO_OUTPUT) {
		if (HDMI_RX_INT1 & CKDT_CHANGE)
			hdmi_rx_clk_det_int();

		if (HDMI_RX_INT1 & SCDT_CHANGE)
			hdmi_rx_sync_det_int();

		if (HDMI_RX_INT1 & HDMI_DVI)
			hdmi_rx_hdmi_dvi_int();

		/*
		if (HDMI_RX_INT1 & SET_MUTE)
			hdmi_rx_avmute_int();
		*/

		if ((HDMI_RX_INT6 & NEW_AUD)
		    || (HDMI_RX_INT3 & AUD_MODE_CHANGE))
			hdmi_rx_restart_audio_chk();

		if (HDMI_RX_INT6 & CTS_RCV)
			hdmi_rx_cts_rcv_int();

		if (HDMI_RX_INT5 & AUDIO_RCV)
			hdmi_rx_audio_rcv_int();

		if (HDMI_RX_INT2 & HDCP_ERR)
			hdmi_rx_hdcp_error_int();

		if (HDMI_RX_INT6 & NEW_CP)
			hdmi_rx_new_gcp_int();

		if (HDMI_RX_INT2 & AUDIO_SAMPLE_CHANGE)
			hdmi_rx_audio_samplechg_int();
	}

}

void sp_tx_show_infomation(void)
{
	unsigned char c, c1;
	unsigned int h_res, h_act, v_res, v_act;
	unsigned int h_fp, h_sw, h_bp, v_fp, v_sw, v_bp;
	unsigned long fresh_rate;
	unsigned long pclk;

	pr_info
	    (" \n*******SP Video Information**********\n");

	sp_read_reg(TX_P0, SP_TX_LINK_BW_SET_REG, &c);

	switch (c) {
	case 0x06:
		pr_info("%s %s : BW = 1.62G\n", LOG_TAG, __func__);
		break;
	case 0x0a:
		pr_info("%s %s : BW = 2.7G\n", LOG_TAG, __func__);
		break;
	case 0x14:
		pr_info("%s %s : BW = 5.4G\n", LOG_TAG, __func__);
		break;
	case 0x19:
		pr_info("%s %s : BW = 6.75G\n", LOG_TAG, __func__);
		break;
	default:
		break;
	}

	pclk = sp_tx_pclk_calc();
	pclk = pclk / 10;
	/* pr_info("%s %s : SSC On\n", LOG_TAG, __func__); */

	sp_read_reg(TX_P2, SP_TX_TOTAL_LINE_STA_L, &c);
	sp_read_reg(TX_P2, SP_TX_TOTAL_LINE_STA_H, &c1);

	v_res = c1;
	v_res = v_res << 8;
	v_res = v_res + c;

	sp_read_reg(TX_P2, SP_TX_ACT_LINE_STA_L, &c);
	sp_read_reg(TX_P2, SP_TX_ACT_LINE_STA_H, &c1);

	v_act = c1;
	v_act = v_act << 8;
	v_act = v_act + c;

	sp_read_reg(TX_P2, SP_TX_TOTAL_PIXEL_STA_L, &c);
	sp_read_reg(TX_P2, SP_TX_TOTAL_PIXEL_STA_H, &c1);

	h_res = c1;
	h_res = h_res << 8;
	h_res = h_res + c;

	sp_read_reg(TX_P2, SP_TX_ACT_PIXEL_STA_L, &c);
	sp_read_reg(TX_P2, SP_TX_ACT_PIXEL_STA_H, &c1);

	h_act = c1;
	h_act = h_act << 8;
	h_act = h_act + c;

	sp_read_reg(TX_P2, SP_TX_H_F_PORCH_STA_L, &c);
	sp_read_reg(TX_P2, SP_TX_H_F_PORCH_STA_H, &c1);

	h_fp = c1;
	h_fp = h_fp << 8;
	h_fp = h_fp + c;

	sp_read_reg(TX_P2, SP_TX_H_SYNC_STA_L, &c);
	sp_read_reg(TX_P2, SP_TX_H_SYNC_STA_H, &c1);

	h_sw = c1;
	h_sw = h_sw << 8;
	h_sw = h_sw + c;

	sp_read_reg(TX_P2, SP_TX_H_B_PORCH_STA_L, &c);
	sp_read_reg(TX_P2, SP_TX_H_B_PORCH_STA_H, &c1);

	h_bp = c1;
	h_bp = h_bp << 8;
	h_bp = h_bp + c;

	sp_read_reg(TX_P2, SP_TX_V_F_PORCH_STA, &c);
	v_fp = c;

	sp_read_reg(TX_P2, SP_TX_V_SYNC_STA, &c);
	v_sw = c;

	sp_read_reg(TX_P2, SP_TX_V_B_PORCH_STA, &c);
	v_bp = c;

	pr_info("%s %s : Total resolution is %d * %d\n", LOG_TAG, __func__,
		h_res, v_res);

	pr_info("%s %s : HF=%d, HSW=%d, HBP=%d\n", LOG_TAG, __func__, h_fp,
		h_sw, h_bp);
	pr_info("%s %s : VF=%d, VSW=%d, VBP=%d\n", LOG_TAG, __func__, v_fp,
		v_sw, v_bp);
	pr_info("%s %s : Active resolution is %d * %d", LOG_TAG, __func__,
		h_act, v_act);

	if (h_res == 0 || v_res == 0)
		fresh_rate = 0;
	else {
		fresh_rate = pclk * 1000;
		fresh_rate = fresh_rate / h_res;
		fresh_rate = fresh_rate * 1000;
		fresh_rate = fresh_rate / v_res;
	}

	pr_info("   @ %ldHz\n", fresh_rate);

	sp_read_reg(TX_P0, SP_TX_VID_CTRL, &c);

	if ((c & 0x06) == 0x00)
		pr_info("%s %s : ColorSpace: RGB,", LOG_TAG, __func__);
	else if ((c & 0x06) == 0x02)
		pr_info("%s %s : ColorSpace: YCbCr422,", LOG_TAG, __func__);
	else if ((c & 0x06) == 0x04)
		pr_info("%s %s : ColorSpace: YCbCr444,", LOG_TAG, __func__);

	sp_read_reg(TX_P0, SP_TX_VID_CTRL, &c);

	if ((c & 0xe0) == 0x00)
		pr_info("6 BPC\n");
	else if ((c & 0xe0) == 0x20)
		pr_info("8 BPC\n");
	else if ((c & 0xe0) == 0x40)
		pr_info("10 BPC\n");
	else if ((c & 0xe0) == 0x60)
		pr_info("12 BPC\n");


	if(is_anx_dongle()) {
		sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x23, 1, &c);
		pr_info("%s %s : Analogix Dongle FW Ver %.2x \n",
			LOG_TAG, __func__, (unsigned int)(c & 0x7f));
	}

	pr_info("\n**************************************************\n");

}

void hdmi_rx_show_video_info(void)
{
	unsigned char c, c1;
	unsigned char cl, ch;
	unsigned int n;
	unsigned int h_res, v_res;

	sp_read_reg(RX_P0, HDMI_RX_HACT_LOW, &cl);
	sp_read_reg(RX_P0, HDMI_RX_HACT_HIGH, &ch);
	n = ch;
	n = (n << 8) + cl;
	h_res = n;

	sp_read_reg(RX_P0, HDMI_RX_VACT_LOW, &cl);
	sp_read_reg(RX_P0, HDMI_RX_VACT_HIGH, &ch);
	n = ch;
	n = (n << 8) + cl;
	v_res = n;

	pr_info("%s %s : >HDMI_RX Info<\n", LOG_TAG, __func__);
	sp_read_reg(RX_P0, HDMI_STATUS, &c);

	if (c & HDMI_MODE)
		pr_info("%s %s : HDMI_RX Mode = HDMI Mode.\n", LOG_TAG,
			__func__);
	else
		pr_info("%s %s : HDMI_RX Mode = DVI Mode.\n", LOG_TAG,
			__func__);

	sp_read_reg(RX_P0, HDMI_RX_VIDEO_STATUS_REG1, &c);

	if (c & VIDEO_TYPE)
		v_res += v_res;

	pr_info("%s %s : HDMI_RX Video Resolution = %d * %d ", LOG_TAG,
		__func__, h_res, v_res);
	sp_read_reg(RX_P0, HDMI_RX_VIDEO_STATUS_REG1, &c);

	if (c & VIDEO_TYPE)
		pr_info("Interlace Video.\n");
	else
		pr_info("Progressive Video.\n");

	sp_read_reg(RX_P0, HDMI_RX_SYS_CTRL1_REG, &c);

	if ((c & 0x30) == 0x00)
		pr_info("%s %s : Input Pixel Clock = Not Repeated.\n", LOG_TAG,
			__func__);
	else if ((c & 0x30) == 0x10)
		pr_info
		    ("%s %s : Input Pixel Clock = 2x Video Clock. Repeated.\n",
		     LOG_TAG, __func__);
	else if ((c & 0x30) == 0x30)
		pr_info
		    ("%s %s : Input Pixel Clock = 4x Vvideo Clock. Repeated.\n",
		     LOG_TAG, __func__);

	if ((c & 0xc0) == 0x00)
		pr_info("%s %s : Output Video Clock = Not Divided.\n", LOG_TAG,
			__func__);
	else if ((c & 0xc0) == 0x40)
		pr_info("%s %s : Output Video Clock = Divided By 2.\n", LOG_TAG,
			__func__);
	else if ((c & 0xc0) == 0xc0)
		pr_info("%s %s : Output Video Clock = Divided By 4.\n", LOG_TAG,
			__func__);

	if (c & 0x02)
		pr_info
		    ("%s %s : Output Video Using Rising Edge To Latch Data.\n",
		     LOG_TAG, __func__);
	else
		pr_info
		    ("%s %s : Output Video Using Falling Edge To Latch Data.\n",
		     LOG_TAG, __func__);

	pr_info("Input Video Color Depth = ");
	sp_read_reg(RX_P0, 0x70, &c1);
	c1 &= 0xf0;

	if (c1 == 0x00)
		pr_info("%s %s : Legacy Mode.\n", LOG_TAG, __func__);
	else if (c1 == 0x40)
		pr_info("%s %s : 24 Bit Mode.\n", LOG_TAG, __func__);
	else if (c1 == 0x50)
		pr_info("%s %s : 30 Bit Mode.\n", LOG_TAG, __func__);
	else if (c1 == 0x60)
		pr_info("%s %s : 36 Bit Mode.\n", LOG_TAG, __func__);
	else if (c1 == 0x70)
		pr_info("%s %s : 48 Bit Mode.\n", LOG_TAG, __func__);

	pr_info("%s %s : Input Video Color Space = ", LOG_TAG, __func__);
	sp_read_reg(RX_P1, HDMI_RX_AVI_DATA00_REG, &c);
	c &= 0x60;

	if (c == 0x20)
		pr_info("YCbCr4:2:2 .\n");
	else if (c == 0x40)
		pr_info("YCbCr4:4:4 .\n");
	else if (c == 0x00)
		pr_info("RGB.\n");
	else
		pr_err("Unknow 0x44 = 0x%.2x\n", (int)c);

	sp_read_reg(RX_P1, HDMI_RX_HDCP_STATUS_REG, &c);

	if (c & AUTH_EN)
		pr_info("%s %s : Authentication is attempted.\n", LOG_TAG,
			__func__);
	else
		pr_info("%s %s : Authentication is not attempted.\n", LOG_TAG,
			__func__);

	for (cl = 0; cl < 20; cl++) {
		sp_read_reg(RX_P1, HDMI_RX_HDCP_STATUS_REG, &c);

		if (c & DECRYPT_EN)
			break;
		else
			mdelay(10);
	}

	if (cl < 20)
		pr_info("%s %s : Decryption is active.\n", LOG_TAG, __func__);
	else
		pr_info("%s %s : Decryption is not active.\n", LOG_TAG,
			__func__);

}

void clean_system_status(void)
{
	if (g_need_clean_status) {
		pr_info("%s %s : clean_system_status. A -> B;\n", LOG_TAG,
			__func__);
		pr_info("A:");
		print_sys_state(sp_tx_system_state_bak);
		pr_info("B:");
		print_sys_state(sp_tx_system_state);

		g_need_clean_status = 0;

		if (sp_tx_system_state_bak >= STATE_LINK_TRAINING) {
			if (sp_tx_system_state >= STATE_AUDIO_OUTPUT)
				hdmi_rx_mute_audio(1);
			else {
				hdmi_rx_mute_video(1);
				sp_tx_video_mute(1);
			}
		}

		if (!HDCP_REPEATER_MODE) {
			if (sp_tx_system_state_bak >= STATE_HDCP_AUTH
			    && sp_tx_system_state <= STATE_HDCP_AUTH) {
				if (__i2c_read_byte(TX_P0, TX_HDCP_CTRL0) &
				    0xFC)
					sp_tx_clean_hdcp_status();
			}
		} else {
			if (sp_tx_system_state_bak > STATE_LINK_TRAINING
			    && sp_tx_system_state <= STATE_LINK_TRAINING) {
				/*Inform AP to re-auth */
				hdmi_rx_set_hpd(0);
				pr_info("%s %s : hdmi_rx_set_hpd 0 !\n",
					LOG_TAG, __func__);
				hdmi_rx_set_termination(0);
				msleep(50);
			}
		}

		if (HDCP_state != HDCP_CAPABLE_CHECK)
			HDCP_state = HDCP_CAPABLE_CHECK;

		if (sp_tx_sc_state != SC_INIT)
			sp_tx_sc_state = SC_INIT;

		if (sp_tx_LT_state != LT_INIT)
			sp_tx_LT_state = LT_INIT;

		if (sp_tx_vo_state != VO_WAIT_VIDEO_STABLE)
			sp_tx_vo_state = VO_WAIT_VIDEO_STABLE;
		audio_stable_count = 0;
	}
}

/*********add for HDCP cap check***************/
void sp_tx_get_ddc_hdcp_BCAP(unsigned char *bcap)
{
	unsigned char c;

	/* 0X74 FOR HDCP DDC */
	sp_write_reg(TX_P0, AUX_ADDR_7_0, 0X3A);
	sp_write_reg(TX_P0, AUX_ADDR_15_8, 0);
	sp_read_reg(TX_P0, AUX_ADDR_19_16, &c);
	c &= 0xf0;
	sp_write_reg(TX_P0, AUX_ADDR_19_16, c);

	sp_tx_aux_wr(0x00);
	sp_tx_aux_rd(0x01);
	sp_read_reg(TX_P0, BUF_DATA_0, &c);

	/* 0X40: bcap */
	sp_tx_aux_wr(0x40);
	sp_tx_aux_rd(0x01);
	sp_read_reg(TX_P0, BUF_DATA_0, bcap);
	/* pr_info("%s %s :HDCP BCAP = %.2x\n",
		LOG_TAG, __func__, (unsigned int)*bcap); */

}

void sp_tx_get_ddc_hdcp_BKSV(unsigned char *bksv)
{
	unsigned char c;

	/* 0X74 FOR HDCP DDC */
	sp_write_reg(TX_P0, AUX_ADDR_7_0, 0X3A);
	sp_write_reg(TX_P0, AUX_ADDR_15_8, 0);
	sp_read_reg(TX_P0, AUX_ADDR_19_16, &c);
	c &= 0xf0;
	sp_write_reg(TX_P0, AUX_ADDR_19_16, c);

	sp_tx_aux_wr(0x00);
	sp_tx_aux_rd(0x01);
	sp_read_reg(TX_P0, BUF_DATA_0, &c);

	/* 0X00: bksv */
	sp_tx_aux_wr(0x00);
	sp_tx_aux_rd(0x01);
	sp_read_reg(TX_P0, BUF_DATA_0, bksv);
	/* pr_info("%s %s :HDCP BKSV0 = %.2x\n",
		LOG_TAG, __func__, (unsigned int)*bksv); */

}

unsigned char slimport_hdcp_cap_check(void)   
{
	unsigned char g_hdcp_cap = 0;
	unsigned char temp;

	if(AUX_OK == sp_tx_aux_dpcdread_bytes(0x06, 0x80, 0x28, 1, &temp))
		g_hdcp_cap = (temp & 0x01) ? 1 : 0;
	else
		pr_info("HDCP CAPABLE: read AUX err! \n");

	pr_info("%s %s :hdcp cap check: %s Supported\n",
		LOG_TAG, __func__, g_hdcp_cap ? "" : "No");
	return g_hdcp_cap;
}

/******************End HDCP cap check********************/
void slimport_hdcp_repeater_reauth(void)
{				/* A patch for HDCP repeater mode */
	unsigned char c, hdcp_ctrl, hdcp_staus;

	if (HDCP_REPEATER_MODE
		&& (sp_tx_system_state > STATE_HDCP_AUTH)) {
		msleep(50);

		sp_read_reg(RX_P1, HDMI_RX_HDCP_STATUS_REG, &c);

		if ((c & AUTH_EN) != 0) {
			/* debug_puts("Upstream HDMI HDCP request!\n"); */

			sp_read_reg(TX_P0, TX_HDCP_CTRL0, &hdcp_ctrl);

			if ((hdcp_ctrl & HARD_AUTH_EN) == 0) {
				pr_info("Repeater Mode: Enable HW HDCP\n");
				assisted_HDCP_repeater = HDCP_ERROR;

			} else {
				sp_read_reg(TX_P0, SP_TX_HDCP_STATUS,
					    &hdcp_staus);

				if (((hdcp_staus & SP_TX_HDCP_AUTH_PASS) == 0)
				    & ((hdcp_staus & SP_TX_HDCP_AUTH_FAIL) ==
				       1)) {
					assisted_HDCP_repeater = HDCP_ERROR;
					pr_info("Clean HDCP and re-auth\n");
				} else if ((hdcp_staus & SP_TX_HDCP_AUTH_PASS)
					   == 1) {
					pr_info
					    ("$$$$$HDCP Pass$$$$$\n");
				}
			}
		}

		sp_read_reg(TX_P0, TX_HDCP_CTRL0, &hdcp_ctrl);
		sp_read_reg(TX_P0, SP_TX_HDCP_STATUS, &hdcp_staus);

		if ((hdcp_ctrl == TX_HDCP_FUNCTION_ENABLE)
		    && ((hdcp_staus & SP_TX_HDCP_AUTH_FAIL))) {
			pr_info
			    ("main link error: HDCP encryption failure %x\n",
			     (int)hdcp_staus);
			assisted_HDCP_repeater = HDCP_ERROR;
			/* sp_tx_set_sys_state(STATE_LINK_TRAINING); */

		}

		if (assisted_HDCP_repeater == HDCP_ERROR) {
			sp_tx_clean_hdcp_status();
			msleep(50);
			/*Clear HDCP AUTH interrupt */
			sp_write_reg_or(TX_P2, SP_COMMON_INT_STATUS1 + 1,
					HDCP_AUTH_DONE);
			sp_tx_hw_hdcp_enable();
			assisted_HDCP_repeater = HDCP_DOING;
		}
	}

}

#ifdef USING_HPD_FOR_POWER_MANAGEMENT
void slimport_cable_monitor(void)
{	
	if (is_cable_detected())
	{
		msleep(50);
		if (is_cable_detected())
		{
			if (sleepdelay_count>0)
			{
				sleepdelay_count = 0;
			}
		}
	} else {
		pr_info("SLIMPORT_CABLE_DETECT = 0\n");
		if (sleepdelay_count > 0) {
			sleepdelay_count--;
			pr_info("sleepdelay_count --\n");
			msleep(5);
		}
	
		if (sleepdelay_count == 1) {
			sleepdelay_count = 0;
			pr_info("sleepdelay_count finish\n");
			sp_tx_set_sys_state(STATE_WAITTING_CABLE_PLUG);
		}
	}

}
#endif

void slimport_tasks_handler(void)
{
	/* isr */
	if (sp_tx_system_state > STATE_WAITTING_CABLE_PLUG)
		system_isr_handler();

	if (HDCP_REPEATER_MODE)
		slimport_hdcp_repeater_reauth();

	hdcp_external_ctrl_flag_monitor();
	clean_system_status();
	
	#ifdef USING_HPD_FOR_POWER_MANAGEMENT
	//slimport_cable_monitor();
	#endif
	
	/*clear up backup system state */
	if (sp_tx_system_state_bak != sp_tx_system_state)
		sp_tx_system_state_bak = sp_tx_system_state;
}

/******************End task  process********************/

void slimport_main_process(void)
{
	slimport_state_process();
	if(sp_tx_system_state > STATE_WAITTING_CABLE_PLUG){
		slimport_int_rec();
		slimport_tasks_handler();
	}
}

/* ***************************************************************** */
/* Functions for slimport_rx anx7730 */
/* ***************************************************************** */

#define SOURCE_AUX_OK 1
#define SOURCE_AUX_ERR 0
#define SOURCE_REG_OK 1
#define SOURCE_REG_ERR 0

#define SINK_DEV_SEL  0x005f0
#define SINK_ACC_OFS  0x005f1
#define SINK_ACC_REG  0x005f2

bool source_aux_read_7730dpcd(long addr, unsigned char cCount, unsigned char *pBuf)
{
	unsigned char c;
	unsigned char addr_l;
	unsigned char addr_m;
	unsigned char addr_h;
	addr_l = (unsigned char) addr;
	addr_m = (unsigned char) (addr >> 8);
	addr_h = (unsigned char) (addr >> 16);
	c = 0;

	while (1) {
		if (sp_tx_aux_dpcdread_bytes
		    (addr_h, addr_m, addr_l, cCount, pBuf) == AUX_OK)
			return SOURCE_AUX_OK;

		c++;
		if (c > 3)
			return SOURCE_AUX_ERR;
	}
}

bool source_aux_write_7730dpcd(long addr, unsigned char cCount, unsigned char *pBuf)
{
	unsigned char c;
	unsigned char addr_l;
	unsigned char addr_m;
	unsigned char addr_h;
	addr_l = (unsigned char) addr;
	addr_m = (unsigned char) (addr >> 8);
	addr_h = (unsigned char) (addr >> 16);
	c = 0;

	while (1) {
		if (sp_tx_aux_dpcdwrite_bytes
		    (addr_h, addr_m, addr_l, cCount, pBuf) == AUX_OK)
			return SOURCE_AUX_OK;

		c++;
		if (c > 3)
			return SOURCE_AUX_ERR;
	}
}

bool i2c_master_read_reg(unsigned char Sink_device_sel, unsigned char offset, unsigned char *Buf)
{
	unsigned char sbytebuf[2] = { 0 };
	long a0, a1;
	a0 = SINK_DEV_SEL;
	a1 = SINK_ACC_REG;
	sbytebuf[0] = Sink_device_sel;
	sbytebuf[1] = offset;

	if (source_aux_write_7730dpcd(a0, 2, sbytebuf) == SOURCE_AUX_OK) {
		if (source_aux_read_7730dpcd(a1, 1, Buf) == SOURCE_AUX_OK)
			return SOURCE_REG_OK;
	}

	return SOURCE_REG_ERR;
}

bool i2c_master_write_reg(unsigned char Sink_device_sel, unsigned char offset, unsigned char value)
{
	unsigned char sbytebuf[3] = { 0 };
	long a0;
	a0 = SINK_DEV_SEL;
	sbytebuf[0] = Sink_device_sel;
	sbytebuf[1] = offset;
	sbytebuf[2] = value;

	if (source_aux_write_7730dpcd(a0, 3, sbytebuf) == SOURCE_AUX_OK)
		return SOURCE_REG_OK;
	else
		return SOURCE_REG_ERR;
}

#ifdef ANX_LINUX_ENV
MODULE_DESCRIPTION("Slimport transmitter ANX7816 driver");
MODULE_AUTHOR("<swang@analogixsemi.com>");
MODULE_LICENSE("GPL");
#endif
