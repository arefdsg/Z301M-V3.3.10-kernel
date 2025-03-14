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

#ifdef CONFIG_MTK_CLKMGR
#include <mach/mt_clkmgr.h>
#else
#include <linux/clk.h>
#endif
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/spinlock.h>
#include "mtk_musb.h"
#include "musb_core.h"
#include "usb20.h"

/*#include <mach/mt_gpio.h>*/

#define FRA (48)
#define PARA (28)

#ifdef FPGA_PLATFORM
bool usb_enable_clock(bool enable)
{
	return true;
}

void usb_phy_poweron(void)
{
}

void usb_phy_savecurrent(void)
{
}

void usb_phy_recover(void)
{
}

/* BC1.2*/
void Charger_Detect_Init(void)
{
}

void Charger_Detect_Release(void)
{
}

void usb_phy_context_save(void)
{
}

void usb_phy_context_restore(void)
{
}

#ifdef CONFIG_MTK_UART_USB_SWITCH
bool usb_phy_check_in_uart_mode(void)
{
	UINT8 usb_port_mode;

	usb_enable_clock(true);
	udelay(50);

	usb_port_mode = USB_PHY_Read_Register8(0x6B);
	usb_enable_clock(false);

	if ((usb_port_mode == 0x5C) || (usb_port_mode == 0x5E))
		return true;
	else
		return false;
}
void usb_phy_switch_to_usb(void)
{
	usb_enable_clock(true);
	udelay(50);
	/* clear force_uart_en */
	USBPHY_WRITE8(0x6B, 0x00);
	usb_enable_clock(false);
	usb_phy_poweron();
	/* disable the USB clock turned on in usb_phy_poweron() */
	usb_enable_clock(false);
}

void usb_phy_switch_to_uart(void)
{

	if (usb_phy_check_in_uart_mode())
		return;
	DBG(0, "Mask PMIC charger detection in UART mode.\n");
	/*ALPS00775710 */
	/*ALPS00775710*/

	usb_enable_clock(true);
	udelay(50);

	/* RG_USB20_BC11_SW_EN = 1'b0 */
	USBPHY_CLR8(0x1a, 0x80);

	/* Set RG_SUSPENDM to 1 */
	USBPHY_SET8(0x68, 0x08);

	/* force suspendm = 1 */
	USBPHY_SET8(0x6a, 0x04);

	/* Set ru_uart_mode to 2'b01 */
	USBPHY_SET8(0x6B, 0x5C);

	/* Set RG_UART_EN to 1 */
	USBPHY_SET8(0x6E, 0x07);

	/* Set RG_USB20_DM_100K_EN to 1 */
	USBPHY_SET8(0x22, 0x02);
	usb_enable_clock(false);
}

#endif

#else

#ifdef CONFIG_MTK_UART_USB_SWITCH
bool in_uart_mode = false;
#endif

static DEFINE_SPINLOCK(musb_reg_clock_lock);
static DEFINE_SPINLOCK(musb_phy_clock_lock);
static DEFINE_SPINLOCK(musb_mcu_clock_lock);

void enable_phy_clock(bool enable)
{
	static int count;
	unsigned long flags;

	spin_lock_irqsave(&musb_phy_clock_lock, flags);
	/* USB phy 48M clock , UNIVPLL_CON0[26] */
	if (enable) {
		if (0 == count) {
			DBG(3, "enable phy clock\n");
			clk_enable(usbpll_clk);
		}
		count++;
	} else {
		if (1 == count) {
			DBG(3, "disable phy clock\n");
			clk_disable(usbpll_clk);
		}
		count = (count == 0) ? 0 : (count - 1);
	}
	spin_unlock_irqrestore(&musb_phy_clock_lock, flags);
}

void enable_mcu_clock(bool enable)
{
	static int count;
	unsigned long flags;

	spin_lock_irqsave(&musb_mcu_clock_lock, flags);
	/* USB phy 48M clock , UNIVPLL_CON0[26] */
	if (enable) {
		if (0 == count) {
			DBG(3, "enable mcu clock\n");
			clk_enable(usbmcu_clk);
		}
		count++;
	} else {
		if (1 == count) {
			DBG(3, "disable mcu clock\n");
			clk_disable(usbmcu_clk);
		}
		count = (count == 0) ? 0 : (count - 1);
	}
	spin_unlock_irqrestore(&musb_mcu_clock_lock, flags);
}

bool usb_enable_clock(bool enable)
{
	static int count;
	unsigned long flags;

	spin_lock_irqsave(&musb_reg_clock_lock, flags);

	if (enable && count == 0) {
		enable_phy_clock(true);
#ifdef CONFIG_MTK_CLKMGR
		enable_clock(MT_CG_INFRA_USB, "INFRA_USB");
		enable_clock(MT_CG_INFRA_USB_MCU, "INFRA_USB_MCU");
		enable_clock(MT_CG_INFRA_ICUSB, "INFRA_ICUSB");
#else
		DBG(3, "enable usb0 clock\n");
		clk_enable(usb_clk);
		enable_mcu_clock(true);
#endif
	} else if (!enable && count == 1) {
#ifdef CONFIG_MTK_CLKMGR
		disable_clock(MT_CG_INFRA_USB_MCU, "INFRA_USB_MCU");
		disable_clock(MT_CG_INFRA_USB, "INFRA_USB");
		disable_clock(MT_CG_INFRA_ICUSB, "INFRA_ICUSB");
#else
		DBG(3, "disable usb0 clock\n");
		clk_disable(usb_clk);
		enable_mcu_clock(false);
#endif
		enable_phy_clock(false);
	}

	if (enable)
		count++;
	else
		count = (count == 0) ? 0 : (count-1);

	spin_unlock_irqrestore(&musb_reg_clock_lock, flags);

	return 1;
}

static void hs_slew_rate_cal(void)
{
	unsigned long data;
	unsigned long x;
	unsigned char value;
	unsigned long start_time, timeout;
	unsigned int timeout_flag = 0;
	/*4 s1:enable usb ring oscillator.*/
	USBPHY_WRITE8(0x15, 0x80);

	/*4 s2:wait 1us.*/
	udelay(1);

	/*4 s3:enable free run clock*/
	USBPHY_WRITE8(0xf00-0x800+0x11, 0x01);
	/*4 s4:setting cyclecnt.*/
	USBPHY_WRITE8(0xf00-0x800+0x01, 0x04);
	/*4 s5:enable frequency meter*/
	USBPHY_SET8(0xf00-0x800+0x03, 0x01);

	/*4 s6:wait for frequency valid.*/
	start_time = jiffies;
	timeout = jiffies + 3 * HZ;

	while (!(USBPHY_READ8(0xf00-0x800+0x10)&0x1)) {
		if (time_after(jiffies, timeout)) {
			timeout_flag = 1;
			break;
		}
	}

	/*4 s7: read result.*/
	if (timeout_flag) {
		DBG(0, "[USBPHY] Slew Rate Calibration: Timeout\n");
		value = 0x4;
	}	else{
		data = USBPHY_READ32(0xf00-0x800+0x0c);
		x = ((1024*FRA*PARA)/data);
		value = (unsigned char)(x/1000);
		if ((x-value*1000)/100 >= 5)
			value += 1;
		DBG(0, "[USBPHY]slew calibration: FM_OUT = %lu, x= %lu, value= %d\n", data, x, value);
	}

	/*4 s8: disable Frequency and run clock.*/
	USBPHY_CLR8(0xf00-0x800+0x03, 0x01);/*disable frequency meter*/
	USBPHY_CLR8(0xf00-0x800+0x11, 0x01);/*disable free run clock*/

	/*4 s9:*/
	USBPHY_WRITE8(0x15, value<<4);

	/*4 s10:disable usb ring oscillator.*/
	USBPHY_CLR8(0x15, 0x80);
}

#ifdef CONFIG_MTK_UART_USB_SWITCH
bool usb_phy_check_in_uart_mode(void)
{
	UINT8 usb_port_mode;

	usb_enable_clock(true);
	udelay(50);
	usb_port_mode = USBPHY_READ8(0x6B);
	/*usb_port_mode = 1;*/
	usb_enable_clock(false);

	if ((usb_port_mode == 0x5C) ||
		(usb_port_mode == 0x5E) || (usb_port_mode_temp == 1)) {
		usb_port_mode_temp = 1;
		return true;
	} else
		return false;
}

void usb_phy_switch_to_uart(void)
{
	if (usb_phy_check_in_uart_mode())
		return;

	usb_enable_clock(true);
	udelay(50);

	/* RG_USB20_BC11_SW_EN = 1'b0 */
	USBPHY_CLR8(0x1a, 0x80);

	/* Set RG_SUSPENDM to 1 */
	USBPHY_SET8(0x68, 0x08);

	/* force suspendm = 1 */
	USBPHY_SET8(0x6a, 0x04);

	/* Set ru_uart_mode to 2'b01 */
	USBPHY_SET8(0x6B, 0x5C);

	/* Set RG_UART_EN to 1 */
	USBPHY_SET8(0x6E, 0x07);

	/* Set RG_USB20_DM_100K_EN to 1 */
	USBPHY_SET8(0x22, 0x02);
	usb_enable_clock(false);

	/*set uart rx path*/
	mtk_uart_usb_rx_sel(1, 1);
	usb_port_mode_temp = 1;
	DBG(0, "usb port value in uart function:%d\n", usb_port_mode_temp);
	/* GPIO Selection */
	/* DRV_WriteReg32(GPIO_BASE + 0x504, 0x10);	//set */
}


void usb_phy_switch_to_usb(void)
{
	/* GPIO Selection */
	/*DRV_WriteReg32(GPIO_BASE + 0x508, 0x10);		//set */
	mtk_uart_usb_rx_sel(1, 0);
	usb_enable_clock(true);
	udelay(50);
	/* clear force_uart_en */
	USBPHY_WRITE8(0x6B, 0x00);
	usb_enable_clock(false);
	usb_phy_poweron();
	/* disable the USB clock turned on in usb_phy_poweron() */
	usb_enable_clock(false);

	usb_port_mode_temp = 0;
	DBG(0, "usb port value in usb function:%d\n", usb_port_mode_temp);
}
#endif

void usb_phy_poweron(void)
{
	#ifdef CONFIG_MTK_UART_USB_SWITCH
	if (usb_phy_check_in_uart_mode())
		return;
	#endif

	/* enable USB MAC clock. */
	usb_enable_clock(true);

	/* wait 50 usec for PHY3.3v/1.8v stable. */
	udelay(50);

	/* force_uart_en, 1'b0 */
	USBPHY_CLR8(0x6b, 0x04);
	/* RG_UART_EN, 1'b0 */
	USBPHY_CLR8(0x6e, 0x01);
	/* rg_usb20_gpio_ctl, 1'b0, usb20_gpio_mode, 1'b0 */
	USBPHY_CLR8(0x21, 0x03);
	/*USBPHY_CLR8(0x21, 0x01);*/

	/* RG_USB20_BC11_SW_EN, 1'b0 */
	USBPHY_CLR8(0x1a, 0x80);

	/* rg_usb20_dp_100k_mode, 1'b1 */
	USBPHY_SET8(0x22, 0x04);
	USBPHY_CLR8(0x22, 0x03);

	/*OTG enable*/
	USBPHY_SET8(0x20, 0x10);
	/* force_suspendm, 1'b0 */
	USBPHY_CLR8(0x6a, 0x04);

	/*6-1. PASS RX sensitivity HQA requirement*/
	USBPHY_SET8(0x18, 0x06);

	/* 7 s7: wait for 800 usec. */
	udelay(800);

	/* force enter device mode, from K2, FIXME */
	USBPHY_CLR8(0x6c, 0x10);
	USBPHY_SET8(0x6c, 0x2E);
	USBPHY_SET8(0x6d, 0x3E);

	printk("VRT & TERM usb phy read default eye value, Addr: 0x5:=0x%x\n", USBPHY_READ8(0x05));
	USBPHY_SET8(0x05, 0x77);
	printk("VRT & TERM usb phy read eye value after set, Addr: 0x5:=0x%x\n", USBPHY_READ8(0x05));

	DBG(0, "usb power on success\n");
}

#ifdef CONFIG_MTK_UART_USB_SWITCH
static bool skipDisableUartMode;
#endif

static void usb_phy_savecurrent_internal(void)
{

	/* 4 1. swtich to USB function. (system register, force ip into usb mode. */

	#ifdef CONFIG_MTK_UART_USB_SWITCH
	if (!usb_phy_check_in_uart_mode()) {
		/* enable USB MAC clock. */
		usb_enable_clock(true);

		/* wait 50 usec for PHY3.3v/1.8v stable. */
		udelay(50);

		/* force_uart_en, 1'b0 */
		USBPHY_CLR8(0x6b, 0x04);
		/* RG_UART_EN, 1'b0 */
		USBPHY_CLR8(0x6e, 0x01);
		/* rg_usb20_gpio_ctl, 1'b0, usb20_gpio_mode, 1'b0 */
		USBPHY_CLR8(0x21, 0x03);

		/*4 2. release force suspendm.*/
		/*USBPHY_CLR8(0x6a, 0x04);*/
		USBPHY_SET8(0x6a, 0x04);
		/* RG_SUSPENDM, 1'b1 */
		USBPHY_SET8(0x68, 0x08);
		usb_enable_clock(false);
	} else {
		if (skipDisableUartMode)
			skipDisableUartMode = false;
		else
			return;
	}
	#else
	/* force_uart_en, 1'b0 */
	USBPHY_CLR8(0x6b, 0x04);
	/* RG_UART_EN, 1'b0 */
	USBPHY_CLR8(0x6e, 0x01);
	/* rg_usb20_gpio_ctl, 1'b0, usb20_gpio_mode, 1'b0 */
	USBPHY_CLR8(0x21, 0x03);

	/* RG_USB20_BC11_SW_EN, 1'b0 */
	/* USBPHY_CLR8(0x6a, 0x04);*/
	USBPHY_SET8(0x6a, 0x04);
	USBPHY_SET8(0x68, 0x08);
	#endif

	/* RG_DPPULLDOWN, 1'b1, RG_DMPULLDOWN, 1'b1 */
	USBPHY_SET8(0x68, 0xc0);
	/* RG_XCVRSEL[1:0], 2'b01. */
	USBPHY_CLR8(0x68, 0x30);
	USBPHY_SET8(0x68, 0x10);
	/* RG_TERMSEL, 1'b1 */
	USBPHY_SET8(0x68, 0x04);
	/* RG_DATAIN[3:0], 4'b0000 */
	USBPHY_CLR8(0x69, 0x3c);

	/* force_dp_pulldown, 1'b1, force_dm_pulldown, 1'b1,
	force_xcversel, 1'b1, force_termsel, 1'b1, force_datain, 1'b1 */
	USBPHY_SET8(0x6a, 0xba);

	/*4 8.RG_USB20_BC11_SW_EN 1'b0*/
	USBPHY_CLR8(0x1a, 0x80);
	/*4 9.RG_USB20_OTG_VBUSSCMP_EN 1'b0*/
	USBPHY_CLR8(0x1a, 0x10);
	/*4 10. delay 800us.*/
	udelay(800);
	/*4 11. rg_usb20_pll_stable = 1*/
	USBPHY_CLR8(0x68, 0x08);
	/*
		USBPHY_SET8(0x63, 0x02);

		ALPS00427972, implement the analog register formula
		ALPS00427972, implement the analog register formula
	*/

	udelay(1);
	/*4 12.  force suspendm = 1.*/
	/* USBPHY_SET8(0x6a, 0x04);*/
	/*4 13.  wait 1us*/
	udelay(1);

	/* force enter device mode, from K2, FIXME */
	/* force enter device mode */
	/*USBPHY_CLR8(0x6c, 0x10);*/
	/*USBPHY_SET8(0x6c, 0x2E);*/
	/*USBPHY_SET8(0x6d, 0x3E);*/
}

void usb_phy_savecurrent(void)
{
	usb_phy_savecurrent_internal();
	/* 4 14. turn off internal 48Mhz PLL. */
	usb_enable_clock(false);
	DBG(0, "usb save current success\n");
}

void usb_phy_recover(void)
{
#if defined(CONFIG_Z301M) || defined(CONFIG_Z301MF)
	unsigned char value;
#endif
	/* turn on USB reference clock. */
	usb_enable_clock(true);
	/* wait 50 usec. */
	udelay(50);

	#ifdef CONFIG_MTK_UART_USB_SWITCH
	if (!usb_phy_check_in_uart_mode()) {
		/* clean PUPD_BIST_EN */
		/* PUPD_BIST_EN = 1'b0 */
		/* PMIC will use it to detect charger type */
		USBPHY_CLR8(0x1d, 0x10);

		/* force_uart_en, 1'b0 */
		USBPHY_CLR8(0x6b, 0x04);
		/* RG_UART_EN, 1'b0 */
		USBPHY_CLR8(0x6e, 0x1);
		/* force_suspendm, 1'b0 */
		USBPHY_CLR8(0x6a, 0x04);
		USBPHY_CLR8(0x22, 0x02);

		skipDisableUartMode = false;
	} else {
		/*if (!skipDisableUartMode)*/
			return;
	}
	#else
	/* clean PUPD_BIST_EN */
	/* PUPD_BIST_EN = 1'b0 */
	/* PMIC will use it to detect charger type */
	USBPHY_CLR8(0x1d, 0x10);

	/* force_uart_en, 1'b0 */
	USBPHY_CLR8(0x6b, 0x04);
	/* RG_UART_EN, 1'b0 */
	USBPHY_CLR8(0x6e, 0x1);
	/* rg_usb20_gpio_ctl, 1'b0, usb20_gpio_mode, 1'b0 */
	/* force_suspendm, 1'b0 */
	USBPHY_CLR8(0x6a, 0x04);

	USBPHY_CLR8(0x21, 0x03);
	#endif

	/* RG_DPPULLDOWN, 1'b0, RG_DMPULLDOWN, 1'b0 */
	USBPHY_CLR8(0x68, 0x40);
	/* 4 7. RG_DMPULLDOWN = 1'b0 */
	USBPHY_CLR8(0x68, 0x80);
	/* RG_XCVRSEL[1:0], 2'b00. */
	USBPHY_CLR8(0x68, 0x30);
	/* RG_TERMSEL, 1'b0 */
	USBPHY_CLR8(0x68, 0x04);
	/* RG_DATAIN[3:0], 4'b0000 */
	USBPHY_CLR8(0x69, 0x3c);

	/* force_dp_pulldown, 1'b0, force_dm_pulldown, 1'b0*/
	USBPHY_CLR8(0x6a, 0x10);
	/* 4 12. force_dm_pulldown = 1b'0 */
	USBPHY_CLR8(0x6a, 0x20);
	/* 4 13. force_xcversel = 1b'0 */
	USBPHY_CLR8(0x6a, 0x08);
	/* 4 14. force_termsel = 1b'0*/
	USBPHY_CLR8(0x6a, 0x02);
	/* 4 15. force_datain = 1b'0*/
	USBPHY_CLR8(0x6a, 0x80);

	/* RG_USB20_BC11_SW_EN, 1'b0 */
	USBPHY_CLR8(0x1a, 0x80);
	/* RG_USB20_OTG_VBUSCMP_EN, 1'b1 */
	USBPHY_SET8(0x1a, 0x10);
	/*18. PASS RX sensitivity HQA requirement*/
	USBPHY_CLR8(0x18, 0x08);
	USBPHY_SET8(0x18, 0x06);

	/* wait 800 usec. */
	udelay(800);

	/* force enter device mode, from K2, FIXME */
	USBPHY_CLR8(0x6c, 0x10);
	USBPHY_SET8(0x6c, 0x2E);
	USBPHY_SET8(0x6d, 0x3E);

	/* from K2, FIXME */
	#if defined(MTK_HDMI_SUPPORT)
	USBPHY_SET8(0x05, 0x05);
	USBPHY_SET8(0x05, 0x50);
	#endif
	hs_slew_rate_cal();

#if defined(CONFIG_Z301M) || defined(CONFIG_Z301MF)
               printk("Slew Rate usb phy read default eye value, Addr: 0x15=0x%x\n", USBPHY_READ8(0x15));
               value = USBPHY_READ8(0x15);
               USBPHY_CLR8(0x15, 0x70);
               USBPHY_WRITE8(0x15, (value)<<4);
               printk("Slew Rate usb phy read eye value after set, Addr: 0x15=0x%x\n", USBPHY_READ8(0x15));
#endif

	/* adjust VRT_VREF_SEL to 820mv, adjust TERM_VREF_SEL to 460mv */
	printk("VRT & TERM usb phy read default eye value, Addr: 0x5:=0x%x\n", USBPHY_READ8(0x05));
        USBPHY_SET8(0x05, 0x77);
        printk("VRT & TERM usb phy read eye value after set, Addr: 0x5:=0x%x\n", USBPHY_READ8(0x05));

	DBG(0, "usb recovery success\n");
}

/* BC1.2 */
void Charger_Detect_Init(void)
{
	/* turn on USB reference clock. */
	usb_enable_clock(true);
	/* wait 50 usec. */
	udelay(50);
	/* RG_USB20_BC11_SW_EN = 1'b1 */
	USBPHY_SET8(0x1a, 0x80);
	DBG(0, "Charger_Detect_Init\n");
}

void Charger_Detect_Release(void)
{
	/* RG_USB20_BC11_SW_EN = 1'b0 */
	USBPHY_CLR8(0x1a, 0x80);
	udelay(1);
	/* 4 14. turn off internal 48Mhz PLL. */
	usb_enable_clock(false);
	DBG(0, "Charger_Detect_Release\n");
}

#if defined(CONFIG_USB_MTK_CHARGER_DETECT)
CHARGER_TYPE usb_charger_type_detect(void)
{
	UINT8 line_state = 0;
	CHARGER_TYPE CHR_Type_num = CHARGER_UNKNOWN;

	usb_enable_clock(true);
	/*musb_set_soft_conn(true); */
	USBPHY_SET8(0x6a, 0x10);
	/*USBPHY_SET8(0x68, 0x40);*/
	mdelay(100);
	USBPHY_SET8(0x6a, 0x30);
	mdelay(100);
	USBPHY_CLR8(0x68, 0x40);
	USBPHY_CLR8(0x68, 0xc0);
	USBPHY_SET8(0x22, 0x02);
	mdelay(100);

	DBG(0, "======PHY:0x68 = 0x%x\n", USBPHY_READ8(0x68));
	line_state = USBPHY_READ8(0x76) >> 6;
	DBG(0, "==================usb_charger_type_detect:0x76 = 0x%x\n", USBPHY_READ8(0x76));
	DBG(0, "==================usb_charger_type_detect:line_state = 0x%x\n", line_state);

	switch (line_state) {
	case 0x0:
	case 0x1:
		DBG(0, "************charger_type = Host **********\n");
		/*musb_set_soft_conn(false);*/
		USBPHY_CLR8(0x6a, 0x30);
		USBPHY_CLR8(0x22, 0x02);
		CHR_Type_num = STANDARD_HOST;
		break;
	case 0x2:
	case 0x3:
		DBG(0, "======enter adapter type detect ========\n");
		/*musb_set_soft_conn(false);*/
		USBPHY_SET8(0x6a, 0x10);
		mdelay(100);
		USBPHY_SET8(0x6a, 0x30);
		USBPHY_CLR8(0x68, 0x40);
		USBPHY_SET8(0x68, 0x80);
		mdelay(100);
		DBG(0, "======0x68 = 0x%x========\n", USBPHY_READ8(0x68));
		USBPHY_CLR8(0x22, 0x02);
		mdelay(100);

		line_state = USBPHY_READ8(0x76) >> 6;
		DBG(0, "======adapter type: 0x76 = 0x%x========\n", USBPHY_READ8(0x76));
		if (line_state == 0x1) {
			DBG(0, "************charger_type = Non-standard **********\n");
			CHR_Type_num = NONSTANDARD_CHARGER;
		} else {
			DBG(0, "************charger_type = Standard **********\n");
			CHR_Type_num = STANDARD_CHARGER;
		}

		USBPHY_CLR8(0x6a, 0x30);
		USBPHY_CLR8(0x68, 0x80);
		break;
	default:
		break;
	}
	usb_enable_clock(false);
	return CHR_Type_num;
}
#endif

void usb_phy_context_save(void)
{
#ifdef CONFIG_MTK_UART_USB_SWITCH
	in_uart_mode = usb_phy_check_in_uart_mode();
#endif
}

void usb_phy_context_restore(void)
{
#ifdef CONFIG_MTK_UART_USB_SWITCH
	if (in_uart_mode)
		usb_phy_switch_to_uart();
#endif
		usb_phy_savecurrent_internal();
}

#endif
