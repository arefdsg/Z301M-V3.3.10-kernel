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

#ifndef _MT_SPM_INTERNAL_
#define _MT_SPM_INTERNAL_

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/atomic.h>
#include <linux/io.h>
#include <mt-plat/aee.h>

#include "mt_spm.h"
#include "mt_lpae.h"

/**************************************
 * Config and Parameter
 **************************************/
#ifdef MTK_FORCE_CLUSTER1
#define SPM_CTRL_BIG_CPU	1
#else
#define SPM_CTRL_BIG_CPU	0
#endif

#define POWER_ON_VAL1_DEF	0x61015830
#define PCM_FSM_STA_DEF		0x48490

#define PCM_WDT_TIMEOUT		(30 * 32768)	/* 30s */
#define PCM_TIMER_MAX		(0xffffffff - PCM_WDT_TIMEOUT)


/**************************************
 * Define and Declare
 **************************************/
#define CON0_PCM_KICK		(1U << 0)
#define CON0_IM_KICK		(1U << 1)
#define CON0_IM_SLEEP_DVS	(1U << 3)
#define CON0_PCM_SW_RESET	(1U << 15)
#define CON0_CFG_KEY		(SPM_PROJECT_CODE << 16)

#define CON1_IM_SLAVE		(1U << 0)
#define CON1_MIF_APBEN		(1U << 3)
#define CON1_PCM_TIMER_EN	(1U << 5)
#define CON1_IM_NONRP_EN	(1U << 6)
#define CON1_PCM_WDT_EN		(1U << 8)
#define CON1_PCM_WDT_WAKE_MODE	(1U << 9)
#define CON1_SPM_SRAM_SLP_B	(1U << 10)
#define CON1_SPM_SRAM_ISO_B	(1U << 11)
#define CON1_EVENT_LOCK_EN	(1U << 12)
#define CON1_SRCCLKEN_FAST_RESP	  (1U << 13)
#define CON1_MD32_APB_INTERNAL_EN (1U << 14)
#define CON1_CFG_KEY		(SPM_PROJECT_CODE << 16)

#define PCM_PWRIO_EN_R0		(1U << 0)
#define PCM_PWRIO_EN_R7		(1U << 7)
#define PCM_RF_SYNC_R0		(1U << 16)
#define PCM_RF_SYNC_R2		(1U << 18)
#define PCM_RF_SYNC_R6		(1U << 22)
#define PCM_RF_SYNC_R7		(1U << 23)

#define R7_AP_MDSRC_REQ		(1U << 17)

#define R13_EXT_SRCLKENA_0	(1U << 0)
#define R13_EXT_SRCLKENA_1	(1U << 1)
#define R13_MD1_SRCLKENA	(1U << 2)
#define R13_MD1_APSRC_REQ	(1U << 3)
#define R13_AP_MD1SRC_ACK	(1U << 4)
#define R13_MD2_SRCLKENA	(1U << 5)
#define R13_MD32_SRCLKENA	(1U << 6)
#define R13_MD32_APSRC_REQ	(1U << 7)
#define R13_MD_DDR_EN		(1U << 12)
#define R13_CONN_SRCLKENA	(1U << 14)
#define R13_CONN_APSRC_REQ	(1U << 15)

#define PCM_SW_INT0		(1U << 0)
#define PCM_SW_INT1		(1U << 1)
#define PCM_SW_INT2		(1U << 2)
#define PCM_SW_INT3		(1U << 3)
#define PCM_SW_INT4		(1U << 4)
#define PCM_SW_INT5		(1U << 5)
#define PCM_SW_INT6		(1U << 6)
#define PCM_SW_INT7		(1U << 7)
#define PCM_SW_INT_ALL		(PCM_SW_INT7 | PCM_SW_INT6 | PCM_SW_INT5 |	\
				 PCM_SW_INT4 | PCM_SW_INT3 | PCM_SW_INT2 |	\
				 PCM_SW_INT1 | PCM_SW_INT0)

#define CC_SYSCLK0_EN_0		(1U << 0)
#define CC_SYSCLK0_EN_1		(1U << 1)
#define CC_SYSCLK1_EN_0		(1U << 2)
#define CC_SYSCLK1_EN_1		(1U << 3)
#define CC_SYSSETTLE_SEL	(1U << 4)
#define CC_LOCK_INFRA_DCM	(1U << 5)
#define CC_SRCLKENA_MASK_0	(1U << 6)
#define CC_CXO32K_RM_EN_MD1	(1U << 9)
#define CC_CXO32K_RM_EN_MD2	(1U << 10)
#define CC_CLKSQ1_SEL		(1U << 12)
#define CC_DISABLE_DORM_PWR	(1U << 14)
#define CC_MD32_DCM_EN		(1U << 18)

#define ASC_MD_DDR_EN_SEL	(1U << 22)
#define ASC_SRCCLKENI_MASK      (1U << 25)

#define WFI_OP_AND		1
#define WFI_OP_OR		0
#define SEL_MD_DDR_EN		1
#define SEL_MD_APSRC_REQ	0

#define TWAM_CON_EN		(1U << 0)
#define TWAM_CON_SPEED_EN	(1U << 4)

#define PCM_MD32_IRQ_SEL	(1U << 4)

#define ISRM_TWAM		(1U << 2)
#define ISRM_PCM_RETURN		(1U << 3)
#define ISRM_RET_IRQ0		(1U << 8)
#define ISRM_RET_IRQ1		(1U << 9)
#define ISRM_RET_IRQ2		(1U << 10)
#define ISRM_RET_IRQ3		(1U << 11)
#define ISRM_RET_IRQ4		(1U << 12)
#define ISRM_RET_IRQ5		(1U << 13)
#define ISRM_RET_IRQ6		(1U << 14)
#define ISRM_RET_IRQ7		(1U << 15)

#define ISRM_RET_IRQ	(ISRM_RET_IRQ7 | ISRM_RET_IRQ6 |	\
				 ISRM_RET_IRQ5 | ISRM_RET_IRQ4 |	\
				 ISRM_RET_IRQ3 | ISRM_RET_IRQ2 |	\
				 ISRM_RET_IRQ1)
#define ISRM_ALL_EXC_TWAM	(ISRM_RET_IRQ | ISRM_RET_IRQ0 | ISRM_PCM_RETURN)
#define ISRM_ALL		(ISRM_ALL_EXC_TWAM | ISRM_TWAM)

#define ISRS_TWAM		(1U << 2)
#define ISRS_PCM_RETURN		(1U << 3)
#define ISRS_SW_INT0		(1U << 4)

#define ISRC_TWAM		ISRS_TWAM
#define ISRC_ALL_EXC_TWAM	ISRS_PCM_RETURN
#define ISRC_ALL		(ISRC_ALL_EXC_TWAM | ISRC_TWAM)

#define WAKE_MISC_TWAM		(1U << 18)
#define WAKE_MISC_PCM_TIMER	(1U << 19)
#define WAKE_MISC_CPU_WAKE	(1U << 20)

#define SR_PCM_APSRC_REQ	(1U << 0)
#define SR_PCM_F26M_REQ	        (1U << 1)
#define SR_CCIF0_TO_MD_MASK_B	(1U << 2)
#define SR_CCIF0_TO_AP_MASK_B	(1U << 3)
#define SR_CCIF1_TO_MD_MASK_B	(1U << 4)
#define SR_CCIF1_TO_AP_MASK_B	(1U << 5)

struct pcm_desc {
	const char *version;	/* PCM code version */
	const u32 *base;	/* binary array base */
	const u16 size;		/* binary array size */
	const u8 sess;		/* session number */
	const u8 replace;	/* replace mode */

	u32 vec0;		/* event vector 0 config */
	u32 vec1;		/* event vector 1 config */
	u32 vec2;		/* event vector 2 config */
	u32 vec3;		/* event vector 3 config */
	u32 vec4;		/* event vector 4 config */
	u32 vec5;		/* event vector 5 config */
	u32 vec6;		/* event vector 6 config */
	u32 vec7;		/* event vector 7 config */
};

struct pwr_ctrl {
	/* for SPM */
	u32 pcm_flags;
	u32 pcm_flags_cust;	/* can override pcm_flags */
	u32 pcm_reserve;
	u32 timer_val;		/* @ 1T 32K */
	u32 timer_val_cust;	/* @ 1T 32K, can override timer_val */
	u32 wake_src;
	u32 wake_src_cust;	/* can override wake_src */
	u32 wake_src_md32;
	u8 r0_ctrl_en;
	u8 r7_ctrl_en;
	u8 infra_dcm_lock;
	u8 pcm_apsrc_req;
	u8 pcm_f26m_req;

	/* for AP */
	u8 mcusys_idle_mask;
	u8 ca15top_idle_mask;
	u8 ca7top_idle_mask;
	u8 wfi_op;		/* 1:WFI_OP_AND, 0:WFI_OP_OR */
	u8 ca15_wfi0_en;
	u8 ca15_wfi1_en;
	u8 ca15_wfi2_en;
	u8 ca15_wfi3_en;
	u8 ca7_wfi0_en;
	u8 ca7_wfi1_en;
	u8 ca7_wfi2_en;
	u8 ca7_wfi3_en;

	/* for CONN */
	u8 conn_mask;

	/* for MM */
	u8 disp_req_mask;
	u8 mfg_req_mask;
	u8 dsi0_ddr_en_mask;	/* E2 */
	u8 dsi1_ddr_en_mask;	/* E2 */
	u8 dpi_ddr_en_mask;	/* E2 */
	u8 isp0_ddr_en_mask;	/* E2 */
	u8 isp1_ddr_en_mask;	/* E2 */

	/* for other SYS */
	u8 md32_req_mask;
	u8 syspwreq_mask;	/* make 26M off when attach ICE */
	u8 srclkenai_mask;

	/* for scenario */
	u32 param1;
	u32 param2;
	u32 param3;

	/* for logging */
	u8 enable_log;
};

struct wake_status {
	u32 assert_pc;		/* PCM_REG_DATA_INI */
	u32 r12;		/* PCM_REG12_DATA */
	u32 raw_sta;		/* SLEEP_ISR_RAW_STA */
	u32 wake_misc;		/* SLEEP_WAKEUP_MISC */
	u32 timer_out;		/* PCM_TIMER_OUT */
	u32 r13;		/* PCM_REG13_DATA */
	u32 idle_sta;		/* SLEEP_SUBSYS_IDLE_STA */
	u32 debug_flag;		/* PCM_PASR_DPD_3 */
	u32 event_reg;		/* PCM_EVENT_REG_STA */
	u32 isr;		/* SLEEP_ISR_STATUS */
	u32 r9;			/* PCM_REG9_DATA */
	u32 log_index;
};

struct spm_lp_scen {
	struct pcm_desc *pcmdesc;
	struct pwr_ctrl *pwrctrl;
	struct wake_status *wakestatus;
};

extern spinlock_t __spm_lock;
extern atomic_t __spm_mainpll_req;

extern struct spm_lp_scen __spm_suspend;
extern struct spm_lp_scen __spm_dpidle;
extern struct spm_lp_scen __spm_sodi;
extern struct spm_lp_scen __spm_ddrdfs;

extern void __spm_reset_and_init_pcm(const struct pcm_desc *pcmdesc);
extern void __spm_kick_im_to_fetch(const struct pcm_desc *pcmdesc);

extern void __spm_init_pcm_register(void);	/* init r0 and r7 */
extern void __spm_init_event_vector(const struct pcm_desc *pcmdesc);
extern void __spm_set_power_control(const struct pwr_ctrl *pwrctrl);
extern void __spm_set_wakeup_event(const struct pwr_ctrl *pwrctrl);
extern void __spm_kick_pcm_to_run(const struct pwr_ctrl *pwrctrl);

extern void __spm_get_wakeup_status(struct wake_status *wakesta);
extern void __spm_clean_after_wakeup(void);
extern wake_reason_t __spm_output_wake_reason(const struct wake_status *wakesta,
					      const struct pcm_desc *pcmdesc, bool suspend);

extern int spm_fs_init(void);
/* extern int is_ext_buck_exist(void); */

/* TODO: wait irq driver ready
extern int mt_irq_mask_all(struct mtk_irq_mask *mask);
extern int mt_irq_mask_restore(struct mtk_irq_mask *mask);
extern void mt_irq_unmask_for_sleep(unsigned int irq);
*/

extern int request_uart_to_sleep(void);
extern int request_uart_to_wakeup(void);
extern void mtk_uart_restore(void);
extern void dump_uart_reg(void);

extern struct wake_status spm_wakesta;
extern u32 spm_get_last_wakeup_src(void);
/**************************************
 * Macro and Inline
 **************************************/
#define EVENT_VEC(event, resume, imme, pc)	\
	(((pc) << 16) |				\
	 (!!(imme) << 6) |			\
	 (!!(resume) << 5) |			\
	 ((event) & 0x1f))

#define spm_emerg(fmt, args...)		pr_emerg("[SPM] " fmt, ##args)
#define spm_alert(fmt, args...)		pr_alert("[SPM] " fmt, ##args)
#define spm_crit(fmt, args...)		pr_crit("[SPM] " fmt, ##args)
#define spm_err(fmt, args...)		pr_err("[SPM] " fmt, ##args)
#define spm_warn(fmt, args...)		pr_warn("[SPM] " fmt, ##args)
#define spm_notice(fmt, args...)	pr_notice("[SPM] " fmt, ##args)
#define spm_info(fmt, args...)		pr_info("[SPM] " fmt, ##args)
#define spm_debug(fmt, args...)		pr_debug("[SPM] " fmt, ##args)	/* pr_debug show nothing */

/* just use in suspend flow for important log due to console suspend */
#define spm_crit2(fmt, args...)		\
do {					\
	aee_sram_printk(fmt, ##args);	\
	spm_crit(fmt, ##args);		\
} while (0)

#define wfi_with_sync()					\
do {							\
	isb();						\
	mb();						\
	__asm__ __volatile__("wfi" : : : "memory");	\
} while (0)

static inline u32 base_va_to_pa(const u32 *base)
{
	phys_addr_t pa = virt_to_phys(base);

	MAPPING_DRAM_ACCESS_ADDR(pa);	/* for 4GB mode */
	return (u32) pa;
}

static inline int is_ext_buck_exist(void)
{
	return 1;
}

static inline void set_pwrctrl_pcm_flags(struct pwr_ctrl *pwrctrl, u32 flags)
{
	if (is_ext_buck_exist())
		flags &= ~SPM_BUCK_SEL;
	else
		flags |= SPM_BUCK_SEL;

	if (pwrctrl->pcm_flags_cust == 0)
		pwrctrl->pcm_flags = flags;
	else
		pwrctrl->pcm_flags = pwrctrl->pcm_flags_cust;
}

static inline void set_pwrctrl_pcm_data(struct pwr_ctrl *pwrctrl, u32 data)
{
	pwrctrl->pcm_reserve = data;
}

static inline void set_flags_for_mainpll(u32 *flags)
{
	if (atomic_read(&__spm_mainpll_req) != 0)
		*flags |= SPM_MAINPLL_PDN_DIS;
	else
		*flags &= ~SPM_MAINPLL_PDN_DIS;
}

#endif
