#include <linux/types.h>
#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <mt-plat/mt_boot.h>
#include <mt-plat/battery_meter.h>
#include <mach/mt_battery_meter.h>
#include <mach/mt_charging.h>
#include <mach/mt_pmic.h>
#include "bq25896_Z301M.h"
#include <mt-plat/mt_gpio.h>
#include <mt-plat/charging.h>
#include <mt-plat/battery_common.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <linux/rtc.h>
#include <linux/ktime.h>
#include <linux/atomic.h>
#include <linux/alarmtimer.h>

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#include <mt-plat/diso.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#ifdef MTK_DISCRETE_SWITCH
#include <mt-plat/mt_gpio.h>
#endif
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#endif
#endif

/* ============================================================*/
/*define*/
/* ============================================================*/
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))
#define bq25896_TAG "ChargerSetting"

bool bootup_init_done = false;
int init_delay = 15000;
/* extern */
extern bool b_eng_charging_limit;
extern int eng_charging_limit_soc;
extern int b_stop_thermal;
extern int b_stop_jeita;
extern int b_otg_probe;

/* delay work */
static struct delayed_work adapter_detect_work;
static struct delayed_work jeita_work;
static struct delayed_work initial_work;

/* AC alarm */
struct wake_lock alarm_cable_lock;
struct alarm AC_charging_alarm_timer;
atomic_t AC_charging_alarm;
kal_bool chr_wake_up_charging = KAL_TRUE;
kal_bool chr_wake_up_discharging = KAL_TRUE;
enum Adapter_flags{
	Z301M_UNFEFINED,
	Z301M_SDP,
	Z301M_CDP,
	Z301M_DCP,
	Z301M_TYPEC_1P5A,
	Z301M_TYPEC_3A,
	Z301M_TYPEC_PD,
	Z301M_TYPEC_OTHERS,
	Z301M_Samsung,
	Z301M_PowerBank,
	Z301M_DCP_750k,
	Z301M_DCP_200k,
	Z301M_DCP_others,
};

/* Charging TimeOut Limit */
struct timespec last_chg_time;
struct timespec now_chg_time;
int last_chg_time_toggle = 0;
int chg_timeout_threshold = 259200;//COS:over 72hr.
//int chg_timeout_threshold = 120;//COS:over 72hr.
int chg_timeout_mos = 0;//MOS:over 72hr.
EXPORT_SYMBOL(chg_timeout_mos);
int chg_timeout_cos = 0;//MOS:over 72hr.
int chg_timeout_limit_old = 0;//COS:chg_timeout && BAT_SOC over 58%; MOS:property
EXPORT_SYMBOL(chg_timeout_limit_old);
int chg_timeout_limit = 0;//COS:chg_timeout && BAT_SOC over 58%; MOS:property
EXPORT_SYMBOL(chg_timeout_limit);
int chg_timeout_limit_stopFG = 0;
EXPORT_SYMBOL(chg_timeout_limit_stopFG);

static char *Adapter_flag_str[] ={
	"UNDEFINED",
	"SDP",
	"CDP",
	"DCP",
	"Type-c 1.5A",
	"Type-c 3A",
	"Type-c PD",
	"Type-c Others",
	"Samsung",
	"Power Bank",
	"DCP_750k",
	"DCP_200k",
	"DCP_others",
};

/* type C */
enum type_c_type {
        TYPE_C_Unknown=0,
        TYPE_C_1_5_A,
        TYPE_C_3_A,
        TYPE_C_PD,
	TYPE_C_OTHERS,
	TYPE_C_DEFAULT,
};

static char *Type_c_str[]={
	"Type-C Unknown",
	"Type-C 1.5A",
	"Type-C 3A",
	"Type-C PD",
	"Type-C Others",
	"Type-C Default",
};

struct power_supply *typec_psy = NULL, *pd_psy = NULL;
static int g_type_c_flag = TYPE_C_Unknown;
static int typec_online = 0;//1: Attached as UFP ; 0: Detached or attached as DFP
static unsigned int typec_current_max;//uA
static unsigned int typec_voltage_max;//uV
static unsigned int typec_voltage_min;//uV

/* PD */
static int pd_online = 0;//1: Attached as UFP ; 0: Detached or attached as DFP
static unsigned int pd_current_max;//uA
static unsigned int pd_voltage_max;//uV

/* Thermal policy : aux_in5*/
int thermal_level = 0;
int charger_thermal_level1 = 50;
int charger_thermal_level2 = 52;
int charger_thermal_level3 = 60;
EXPORT_SYMBOL(charger_thermal_level1);
EXPORT_SYMBOL(charger_thermal_level2);
EXPORT_SYMBOL(charger_thermal_level3);

#define CHARGER_NTC_CHANNEL_NUM 15
struct vadc_map_pt {
	int32_t x;
	int32_t y;
};
static const struct vadc_map_pt aux_in5_adc[] = {
	{	-10 ,	1557098 	},
	{	-9 ,	1544333 	},
	{	-8 ,	1531219 	},
	{	-7 ,	1488732 	},
	{	-6 ,	1473700 	},
	{	-5 ,	1458224 	},
	{	-4 ,	1442311 	},
	{	-3 ,	1458224 	},
	{	-2 ,	1442311 	},
	{	-1 ,	1425958 	},
	{	0 ,	1409188 	},
	{	1 ,	1392003 	},
	{	2 ,	1374408 	},
	{	3 ,	1356421 	},
	{	4 ,	1338059 	},
	{	5 ,	1319321 	},
	{	6 ,	1300250 	},
	{	7 ,	1280834 	},
	{	8 ,	1261110 	},
	{	9 ,	1241098 	},
	{	10 ,	1220794 	},
	{	11 ,	1200260 	},
	{	12 ,	1179482 	},
	{	13 ,	1158495 	},
	{	14 ,	1137358 	},
	{	15 ,	1116031 	},
	{	16 ,	1094588 	},
	{	17 ,	1073050 	},
	{	18 ,	1051404 	},
	{	19 ,	1029716 	},
	{	20 ,	1008025 	},
	{	21 ,	986294 	},
	{	22 ,	964617 	},
	{	23 ,	943020 	},
	{	24 ,	921437 	},
	{	25 ,	900000 	},
	{	26 ,	878676 	},
	{	27 ,	857503 	},
	{	28 ,	836501 	},
	{	29 ,	815694 	},
	{	30 ,	795098 	},
	{	31 ,	774733 	},
	{	32 ,	754619 	},
	{	33 ,	734773 	},
	{	34 ,	715212 	},
	{	35 ,	695949 	},
	{	36 ,	676993 	},
	{	37 ,	658361 	},
	{	38 ,	640057 	},
	{	39 ,	617221 	},
	{	40 ,	604495 	},
	{	41 ,	587242 	},
	{	42 ,	570349 	},
	{	43 ,	553825 	},
	{	44 ,	537675 	},
	{	45 ,	521890 	},
	{	46 ,	506488 	},
	{	47 ,	491452 	},
	{	48 ,	476801 	},
	{	49 ,	462506 	},
	{	50 ,	448598 	},
	{	51 ,	435050 	},
	{	52 ,	421862 	},
	{	53 ,	409040 	},
	{	54 ,	396568 	},
	{	55 ,	384449 	},
	{	56 ,	372675 	},
	{	57 ,	361243 	},
	{	58 ,	350153 	},
	{	59 ,	339376 	},
	{	60 ,	328931 	},
	{	61 ,	318787 	},
	{	62 ,	308959 	},
	{	63 ,	299425 	},
	{	64 ,	290199 	},
	{	65 ,	281243 	},
	{	66 ,	272570 	},
	{	67 ,	264164 	},
	{	68 ,	256022 	},
	{	69 ,	248142 	},
	{	70 ,	240519 	},
	{	71 ,	233120 	},
	{	72 ,	225968 	},
	{	73 ,	219042 	},
	{	74 ,	212334 	},
	{	75 ,	205852 	},
	{	76 ,	199559 	},
	{	77 ,	193488 	},
	{	78 ,	187602 	},
	{	79 ,	181906 	},
	{	80 ,	176389 	},
	{	81 ,	171055 	},
	{	82 ,	165894 	},
	{	83 ,	160905 	},
	{	84 ,	156071 	},
	{	85 ,	151393 	},
	{	86 ,	146866 	},
	{	87 ,	142485 	},
	{	88 ,	138245 	},
	{	89 ,	134142 	},
	{	90 ,	130170 	},
	{	91 ,	126324 	},
	{	92 ,	122604 	},
	{	93 ,	119001 	},
	{	94 ,	115515 	},
	{	95 ,	112140 	},
	{	96 ,	108873 	},
	{	97 ,	105709 	},
	{	98 ,	102647 	},
	{	99 ,	99681 	},
	{	100 ,	96811 	},
};

/*============================================================*/
/*global variable*/
/*============================================================*/
/*#if !defined(GPIO_CHR_CE_PIN)
#ifdef GPIO_SWCHARGER_EN_PIN
#define GPIO_CHR_CE_PIN GPIO_SWCHARGER_EN_PIN
#else
#define  GPIO_CHR_CE_PIN (19 | 0x80000000)
#endif
#endif*/
kal_bool charging_type_det_done = KAL_TRUE;

const unsigned int VBAT_CV_VTH[] = {
	3504000,    3520000,    3536000,    3552000,
	3568000,    3584000,    3600000,    3616000,
	3632000,    3648000,    3664000,    3680000,
	3696000,    3712000,	3728000,    3744000,
	3760000,    3776000,    3792000,    3808000,
	3824000,    3840000,    3856000,    3872000,
	3888000,    3904000,    3920000,    3936000,
	3952000,    3968000,    3984000,    4000000,
	4016000,    4032000,    4048000,    4064000,
	4080000,    4096000,    4112000,    4128000,
	4144000,    4160000,    4176000,    4192000,
	4208000,    4224000,    4240000,    4256000,
	4272000,    4288000,    4304000,    4320000,
	4336000,    4352000,    4368000
};

const unsigned int CS_VTH[] = {
	51200,  57600,  64000,  70400,
	76800,  83200,  89600,  96000,
	102400, 108800, 115200, 121600,
	128000, 134400, 140800, 147200,
	153600, 160000, 166400, 172800,
	179200, 185600, 192000, 198400,
	204800, 211200, 217600, 224000
};

const unsigned int INPUT_CS_VTH[] = {
	CHARGE_CURRENT_100_00_MA,  CHARGE_CURRENT_150_00_MA,	CHARGE_CURRENT_500_00_MA,
	CHARGE_CURRENT_900_00_MA,
	CHARGE_CURRENT_1000_00_MA, CHARGE_CURRENT_1500_00_MA,  CHARGE_CURRENT_2000_00_MA,
	CHARGE_CURRENT_MAX
};

const unsigned int VCDT_HV_VTH[] = {
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V,	  BATTERY_VOLT_04_300000_V,
	BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V,	  BATTERY_VOLT_04_500000_V,
	BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V,	  BATTERY_VOLT_06_500000_V,
	BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V,	  BATTERY_VOLT_09_500000_V,
	BATTERY_VOLT_10_500000_V
};

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
#define SW_POLLING_PERIOD 100 /*100 ms*/
#define MSEC_TO_NSEC(x)		(x * 1000000UL)

static DEFINE_MUTEX(diso_polling_mutex);
static DECLARE_WAIT_QUEUE_HEAD(diso_polling_thread_wq);
static struct hrtimer diso_kthread_timer;
static kal_bool diso_thread_timeout = KAL_FALSE;
static struct delayed_work diso_polling_work;
static void diso_polling_handler(struct work_struct *work);
static DISO_Polling_Data DISO_Polling;
#else
DISO_IRQ_Data DISO_IRQ;
#endif
int g_diso_state	= 0;

static char *DISO_state_s[8] = {
		  "IDLE",
		  "OTG_ONLY",
		  "USB_ONLY",
		  "USB_WITH_OTG",
		  "DC_ONLY",
		  "DC_WITH_OTG",
		  "DC_WITH_USB",
		  "DC_USB_OTG",
};
#endif


/*============================================================*/
/*function prototype*/
/*============================================================*/
/*============================================================*/
/*extern variable*/
/*============================================================*/
/*============================================================*/
/*extern function*/
/*============================================================*/
static unsigned int charging_error;
static unsigned int charging_get_error_state(void);
static unsigned int charging_set_error_state(void *data);
int proj_id = 5;
extern int hw_id_charger;
extern int Read_PROJ_ID(void);
extern void charging_pad_or_standac(void);
extern int ADCPWR_enable(bool bEnable);
extern int DPM_SW_enable(bool bEnable);
extern int ads1013_write_byte(int value);
extern int thermal_ads1013_adc_temp(int mode);
extern kal_bool upmu_is_chr_det(void);
extern int ft5726_cable_status_handler(int state);
/*============================================================*/
unsigned int charging_value_to_parameter(const unsigned int *parameter, const unsigned int array_size,
const unsigned int val)
{
	unsigned int temp_param;

	if (val < array_size) {
		temp_param = parameter[val];
	} else {
		battery_log(BAT_LOG_CRTI, "Can't find the parameter \r\n");
		temp_param = parameter[0];
	}

	return temp_param;
}

unsigned int charging_parameter_to_value(const unsigned int *parameter, const unsigned int array_size,
const unsigned int val)
{
	unsigned int i;

	pr_debug("array_size = %d\r\n", array_size);

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	pr_debug("NO register value match. val=%d\r\n", val);
	/*TODO: ASSERT(0);*/
	return 0;
}

static unsigned int bmt_find_closest_level(const unsigned int *pList, unsigned int number, unsigned int level)
{
	unsigned int i;
	unsigned int max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = KAL_TRUE;
	else
		max_value_in_last_element = KAL_FALSE;

	if (max_value_in_last_element == KAL_TRUE) {
		/*max value in the last element*/
		for (i = (number-1); i != 0; i--) {
			if (pList[i] <= level)
				return pList[i];
		}

		pr_debug("Can't find closest level, small value first \r\n");
		return pList[0];
		/*return CHARGE_CURRENT_0_00_MA;*/
	} else {
		/*max value in the first element*/
		for (i = 0; i < number; i++) {
			if (pList[i] <= level)
				return pList[i];
		}

		pr_debug("Can't find closest level, large value first\r\n");
		return pList[number - 1];
		/*return CHARGE_CURRENT_0_00_MA;*/
	}
}

static unsigned int charging_hw_init(void *data)
{
	unsigned int status = STATUS_OK;

	charging_pad_or_standac();
	return status;
}

static unsigned int charging_dump_register(void *data)
{
	unsigned int status = STATUS_OK;

	pr_debug("charging_dump_register\r\n");

	bq25896_dump_register();

	return status;
}

static unsigned int charging_enable(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int enable = *(unsigned int *)(data);

	if (KAL_TRUE == enable) {
		bq25896_set_en_hiz(0x0);
		bq25896_set_chg_config(0x1); /*charger enable*/
		battery_log(BAT_LOG_FULL, "[charging_enable] bq25896_set_en_hiz(0x0)\n");
	} else {
	#if defined(CONFIG_USB_MTK_HDRC_HCD)
		if (mt_usb_is_device())
	#endif
			bq25896_set_chg_config(0x0);
		if (charging_get_error_state()) {
			battery_log(BAT_LOG_FULL, "[charging_enable] bq25896_set_en_hiz(0x1)\n");
			bq25896_set_en_hiz(0x1);	/* disable power path */
		}
	}
	return status;
}

static unsigned int charging_set_cv_voltage(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned short register_value;
	unsigned int cv_value = *(unsigned int *)(data);

	if (cv_value == BATTERY_VOLT_04_200000_V) {
	#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
		/*highest of voltage will be 4.3V, because powerpath limitation*/
#if 1 /*if use BQ24296M, modify this condition to 0*/
		cv_value = 4304000;
#else
		cv_value = 4352000;
#endif
	#else
		/*use nearest value*/
		cv_value = 4208000;
	#endif
	}
	register_value = charging_parameter_to_value(VBAT_CV_VTH, GETARRAYNUM(VBAT_CV_VTH), cv_value);
	bq25896_set_vreg(register_value);
	
	return status;
}

static unsigned int charging_get_current(void *data)
{
	unsigned int status = STATUS_OK;

	unsigned char ret_val = 0;

	/*Get current level*/
	bq25896_read_interface((unsigned char)(bq25896_CON4), (&ret_val), (unsigned char)(CON4_ICHG_MASK),(unsigned char)(CON4_ICHG_SHIFT));

	/*Parsing*/
	ret_val = (ret_val*64) + 512;

	*(unsigned int *)data = ret_val;
	return status;
}

static unsigned int charging_set_current(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;
	unsigned int current_value = *(unsigned int *)data;

	array_size = GETARRAYNUM(CS_VTH);
	set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value);
	register_value = charging_parameter_to_value(CS_VTH, array_size, set_chr_current);
	bq25896_set_ichg(register_value);

	return status;
}

static unsigned int charging_set_input_current(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	array_size = GETARRAYNUM(INPUT_CS_VTH);
	set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size, *(unsigned int *)data);
	register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size, set_chr_current);
	
	bq25896_set_iinlim(register_value);

	return status;
}

static unsigned int charging_get_charging_status(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int ret_val;

	ret_val = bq25896_get_chrg_stat();

	if (ret_val == 0x3)
		*(unsigned int *)data = KAL_TRUE;
	else
		*(unsigned int *)data = KAL_FALSE;

	return status;
}

static unsigned int charging_get_temp(void *data)
{
       unsigned int status = STATUS_OK;
       unsigned int ret_val;

       ret_val = bq25896_get_tspct();
       *(unsigned int *)data = ret_val;

       return status;
}

static unsigned int charging_get_power_good_status(void *data)
{
	unsigned int status = STATUS_OK;

	*(unsigned int *)data = bq25896_get_pg_stat();

	return status;
}

static unsigned int charging_reset_watch_dog_timer(void *data)
{
	unsigned int status = STATUS_OK;

	battery_log(BAT_LOG_CRTI, "[%s][%s]\n", bq25896_TAG, __func__);

	bq25896_set_wd_rst(0x1); /*Kick watchdog*/

	return status;
}

static unsigned int charging_set_hv_threshold(void *data)
{
	unsigned int status = STATUS_OK;

	unsigned int set_hv_voltage;
	unsigned int array_size;
	unsigned short register_value;
	unsigned int voltage = *(unsigned int *)(data);

	array_size = GETARRAYNUM(VCDT_HV_VTH);
	set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size, set_hv_voltage);
	upmu_set_rg_vcdt_hv_vth(register_value);

	return status;
}

static unsigned int charging_get_hv_status(void *data)
{
	unsigned int status = STATUS_OK;

	*(kal_bool *)(data) = upmu_get_rgs_vcdt_hv_det();

	return status;
}


static unsigned int charging_get_battery_status(void *data)
{
	unsigned int status = STATUS_OK;

	upmu_set_baton_tdet_en(1);
	upmu_set_rg_baton_en(1);
	*(kal_bool *)(data) = upmu_get_rgs_baton_undet();

	return status;
}

static unsigned int charging_get_charger_det_status(void *data)
{
	unsigned int status = STATUS_OK;

	*(kal_bool *)(data) = upmu_get_rgs_chrdet();

	return status;
}

kal_bool charging_type_detection_done(void)
{
	return charging_type_det_done;
}

static unsigned int charging_get_charger_type(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(CHARGER_TYPE *) (data) = STANDARD_HOST;
#else
	*(CHARGER_TYPE *) (data) = hw_charging_get_charger_type();
#endif
	return status;
}

static unsigned int charging_get_is_pcm_timer_trigger(void *data)
{
	unsigned int status = STATUS_OK;

	return status;
}

static unsigned int charging_set_platform_reset(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	pr_debug("charging_set_platform_reset\n");
	kernel_restart("battery service reboot system");
#endif
	return status;
}

static unsigned int charging_get_platform_boot_mode(void *data)
{
	unsigned int status = STATUS_OK;

	*(unsigned int *)(data) = get_boot_mode();

	pr_debug("get_boot_mode=%d\n", get_boot_mode());

	return status;
}

static unsigned int charging_set_power_off(void *data)
{
	unsigned int status = STATUS_OK;

	pr_debug("charging_set_power_off\n");
	kernel_power_off();

	return status;
}

static unsigned int charging_get_power_source(void *data)
{
	unsigned int status = STATUS_OK;

#if 0	/*#if defined(MTK_POWER_EXT_DETECT)*/
	if (MT_BOARD_PHONE == mt_get_board_type())
		*(kal_bool *)data = KAL_FALSE;
	else
		*(kal_bool *)data = KAL_TRUE;
#else
	*(kal_bool *)data = KAL_FALSE;
#endif

	return status;
}

static unsigned int charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;
}

static unsigned int charging_set_ta_current_pattern(void *data)
{
	return STATUS_UNSUPPORTED;
}

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
void set_vusb_auxadc_irq(bool enable, bool flag)
{
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
	hrtimer_cancel(&diso_kthread_timer);

	DISO_Polling.reset_polling = KAL_TRUE;
	DISO_Polling.vusb_polling_measure.notify_irq_en = enable;
	DISO_Polling.vusb_polling_measure.notify_irq = flag;

	hrtimer_start(&diso_kthread_timer, ktime_set(0, MSEC_TO_NSEC(SW_POLLING_PERIOD)), HRTIMER_MODE_REL);
#else
	unsigned short threshold = 0;

	if (enable) {
		if (flag == 0)
			threshold = DISO_IRQ.vusb_measure_channel.falling_threshold;
		else
			threshold = DISO_IRQ.vusb_measure_channel.rising_threshold;

		threshold = (threshold * R_DISO_VBUS_PULL_DOWN)/(R_DISO_VBUS_PULL_DOWN + R_DISO_VBUS_PULL_UP);
		mt_auxadc_enableBackgroundDection(DISO_IRQ.vusb_measure_channel.number, threshold,
		DISO_IRQ.vusb_measure_channel.period, DISO_IRQ.vusb_measure_channel.debounce, flag);
	} else {
		mt_auxadc_disableBackgroundDection(DISO_IRQ.vusb_measure_channel.number);
	}
#endif
	pr_debug(" [%s] enable: %d, flag: %d!\n", __func__, enable, flag);
}

void set_vdc_auxadc_irq(bool enable, bool flag)
{
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
	hrtimer_cancel(&diso_kthread_timer);

	DISO_Polling.reset_polling = KAL_TRUE;
	DISO_Polling.vdc_polling_measure.notify_irq_en = enable;
	DISO_Polling.vdc_polling_measure.notify_irq = flag;

	hrtimer_start(&diso_kthread_timer, ktime_set(0, MSEC_TO_NSEC(SW_POLLING_PERIOD)), HRTIMER_MODE_REL);
#else
	unsigned short threshold = 0;

	if (enable) {
		if (flag == 0)
			threshold = DISO_IRQ.vdc_measure_channel.falling_threshold;
		else
			threshold = DISO_IRQ.vdc_measure_channel.rising_threshold;

		threshold = (threshold * R_DISO_DC_PULL_DOWN)/(R_DISO_DC_PULL_DOWN + R_DISO_DC_PULL_UP);
		mt_auxadc_enableBackgroundDection(DISO_IRQ.vdc_measure_channel.number, threshold,
		DISO_IRQ.vdc_measure_channel.period, DISO_IRQ.vdc_measure_channel.debounce, flag);
	} else {
		mt_auxadc_disableBackgroundDection(DISO_IRQ.vdc_measure_channel.number);
	}
#endif
	pr_debug(" [%s] enable: %d, flag: %d!\n", __func__, enable, flag);
}

#if !defined(MTK_AUXADC_IRQ_SUPPORT)
static void diso_polling_handler(struct work_struct *work)
{
	int trigger_channel = -1;
	int trigger_flag = -1;

	if (DISO_Polling.vdc_polling_measure.notify_irq_en)
		trigger_channel = AP_AUXADC_DISO_VDC_CHANNEL;
	else if (DISO_Polling.vusb_polling_measure.notify_irq_en)
		trigger_channel = AP_AUXADC_DISO_VUSB_CHANNEL;

	pr_debug("[DISO]auxadc handler triggered\n");
	switch (trigger_channel) {
	case AP_AUXADC_DISO_VDC_CHANNEL:
		trigger_flag = DISO_Polling.vdc_polling_measure.notify_irq;
		pr_debug("[DISO]VDC IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel, trigger_flag);
#ifdef MTK_DISCRETE_SWITCH /*for DSC DC plugin handle */
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_FALLING);
		if (trigger_flag == DISO_IRQ_RISING) {
			DISO_data.diso_state.pre_vusb_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[2]);
		}
#else /*for load switch OTG leakage handle*/
		set_vdc_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag) & 0x1);
		if (trigger_flag == DISO_IRQ_RISING) {
			DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_ONLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[5]);
		} else if (trigger_flag == DISO_IRQ_FALLING) {
			DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_ONLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[1]);
		} else
			pr_debug("[%s] wrong trigger flag!\n", __func__);
#endif
		break;
	case AP_AUXADC_DISO_VUSB_CHANNEL:
		trigger_flag = DISO_Polling.vusb_polling_measure.notify_irq;
		pr_debug("[DISO]VUSB IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel, trigger_flag);
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		if (trigger_flag == DISO_IRQ_FALLING) {
			DISO_data.diso_state.pre_vusb_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[4]);
		} else if (trigger_flag == DISO_IRQ_RISING) {
			DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[6]);
		} else
			pr_debug("[%s] wrong trigger flag!\n", __func__);
			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag)&0x1);
		break;
	default:
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		pr_debug("[DISO]VUSB auxadc IRQ triggered ERROR OR TEST\n");
		return; /* in error or unexecpt state just return */
	}

	g_diso_state = *(int *)&DISO_data.diso_state;
	pr_debug("[DISO]g_diso_state: 0x%x\n", g_diso_state);
	DISO_data.irq_callback_func(0, NULL);
}
#else
static irqreturn_t diso_auxadc_irq_handler(int irq, void *dev_id)
{
	int trigger_channel = -1;
	int trigger_flag = -1;

	trigger_channel = mt_auxadc_getCurrentChannel();
	pr_debug("[DISO]auxadc handler triggered\n");
	switch (trigger_channel) {
	case AP_AUXADC_DISO_VDC_CHANNEL:
		trigger_flag = mt_auxadc_getCurrentTrigger();
		pr_debug("[DISO]VDC IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel, trigger_flag);
#ifdef MTK_DISCRETE_SWITCH /*for DSC DC plugin handle */
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_FALLING);
		if (trigger_flag == DISO_IRQ_RISING) {
			DISO_data.diso_state.pre_vusb_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[2]);
		}
#else /*for load switch OTG leakage handle*/
		set_vdc_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag) & 0x1);
		if (trigger_flag == DISO_IRQ_RISING) {
			DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_ONLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[5]);
			} else {
			DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_ONLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[1]);
		}
#endif
		break;
	case AP_AUXADC_DISO_VUSB_CHANNEL:
		trigger_flag = mt_auxadc_getCurrentTrigger();
		pr_debug("[DISO]VUSB IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel, trigger_flag);
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		if (trigger_flag == DISO_IRQ_FALLING) {
			DISO_data.diso_state.pre_vusb_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[4]);
		} else {
			DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[6]);
		}
		set_vusb_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag)&0x1);
		break;
	default:
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		pr_debug("[DISO]VUSB auxadc IRQ triggered ERROR OR TEST\n");
		return IRQ_HANDLED; /* in error or unexecpt state just return */
	}
	g_diso_state = *(int *)&DISO_data.diso_state;
	return IRQ_WAKE_THREAD;
}
#endif

static unsigned int diso_get_current_voltage(int Channel)
{
	int ret = 0, data[4], i, ret_value = 0, ret_temp = 0, times = 5;

	if (IMM_IsAdcInitReady() == 0) {
		pr_debug("[DISO] AUXADC is not ready");
		return 0;
	}

	i = times;
	while (i--) {
		ret_value = IMM_GetOneChannelValue(Channel, data, &ret_temp);
		if (ret_value == 0) {
			ret += ret_temp;
		} else {
			times = times > 1 ? times - 1 : 1;
			pr_debug("[diso_get_current_voltage] ret_value=%d, times=%d\n",
			ret_value, times);
		}
	}

	ret = ret*1500/4096;
	ret = ret/times;

	return  ret;
}

static void _get_diso_interrupt_state(void)
{
	int vol = 0;
	int diso_state = 0;

	mdelay(AUXADC_CHANNEL_DELAY_PERIOD);

	vol = diso_get_current_voltage(AP_AUXADC_DISO_VDC_CHANNEL);
	vol = (R_DISO_DC_PULL_UP + R_DISO_DC_PULL_DOWN)*100*vol/(R_DISO_DC_PULL_DOWN)/100;
	pr_debug("[DISO]	Current DC voltage mV = %d\n", vol);

	/* force delay for switching as no flag for check switching done */
	mdelay(SWITCH_RISING_TIMING + LOAD_SWITCH_TIMING_MARGIN);
	if (vol > VDC_MIN_VOLTAGE/1000 && vol < VDC_MAX_VOLTAGE/1000)
		diso_state |= 0x4; /*SET DC bit as 1*/
	else
		diso_state &= ~0x4; /*SET DC bit as 0*/


	vol = diso_get_current_voltage(AP_AUXADC_DISO_VUSB_CHANNEL);
	vol = (R_DISO_VBUS_PULL_UP + R_DISO_VBUS_PULL_DOWN)*100*vol/(R_DISO_VBUS_PULL_DOWN)/100;
	pr_debug("[DISO]	Current VBUS voltage  mV = %d\n", vol);

	if (vol > VBUS_MIN_VOLTAGE/1000 && vol < VBUS_MAX_VOLTAGE/1000) {
		if (!mt_usb_is_device()) {
			diso_state |= 0x1; /*SET OTG bit as 1*/
			diso_state &= ~0x2; /*SET VBUS bit as 0*/
		} else {
			diso_state &= ~0x1; /*SET OTG bit as 0*/
			diso_state |= 0x2; /*SET VBUS bit as 1;*/
		}

	} else {
		diso_state &= 0x4; /*SET OTG and VBUS bit as 0*/
	}
	pr_debug("[DISO] DISO_STATE==0x%x\n", diso_state);
	g_diso_state = diso_state;
}
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
int _get_irq_direction(int pre_vol, int cur_vol)
{
	int ret = -1;

	/*threshold 1000mv*/
	if ((cur_vol - pre_vol) > 1000)
		ret = DISO_IRQ_RISING;
	else if ((pre_vol - cur_vol) > 1000)
		ret = DISO_IRQ_FALLING;

	return ret;
}

static void _get_polling_state(void)
{
	int vdc_vol = 0, vusb_vol = 0;
	int vdc_vol_dir = -1;
	int vusb_vol_dir = -1;

	DISO_polling_channel *VDC_Polling = &DISO_Polling.vdc_polling_measure;
	DISO_polling_channel *VUSB_Polling = &DISO_Polling.vusb_polling_measure;

	vdc_vol = diso_get_current_voltage(AP_AUXADC_DISO_VDC_CHANNEL);
	vdc_vol = (R_DISO_DC_PULL_UP + R_DISO_DC_PULL_DOWN)*100*vdc_vol/(R_DISO_DC_PULL_DOWN)/100;

	vusb_vol = diso_get_current_voltage(AP_AUXADC_DISO_VUSB_CHANNEL);
	vusb_vol = (R_DISO_VBUS_PULL_UP + R_DISO_VBUS_PULL_DOWN)*100*vusb_vol/(R_DISO_VBUS_PULL_DOWN)/100;

	VDC_Polling->preVoltage = VDC_Polling->curVoltage;
	VUSB_Polling->preVoltage = VUSB_Polling->curVoltage;
	VDC_Polling->curVoltage = vdc_vol;
	VUSB_Polling->curVoltage = vusb_vol;

	if (DISO_Polling.reset_polling) {
		DISO_Polling.reset_polling = KAL_FALSE;
		VDC_Polling->preVoltage = vdc_vol;
		VUSB_Polling->preVoltage = vusb_vol;

		if (vdc_vol > 1000)
			vdc_vol_dir = DISO_IRQ_RISING;
		else
			vdc_vol_dir = DISO_IRQ_FALLING;

		if (vusb_vol > 1000)
			vusb_vol_dir = DISO_IRQ_RISING;
		else
			vusb_vol_dir = DISO_IRQ_FALLING;
	} else {
		/*get voltage direction*/
		vdc_vol_dir = _get_irq_direction(VDC_Polling->preVoltage, VDC_Polling->curVoltage);
		vusb_vol_dir = _get_irq_direction(VUSB_Polling->preVoltage, VUSB_Polling->curVoltage);
	}

	if (VDC_Polling->notify_irq_en &&
	(vdc_vol_dir == VDC_Polling->notify_irq)) {
		schedule_delayed_work(&diso_polling_work, 10*HZ/1000); /*10ms*/
		pr_debug("[%s] ready to trig VDC irq, irq: %d\n",
		__func__, VDC_Polling->notify_irq);
	} else if (VUSB_Polling->notify_irq_en && (vusb_vol_dir == VUSB_Polling->notify_irq)) {
		schedule_delayed_work(&diso_polling_work, 10*HZ/1000);
		pr_debug("[%s] ready to trig VUSB irq, irq: %d\n",
		__func__, VUSB_Polling->notify_irq);
	} else if ((vdc_vol == 0) && (vusb_vol == 0)) {
		VDC_Polling->notify_irq_en = 0;
		VUSB_Polling->notify_irq_en = 0;
	}

}

enum hrtimer_restart diso_kthread_hrtimer_func(struct hrtimer *timer)
{
	diso_thread_timeout = KAL_TRUE;
	wake_up(&diso_polling_thread_wq);

	return HRTIMER_NORESTART;
}

int diso_thread_kthread(void *x)
{
	/* Run on a process content */
	while (1) {
		wait_event(diso_polling_thread_wq, (diso_thread_timeout == KAL_TRUE));

		diso_thread_timeout = KAL_FALSE;

		mutex_lock(&diso_polling_mutex);

		_get_polling_state();

		if (DISO_Polling.vdc_polling_measure.notify_irq_en || DISO_Polling.vusb_polling_measure.notify_irq_en)
			hrtimer_start(&diso_kthread_timer, ktime_set(0, MSEC_TO_NSEC(SW_POLLING_PERIOD)),
			HRTIMER_MODE_REL);
		else
			hrtimer_cancel(&diso_kthread_timer);

		mutex_unlock(&diso_polling_mutex);
	}

	return 0;
}
#endif
#endif


static unsigned int charging_get_error_state(void)
{
	return charging_error;
}

static unsigned int charging_set_error_state(void *data)
{
	unsigned int status = STATUS_OK;

	charging_error = *(unsigned int *)(data);

	return status;
}

static unsigned int charging_diso_init(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *)data;

	/* Initialization DISO Struct */
	pDISO_data->diso_state.cur_otg_state	 = DISO_OFFLINE;
	pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
	pDISO_data->diso_state.cur_vdc_state	 = DISO_OFFLINE;

	pDISO_data->diso_state.pre_otg_state	 = DISO_OFFLINE;
	pDISO_data->diso_state.pre_vusb_state = DISO_OFFLINE;
	pDISO_data->diso_state.pre_vdc_state	 = DISO_OFFLINE;

	pDISO_data->chr_get_diso_state = KAL_FALSE;
	pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;

#if !defined(MTK_AUXADC_IRQ_SUPPORT)
	hrtimer_init(&diso_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	diso_kthread_timer.function = diso_kthread_hrtimer_func;
	INIT_DELAYED_WORK(&diso_polling_work, diso_polling_handler);

	kthread_run(diso_thread_kthread, NULL, "diso_thread_kthread");
	pr_debug("[%s] done\n", __func__);
#else
	struct device_node *node;
	int ret;

	/*Initial AuxADC IRQ*/
	DISO_IRQ.vdc_measure_channel.number   = AP_AUXADC_DISO_VDC_CHANNEL;
	DISO_IRQ.vusb_measure_channel.number  = AP_AUXADC_DISO_VUSB_CHANNEL;
	DISO_IRQ.vdc_measure_channel.period   = AUXADC_CHANNEL_DELAY_PERIOD;
	DISO_IRQ.vusb_measure_channel.period  = AUXADC_CHANNEL_DELAY_PERIOD;
	DISO_IRQ.vdc_measure_channel.debounce	  = AUXADC_CHANNEL_DEBOUNCE;
	DISO_IRQ.vusb_measure_channel.debounce  = AUXADC_CHANNEL_DEBOUNCE;

	/* use default threshold voltage, if use high voltage,maybe refine*/
	DISO_IRQ.vusb_measure_channel.falling_threshold = VBUS_MIN_VOLTAGE/1000;
	DISO_IRQ.vdc_measure_channel.falling_threshold = VDC_MIN_VOLTAGE/1000;
	DISO_IRQ.vusb_measure_channel.rising_threshold = VBUS_MIN_VOLTAGE/1000;
	DISO_IRQ.vdc_measure_channel.rising_threshold = VDC_MIN_VOLTAGE/1000;

	node = of_find_compatible_node(NULL, NULL, "mediatek,AUXADC");
	if (!node) {
		pr_debug("[diso_adc]: of_find_compatible_node failed!!\n");
	} else {
		pDISO_data->irq_line_number = irq_of_parse_and_map(node, 0);
		pr_debug("[diso_adc]: IRQ Number: 0x%x\n", pDISO_data->irq_line_number);
	}

	mt_irq_set_sens(pDISO_data->irq_line_number, MT_EDGE_SENSITIVE);
	mt_irq_set_polarity(pDISO_data->irq_line_number, MT_POLARITY_LOW);

	ret = request_threaded_irq(pDISO_data->irq_line_number, diso_auxadc_irq_handler,
	pDISO_data->irq_callback_func, IRQF_ONESHOT, "DISO_ADC_IRQ", NULL);

	if (ret) {
		pr_debug("[diso_adc]: request_irq failed.\n");
	} else {
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		pr_debug("[diso_adc]: diso_init success.\n");
	}
#endif
#endif

	return status;
}

static unsigned int charging_get_diso_state(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	int diso_state = 0x0;
	DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *)data;

	_get_diso_interrupt_state();
	diso_state = g_diso_state;
	pr_debug("[do_chrdet_int_task] current diso state is %s!\n", DISO_state_s[diso_state]);
	if (((diso_state >> 1) & 0x3) != 0x0) {
		switch (diso_state) {
		case USB_ONLY:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
#ifdef MTK_DISCRETE_SWITCH
			set_vdc_auxadc_irq(DISO_IRQ_ENABLE, 1);
#endif
			pDISO_data->diso_state.cur_vusb_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_ONLY:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_RISING);
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_WITH_USB:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_FALLING);
			pDISO_data->diso_state.cur_vusb_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_WITH_OTG:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_ONLINE;
			break;
		default: /*OTG only also can trigger vcdt IRQ*/
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_ONLINE;
			pr_debug(" switch load vcdt irq triggerd by OTG Boost!\n");
			break; /*OTG plugin no need battery sync action*/
		}
	}

	if (DISO_ONLINE == pDISO_data->diso_state.cur_vdc_state)
		pDISO_data->hv_voltage = VDC_MAX_VOLTAGE;
	else
		pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;
#endif

	return status;
}

static unsigned int charging_set_vindpm_os(void *data)
{
	unsigned int status = STATUS_OK;

	bq25896_set_vindpm_os(0x7);
	return status;
}

static unsigned int charging_set_sys_min(void *data)
{
	unsigned int status = STATUS_OK;

	bq25896_set_sys_min(0x6);
	return status;
}

/* ++++++ ASUS portting guide ++++++ */
unsigned int ICL_select(unsigned int charger_type)
{
	if (charger_type == NONSTANDARD_CHARGER || charger_type == STANDARD_CHARGER)
		return 0x12;	//Set Input Current Limit = 1000 mA
	else 
		return 0x1C;	//Set Input Current Limit = 1500 mA
}

void jeita_wakelock(bool lock)
{
	if (lock){
		if (!wake_lock_active(&alarm_cable_lock)){
			battery_log(BAT_LOG_CRTI, "[%s]Cable WakeLock: *LOCK*\n", bq25896_TAG);
			wake_lock(&alarm_cable_lock);
		}
	} else {
		if (wake_lock_active(&alarm_cable_lock)){
			battery_log(BAT_LOG_CRTI, "[%s]Cable WakeLock: *UNLOCK*\n", bq25896_TAG);
			wake_unlock(&alarm_cable_lock);
		}	
	}

}

void Recharge(unsigned int bq25896_temp)
{
	battery_log(BAT_LOG_CRTI, "[%s][%s]\n", bq25896_TAG, __func__);
	if(bq25896_temp <= 51){			//Temp>=45 ( 51=0x0110011 )
		bq25896_set_chg_config(0x1);	// 1. Enable charging
		bq25896_set_vreg(0x1D);		// 2. Set Charge Voltage Limit = 4.3 V
	}else{					//Temp<45
		bq25896_set_chg_config(0x1);	// 1. Enable charging
		bq25896_set_vreg(0x21);		// 2. Set Charge Voltage Limit = 4.368 V

		if ((bq25896_get_chrg_stat() & 0x3) && BMT_status.UI_SOC <= 98)
		{
			bq25896_set_chg_config(0x0);	// 1. Disable charge
			bq25896_set_chg_config(0x1);	// 2. Enable charge
		}
	}
}

static unsigned int charging_JEITA(void *data){
#ifdef ENG_BUILD
	unsigned int b_charging_enable;
	bq25896_set_conv_rate(0x1);	// 1. Start 1s Continuous Conversion
	if (BMT_status.SOC > eng_charging_limit_soc && b_eng_charging_limit == true){
		battery_log(BAT_LOG_CRTI, "[%s][ENG] STOP charging (%d)(%d)(%d)!\n", __func__, b_eng_charging_limit, BMT_status.SOC, eng_charging_limit_soc);
		b_charging_enable = KAL_FALSE;
	} else {
		battery_log(BAT_LOG_CRTI, "[%s][ENG] charging (%d)(%d)(%d)\n", __func__,b_eng_charging_limit, BMT_status.SOC, eng_charging_limit_soc);
		b_charging_enable = KAL_TRUE;
	}
	charging_enable(&b_charging_enable);
#else
	int temp;
	unsigned int bq25896_temp;
	static int bq25896_recharging = 0;

	bq25896_set_conv_rate(0x1);	// 1. Start 1s Continuous Conversion
	bq25896_set_jeita_iset(0x0);	// 2. Soft cold charge current = 0.5C

	bq25896_temp = bq25896_get_tspct();
	temp = battery_meter_get_battery_temperature();
	battery_log(BAT_LOG_CRTI, "[%s][%s]\n", bq25896_TAG, __func__);
	battery_log(BAT_LOG_CRTI, "[%s][%s] BAT_temp = %d, bq25896_get_tspct = %d\n", bq25896_TAG, __func__, temp, bq25896_temp);

	if (b_stop_jeita){
		battery_log(BAT_LOG_CRTI,"[%s][%s] stop JEITA by command, skip this function\n", bq25896_TAG, __func__);
		return 0;
	}

	if(chg_timeout_limit == 0 || chg_timeout_limit == 3){
			if(bq25896_recharging == 0){
					if (temp >= 55 || temp <= 0){
							bq25896_set_chg_config(0x0);	// 1. Disable charging
							bq25896_set_vreg(0x21);			// 2. Set Charge Voltage Limit = 4.368 V
							bq25896_recharging = 1;
					}else{
							Recharge(bq25896_temp);			// Normal temp & Soft cold temp
					}
			}else{
					if (3 < temp && temp < 52){
							bq25896_recharging = 0;
							Recharge(bq25896_temp);			// Normal temp & Soft cold temp
					}
			}
	}else{
			bq25896_set_chg_config(0x0);    				// Disable charging
	}
#endif
        return 0;
}

int voltage2temp(void)
{
	int table_size = 0;
	int i = 0;
	int ret = 0;
	int adc_volt = 0;
	int temp_therm = 0;

	table_size = ARRAY_SIZE(aux_in5_adc);
	ret = IMM_GetOneChannelValue_Cali(CHARGER_NTC_CHANNEL_NUM, &adc_volt);
	if (ret != 0){
		battery_log(BAT_LOG_CRTI,"[%s][%s] adc_volt read fail\n", bq25896_TAG, __func__);
	} else {
		battery_log(BAT_LOG_CRTI,"[%s][%s] adc_volt = %d\n", bq25896_TAG, __func__, adc_volt);
	}

	while ( i < table_size ) {
		if (aux_in5_adc[i].y < adc_volt){
			break;
		} else {
			i++;
		}

		if (i ==0 ) {
			temp_therm = aux_in5_adc[0].x;
		} else if ( i == table_size ) {
			temp_therm = aux_in5_adc[table_size-1].x;
		} else {
			temp_therm = (((int32_t) ((aux_in5_adc[i].x - aux_in5_adc[i-1].x)*(adc_volt - aux_in5_adc[i-1].y))/(aux_in5_adc[i].y - aux_in5_adc[i-1].y))+aux_in5_adc[i-1].x);
		}
	}
//	battery_log(BAT_LOG_CRTI,"[%s][%s] Voltage to Temp = %d, index = %d\n", bq25896_TAG, __func__, temp_therm, i);
	return temp_therm;
}

static void ASUS_Thermal_Policy(void)
{
#ifdef ENG_BUILD
	battery_log(BAT_LOG_CRTI,"[%s][%s] this is ENG build, we'll disable this function\n", bq25896_TAG, __func__);
	return;
#else
	int charger_temp = 0;

	if (b_stop_thermal){
		battery_log(BAT_LOG_CRTI,"[%s][%s] stop charger thermal policy by command, skip this function\n", bq25896_TAG, __func__);
		return;
	}

	if(proj_id != 3){
		battery_log(BAT_LOG_CRTI,"[%s][%s] Z301M doesn't need, skip this function", bq25896_TAG, __func__);
		return;
	}

	if(hw_id_charger < 2 ){
		battery_log(BAT_LOG_CRTI,"[%s][%s] HW_ID < 2 (before PR) , skip this function", bq25896_TAG, __func__);
		return;
	}

	if (BMT_status.charger_type != STANDARD_HOST)
	{
		charger_temp = voltage2temp();
	
		// need to do: check thermal level threshold
		if (charger_temp < charger_thermal_level1)
			thermal_level = 0;
		else if (charger_temp < charger_thermal_level2)
			thermal_level = 1;
		else if (charger_temp < charger_thermal_level3)
			thermal_level = 2;
		else 
			thermal_level = 3;
	
		battery_log(BAT_LOG_CRTI, "[%s][%s] charger temp = %d, thermal level = %d\n", bq25896_TAG, __func__, charger_temp, thermal_level);

		switch(thermal_level)
		{
			case 0:
				if(BMT_status.Adapter == Z301M_DCP || BMT_status.Adapter == Z301M_DCP_others|| BMT_status.Adapter == Z301M_UNFEFINED || BMT_status.Adapter == Z301M_Samsung)
					bq25896_set_iinlim(0x12);	// 1. Set Input Current = 1000 mA
				else if (BMT_status.Adapter == Z301M_CDP || BMT_status.Adapter == Z301M_TYPEC_1P5A)
					bq25896_set_iinlim(0x1C);	// 1. Set Input Current = 1500 mA
				else if (BMT_status.Adapter == Z301M_DCP_750k || BMT_status.Adapter == Z301M_DCP_200k || BMT_status.Adapter == Z301M_PowerBank)
					bq25896_set_iinlim(0x26);	// 1. Set Input Current = 2000 mA
				else if (BMT_status.Adapter == Z301M_TYPEC_3A)
					bq25896_set_iinlim(0x3A);	// 1. Set Input Current = 3000 mA
					
				bq25896_set_en_hiz(0x0);	// 2. Disable HIZ mode
				break;
			case 1:
				bq25896_set_iinlim(0x12);	// 1. Set Input Current = 1000 mA
				bq25896_set_en_hiz(0x0);	// 2. Disable HIZ mode
				break;
			case 2:
				if (BMT_status.UI_SOC < 8)
					bq25896_set_iinlim(0x12);	// 1. Set Input Current = 1000 mA
				else if (BMT_status.UI_SOC < 15)
					bq25896_set_iinlim(0xC);	// 1. Set Input Current = 700 mA
				else
					bq25896_set_iinlim(0x8);	// 1. Set Input Current = 500 mA
				bq25896_set_en_hiz(0x0);	// 2. Disable HIZ mode
				break;
			case 3:
				bq25896_set_en_hiz(0x1);	// 1. Enable HIZ mode
				break;
		}
	} else {
		battery_log(BAT_LOG_CRTI, "[%s][%s] ByPass SDP!\n", bq25896_TAG, __func__);
	}
#endif
}

#define COS_CHARGING_TIMEOUT_LIMIT_FILE   "/cache/charger/CHGLimit"
void Write_CHG_TimeOut_Limit(void){
	struct file *fp = NULL;
	mm_segment_t old_fs;
	static char b_buf[sizeof(int)];

	fp = filp_open(COS_CHARGING_TIMEOUT_LIMIT_FILE, O_RDWR|O_CREAT, 0666);
	if (!IS_ERR_OR_NULL(fp) ) {
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			sprintf(b_buf, "%d", chg_timeout_cos);//cache/charger/CHGLimit depends on chg_timeout; MOS should clear it at shutdown; COS will count timer
			fp->f_op->llseek(fp, 0, 0);
			fp->f_op->write(fp,b_buf,sizeof(b_buf),&fp->f_pos);
			/* Restore segment descriptor */
			set_fs(old_fs);
			/* Close file operation */
			filp_close(fp, NULL);
	}else{
			battery_log(BAT_LOG_CRTI,"%s: file open error (%s)\n", __func__, COS_CHARGING_TIMEOUT_LIMIT_FILE);
	}

	if(chg_timeout_cos || chg_timeout_mos){
			if(BMT_status.UI_SOC > 60){
					chg_timeout_limit = 1;	//Enable HIZ mode
					chg_timeout_limit_stopFG = 0;
			}else if(BMT_status.UI_SOC == 60){
					chg_timeout_limit = 2;		//Disable HIZ mode but not charging
					chg_timeout_limit_stopFG = 1;//Ever <= 60%.
			}else{
					chg_timeout_limit = 3;		//Because SOC < 60%, Disable HIZ mode and charging
					chg_timeout_limit_stopFG = 1;//Ever <= 60%.
			}
	}else{
			chg_timeout_limit = 0;					//Disable HIZ mode and charging
			chg_timeout_limit_stopFG = 0;
	}

	if(thermal_level == 3){							//If thermal level is too high, HIZ mode should keep
		bq25896_set_en_hiz(0x1);    				// Enable HIZ mode
	}else{
		if(chg_timeout_limit == 1)
			bq25896_set_en_hiz(1);
		else
			bq25896_set_en_hiz(0);
	}
	battery_log(BAT_LOG_CRTI, "[%s][%s] chg_timeout_cos(%d) chg_timeout_mos(%d) chg_timeout_limit(%d) chg_timeout_limit_old(%d) thermal_level(%d) chg_timeout_limit_stopFG(%d)\n", bq25896_TAG, __func__, chg_timeout_cos, chg_timeout_mos, chg_timeout_limit, chg_timeout_limit_old, thermal_level, chg_timeout_limit_stopFG);
}
EXPORT_SYMBOL(Write_CHG_TimeOut_Limit);

static void ASUS_JEITA_work(struct work_struct *work)
{
	if (upmu_is_chr_det() == KAL_TRUE){
		jeita_wakelock(true);
		/*Add By Michael_Nieh@asus.com+++*/
		if(g_platform_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT || g_platform_boot_mode == LOW_POWER_OFF_CHARGING_BOOT){
				if(!chg_timeout_cos){
				//COS timeout not yet
						if(!last_chg_time_toggle){
						//100% not yet
								if(BMT_status.UI_SOC == 100){
								//record the first time to 100%
										get_monotonic_boottime(&last_chg_time);
										battery_log(BAT_LOG_CRTI, "[%s][%s] charging && 100%% !!(%ld)\n", bq25896_TAG, __func__, last_chg_time.tv_sec);
										last_chg_time_toggle = 1;
								}
						}else{
						//already 100% and count the time whether its timeout or not
								get_monotonic_boottime(&now_chg_time);
								battery_log(BAT_LOG_CRTI, "[%s][%s] now_chg_time(%ld) last_chg_time(%ld) chg_timeout_threshold(%d)\n", bq25896_TAG, __func__, now_chg_time.tv_sec, last_chg_time.tv_sec,chg_timeout_threshold);
								if((now_chg_time.tv_sec - last_chg_time.tv_sec) > chg_timeout_threshold){//60 * 60 * 24 * 3 = 259200 = 72hr
										chg_timeout_cos = 1;
								}
						}
				}
				Write_CHG_TimeOut_Limit();
		}
		battery_log(BAT_LOG_CRTI, "[%s][%s] chg_timeout_cos(%d) chg_timeout_mos(%d) chg_timeout_limit(%d) thermal_level(%d)\n", bq25896_TAG, __func__, chg_timeout_cos, chg_timeout_mos, chg_timeout_limit, thermal_level);
		if(chg_timeout_mos){
				Write_CHG_TimeOut_Limit();
		}
		/*Add By Michael_Nieh@asus.com---*/
		ASUS_Thermal_Policy();
		charging_JEITA(NULL);
		charging_reset_watch_dog_timer(NULL);
		jeita_wakelock(false);
	}
}

static void ASUS_Adapter_Detect_work(struct work_struct *work)
{
	int ads1013_reg = 0;
	int firstbit = 0;
	int i = 0;
	//need to do: cable out in waiting 30s
	ADCPWR_enable(0);		// 1. Soc GPIO【ADCPWREN_PMI_GP1】 = "L"
	DPM_SW_enable(1);		// 2. SoC GPIO【ADC_SW_EN 】 = "H"

	battery_log(BAT_LOG_CRTI, "[%s][%s]Waitting ADC stable 30s \n", bq25896_TAG, __func__);
	do{
		i++;
		msleep(500);
		
		if (upmu_is_chr_det() == KAL_FALSE){
			battery_log(BAT_LOG_CRTI, "[%s][%s]charger bunk disable or loss (%d)*0.5s\n", bq25896_TAG, __func__, i);
			ADCPWR_enable(0);
			DPM_SW_enable(0);
			return;
		}
	}while (i<60);

	ads1013_reg = thermal_ads1013_adc_temp(2);	// Read ADC INA0 to check VADC
	firstbit = ads1013_reg >> 11 ;
	battery_log(BAT_LOG_CRTI, "[%s][%s] ads1013_reg = %d, firstbit = %d\n", bq25896_TAG, __func__, ads1013_reg, firstbit);
	if (ads1013_reg <= 0x258 || firstbit == 1){			// VADC < 0.6V
		ADCPWR_enable(1);			// 1. Soc GPIO【ADCPWREN_PMI_GP1】 = "H"
		msleep(5);
		ads1013_reg = thermal_ads1013_adc_temp(2);
		if (ads1013_reg < 0x7D0){		//VADC > 2V
			if (ads1013_reg >= 0x5D6 && ads1013_reg <= 0x766){
				BMT_status.Adapter = Z301M_DCP_750k;
				bq25896_set_iinlim(0x26); 	// 1. Set Input Current Limit = 2000mA
        			bq25896_set_en_ilim(0x0); 	// 2. Disable Current Limit
			} else if (ads1013_reg >= 0x2A9 && ads1013_reg <= 0x371){
				BMT_status.Adapter = Z301M_DCP_200k;
				bq25896_set_iinlim(0x26); 	// 1. Set Input Current Limit = 2000mA
        			bq25896_set_en_ilim(0x0); 	// 2. Disable Current Limit
			} else {
				BMT_status.Adapter = Z301M_DCP_others;
				bq25896_set_iinlim(0x12); 	// 1. Set Input Current Limit = 1000mA
        			bq25896_set_en_ilim(0x0); 	// 2. Disable Current Limit
			}
		} else {
			BMT_status.Adapter = Z301M_DCP;
			bq25896_set_iinlim(0x12); 	// 1. Set Input Current Limit = 1000mA
			bq25896_set_en_ilim(0x0); 	// 2. Disable Current Limit
		}
	} else {
		ads1013_reg = thermal_ads1013_adc_temp(2);
		if (ads1013_reg >= 0x708){		// VADC > 1.8V
			BMT_status.Adapter = Z301M_PowerBank;
			bq25896_set_iinlim(0x26); 	// 1. Set Input Current Limit = 2000mA
        		bq25896_set_en_ilim(0x0); 	// 2. Disable Current Limit
		} else {
			BMT_status.Adapter = Z301M_Samsung;
			bq25896_set_iinlim(0x12); 	// 1. Set Input Current Limit = 1000mA
        		bq25896_set_en_ilim(0x0); 	// 2. Disable Current Limit
		}
	}
	ADCPWR_enable(0);		// 1. Soc GPIO【ADCPWREN_PMI_GP1】 = "L"
	DPM_SW_enable(0);		// 2. SoC GPIO【ADC_SW_EN 】 = "L"

	battery_log(BAT_LOG_CRTI, "[%s][%s] %s\n", bq25896_TAG, __func__, Adapter_flag_str[BMT_status.Adapter]);
	ASUS_Thermal_Policy();
	charging_JEITA(NULL);
	charging_reset_watch_dog_timer(NULL);
	jeita_wakelock(false);

	if (atomic_read(&AC_charging_alarm) == 0)
	{
		alarm_start_relative(&AC_charging_alarm_timer, ns_to_ktime(60000000000)); //60s
		battery_log(BAT_LOG_CRTI, "[%s][%s] Set Wake_BAT_alarm_timer = 60\n", bq25896_TAG, __func__);
		atomic_set(&AC_charging_alarm,1);
	}
}

void Initial_Charger_Setting(void)
{
	battery_log(BAT_LOG_CRTI, "[%s][%s]\n", bq25896_TAG, __func__);

	bq25896_set_vreg(0x21);		// 1. Set Charge Voltage Limit = 4.368 V
	bq25896_set_sys_min(0x6);	// 2. Minimum System Voltage = 3.6 V
	bq25896_set_iinlim(0x8);	// 3. set input current limit = 500 mA
	bq25896_set_ichg(0x22);		// 4. Set Fast Charge Current Limit = 2176 mA
	bq25896_set_iprechg(0x5);	//    Set Pre-Charge Current Limit = 384 mA
	bq25896_set_iterm(0x2);		//    Set Termination Current Limit = 192 mA
	bq25896_set_watchdog(0x3);	// 5. watch dog timer = 160 sec
	bq25896_set_en_ilim(0x1);	// 6. Enable Current Limit
	bq25896_set_auto_dpmp_en(0x0);	// 7. Disable AUTO_DPDM
	bq25896_set_vindpm_os(0x7);	// 8. Voltage DPM offset = 700 mV
	if(hw_id_charger < 2 ){
		bq25896_LID(1);			// 9. LID = Disable
	}
}

static unsigned int charging_PAD_only(void *data)
{
	battery_log(BAT_LOG_CRTI, "[%s][%s]\n", bq25896_TAG, __func__);
	cancel_delayed_work(&adapter_detect_work);
	cancel_delayed_work(&jeita_work);
	bq25896_set_watchdog(0x3);	// 1. watch dog timer = 160 sec
	bq25896_set_en_ilim(0x1);	// 2. Enable Current Limit
	bq25896_set_iinlim(0x8);	// 3. set input current limit = 500 mA
	bq25896_set_auto_dpmp_en(0x0);	// 4. Disable AUTO_DPDM
	bq25896_set_watchdog(0x0);	// 5. Disable watch dog timer
	if(hw_id_charger < 2 ){
		bq25896_LID(0);			// 6. LID = Enable
	}
	bq25896_set_conv_rate(0x0);	// 7. ADC mode = single-shot mode
	atomic_set(&AC_charging_alarm,0);
	BMT_status.Adapter = Z301M_UNFEFINED;
	return 0;
}
/*++++ type C porting ++++*/
static void typec_supply_dump_status(struct power_supply *psy)
{
	static char *typec_mode_text[] = {
		"Unknown", "Source", "Sink", "DFP", "UFP"
	};
	int rc;
	union power_supply_propval value;
        
	battery_log(BAT_LOG_CRTI, "[%s][%s]\n", bq25896_TAG, __func__);

	rc = psy->get_property(psy, POWER_SUPPLY_PROP_TYPEC_MODE, &value);
	if (rc == 0) {
		battery_log(BAT_LOG_CRTI, "[%s][%s]TypeC supply: TYPEC_MODE=%s\n", bq25896_TAG, __func__, typec_mode_text[value.intval]);
	}

	rc = psy->get_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
	if (rc == 0) {
		battery_log(BAT_LOG_CRTI, "[%s][%s]TypeC supply: ONLINE=%d\n", bq25896_TAG, __func__, value.intval);
		typec_online = value.intval;
	}

	rc = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &value);
	if (rc == 0) {
		battery_log(BAT_LOG_CRTI, "[%s][%s]TypeC supply: VOLTAGE_MAX=%u uV\n", bq25896_TAG, __func__, value.intval);  
		typec_voltage_max = value.intval;
	}

	rc = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_MIN, &value);
	if (rc == 0) {
		battery_log(BAT_LOG_CRTI, "[%s][%s]TypeC supply: VOLTAGE_MIN=%u uV\n", bq25896_TAG, __func__, value.intval);  
		typec_voltage_min = value.intval;
	}

	rc = psy->get_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &value);
	if (rc == 0) {
		battery_log(BAT_LOG_CRTI, "[%s][%s]TypeC supply: CURRENT_MAX=%u uA\n", bq25896_TAG, __func__, value.intval);
		typec_current_max = value.intval;
	}
}
/*---- type C porting ----*/

/*++++ PD portting ++++*/
static void pd_supply_dump_status(struct power_supply *psy)
{
	static char *scope_text[] = {
		"Unknown", "System", "Sink", "DFP", "UFP"
	};
	int rc;
	union power_supply_propval value;

	battery_log(BAT_LOG_CRTI, "[%s][%s]\n", bq25896_TAG, __func__);

	rc = psy->get_property(psy, POWER_SUPPLY_PROP_SCOPE, &value);
	if (rc == 0) {
		battery_log(BAT_LOG_CRTI, "[%s][%s] PD supply: SCOPE=%s\n", bq25896_TAG, __func__, scope_text[value.intval]);
	}

	if (rc == 0) {
		battery_log(BAT_LOG_CRTI, "[%s][%s] PD supply: ONLINE=%d\n", bq25896_TAG, __func__, value.intval);
		pd_online = value.intval;
	}

	if (rc == 0) {
		battery_log(BAT_LOG_CRTI, "[%s][%s] PD supply: VOLTAGE_MAX=%u uV\n", bq25896_TAG, __func__, value.intval);
		pd_voltage_max = value.intval;
	}

	rc = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_MIN, &value);
	if (rc == 0) {
		battery_log(BAT_LOG_CRTI, "[%s][%s] PD supply: VOLTAGE_MIN=%u uV\n", bq25896_TAG, __func__, value.intval);
	}

	rc = psy->get_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &value);
	if (rc == 0) {
		battery_log(BAT_LOG_CRTI, "[%s][%s] PD supply: CURRENT_MAX=%u uA\n", bq25896_TAG, __func__, value.intval);
		pd_current_max = value.intval;
	}
}
/*---- PD portting ----*/

void DFP_setting(void)
{
	battery_log(BAT_LOG_CRTI, "[%s][%s] %s\n", bq25896_TAG, __func__, Type_c_str[g_type_c_flag]);
	if (g_type_c_flag == TYPE_C_DEFAULT){
	//battery_log(BAT_LOG_CRTI, "[%s][%s] Do Type-C Default setting(skip)\n", bq25896_TAG, __func__);
		return;
	} else if (g_type_c_flag == TYPE_C_1_5_A) {
		bq25896_set_iinlim(0x1C); 	// 1. Set Input Current Limit = 1500 mA
		BMT_status.Adapter = Z301M_TYPEC_1P5A;
	} else if (g_type_c_flag == TYPE_C_3_A) {
		bq25896_set_iinlim(0x3A); 	// 1. Set Input Current Limit = 3000 mA
		BMT_status.Adapter = Z301M_TYPEC_3A;
	} else if (g_type_c_flag == TYPE_C_OTHERS) {
		bq25896_set_iinlim(0x12);       // 1. Set Input Current Limit = 1000 mA
		BMT_status.Adapter = Z301M_TYPEC_OTHERS;
	} else if (g_type_c_flag == TYPE_C_PD){
		bq25896_set_iinlim(0x12);       // 1. Set Input Current Limit = 1000 mA
		BMT_status.Adapter = Z301M_TYPEC_PD;
	}
       	bq25896_set_en_ilim(0x0); 	// 2. Disable Current Limit
}

int asus_DFP_detect(void)
{
        int rc;
        battery_log(BAT_LOG_CRTI, "[%s][%s]\n", bq25896_TAG, __func__);
	if (pd_psy)
		pd_supply_dump_status(pd_psy);
        /*
         * call external function for reading CC logic IC
         * TYPE_C_Unknown=0, TYPE_C_1_5_A=1, TYPE_C_3_A=2, TYPE_C_PD=3, TYPE_C_OTHERS=4, TYPE_C_DEFAULT=5
         */
	if (typec_psy)
	        typec_supply_dump_status(typec_psy);

        if (typec_online == 1) {
		if (typec_current_max == 1500000){	//TypeC_1.5A
	                g_type_c_flag = TYPE_C_1_5_A;
	        }else if(typec_current_max == 3000000){	//TypeC_3.0A
	                g_type_c_flag = TYPE_C_3_A;
		}else if (typec_current_max == 500000){	//Default 500mA
			g_type_c_flag = TYPE_C_DEFAULT;
	        }else{
	                g_type_c_flag = TYPE_C_OTHERS;
		}

		if(typec_online == 1 && pd_online ==1){
			g_type_c_flag = TYPE_C_PD;
		}
	} else {
		g_type_c_flag = TYPE_C_Unknown;
	}

	rc = g_type_c_flag;
	battery_log(BAT_LOG_CRTI, "[%s][%s]TypeC flag = %d; pd_online = %d\n", bq25896_TAG, __func__, g_type_c_flag, pd_online);

	return rc;
}

unsigned int Adapter_check(void)
{
	if (BMT_status.charger_type == STANDARD_HOST)
		return Z301M_SDP;
	else if (BMT_status.charger_type == NONSTANDARD_CHARGER)
		return Z301M_DCP_others;
	else if (BMT_status.charger_type == CHARGING_HOST)
		return Z301M_CDP;
	else{
		if(proj_id == 3)
			queue_delayed_work(adapter_wq ,&adapter_detect_work, msecs_to_jiffies(0));
		return Z301M_DCP;
	}	
}

int dfp_type = 0;
static unsigned int charging_setting(void *data)
{
	unsigned int icl;
	
	Initial_Charger_Setting();
	jeita_wakelock(true);

	if (BMT_status.charger_type != STANDARD_HOST){
		msleep(500);
		icl = ICL_select(BMT_status.charger_type);
		bq25896_set_iinlim(icl); 	// 1. Set Input Current Limit
        	bq25896_set_en_ilim(0x0); 	// 2. Disable Current Limit
		dfp_type = asus_DFP_detect();
		if (dfp_type!=TYPE_C_Unknown && dfp_type!=TYPE_C_DEFAULT){
			DFP_setting();
		}else{
			BMT_status.Adapter = Adapter_check();
		}
	} else if (BMT_status.charger_type == STANDARD_HOST){
			BMT_status.Adapter = Z301M_SDP;
		//need to do: retry
	}

	battery_log(BAT_LOG_CRTI, "[%s][%s] %s\n", bq25896_TAG, __func__, Adapter_flag_str[BMT_status.Adapter]);
	
	if (proj_id != 3 || (proj_id == 3 && BMT_status.Adapter != Z301M_DCP)){
		ASUS_Thermal_Policy();
		charging_JEITA(NULL);
		charging_reset_watch_dog_timer(NULL);
		jeita_wakelock(false);
		if (atomic_read(&AC_charging_alarm) == 0)
		{
			alarm_start_relative(&AC_charging_alarm_timer, ns_to_ktime(60000000000)); //60s
			battery_log(BAT_LOG_CRTI, "[%s][%s] Set Wake_BAT_alarm_timer = 60\n", bq25896_TAG, __func__);
			atomic_set(&AC_charging_alarm,1);
		}
	}

	return 0;
}

void init_delay_work(void);
void do_chrdet_int_task_bq25896(void)
{
	battery_log(BAT_LOG_CRTI, "[%s][%s]", bq25896_TAG, __func__);

	if (!bootup_init_done){
		init_delay_work();
		init_delay = 5000;
		return;
	} else {
		do_chrdet_int_task();	
	}

	if(chg_timeout_cos || chg_timeout_mos){
			if(BMT_status.UI_SOC > 60){
					chg_timeout_limit = 1;	//Enable HIZ mode
					chg_timeout_limit_stopFG = 0;
			}else if(BMT_status.UI_SOC == 60){
					chg_timeout_limit = 2;		//Disable HIZ mode but not charging
					chg_timeout_limit_stopFG = 1;//Ever <= 60%.
			}else{
					chg_timeout_limit = 3;		//Because SOC < 60%, Disable HIZ mode and charging
					chg_timeout_limit_stopFG = 1;//Ever <= 60%.
			}
	}else{
			chg_timeout_limit = 0;					//Disable HIZ mode and charging
			chg_timeout_limit_stopFG = 0;
	}

	if (upmu_is_chr_det() == KAL_TRUE && chr_wake_up_charging == 1){
		battery_log(BAT_LOG_CRTI, "[%s][%s] insertion !!\n", bq25896_TAG, __func__);
		chr_wake_up_charging = 0;
		chr_wake_up_discharging = 1;
		charging_setting(NULL);
		if((g_platform_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT || g_platform_boot_mode == LOW_POWER_OFF_CHARGING_BOOT) && (!last_chg_time_toggle)){
				//in COS and 100% not yet
				if(BMT_status.UI_SOC == 100){
						//record the first time to 100%
						get_monotonic_boottime(&last_chg_time);
						battery_log(BAT_LOG_CRTI, "[%s][%s] insertion && 100%% !! (%ld)\n", bq25896_TAG, __func__, last_chg_time.tv_sec);
						last_chg_time_toggle = 1;
				}
		}
	} else if (upmu_is_chr_det() == KAL_FALSE && chr_wake_up_discharging == 1){
		battery_log(BAT_LOG_CRTI, "[%s][%s] removal !!\n", bq25896_TAG, __func__);
		bq25896_set_en_hiz(0x0);        // Disable HIZ mode
		chr_wake_up_charging = 1;
		chr_wake_up_discharging = 0;
		atomic_set(&AC_charging_alarm,0);
		jeita_wakelock(false);

		charging_PAD_only(NULL);
	}
}
EXPORT_SYMBOL(do_chrdet_int_task_bq25896);

static enum alarmtimer_restart AC_charging_polling_atimer(struct alarm *alarm, ktime_t t)
{
	jeita_wakelock(true);
	battery_log(BAT_LOG_CRTI, "[%s][%s]\n", bq25896_TAG, __func__);

	if(atomic_read(&AC_charging_alarm) == 1){
		cancel_delayed_work(&jeita_work);
		queue_delayed_work(adapter_wq ,&jeita_work, msecs_to_jiffies(0));
		alarm_forward_now(&AC_charging_alarm_timer, ns_to_ktime(60000000000)); //60s
		battery_log(BAT_LOG_CRTI, "[%s][%s] done\n", bq25896_TAG, __func__);
		return ALARMTIMER_RESTART;
	} else {
		jeita_wakelock(false);
		return ALARMTIMER_NORESTART;
	}
}


static void int_task_initial_work(struct work_struct *work)
{
	proj_id = Read_PROJ_ID();

	pd_psy = power_supply_get_by_name("power_delivery");
	if (!pd_psy) {
		battery_log(BAT_LOG_CRTI, "[%s][%s] PD supply not found!", bq25896_TAG, __func__);
	}

	typec_psy = power_supply_get_by_name("typec");
	if (!typec_psy){
		battery_log(BAT_LOG_CRTI, "[%s][%s] typec supply not found!", bq25896_TAG, __func__);
	}

	b_otg_probe = 0;
	BMT_status.Adapter = Z301M_UNFEFINED;
	battery_log(BAT_LOG_CRTI, "[%s][%s]\n", bq25896_TAG, __func__);
	if (upmu_is_chr_det() == KAL_TRUE){
		do_chrdet_int_task_bq25896();
	}
}

void init_delay_work(void)
{
	INIT_DELAYED_WORK(&adapter_detect_work, ASUS_Adapter_Detect_work);
	INIT_DELAYED_WORK(&jeita_work, ASUS_JEITA_work);
	INIT_DELAYED_WORK(&initial_work, int_task_initial_work);
	alarm_init(&AC_charging_alarm_timer, ALARM_REALTIME, AC_charging_polling_atimer);
	wake_lock_init(&alarm_cable_lock,WAKE_LOCK_SUSPEND, "alarm_cable_wakelock");
	cancel_delayed_work(&initial_work);
	queue_delayed_work(adapter_wq ,&initial_work, msecs_to_jiffies(init_delay));	
	bootup_init_done = true;
	battery_log(BAT_LOG_CRTI, "[%s][%s] init_delay = %d\n", bq25896_TAG, __func__, init_delay);
	init_delay = 10000;
}
EXPORT_SYMBOL(init_delay_work);
/* ------ ASUS portting guide ------ */

static unsigned int charging_get_VREG(void *data){
	int vreg = 0;
	int value;
	int i;
	int mV = 16;
	value = bq25896_get_vreg();
	for( i=0; i<6; i++){
		if ((value & BIT(i))==BIT(i))
			vreg += mV;
			mV *= 2;
	}
	vreg = vreg + 3840;
	*(unsigned int *)(data) = vreg;
	battery_log(BAT_LOG_CRTI, "[%s] Charge Voltage Limit =%d\n", __func__,vreg);
	return 0;
}

static unsigned int charging_get_bq25896_ICharging(void *data){
        int current1 = 0;
        int value ;
        int i;
        int mA = 50;
	//bq25896_set_conv_rate(0x1);	// 1. Start 1s Continuous Conversion
	bq25896_set_conv_start(0x1);	 
        value = bq25896_get_ichgr();
        for(i=0;i<7;i++)
        {
                if ((value & BIT(i))==BIT(i))
                        current1 += mA;
                mA *= 2;
        }
        battery_log(BAT_LOG_CRTI, "[%s][show_ICHGR] current =%d\n", bq25896_TAG, current1);
	*(unsigned int *)(data) = current1;
        return 0;
}

static unsigned int charging_func_null(void *data){
	battery_log(BAT_LOG_CRTI, "[Z301M][Z301MF] has no this function!\n");
	return 0;
}

static unsigned int (* const charging_func[CHARGING_CMD_NUMBER]) (void *data) = {
	charging_hw_init,
	charging_dump_register,
	charging_enable,
	charging_set_cv_voltage,
	charging_get_current,
	charging_set_current,
	charging_set_input_current,
	charging_get_charging_status,
	charging_reset_watch_dog_timer,
	charging_set_hv_threshold,
	charging_get_hv_status,
	charging_get_battery_status,
	charging_get_charger_det_status,
	charging_get_charger_type,
	charging_get_is_pcm_timer_trigger,
	charging_set_platform_reset,
	charging_get_platform_boot_mode,
	charging_set_power_off,
	charging_get_power_source,
	charging_get_csdac_full_flag,
	charging_set_ta_current_pattern,
	charging_set_error_state,
	charging_diso_init,
	charging_get_diso_state,
	charging_set_vindpm_os,
	charging_set_sys_min,
	charging_func_null,
	charging_func_null,
	charging_func_null,
	charging_func_null,
	charging_func_null,
	charging_func_null,
	charging_func_null,
	charging_func_null,
	charging_func_null,
	charging_JEITA,
	charging_get_VREG,
	charging_func_null,
	charging_get_bq25896_ICharging,
	charging_get_power_good_status,
	charging_setting,
	charging_PAD_only,
	charging_get_temp,
};

 /*
 *FUNCTION
 *		Internal_chr_control_handler
 *
 *DESCRI
 *		 This function is called to set the charger hw
 *
 *CALLS
 *
 * PARAMETERS
 *		None
 *
 *RETURNS
 *
 * GLOBALS AFFECTED
 *	   None
 */
signed int chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	signed int status;

	if (cmd < CHARGING_CMD_NUMBER)
		status = charging_func[cmd](data);
	else
		return STATUS_UNSUPPORTED;

	return status;
}
