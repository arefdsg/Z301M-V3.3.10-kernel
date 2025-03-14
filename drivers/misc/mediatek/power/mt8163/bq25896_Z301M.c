#include <linux/delay.h>
#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <mt-plat/charging.h>
#include "bq25896.h"
#include <mt-plat/battery_common.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/switch.h>
//michael_nieh@asus.com
extern int chg_timeout_mos;
extern void Write_CHG_TimeOut_Limit(void);
//shilun_huang@asus.com
int b_otg_probe = 0;
int b_stop_thermal = 0;
int b_stop_jeita = 0;
EXPORT_SYMBOL(b_otg_probe);
EXPORT_SYMBOL(b_stop_thermal);
EXPORT_SYMBOL(b_stop_jeita);
extern int charger_thermal_level1;
extern int charger_thermal_level2;
extern int charger_thermal_level3;
static int g_usb_connector_event = 0; //for usb connector thermal event
struct switch_dev charger_dev;

int g_stand_int = 0;
EXPORT_SYMBOL(g_stand_int);
struct workqueue_struct *adapter_wq;
#define CHARGING_LIMIT_THRESHOLD 60
struct charger_gpio{
	struct device	*dev;
	unsigned int 	PG_OTG_EN_SOC;
	unsigned int 	LID_EN;
	unsigned int 	USBCON_LID;
	unsigned int 	USBCON_TEMP;
	struct 		delayed_work charger_limit_work;
};
struct charger_gpio *charger_gpio;
#define DEBUG_REG 1 
/**********************************************************
 *
 *	[I2C Slave Setting]
 *
 *********************************************************/
#define bq25896_SLAVE_ADDR_WRITE   0xD6
#define bq25896_SLAVE_ADDR_READ    0xD7

#ifdef CONFIG_OF
static const struct of_device_id bq25896_id[] = {
		{ .compatible = "ti,bq25896" },
		{},
};
MODULE_DEVICE_TABLE(of, bq25896_id);
#endif

static struct i2c_client *new_client;
static const struct i2c_device_id bq25896_i2c_id[] = { {"bq25896", 0}, {} };

kal_bool chargin_hw_init_done = KAL_FALSE;
static int bq25896_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

static void bq25896_shutdown(struct i2c_client *client)
{
	battery_log(BAT_LOG_CRTI, "[bq25896_shutdown] driver shutdown\n");
//	bq25896_set_chg_config(0x0);
}
static struct i2c_driver bq25896_driver = {
		.driver = {
				.name    = "bq25896",
#ifdef CONFIG_OF
				.of_match_table = of_match_ptr(bq25896_id),
#endif
		},
		.probe       = bq25896_driver_probe,
		.id_table    = bq25896_i2c_id,
		.shutdown    = bq25896_shutdown,
};

/**********************************************************
 *
 *[Global Variable]
 *
 *********************************************************/
#define bq25896_REG_NUM 21 
unsigned char bq25896_reg[bq25896_REG_NUM] = {0};
extern void do_chrdet_int_task(void);
extern void do_chrdet_int_task_bq25896(void);
int hw_id_charger = 0;
EXPORT_SYMBOL(hw_id_charger);
extern int Read_HW_ID(void);
extern int g_otg_enable;
static DEFINE_MUTEX(bq25896_i2c_access);
int usb_in_int;
EXPORT_SYMBOL(usb_in_int);
struct regulator *VIBR_PMU;
bool b_vibr_enable = false;
extern kal_bool upmu_is_chr_det(void);
extern void init_delay_work(void);

/**********************************************************
 *
 *	[I2C Function For Read/Write bq25896]
 *
 *********************************************************/
int bq25896_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char     readData = 0;
	int      ret = 0;
	struct i2c_msg msg[2];
	struct i2c_adapter *adap = new_client->adapter;

	mutex_lock(&bq25896_i2c_access);
	msg[0].addr = new_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &cmd;

	msg[1].addr = new_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &readData;

	ret = i2c_transfer(adap, msg, 2);
	if (ret < 0) {
		mutex_unlock(&bq25896_i2c_access);
		return 0;
	}
	*returnData = readData;

	mutex_unlock(&bq25896_i2c_access);
	return 1;
}

int bq25896_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;
	struct i2c_msg msg;
	struct i2c_adapter *adap = new_client->adapter;

	mutex_lock(&bq25896_i2c_access);
	write_data[0] = cmd;
	write_data[1] = writeData;
	msg.addr = new_client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = (char *)write_data;

	ret = i2c_transfer(adap, &msg, 1);
	if (ret < 0) {
		mutex_unlock(&bq25896_i2c_access);
		return 0;
	}

	mutex_unlock(&bq25896_i2c_access);
	return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int bq25896_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char bq25896_reg = 0;
	int ret = 0;

	battery_log(BAT_LOG_FULL, "--------------------------------------------------\n");

	ret = bq25896_read_byte(RegNum, &bq25896_reg);

	battery_log(BAT_LOG_FULL, "[bq25896_read_interface] Reg[%x]=0x%x\n", RegNum, bq25896_reg);

	bq25896_reg &= (MASK << SHIFT);
	*val = (bq25896_reg >> SHIFT);

	battery_log(BAT_LOG_FULL, "[bq25896_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int bq25896_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				    unsigned char SHIFT)
{
	unsigned char bq25896_reg = 0;
	int ret = 0;

	battery_log(BAT_LOG_FULL, "--------------------------------------------------\n");

	ret = bq25896_read_byte(RegNum, &bq25896_reg);
	battery_log(BAT_LOG_FULL, "[bq25896_config_interface] Reg[%x]=0x%x\n", RegNum, bq25896_reg);

	bq25896_reg &= ~(MASK << SHIFT);
	bq25896_reg |= (val << SHIFT);

	ret = bq25896_write_byte(RegNum, bq25896_reg);
	battery_log(BAT_LOG_CRTI, "[bq25896_config_interface] write Reg[%x]=0x%x\n", RegNum, bq25896_reg);

	/* Check */
	/* bq25896_read_byte(RegNum, &bq25896_reg); */
	/* battery_log(BAT_LOG_FULL, "[bq25896_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq25896_reg); */

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0--00------------------------------------------------ */

void bq25896_set_en_hiz(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_HIZ_MASK),
				       (unsigned char) (CON0_EN_HIZ_SHIFT)
	    );
}
}
void bq25896_set_en_ilim(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_ILIM_MASK),
				       (unsigned char) (CON0_EN_ILIM_SHIFT)
	    );
}
}

void bq25896_set_iinlim(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_IINLIM_MASK),
				       (unsigned char) (CON0_IINLIM_SHIFT)
	    );
}
}

/* CON1--01------------------------------------------------ */
void bq25896_set_bhot(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_BHOT_MASK), 
                                       (unsigned char) (CON1_BHOT_SHIFT)
	    );
}
}

void bq25896_set_bcold(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_BCOLD_MASK), 
                                       (unsigned char) (CON1_BCOLD_SHIFT)
	    );
}
}

void bq25896_set_vindpm_os(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_VINDPM_OS_MASK), 
                                       (unsigned char) (CON1_VINDPM_OS_SHIFT)
	    );
}
}

/* CON2--02------------------------------------------------ */
void bq25896_set_conv_start(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_CONV_START_MASK),
                                       (unsigned char) (CON2_CONV_START_SHIFT)
	    );
}
}

void bq25896_set_conv_rate(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_CONV_RATE_MASK), 
                                       (unsigned char) (CON2_CONV_RATE_SHIFT)
	    );
}
}

void bq25896_set_boost_freq(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_BOOST_FREQ_MASK), 
                                       (unsigned char) (CON2_BOOST_FREQ_SHIFT)
	    );
}
}

void bq25896_set_ico_en(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_ICO_EN_MASK), 
                                       (unsigned char) (CON2_ICO_EN_SHIFT)
	    );
}
}

void bq25896_set_force_dpmp(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_FORCE_DPDM_MASK), 
                                       (unsigned char) (CON2_FORCE_DPDM_SHIFT)
	    );
}
}

void bq25896_set_auto_dpmp_en(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_AUTO_DPDM_EN_MASK), 
                                       (unsigned char) (CON2_AUTO_DPDM_EN_SHIFT)
	    );
}
}

/* CON3--03------------------------------------------------ */
void bq25896_set_bat_loaden(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){
	ret = bq25896_config_interface((unsigned char) (bq25896_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_BAT_LOADEN_MASK), 
                                       (unsigned char) (CON3_BAT_LOADEN_SHIFT)
	    );
}
}

void bq25896_set_wd_rst(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){
	ret = bq25896_config_interface((unsigned char) (bq25896_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_WD_RST_MASK), 
                                       (unsigned char) (CON3_WD_RST_SHIFT)
	    );
}
}

void bq25896_set_otg_config(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){
	ret = bq25896_config_interface((unsigned char) (bq25896_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_OTG_CONFIG_MASK), 
                                       (unsigned char) (CON3_OTG_CONFIG_SHIFT)
	    );
}
}

void bq25896_set_chg_config(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_CHG_CONFIG_MASK), 
                                       (unsigned char) (CON3_CHG_CONFIG_SHIFT)
	    );
}
}

void bq25896_set_sys_min(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_SYS_MIN_MASK), 
                                       (unsigned char) (CON3_SYS_MIN_SHIFT)
	    );
}
}

void bq25896_set_min_vbat_sel(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_MIN_VBAT_SEL_MASK), 
                                       (unsigned char) (CON3_MIN_VBAT_SEL_SHIFT)
	    );
}
}

/* CON4--04------------------------------------------------ */
void bq25896_set_en_pumpx(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_EN_PUMPX_MASK), 
                                       (unsigned char) (CON4_EN_PUMPX_VREG_SHIFT)
	    );
}
}

void bq25896_set_ichg(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_ICHG_MASK), 
                                       (unsigned char) (CON4_ICHG_SHIFT)
	    );
}
}

/* CON5--05------------------------------------------------ */
void bq25896_set_iprechg(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_IPRECHG_MASK), 
                                       (unsigned char) (CON5_IPRECHG_SHIFT)
	    );
}
}

void bq25896_set_iterm(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_ITERM_MASK), 
                                       (unsigned char) (CON5_ITERM_SHIFT)
	    );
}
}

/* CON6--06------------------------------------------------ */
void bq25896_set_vreg(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VREG_MASK), 
                                       (unsigned char) (CON6_VREG_SHIFT)
	    );
}
}

unsigned int bq25896_get_vreg(void)
{
        unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){

        ret = bq25896_read_interface((unsigned char) (bq25896_CON6),
                                       (&val),
                                       (unsigned char) (CON6_VREG_MASK),
                                       (unsigned char) (CON6_VREG_SHIFT)
            );
}
	return val;
}

void bq25896_set_batlowv(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_BATLOWV_MASK), 
                                       (unsigned char) (CON6_BATLOWV_SHIFT)
	    );
}
}

void bq25896_set_vrechg(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VRECHG_MASK), 
                                       (unsigned char) (CON6_VRECHG_SHIFT)
	    );
}
}
/* CON7--07------------------------------------------------ */
void bq25896_set_en_term(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_EN_TERM_MASK), 
                                       (unsigned char) (CON7_EN_TERM_SHIFT)
	    );
}
}

void bq25896_set_stat_dis(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_STAT_DIS_MASK), 
                                       (unsigned char) (CON7_STAT_DIS_SHIFT)
	    );
}
}

void bq25896_set_watchdog(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_WATCHDOG_MASK), 
                                       (unsigned char) (CON7_WATCHDOG_SHIFT)
	    );
}
}

void bq25896_set_en_timer(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_EN_TIMER_MASK), 
                                       (unsigned char) (CON7_EN_TIMER_SHIFT)
	    );
}
}

void bq25896_set_chg_timer(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_CHG_TIMER_MASK), 
                                       (unsigned char) (CON7_CHG_TIMER_SHIFT)
	    );
}
}

void bq25896_set_jeita_iset(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_JEITA_ISET_MASK), 
                                       (unsigned char) (CON7_JEITA_ISET_SHIFT)
	    );
}
}

/* CON8--08------------------------------------------------ */
void bq25896_set_bat_comp(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON8),
				       (unsigned char) (val),
				       (unsigned char) (CON8_BAT_COMP_MASK), 
                                       (unsigned char) (CON8_BAT_COMP_SHIFT)
	    );
}
}

void bq25896_set_vclamp(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON8),
				       (unsigned char) (val),
				       (unsigned char) (CON8_VCLAMP_MASK), 
                                       (unsigned char) (CON8_VCLAMP_SHIFT)
	    );
}
}

void bq25896_set_treg(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON8),
				       (unsigned char) (val),
				       (unsigned char) (CON8_TREG_MASK), 
                                       (unsigned char) (CON8_TREG_SHIFT)
	    );
}
}

/* CON9--09------------------------------------------------ */
void bq25896_set_force_ico(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_FORCE_ICO_MASK), 
                                       (unsigned char) (CON9_FORCE_ICO_SHIFT)
	    );
}
}

void bq25896_set_tmr2x_en(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_TRM2X_EN_MASK), 
                                       (unsigned char) (CON9_TRM2X_EN_SHIFT)
	    );
}
}

void bq25896_set_batfet_disable(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_BATFET_DIS_MASK), 
                                       (unsigned char) (CON9_BATFET_DIS_SHIFT)
	    );
}
}

void bq25896_set_jeita_vset(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_JEITA_VSET_MASK), 
                                       (unsigned char) (CON9_JEITA_VSET_SHIFT)
	    );
}
}

void bq25896_set_batfet_ddlay(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_BATFET_DLY_MASK), 
                                       (unsigned char) (CON9_BATFET_DLY_SHIFT)
	    );
}
}

void bq25896_set_batfet_rst_en(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_BATFET_RST_EN_MASK), 
                                       (unsigned char) (CON9_BATFET_RST_EN_SHIFT)
	    );
}
}

void bq25896_set_pumpx_up(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_PUMPX_UP_MASK), 
                                       (unsigned char) (CON9_PUMPX_UP_SHIFT)
	    );
}
}

void bq25896_set_pumpx_dn(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_PUMPX_DN_MASK), 
                                       (unsigned char) (CON9_PUMPX_DN_SHIFT)
	    );
}
}
/* CON10-0A------------------------------------------------ */
void bq25896_set_boostv(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON10),
				       (unsigned char) (val),
				       (unsigned char) (CON10_BOOSTV_MASK), 
                                       (unsigned char) (CON10_BOOSTV_SHIFT)
	    );
}
}

void bq25896_set_pfm_otg_dis(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON10),
				       (unsigned char) (val),
				       (unsigned char) (CON10_PFM_OTG_DIS_MASK), 
                                       (unsigned char) (CON10_PFM_OTG_DIS_SHIFT)
	    );
}
}

void bq25896_set_boost_lim(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON10),
				       (unsigned char) (val),
				       (unsigned char) (CON10_BOOST_LIM_MASK), 
                                       (unsigned char) (CON10_BOOST_LIM_SHIFT)
	    );
}
}

/* CON11-0B------------------------------------------------ */
unsigned int bq25896_get_vbus_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON11),
				       (&val),
				       (unsigned char) (CON11_VBUS_STAT_MASK), 
                                       (unsigned char) (CON11_VBUS_STAT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_chrg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON11),
				       (&val),
				       (unsigned char) (CON11_CHRG_STAT_MASK), 
                                       (unsigned char) (CON11_CHRG_STAT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_pg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON11),
				       (&val),
				       (unsigned char) (CON11_PG_STAT_MASK), 
                                       (unsigned char) (CON11_PG_STAT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_vsys_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON11),
				       (&val),
				       (unsigned char) (CON11_VSYS_STAT_MASK), 
                                       (unsigned char) (CON11_VSYS_STAT_SHIFT)
	    );
}
	return val;
}

/* CON12-0C------------------------------------------------ */
unsigned int bq25896_get_watchdog_fault(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON12),
				       (&val),
				       (unsigned char) (CON12_WATCHDOG_FAULT_MASK), 
                                       (unsigned char) (CON12_WATCHDOG_FAULT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_boost_fault(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON12),
				       (&val),
				       (unsigned char) (CON12_BOOST_FAULT_MASK), 
                                       (unsigned char) (CON12_BOOST_FAULT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_chrg_fault(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON12),
				       (&val),
				       (unsigned char) (CON12_CHRG_FAULT_MASK), 
                                       (unsigned char) (CON12_CHRG_FAULT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_bat_fault(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON12),
				       (&val),
				       (unsigned char) (CON12_BAT_FAULT_MASK), 
                                       (unsigned char) (CON12_BAT_FAULT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_ntc_fault(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON12),
				       (&val),
				       (unsigned char) (CON12_NTC_FAULT_MASK), 
                                       (unsigned char) (CON12_NTC_FAULT_SHIFT)
	    );
}
	return val;
}

/* CON13-0D------------------------------------------------ */
void bq25896_set_force_vindpm(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON13),
				       (unsigned char) (val),
				       (unsigned char) (CON13_FORCE_VINDPM_MASK), 
                                       (unsigned char) (CON13_FORCE_VINSPM_SHIFT)
	    );
}
}

void bq25896_set_vindpm(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON13),
				       (unsigned char) (val),
				       (unsigned char) (CON13_VINDPM_MASK), 
                                       (unsigned char) (CON13_VINDPM_SHIFT)
	    );
}
}

/* CON14-0E------------------------------------------------ */
unsigned int bq25896_get_therm_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON14),
				       (&val),
				       (unsigned char) (CON14_THERM_STAT_MASK), 
                                       (unsigned char) (CON14_THERM_STAT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_batv(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON14),
				       (&val),
				       (unsigned char) (CON14_BATV_MASK), 
                                       (unsigned char) (CON14_BATV_SHIFT)
	    );
}
	return val;
}

/* CON15-0F------------------------------------------------ */
unsigned int bq25896_get_sysv(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON15),
				       (&val),
				       (unsigned char) (CON15_SYSV_MASK), 
                                       (unsigned char) (CON15_SYSV_SHIFT)
	    );
}
	return val;
}

/* CON16-10------------------------------------------------ */
unsigned int bq25896_get_tspct(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON16),
				       (&val),
				       (unsigned char) (CON16_TSPCT_MASK), 
                                       (unsigned char) (CON16_TSPCT_SHIFT)
	    );
}
	return val;
}

/* CON17-11------------------------------------------------ */
unsigned int bq25896_get_vbus_gd(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON17),
				       (&val),
				       (unsigned char) (CON17_VBUS_GD_MASK), 
                                       (unsigned char) (CON17_VBUS_GD_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_vbusv(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON17),
				       (&val),
				       (unsigned char) (CON17__MASK), 
                                       (unsigned char) (CON17__SHIFT)
	    );
}
	return val;
}

/* CON18-12------------------------------------------------ */
unsigned int bq25896_get_ichgr(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON18),
				       (&val),
				       (unsigned char) (CON18_ICHGR_MASK), 
                                       (unsigned char) (CON18_ICHGR_SHIFT)
	    );
}
	return val;
}

/* CON19-13------------------------------------------------ */
unsigned int bq25896_get_vdpm_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON19),
				       (&val),
				       (unsigned char) (CON19_VDPM_STAT_MASK), 
                                       (unsigned char) (CON19_VDPM_STAT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_idpm_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON19),
				       (&val),
				       (unsigned char) (CON19_IDPM_STAT_MASK), 
                                       (unsigned char) (CON19_IDPM_STAT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_idpm_lim(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON19),
				       (&val),
				       (unsigned char) (CON19_IDPM_LIM_MASK), 
                                       (unsigned char) (CON19_IDPM_LIM_SHIFT)
	    );
}
	return val;
}

/* CON20-14------------------------------------------------ */
void bq25896_set_reg_rst(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON20),
				       (unsigned char) (val),
				       (unsigned char) (CON20_REG_RST_MASK), 
                                       (unsigned char) (CON20_REG_RST_SHIFT)
	    );
}
}

void bq25896_set_ico_optimized(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON20),
				       (unsigned char) (val),
				       (unsigned char) (CON20_ICO_OPTIMIZED_MASK), 
                                       (unsigned char) (CON20_ICO_OPTIMIZED_SHIFT)
	    );
}
}

unsigned int bq25896_get_pn(void)
{
	unsigned char val = 0;
	unsigned int ret = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON20),
				       (&val),
				       (unsigned char) (CON20_PN_MASK), 
                                       (unsigned char) (CON20_PN_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_ts_profile(void)
{
	unsigned char val = 0;
	unsigned int ret = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON20),
				       (&val),
				       (unsigned char) (CON20_TS_PROFILE_MASK), 
                                       (unsigned char) (CON20_TS_PROFILE_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_dev_rev(void)
{
	unsigned char val = 0;
	unsigned int ret = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON20),
				       (&val),
				       (unsigned char) (CON20_DEV_REV_MASK), 
                                       (unsigned char) (CON20_DEV_REV_SHIFT)
	    );
}
	return val;
}

/**********************************************************
  *
  *   [External Function]
  *
 *********************************************************/
void charging_pad_or_standac(void)
{
                battery_log(BAT_LOG_CRTI, "<BATT>[%s]charging mode: pad only\n", __func__);
                bq25896_set_watchdog(0x3);      // 1. watch dog timer = 160 sec
                bq25896_set_en_ilim(0x1);       // 2. enable current limit
                bq25896_set_iinlim(0x8);        // 3. set input current limit = 500 mA
                bq25896_set_auto_dpmp_en(0x0);  // 4. disable auto_dpdm
		bq25896_set_wd_rst(0x1);
}
EXPORT_SYMBOL(charging_pad_or_standac);
/**********************************************************
  *
  *   [Internal Function]
  *
 *********************************************************/
void bq25896_dump_register(void)
{
	int i = 0;

	battery_log(BAT_LOG_CRTI, "[bq25896] ");
	for (i = 0; i < bq25896_REG_NUM; i++) {
		bq25896_read_byte(i, &bq25896_reg[i]);
		battery_log(BAT_LOG_CRTI, "[0x%x]=0x%x ", i, bq25896_reg[i]);
	}
	battery_log(BAT_LOG_CRTI, "\n");
}

#define CHARGING_LIMIT_THRESHOLD_FILE   "/factory/charging_limit_threshold"
bool b_eng_charging_limit = true;
int eng_charging_limit_soc = CHARGING_LIMIT_THRESHOLD;
static void charger_limit(struct work_struct *work){
        struct file *fp = NULL;
        mm_segment_t old_fs;
        static char b_buf[sizeof(int)];
        int byte_count = 0;
        s32 res;

        battery_log(BAT_LOG_CRTI, "************ CHARGER LIMIT THRESHOLD read from file ************\n");
        fp = filp_open(CHARGING_LIMIT_THRESHOLD_FILE, O_RDWR|O_CREAT, 0666);
        if (!IS_ERR_OR_NULL(fp) ) {
                old_fs = get_fs();
                set_fs(KERNEL_DS);

                /* read file */
                byte_count= fp->f_op->read(fp, b_buf, sizeof(b_buf), &fp->f_pos);
                battery_log(BAT_LOG_CRTI, "read data from file: %s\n", b_buf);

                /* using default 60 if it's empty */
                if (!strncmp("", b_buf, 1))
                        goto file_open_but_error;

                if (kstrtos32(b_buf, 10, &res)) {
                        battery_log(BAT_LOG_CRTI,"%s: kstrtos32 error!", __func__);
                        goto file_open_but_error;
                }
                battery_log(BAT_LOG_CRTI, "read data from file(int): %d\n", res);

                /* replace the charging limit threshold with new value */
                eng_charging_limit_soc = res;

                /* Restore segment descriptor */
                set_fs(old_fs);

                /* Close file operation */
                filp_close(fp, NULL);
        }
        else{
                battery_log(BAT_LOG_CRTI,"%s: file open error (%s)\n", __func__, CHARGING_LIMIT_THRESHOLD_FILE);
                return;
        }

        return;
file_open_but_error:
        /* Restore segment descriptor */
        set_fs(old_fs);

        /* Close file operation */
        filp_close(fp, NULL);
        return;
}

/**
 * usb_connector_therm_handler() - called when the gpio 6 status changes.
 * This is used for detecting usb connector thermal abnormal
 */
static irqreturn_t usb_connector_therm_handler(int irq, void *_chip)
{
	g_usb_connector_event = gpio_get_value(charger_gpio->USBCON_TEMP);
	switch_set_state(&charger_dev, g_usb_connector_event);
	battery_log(BAT_LOG_CRTI, "[ChargerSetting][%s] gpio 6 is %s\n", __func__, g_usb_connector_event ? "high" : "low");
	if (g_usb_connector_event == 1 && upmu_is_chr_det() == KAL_TRUE)
	{
		battery_log(BAT_LOG_CRTI, "[ChargerSetting][%s] set charger suspend due to gpio 6 event\n", __func__);
		bq25896_set_en_hiz(0x1);
	}

	return IRQ_HANDLED;
}

static int bq25896_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct charger_gpio *charger;
	kal_bool chr_status;
	int rc = 0;

	battery_log(BAT_LOG_CRTI, "[bq25896_driver_probe]\n");
	charger = devm_kzalloc(dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;
	charger->dev = dev;
	charger_gpio = charger;

	new_client = client;
	charger_get_gpio_info();
	charger_gpio_request();

	/* register switch device for usb connector status */
	charger_dev.name = "usb_connector";
	if (switch_dev_register(&charger_dev) < 0) {
		battery_log(BAT_LOG_CRTI, "[%s] fail to register charger switch\n", __func__);
	} else {
		g_usb_connector_event = gpio_get_value(charger_gpio->USBCON_TEMP);
		battery_log(BAT_LOG_CRTI, "[%s] check USBCON_TEMP status first: %s\n", __func__, g_usb_connector_event ? "high" : "low");
		switch_set_state(&charger_dev, g_usb_connector_event);
	}

	/* add gpio 6 interrupt func for usb thermal issue */
	rc = gpio_to_irq(charger_gpio->USBCON_TEMP);
	if (rc < 0) {
		battery_log(BAT_LOG_CRTI, "[%s] config USBCON_TEMP as irq fail with rc: %d\n", __func__, rc);
	} else {
		rc = devm_request_threaded_irq(dev, gpio_to_irq(charger_gpio->USBCON_TEMP), NULL,
						usb_connector_therm_handler,
						IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"usb_connector_therm_irq", dev);
		if (rc)
			battery_log(BAT_LOG_CRTI, "[%s] request USBCON_TEMP irq handler failed with rc: %d\n", __func__, rc);
		else
			battery_log(BAT_LOG_CRTI, "[%s] request USBCON_TEMP irq handler successed!!\n", __func__);
	}
	/* --------------------- */
	
	adapter_wq = create_singlethread_workqueue("adapter_wq");
	init_delay_work();
        INIT_DELAYED_WORK(&charger_gpio->charger_limit_work, charger_limit);
        schedule_delayed_work(&charger_gpio->charger_limit_work, msecs_to_jiffies(15000));

	printk("[%s] CHARGING_CMD_GET_CHARGER_DET_STATUS\n",__func__);
	chr_control_interface(CHARGING_CMD_GET_CHARGER_DET_STATUS, &chr_status);
        if(chr_status == TRUE){
		printk("[%s] Can't distinguish Charger Type > Continue\n",__func__);
        }else{
                printk("[%s] PAD only\n",__func__);
                chr_control_interface(CHARGING_CMD_INIT, NULL);
        }
        bq25896_dump_register();
        chargin_hw_init_done = KAL_TRUE;
	b_otg_probe = 1;
	hw_id_charger = Read_HW_ID();
	return 0;
}

/**********************************************************
 *
 *	[platform_driver API]
 *
 *********************************************************/
unsigned char g_reg_value_bq25896 = 1;

static ssize_t show_CHGLimit(struct device *dev, struct device_attribute *attr, char *buf)
{
		battery_log(BAT_LOG_CRTI, "[%s] chg_timeout_mos(%d)\n", __func__, chg_timeout_mos);
		return sprintf(buf, "%d\n", chg_timeout_mos);
}
static ssize_t store_CHGLimit(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
		sscanf(buf, "%d\n", &chg_timeout_mos);
		Write_CHG_TimeOut_Limit();
		battery_log(BAT_LOG_CRTI, "[%s] chg_timeout_mos(%d)\n", __func__, chg_timeout_mos);
		return size;
}
static DEVICE_ATTR(CHGLimit, 0664, show_CHGLimit, store_CHGLimit);

static ssize_t show_ICHGR(struct device *dev, struct device_attribute *attr, char *buf)
{
	int current1 = 0;
	int value ;
	int i;
	int mA = 50;
	value = bq25896_get_ichgr();
	for(i=0;i<7;i++)
	{
		if ((value & BIT(i))==BIT(i))
			current1 += mA; 
		mA *= 2;
	}
	battery_log(BAT_LOG_CRTI, "[show_ICHGR] current =%d\n", current1);
	return sprintf(buf, "%d\n", current1);
}
static ssize_t store_ICHGR(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        return size;
}
static DEVICE_ATTR(ICHGR, 0664, show_ICHGR, store_ICHGR);

static ssize_t show_bq25896_eng_charging_limit_soc(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_log(BAT_LOG_CRTI, "[%s] eng_charging_limit_soc = %d\n", __func__, eng_charging_limit_soc);
	return sprintf(buf, "%d\n", eng_charging_limit_soc);
}

static ssize_t store_bq25896_eng_charging_limit_soc(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        int limit_threshold;
        static char b_buf[sizeof(int)];
        mm_segment_t old_fs;
        struct file *fp = NULL;

        sscanf(buf, "%du", &limit_threshold);
        eng_charging_limit_soc = limit_threshold;
	battery_log(BAT_LOG_CRTI, "[%s] eng_charging_limit_soc = %d\n", __func__, eng_charging_limit_soc);

        fp = filp_open(CHARGING_LIMIT_THRESHOLD_FILE, O_RDWR|O_CREAT , 0666);
        if (!IS_ERR_OR_NULL(fp) ) {
                old_fs = get_fs();
                set_fs(KERNEL_DS);

                sprintf(b_buf, "%d", eng_charging_limit_soc);
                fp->f_op->llseek(fp, 0, 0);
                fp->f_op->write(fp,b_buf,sizeof(b_buf),&fp->f_pos);
                pr_err("writr new eng_charging_limit_soc = %d to the file.\n", eng_charging_limit_soc);
                set_fs(old_fs);
                if(fp!=NULL)
                        filp_close(fp, NULL);
        }

        return size;
}
#ifdef ENG_BUILD
static DEVICE_ATTR(bq25896_eng_charging_limit_soc, 0666, show_bq25896_eng_charging_limit_soc, store_bq25896_eng_charging_limit_soc);
#else
static DEVICE_ATTR(bq25896_eng_charging_limit_soc, 0664, show_bq25896_eng_charging_limit_soc, store_bq25896_eng_charging_limit_soc);
#endif
/*establish file node for eng_charging limit enable*/
static ssize_t show_bq25896_eng_charging_limit(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_log(BAT_LOG_CRTI, "[show_bq25896_eng_charging_limit] eng_charging_limit = %d\n", b_eng_charging_limit);
	return sprintf(buf, "%d\n", b_eng_charging_limit);
}

static ssize_t store_bq25896_eng_charging_limit(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int request;
	unsigned int b_charging_enable;

	sscanf(buf, "%du", &request);
	
	if (!request){
		b_eng_charging_limit = false;
		b_charging_enable = KAL_TRUE;
                battery_log(BAT_LOG_CRTI, "[store_bq25896_eng_charging_limit] eng_charging_limit = %d\n",b_eng_charging_limit);
	}else{
		b_eng_charging_limit = true;
		if (BMT_status.SOC > eng_charging_limit_soc)
			b_charging_enable = KAL_FALSE;  //stop charging if capacity > eng_charging_limit_soc
		else
			b_charging_enable = KAL_TRUE;

		battery_log(BAT_LOG_CRTI, "[store_bq25896_eng_charging_limit] eng_charging_limit = %d\n",b_eng_charging_limit);
	}
	chr_control_interface(CHARGING_CMD_ENABLE, &b_charging_enable);	
	return size;
}
#ifdef ENG_BUILD
static DEVICE_ATTR(bq25896_eng_charging_limit, 0666, show_bq25896_eng_charging_limit, store_bq25896_eng_charging_limit);
#else
static DEVICE_ATTR(bq25896_eng_charging_limit, 0664, show_bq25896_eng_charging_limit, store_bq25896_eng_charging_limit);
#endif
/*end charging limit file node.*/
static ssize_t show_bq25896_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_log(BAT_LOG_CRTI, "[show_bq25896_access] 0x%x\n", g_reg_value_bq25896);
	return sprintf(buf, "%u\n", g_reg_value_bq25896);
}
static ssize_t store_bq25896_access(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	battery_log(BAT_LOG_CRTI, "[store_bq25896_access]\n");

	if (buf != NULL && size != 0) {
		battery_log(BAT_LOG_CRTI, "[store_bq25896_access] buf is %s and size is %zu\n", buf, size);
		/*reg_address = kstrtoul(buf, 16, &pvalue);*/

		pvalue = (char *)buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			ret = kstrtou32(addr, 16, (unsigned int *)&reg_address);
		} else
			ret = kstrtou32(pvalue, 16, (unsigned int *)&reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			ret = kstrtou32(val, 16, (unsigned int *)&reg_value);

			battery_log(BAT_LOG_CRTI,
			    "[store_bq25896_access] write bq25896 reg 0x%x with value 0x%x !\n",
			     reg_address, reg_value);
			ret = bq25896_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = bq25896_read_interface(reg_address, &g_reg_value_bq25896, 0xFF, 0x0);
			battery_log(BAT_LOG_CRTI,
			    "[store_bq25896_access] read bq25896 reg 0x%x with value 0x%x !\n",
			     reg_address, g_reg_value_bq25896);
			battery_log(BAT_LOG_CRTI,
			    "[store_bq25896_access] Please use \"cat bq25896_access\" to get value\r\n");
		}
	}
	return size;
}
static DEVICE_ATTR(bq25896_access, 0664, show_bq25896_access, store_bq25896_access);

static ssize_t show_bq25896_dump (struct device *dev, struct device_attribute *attr, char *buf)
{
        int i = 0;
	char *p = buf;

        p += sprintf(p,"[bq25896]\n");
        for (i = 0; i < bq25896_REG_NUM; i++) {
                bq25896_read_byte(i, &bq25896_reg[i]);
		p += sprintf(p, " [0x%x]=0x%x \n",i,bq25896_reg[i]);
        }
	i = p - buf;		
	return i;
}
static ssize_t store_bq25896_dump(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}
static DEVICE_ATTR(bq25896_dump, 0664, show_bq25896_dump, store_bq25896_dump);

static ssize_t show_otg_on (struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "otg_on\n");;	
}
static ssize_t store_otg_on(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        int ret = 0;
        char *pvalue = NULL, *gpio;
        unsigned int value = 0;

        printk(KERN_ERR "%s\n", __func__);

        pvalue = (char *)buf;
        gpio = strsep(&pvalue, " ");
        ret = kstrtou32(gpio, 16, (unsigned int *)&value);
        gpio_direction_output(charger_gpio->PG_OTG_EN_SOC,value);
	bq25896_set_otg_config(value);	
			
        printk(KERN_ERR "PG_OTG_EN_SOC = %d\n",gpio_get_value(charger_gpio->PG_OTG_EN_SOC));
	
	return size;
}
static DEVICE_ATTR(otg_on, 0664, show_otg_on, store_otg_on);

static ssize_t show_gpio_set(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *p = buf;
	int ret = 0;
	p += sprintf(p,"PG_OTG_EN_SOC = %d\n",gpio_get_value(charger_gpio->PG_OTG_EN_SOC));
	p += sprintf(p,"LID_EN = %d\n",gpio_get_value(charger_gpio->LID_EN));
//shilun	p += sprintf(p,"USBCON_LID = %d\n",gpio_get_value(charger_gpio->USBCON_LID));
	ret = p - buf;
	return ret;
}
static ssize_t store_gpio_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *gpio;
	unsigned int value = 0;

	printk(KERN_ERR "%s\n", __func__);

	pvalue = (char *)buf;
	gpio = strsep(&pvalue, " ");
	ret = kstrtou32(gpio, 16, (unsigned int *)&value);
	gpio_direction_output(charger_gpio->PG_OTG_EN_SOC,value);
	printk(KERN_ERR "PG_OTG_EN_SOC = %d\n",gpio_get_value(charger_gpio->PG_OTG_EN_SOC));
	
	gpio = strsep(&pvalue, " ");
        ret = kstrtou32(gpio, 16, (unsigned int *)&value);
        gpio_direction_output(charger_gpio->LID_EN,value);
        printk(KERN_ERR "LID_EN = %d\n",gpio_get_value(charger_gpio->LID_EN));

//shilun	gpio = strsep(&pvalue, " ");
//        ret = kstrtou32(gpio, 16, (unsigned int *)&value);
//        gpio_direction_output(charger_gpio->USBCON_LID,value);
//        printk(KERN_ERR "USBCON_LID = %d\n",gpio_get_value(charger_gpio->USBCON_LID));
        return size;
}
static DEVICE_ATTR(gpio_set, 0664, show_gpio_set, store_gpio_set);

static ssize_t show_stop_charger_thermal(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "b_stop_thermal = %d\n",b_stop_thermal);
}

static ssize_t store_stop_charger_thermal(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	sscanf(buf, "%d\n", &b_stop_thermal);
	return size;
}

static DEVICE_ATTR(stop_charger_thermal, 0664, show_stop_charger_thermal, store_stop_charger_thermal);

static ssize_t show_stop_jeita(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "b_stop_jeita = %d\n",b_stop_jeita);
}

static ssize_t store_stop_jeita(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	sscanf(buf, "%d\n", &b_stop_jeita);
	return size;
}

static DEVICE_ATTR(stop_jeita, 0664, show_stop_jeita, store_stop_jeita);

static ssize_t show_charger_thermal_threshold(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "charger_thermal_level1 = %d\n charger_thermal_level2 = %d\n charger_thermal_level3 = %d\n ", 
		charger_thermal_level1, charger_thermal_level2, charger_thermal_level3);
}

static ssize_t store_charger_thermal_threshold(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	int level = 0;
	char *pvalue = NULL;
	char *plevel = NULL;

	battery_log(BAT_LOG_CRTI, "[store_charger_thermal_threshold]\n");
	if( buf != NULL && size != 0){
		pvalue = (char *)buf;
		plevel = strsep(&pvalue, " ");
		ret = kstrtou32(plevel, 16, (unsigned int *)&level);

		plevel = strsep(&pvalue, " ");
		switch(level)
		{
			case 1:
				sscanf(plevel, "%d\n", &charger_thermal_level1);
				break;
			case 2:
				sscanf(plevel, "%d\n", &charger_thermal_level2);
				break;
			case 3:
				sscanf(plevel, "%d\n", &charger_thermal_level3);
				break;
			default:
				break;
		}
	}
	return size;
}

static DEVICE_ATTR(charger_thermal_threshold, 0664, show_charger_thermal_threshold, store_charger_thermal_threshold);

static int bq25896_user_space_probe(struct platform_device *dev)
{
		int ret_device_file = 0;
	battery_log(BAT_LOG_CRTI, "******** bq25896_user_space_probe!! ********\n");

		ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq25896_access);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq25896_dump);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq25896_eng_charging_limit);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq25896_eng_charging_limit_soc);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_gpio_set);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_CHGLimit);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ICHGR);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_otg_on);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_stop_charger_thermal);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_stop_jeita);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_charger_thermal_threshold);
		return 0;
}

struct platform_device bq25896_user_space_device = {
		.name   = "bq25896-user",
		.id     = -1,
};

static struct platform_driver bq25896_user_space_driver = {
		.probe      = bq25896_user_space_probe,
		.driver     = {
				.name = "bq25896-user",
		},
};


extern int g_vcdt_irq;
int g_projector_on = 0;

void bq25896_PG_OTG_EN_SOC(int val)
{
	battery_log(BAT_LOG_CRTI, "<BATT>[%s]\n", __func__);
	if (val){
		if (b_otg_probe)
			msleep(5000);
		if (hw_id_charger < 2)
			gpio_direction_output(charger_gpio->LID_EN,val);
		bq25896_set_boost_lim(0x6);	// 1. Set Boost current limit = 2150 mA
		bq25896_set_boostv(0xA);	// 2. Boost Voltage = 5.19 V
		bq25896_set_otg_config(0x1);	// 3. Enable OTG
		gpio_direction_output(charger_gpio->PG_OTG_EN_SOC, val);
		bq25896_set_wd_rst(0x1);	// Watch dog timer reset
	}
	else{
		bq25896_set_otg_config(0x0);	// 1. Disable OTG
		gpio_direction_output(charger_gpio->PG_OTG_EN_SOC, val);
		bq25896_set_en_ilim(0x1);	// 3. enable current limit
                chr_control_interface(CHARGING_CMD_PADONLY, NULL);
	}
}


void bq25896_LID(int val)
{
#ifndef ENG_BUILD
	battery_log(BAT_LOG_CRTI, "<BATT>[%s] LID = %s\n", __func__, val?"Disable":"Enable");
	gpio_direction_output(charger_gpio->LID_EN,val);
#endif
}

void charger_get_gpio_info(void)
{
	static struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "ti,bq25896");
	charger_gpio->PG_OTG_EN_SOC = of_get_named_gpio(node, "er_pg_otg_en_soc", 0);
	battery_log(BAT_LOG_CRTI, "PG_OTG_EN_SOC = %d\n", charger_gpio->PG_OTG_EN_SOC);

	charger_gpio->LID_EN = of_get_named_gpio(node, "er_lid_en", 0);
	battery_log(BAT_LOG_CRTI, "LID_EN = %d\n", charger_gpio->LID_EN);

	charger_gpio->USBCON_LID = of_get_named_gpio(node, "er_usbcon_lid", 0);
	battery_log(BAT_LOG_CRTI, " USBCON_LID = %d\n", charger_gpio->USBCON_LID);

	charger_gpio->USBCON_TEMP = of_get_named_gpio(node, "USBCON_TEMP", 0);
	battery_log(BAT_LOG_CRTI, " USBCON_LID = %d\n", charger_gpio->USBCON_TEMP);
}

void charger_gpio_request(void)
{
	int err = 0;
	
	err = gpio_request(charger_gpio->PG_OTG_EN_SOC, "PG_OTG_EN_SOC");	//GPIO5
        if (err<0)
                battery_log(BAT_LOG_CRTI, "%s: gpio_request failed for gpio LID_EN = %d\n", __func__, charger_gpio->LID_EN);
        else
                gpio_direction_output(charger_gpio->PG_OTG_EN_SOC, 0);

	err = gpio_request(charger_gpio->USBCON_TEMP, "USBCON_TEMP");	//GPIO6
        if (err<0)
                battery_log(BAT_LOG_CRTI, "%s: gpio_request failed for gpio USBCON_TEMP = %d\n", __func__, charger_gpio->USBCON_TEMP);
        else
                gpio_direction_input(charger_gpio->USBCON_TEMP);

	err = gpio_request(charger_gpio->LID_EN, "LID_EN");			//GPIO19
        if (err<0)
                battery_log(BAT_LOG_CRTI, "%s: gpio_request failed for gpio LID_EN = %d\n", __func__, charger_gpio->LID_EN);
        else
                gpio_direction_output(charger_gpio->LID_EN, 1);

        err = gpio_request(charger_gpio->USBCON_LID, "USBCON_LID");		//GPIO38
        if (err<0)
                battery_log(BAT_LOG_CRTI, "%s: gpio_request failed for gpio USBCON_LID = %d\n", __func__, charger_gpio->USBCON_LID);
        else
                gpio_direction_input(charger_gpio->USBCON_LID);
 //shilun				gpio_direction_output(charger_gpio->USBCON_LID, 1);
}

void charger_set_gpio(int gpio_usbbusonnum, int gpio_coverbuson, int gpio_coverbusoff, int gpio_padqbdet) 
{
}
EXPORT_SYMBOL(charger_set_gpio);

static int __init bq25896_init(void)
{
	int ret = 0;

	battery_log(BAT_LOG_CRTI, "[bq25896_init] init start\n");

#ifndef CONFIG_OF
	i2c_register_board_info(BQ25896_BUSNUM, &i2c_bq25896, 1);
#endif

	if (i2c_add_driver(&bq25896_driver) != 0)
		battery_log(BAT_LOG_CRTI, "[bq25896_init] failed to register bq25896 i2c driver.\n");
	else
		battery_log(BAT_LOG_CRTI, "[bq25896_init] Success to register bq25896 i2c driver.\n");

	/*bq25896 user space access interface*/
	ret = platform_device_register(&bq25896_user_space_device);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[bq25896_init] Unable to device register(%d)\n", ret);
		return ret;
	}

	ret = platform_driver_register(&bq25896_user_space_driver);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[bq25896_init] Unable to register driver (%d)\n", ret);
		return ret;
	}

	return 0;
}

static void __exit bq25896_exit(void)
{
		i2c_del_driver(&bq25896_driver);
}
//late_initcall(bq25896_init);
module_init(bq25896_init);
module_exit(bq25896_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq25896 Driver");
MODULE_AUTHOR("Shilun Huang<shilun_huang@asus.com>");
