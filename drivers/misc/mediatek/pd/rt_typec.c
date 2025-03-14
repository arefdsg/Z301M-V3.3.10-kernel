#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <mt-plat/charging.h>
#include <mt-plat/battery_meter.h>
#include <mt-plat/battery_common.h>
#include "inc/tcpm.h"
#include "inc/tcpci.h"
#include "inc/typec.h"
#include <linux/clk.h>
#include <linux/pm_qos.h>
#include <linux/regulator/consumer.h>
#include <linux/of_device.h>

//static struct notifier_block type_c_nb;
//static struct platform_device *plat_dev;
static struct tcpc_device *tcpc_dev;
static int tcpc_pd_state;
static int tcpc_power_role;
static int tcpc_sink_voltage;
static int tcpc_sink_current;
static int tcpc_typec_state;
static int tcpc_pre_typec_state;
static bool tcpc_src_support_pd;
static bool otg_last_time_status;

struct typec_core_state {
        u32 vbus:1,
                vcon:1,
                data:2,
                pd_has_contract:1,
                giveup_get_vbus_reg:1,
                giveup_get_usb_phy:1,
                giveup_get_vconn_reg:1,
                shutdown:1,
                hardreset:1,    /* PD in hardreset */
                pr_swap:1;      /* in swapping power role */
};

typedef enum {
        DATA_ROLE_NONE = 0,
        DATA_ROLE_DOWN = 1,
        DATA_ROLE_UP = 2,
} typec_core_data_role;

struct rt_typec_pd_info {
        struct device *dev;
        struct notifier_block nb;
        struct tcpc_device *tcpc;
        struct mutex state_lock;
        struct tcp_ny_typec_state typec_state;
        struct tcp_ny_pd_state pd_state;
        struct tcp_ny_vbus_state vbus_state;
        struct notifier_block power_nb;
        atomic_t latest_batt_capacity;
        atomic_t latest_batt_temperature;
        struct typec_device_ops *ops;
        struct typec_core_state state;
#if defined(CONFIG_USB_DWC3_PD_DUAL_ROLE_SWAP)
        struct regulator *vbus_reg;
        struct regulator *vcon_reg;
        struct power_supply *usb_psy;
#endif
        bool bus_clks_enabled;
//        struct clk *bus_clks[NUM_BUS_CLOCKS];
        struct pm_qos_request pm_qos_req_dma;
        s32 pm_qos_latency;
        struct delayed_work cancel_qos_work;
};
static struct rt_typec_pd_info rt_info;

#if 0
#define rt_info_set_field(field, v)                     \
        do {                                            \
                mutex_lock(&rt_info.state_lock);        \
                rt_info.field = (v);                    \
                mutex_unlock(&rt_info.state_lock);      \
                                                        \
        } while(0)

#define rt_info_get_field(field, v)                     \
        do {                                            \
                mutex_lock(&rt_info.state_lock);        \
                v = rt_info.field;                      \
                mutex_unlock(&rt_info.state_lock);      \
                                                        \
        } while(0)
#endif

#define rt_info_set_field(field, v)                     \
        do {                                            \
                rt_info.field = (v);                    \
                                                        \
        } while(0)

#define rt_info_get_field(field, v)                     \
        do {                                            \
                v = rt_info.field;                      \
                                                        \
        } while(0)

struct rt1711_chip *chip;

/**
 * Invoked by typec-core/typec_current_mode_detect for charger to detect
 * the termination type when we are SINK_ATTACHED.
 * @return The detected current mode
 *   TYPEC_CURRENT_MODE_UNSPPORTED: When the attached mode is not sink attached
 *   TYPEC_CURRENT_MODE_DEFAULT: 500mA
 *   TYPEC_CURRENT_MODE_MID: 1.5A
 *   TYPEC_CURRENT_MODE_HIGH: 3A
 */
static enum typec_current_mode rt1711_current_mode_detect(void)
{
        enum typec_current_mode cmode;
        struct tcp_ny_typec_state typec_state;

        rt_info_get_field(typec_state, typec_state);
#if defined (CONFIG_PD_DBG_INFO)
        pr_info("RT1711 %s: rp_level=%d, polarity=%d, old_state=%d, new_state=%d\n",
                 __func__, typec_state.rp_level, typec_state.polarity,
                 typec_state.old_state, typec_state.new_state);
#endif
        if (typec_state.new_state != TYPEC_ATTACHED_SNK) {
                return TYPEC_CURRENT_MODE_UNSPPORTED;
        }
        switch (typec_state.rp_level) {
        case TYPEC_CC_VOLT_SNK_1_5:
                cmode = TYPEC_CURRENT_MODE_MID;
                break;
        case TYPEC_CC_VOLT_SNK_3_0:
                cmode = TYPEC_CURRENT_MODE_HIGH;
                break;
        case TYPEC_CC_VOLT_SNK_DFT:
        default:
                cmode = TYPEC_CURRENT_MODE_DEFAULT;

        }
        return cmode;
}

static enum typec_attached_state rt1711_attatched_state_detect(void)
{
        enum typec_attached_state attach_state;
        struct tcp_ny_typec_state typec_state;

        rt_info_get_field(typec_state, typec_state);
#if defined (CONFIG_PD_DBG_INFO)
        pr_info("RT1711 %s: rp_level=%d, polarity=%d, old_state=%d, new_state=%d\n",
                 __func__, typec_state.rp_level, typec_state.polarity,
                 typec_state.old_state, typec_state.new_state);
#endif
        /* map 'enum typec_attach_type' (RT specific) to 'enum typec_attached_state' */
        switch (typec_state.new_state) {
        case TYPEC_ATTACHED_SNK:
                attach_state = TYPEC_ATTACHED_AS_UFP;
                break;
        case TYPEC_ATTACHED_SRC:
                attach_state = TYPEC_ATTACHED_AS_DFP;
                break;
        case TYPEC_ATTACHED_AUDIO:
        case TYPEC_ATTACHED_DEBUG:
                attach_state = TYPEC_ATTACHED_TO_ACCESSORY;
                break;
        default:
                attach_state = TYPEC_NOT_ATTACHED;
        }
        return attach_state;
}
static enum typec_current_mode rt1711_current_advertise_get(void)
{
        WARN(1, "unimplement foo - %s\n", __func__);
        return TYPEC_CURRENT_MODE_DEFAULT;
}

static int rt1711_current_advertise_set(enum typec_current_mode current_mode)
{
        WARN(1, "unimplement foo - %s\n", __func__);
        return 0;
}
/* call from 'cat /sys/class/typec/typec_device/port_mode_ctrl'
 * Besides BSP/ATD tools, no one will access it. */
static enum typec_port_mode rt1711_port_mode_get(void)
{
        WARN(1, "unimplement foo - %s\n", __func__);
        return TYPEC_MODE_ACCORDING_TO_PROT;
}

/* call from 'echo 0|1|2|3 > /sys/class/typec/typec_device/port_mode_ctrl'
 *  0: TYPEC_MODE_ACCORDING_TO_PROT
 *  1: TYPEC_UFP_MODE
 *  2: TYPEC_DFP_MODE
 *  3: TYPEC_DRP_MODE
 * Besides BSP/ATD tools, no one will access it.
 * */
static int rt1711_port_mode_set(enum typec_port_mode port_mode)
{
        WARN(1, "unimplement foo - %s\n", __func__);
        return 0;
}
/* call from 'cat /sys/class/typec/typec_device/dump_regs'
 * Besides BSP/ATD tools, no one will access it.
 * */
static ssize_t rt1711_dump_regs(char *buf)
{
        WARN(1, "unimplement foo - %s\n", __func__);
        return 0;
}

/* call from 'cat /sys/class/typec/typec_device/i2c_status'
 * Besides BSP/ATD tools, no one will access it.
 * */
static bool rt1711_i2c_status(void)
{
        struct i2c_client *client = chip->client;
        u16 vid, pid;
        int ret;

        ret = rt1711_read_device(client, TCPC_V10_REG_VID, 2, &vid);
        if (ret < 0) {
                dev_err(&client->dev, "read chip ID fail\n");
                return 0;
        }

        if (vid != RICHTEK_1711_VID) {
                pr_info("%s failed, VID=0x%04x\n", __func__, vid);
                return 0;
        }

        ret = rt1711_read_device(client, TCPC_V10_REG_PID, 2, &pid);
        if (ret < 0) {
                dev_err(&client->dev, "read product ID fail\n");
                return 0;
        }

        if (pid != RICHTEK_1711_PID) {
                pr_info("%s failed, PID=0x%04x\n", __func__, pid);
                return 0;
        }

        return 1;
}

/* call from 'cat /sys/class/typec/typec_device/cc_status'
 * Besides BSP/ATD tools, no one will access it.
 * */
static int rt1711_cc_status(void)
{
        int ret, cc, cc1, cc2;

        ret = rt_info.tcpc->ops->get_cc(rt_info.tcpc, &cc1, &cc2);
        if (ret < 0)
                return ret;

        cc1 = !!cc1;
        cc2 = !!cc2;

        if (cc1 ^ cc2)
                cc = cc1 ? 1 : 2;
        else
                cc = 0;

        return cc;
}

static enum typec_vbus_state rt1711_get_vbus_state(void)
{
        bool vbus_state = rt_info.state.vbus != 0;
        int ret;
        uint16_t pwr_status;

        if (vbus_state) {
                return TYPEC_VBUS_SRC_5V;
        }

        ret = rt_info.tcpc->ops->get_power_status(rt_info.tcpc, &pwr_status);
        if (!ret && ((pwr_status & TCPC_REG_POWER_STATUS_VBUS_PRES) > 0))
                return TYPEC_VBUS_SINK_5V;
        return TYPEC_VBUS_NONE;
}

struct typec_device_ops rt1711_typec_ops = {
        .current_detect = rt1711_current_mode_detect,
        .attached_state_detect = rt1711_attatched_state_detect,
        .current_advertise_get = rt1711_current_advertise_get,
        .current_advertise_set = rt1711_current_advertise_set,
        .port_mode_get = rt1711_port_mode_get,
        .port_mode_set = rt1711_port_mode_set,
        .dump_regs = rt1711_dump_regs,
        .i2c_status = rt1711_i2c_status,
        .cc_status = rt1711_cc_status,
	.get_vbus_state = rt1711_get_vbus_state,
};
bool mt_usb_pd_support(void)
{
        int dp_id = gpio_get_value(922);//GPIO53
        if (dp_id==0){
                pr_err("%s : on(rt1716 cc)",__func__);
                return true;
        } else {
                pr_err("%s : off(anx cc)",__func__);
                return false;
        }
}

bool mt_is_power_sink(void)
{
	if (tcpc_power_role == PD_ROLE_SINK)
		return true;
	else
		return false;
}

/* unit: mA */
int mt_usb_pd_get_current(void)
{
	return tcpc_sink_current;
}

int mt_pep_custom_hv(bool enable)
{
	if (tcpc_dev)
		tcpm_typec_set_custom_hv(tcpc_dev, enable);
	return 0;
}

static int rt_chg_handle_source_vbus(struct tcp_notify *tcp_noti)
{
	bool enable = (tcp_noti->vbus_state.mv > 0) ? true : false;

	/* if vbus boost comes from charger ic */
	if(enable){
		pr_info("%s: vbus_otg enable\n", __func__);
		otg_last_time_status=true;
		tbl_charger_otg_vbus(1);
	}
	return 0;
}

static int rt_chg_handle_sink_vbus(struct tcp_notify *tcp_noti)
{
	if(otg_last_time_status){
	    pr_info("%s: vbus_otg disable\n", __func__);
	    otg_last_time_status=false;
            tbl_charger_otg_vbus(0);
	}

	tcpc_sink_voltage = tcp_noti->vbus_state.mv;
	tcpc_sink_current = tcp_noti->vbus_state.ma;
	return 0;
}

static int chg_tcp_notifer_call(struct notifier_block *nb, unsigned long event, void *data)
{
	struct tcp_notify *tcp_noti = data;

	pr_info("%s: %d\n", __func__, (int)event);

	switch (event) {
	case TCP_NOTIFY_PR_SWAP:
		break;
	case TCP_NOTIFY_DR_SWAP:
		pr_debug("dr swap: %d\n", tcp_noti->swap_state.new_role);
		break;
	case TCP_NOTIFY_SOURCE_VCONN:
		break;
	case TCP_NOTIFY_VCONN_SWAP:
		break;
	case TCP_NOTIFY_SOURCE_VBUS:
		pr_info("%s: RT1711 TCP_NOTIFY_SOURCE_VBUS  %d mV, %d mA\n",
		__func__, tcp_noti->vbus_state.mv, tcp_noti->vbus_state.ma);
		rt_chg_handle_source_vbus(tcp_noti);
		rt_info_set_field(vbus_state, tcp_noti->vbus_state);
		break;
	case TCP_NOTIFY_SINK_VBUS:
		pr_info("%s: RT1711 TCP_NOTIFY_SINK_VBUS  %d mV, %d mA, type=%d\n",
			 __func__, tcp_noti->vbus_state.mv, tcp_noti->vbus_state.ma,
			tcp_noti->vbus_state.type);
		rt_chg_handle_sink_vbus(tcp_noti);
		rt_info_set_field(vbus_state, tcp_noti->vbus_state);
		if (tcpc_power_role == PD_ROLE_SINK && tcpc_src_support_pd == true)
			bat_update_charger_type(TYPEC_PD_CHARGER);
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		tcpc_pre_typec_state = tcpc_typec_state;
		tcpc_typec_state = tcp_noti->typec_state.new_state;
		rt_info_set_field(typec_state, tcp_noti->typec_state);

		if (tcpc_typec_state == TYPEC_ATTACHED_SRC) {
			tcpc_power_role = PD_ROLE_SOURCE;
			ssusb_mode_switch_typec(1);
		} else
			tcpc_power_role = PD_ROLE_SINK;

		if (tcpc_typec_state == TYPEC_UNATTACHED) {
			tcpc_src_support_pd = false;
			if (tcpc_pre_typec_state == TYPEC_ATTACHED_SRC)
				ssusb_mode_switch_typec(0);
		}
		break;
	case TCP_NOTIFY_PD_STATE:
		tcpc_pd_state = tcp_noti->pd_state.connected;

		rt_info_set_field(typec_state, tcp_noti->typec_state);
		if (tcpc_pd_state == PD_CONNECT_TYPEC_ONLY_SNK) {
			if (tcpc_sink_voltage == 5000 && tcpc_sink_current == 1500)
				bat_update_charger_type(TYPEC_1_5A_CHARGER);
			else if (tcpc_sink_voltage == 5000 && tcpc_sink_current == 3000)
				bat_update_charger_type(TYPEC_3A_CHARGER);
		} else if (tcpc_pd_state == PD_CONNECT_PE_READY_SNK) {
			tcpc_src_support_pd = true;
			bat_update_charger_type(TYPEC_PD_CHARGER);
		} else if (tcpc_pd_state == PD_CONNECT_PE_READY_SRC)
			tcpc_src_support_pd = true;

		break;
	default:
		break;
	};
	return NOTIFY_OK;
}

int rt1711_typec_ops_init(struct rt1711_chip *rt_chip)
{
	int ret;
        struct rt_typec_pd_info *info = &rt_info;

        chip = rt_chip;
        /* Get tcpc device by tcpc_device'name */
        info->tcpc = tcpc_dev_get_by_name("type_c_port0");
        if (!info->tcpc) {
                pr_err("%s: get rt1711-tcpc fail\n", __func__);
                return -ENODEV;
        }

        info->ops = &rt1711_typec_ops;
        rt_info.dev = rt_chip->dev;
#if 0
#if defined(CONFIG_USB_DWC3_PD_DUAL_ROLE_SWAP)
        rt_info.vbus_reg = devm_regulator_get_optional(rt_info.dev, "vbus_otg");
        if (IS_ERR(rt_info.vbus_reg) &&
            PTR_ERR(rt_info.vbus_reg) == -EPROBE_DEFER) {
                /* regulators may not be ready, so retry again later */
                rt_info.vbus_reg = NULL;
        }
#endif
#if defined (PD_BUS_QOS)
        plat_bus_freq_init(info, info->dev);
#endif
        INIT_DELAYED_WORK(&info->cancel_qos_work, rt1711_cancel_qos_work_func);
#if defined (PD_PM_QOS)
        if (rt_chip->irq > 0) {
                rt_info.pm_qos_latency = 41;    /* hardcoded, same as the lowest latency of android_usb@66bf0c8(msm8996.dtsi) */
                rt_info.pm_qos_req_dma.type = PM_QOS_REQ_AFFINE_CORES;
                rt_info.pm_qos_req_dma.irq = rt_chip->irq;
                cpumask_copy(&rt_info.pm_qos_req_dma.cpus_affine, cpumask_of(0));
                pm_qos_add_request(&rt_info.pm_qos_req_dma,
                                   PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
        } else {
                rt_info.pm_qos_req_dma.irq = 0;
                pr_err("%s: QOS for IRQ(%d) is not available\n", __func__, rt_chip->irq);
        }
#endif
        init_completion(&accept_completion);
        rt_info.state.shutdown = 0;
        rt_info.state.hardreset = 0;
        rt_info.state.pr_swap = 0;
        /* register notifier for battery capacity and temperature*/
        rt_info.power_nb.notifier_call = battery_capacity_notify;
//        atomic_set(&rt_info.latest_batt_capacity, POWER_CAPACITY_INIT_VALUE);
//        atomic_set(&rt_info.latest_batt_temperature, POWER_TEMPERATURE_INIT_VALUE);
        power_register_notify(&rt_info.power_nb);

        mutex_init(&rt_info.state_lock);
        /* register tcpc notifier */
#endif
        info->nb.notifier_call = chg_tcp_notifer_call;
        ret = register_tcp_dev_notifier(info->tcpc, &info->nb);
        if (ret < 0) {
                mutex_destroy(&rt_info.state_lock);
                pr_err("%s: register tcpc notifer fail\n", __func__);
                return -EINVAL;
        }
        return 0;
}

void rt1711_typec_ops_remove(struct rt1711_chip *rt_chip)
{
        pr_info("%s: vbus_otg disable\n", __func__);
        tbl_charger_otg_vbus(0);
}
