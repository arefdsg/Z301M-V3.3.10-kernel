/*
 * Copyright (C) 2016 Richtek Technology Corp.
 *
 * TCPC Interface for dual role
 *
 * Author: TH <tsunghan_tsai@richtek.com>
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include "inc/tcpci.h"
#include "inc/tcpci_typec.h"

#ifdef CONFIG_DUAL_ROLE_USB_INTF
static struct completion inquiry_sink_cap_completion;

static enum dual_role_property tcpc_dual_role_props[] = {
	DUAL_ROLE_PROP_SUPPORTED_MODES,
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
	DUAL_ROLE_PROP_VCONN_SUPPLY,
};

static int tcpc_dual_role_get_prop(struct dual_role_phy_instance *dual_role,
			enum dual_role_property prop, unsigned int *val)
{
	struct tcpc_device *tcpc = dev_get_drvdata(dual_role->dev.parent);
	int ret = 0;

	switch (prop) {
	case DUAL_ROLE_PROP_SUPPORTED_MODES:
		*val = tcpc->dual_role_supported_modes;
		break;
	case DUAL_ROLE_PROP_MODE:
		*val = tcpc->dual_role_mode;
		break;
	case DUAL_ROLE_PROP_PR:
		*val = tcpc->dual_role_pr;
		break;
	case DUAL_ROLE_PROP_DR:
		*val = tcpc->dual_role_dr;
		break;
	case DUAL_ROLE_PROP_VCONN_SUPPLY:
		*val = tcpc->dual_role_vconn;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static	int tcpc_dual_role_prop_is_writeable(
	struct dual_role_phy_instance *dual_role, enum dual_role_property prop)
{
	int retval = -ENOSYS;
	struct tcpc_device *tcpc = dev_get_drvdata(dual_role->dev.parent);

	switch (prop) {
#ifdef CONFIG_USB_POWER_DELIVERY
	case DUAL_ROLE_PROP_PR:
	case DUAL_ROLE_PROP_DR:
	case DUAL_ROLE_PROP_VCONN_SUPPLY:
#else
	case DUAL_ROLE_PROP_MODE:
#endif	/* CONFIG_USB_POWER_DELIVERY */
		if (tcpc->dual_role_supported_modes ==
			DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP)
			retval = 1;
		break;
	default:
		break;
	}
	return retval;
}

#ifdef CONFIG_USB_POWER_DELIVERY

static inline bool fixedType_and_dual_role_power(pd_port_power_caps *caps)
{
	u32 pdo;

	if (caps->nr == 0) return false;

	pdo = caps->pdos[0];
	if (0 == pdo >> 30) {
		/* fixed type */
		return (pdo & (1 << 29)) > 0;
	}
	return false;
}

int get_sink_cap_dpm_event_cb(struct tcpc_device *tcpc, int ret,
			      struct tcp_dpm_event *event)
{
	if (event->event_id == TCP_DPM_EVT_GET_SINK_CAP)
		complete(&inquiry_sink_cap_completion);
	return 0;
}

static int tcpc_dual_role_set_prop_pr(
	struct tcpc_device *tcpc, unsigned int val)
{
	int ret;
	uint8_t role;
	unsigned long timeout;
	bool is_remote_pr_swapable = true;

	switch (val) {
	case DUAL_ROLE_PROP_PR_SRC:
		role = PD_ROLE_SOURCE;
		break;
	case DUAL_ROLE_PROP_PR_SNK:
		role = PD_ROLE_SINK;
		break;
	default:
		return 0;
	}

	if (val == tcpc->dual_role_pr) {
		pr_info("%s wrong role (%d->%d)\n",
			__func__, tcpc->dual_role_pr, val);
		return 0;
	}

	if (tcpc->dual_role_pr == DUAL_ROLE_PROP_PR_SRC) {
		/* current power role is source role. */
		if (tcpc->pd_port.remote_snk_cap.nr == 0) {
			/* sink cap is not available, try to inquiry sink cap first */
			reinit_completion(&inquiry_sink_cap_completion);
			tcpm_dpm_pd_get_sink_cap(tcpc, get_sink_cap_dpm_event_cb);
			if (wait_for_completion_timeout(&inquiry_sink_cap_completion,
							msecs_to_jiffies(150))) {
				/* wait until remote_snk_cap.nr is up to date */
				timeout = jiffies + HZ;
				do {
					if (tcpc->pd_port.remote_snk_cap.nr > 0)
						break;
					msleep(10);
				} while(time_before(jiffies, timeout));
			} else {
				pr_err("%s: timeout in waiting sink caps\n", __func__);
			}
		}
		if (tcpc->pd_port.remote_snk_cap.nr > 0) {
			is_remote_pr_swapable = fixedType_and_dual_role_power(
				&tcpc->pd_port.remote_snk_cap);
		} else {
			pr_err("%s: sink cap is not available\n", __func__);
		}
	} else if (tcpc->dual_role_pr == DUAL_ROLE_PROP_PR_SNK) {
		is_remote_pr_swapable = fixedType_and_dual_role_power(
		&tcpc->pd_port.remote_src_cap);
	}

	if (is_remote_pr_swapable) {
		ret = tcpm_dpm_pd_power_swap(tcpc, role, NULL);
		pr_info("%s power role swap (%d->%d): %d\n",
			__func__, tcpc->dual_role_pr, val, ret);
	} else {
		pr_info("%s power role swap (%d->%d): remote, PR is not available\n",
			__func__, tcpc->dual_role_pr, val);
		ret = TCPM_ERROR_NO_PD_CONNECTED;
	}

	if (ret == TCPM_ERROR_NO_PD_CONNECTED) {
		ret = tcpm_typec_role_swap(tcpc);
		pr_info("%s typec role swap (%d->%d): %d\n",
			__func__, tcpc->dual_role_pr, val, ret);
	}

	return ret;
}

static int tcpc_dual_role_set_prop_dr(
	struct tcpc_device *tcpc, unsigned int val)
{
	int ret;
	uint8_t role;

	switch (val) {
	case DUAL_ROLE_PROP_DR_HOST:
		role = PD_ROLE_DFP;
		break;
	case DUAL_ROLE_PROP_DR_DEVICE:
		role = PD_ROLE_UFP;
		break;
	default:
		return 0;
	}

	if (val == tcpc->dual_role_dr) {
		pr_info("%s wrong role (%d->%d)\n",
			__func__, tcpc->dual_role_dr, val);
		return 0;
	}

	ret = tcpm_dpm_pd_data_swap(tcpc, role, NULL);
	pr_info("%s data role swap (%d->%d): %d\n",
		__func__, tcpc->dual_role_dr, val, ret);

	return ret;
}

static int tcpc_dual_role_set_prop_vconn(
	struct tcpc_device *tcpc, unsigned int val)
{
	int ret;
	uint8_t role;

	switch (val) {
	case DUAL_ROLE_PROP_VCONN_SUPPLY_NO:
		role = PD_ROLE_VCONN_OFF;
		break;
	case DUAL_ROLE_PROP_VCONN_SUPPLY_YES:
		role = PD_ROLE_VCONN_ON;
		break;
	default:
		return 0;
	}

	if (val == tcpc->dual_role_vconn) {
		pr_info("%s wrong role (%d->%d)\n",
			__func__, tcpc->dual_role_vconn, val);
		return 0;
	}

	ret = tcpm_dpm_pd_vconn_swap(tcpc, role, NULL);
	pr_info("%s vconn swap (%d->%d): %d\n",
		__func__, tcpc->dual_role_vconn, val, ret);

	return ret;
}

#else	/* TypeC Only */

static int tcpc_dual_role_set_prop_mode(
	struct tcpc_device *tcpc, unsigned int val)
{
	int ret;
	uint8_t role;

	switch (val) {
	case DUAL_ROLE_PROP_MODE_DFP:
		role = PD_ROLE_SOURCE;
		break;
	case DUAL_ROLE_PROP_MODE_UFP:
		role = PD_ROLE_SINK;
		break;
	default:
		return 0;
	}

	if (val == tcpc->dual_role_mode) {
		pr_info("%s wrong role (%d->%d)\n",
			__func__, tcpc->dual_role_mode, val);
		return 0;
	}

	ret = tcpm_typec_role_swap(tcpc);
	pr_info("%s typec role swap (%d->%d): %d\n",
		__func__, tcpc->dual_role_mode, val, ret);

	return ret;
}

#endif	/* CONFIG_USB_POWER_DELIVERY */

static int tcpc_dual_role_set_prop(struct dual_role_phy_instance *dual_role,
			enum dual_role_property prop, const unsigned int *val)
{
	struct tcpc_device *tcpc = dev_get_drvdata(dual_role->dev.parent);

	switch (prop) {
#ifdef CONFIG_USB_POWER_DELIVERY
	case DUAL_ROLE_PROP_PR:
		tcpc_dual_role_set_prop_pr(tcpc, *val);
		break;
	case DUAL_ROLE_PROP_DR:
		tcpc_dual_role_set_prop_dr(tcpc, *val);
		break;
	case DUAL_ROLE_PROP_VCONN_SUPPLY:
		tcpc_dual_role_set_prop_vconn(tcpc, *val);
		break;
#else /* TypeC Only */
	case DUAL_ROLE_PROP_MODE:
		tcpc_dual_role_set_prop_mode(tcpc, *val);
		break;
#endif /* CONFIG_USB_POWER_DELIVERY */

	default:
		break;
	}

	return 0;
}

static void tcpc_get_dual_desc(struct tcpc_device *tcpc)
{
	struct device_node *np = of_find_node_by_name(NULL, tcpc->desc.name);
	u32 val;

	if (!np)
		return;

	if (of_property_read_u32(np, "rt-dual,supported_modes", &val) >= 0) {
		if (val > DUAL_ROLE_PROP_SUPPORTED_MODES_TOTAL)
			tcpc->dual_role_supported_modes =
					DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
		else
			tcpc->dual_role_supported_modes = val;
	}
}

int tcpc_dual_role_phy_init(
			struct tcpc_device *tcpc)
{
	struct dual_role_phy_desc *dual_desc;
	int len;
	char *str_name;

	tcpc->dr_usb = devm_kzalloc(&tcpc->dev,
				sizeof(tcpc->dr_usb), GFP_KERNEL);

	dual_desc = devm_kzalloc(&tcpc->dev, sizeof(*dual_desc), GFP_KERNEL);
	if (!dual_desc)
		return -ENOMEM;

	tcpc_get_dual_desc(tcpc);

	len = strlen(tcpc->desc.name);
	str_name = devm_kzalloc(&tcpc->dev, len+11, GFP_KERNEL);
	sprintf(str_name, "%s", tcpc->desc.name);
	dual_desc->name = str_name;
	init_completion(&inquiry_sink_cap_completion);

	dual_desc->properties = tcpc_dual_role_props;
	dual_desc->num_properties = ARRAY_SIZE(tcpc_dual_role_props);
	dual_desc->get_property = tcpc_dual_role_get_prop;
	dual_desc->set_property = tcpc_dual_role_set_prop;
	dual_desc->property_is_writeable = tcpc_dual_role_prop_is_writeable;

	tcpc->dr_usb = devm_dual_role_instance_register(&tcpc->dev, dual_desc);
	if (IS_ERR(tcpc->dr_usb)) {
		dev_err(&tcpc->dev, "tcpc fail to register dual role usb\n");
		return -EINVAL;
	}
	/* init dual role phy instance property */
	tcpc->dual_role_pr = DUAL_ROLE_PROP_PR_NONE;
	tcpc->dual_role_dr = DUAL_ROLE_PROP_DR_NONE;
	tcpc->dual_role_mode = DUAL_ROLE_PROP_MODE_NONE;
	tcpc->dual_role_vconn = DUAL_ROLE_PROP_VCONN_SUPPLY_NO;
	return 0;
}
EXPORT_SYMBOL(tcpc_dual_role_phy_init);
#endif /* CONFIG_DUAL_ROLE_USB_INTF */
