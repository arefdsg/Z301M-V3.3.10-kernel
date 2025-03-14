/*
 * typec-core.c - Type-C connector Framework
 *
 * Copyright (C) 2015 HUAWEI, Inc.
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 * Author: HUAWEI, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <inc/typec.h>
#include <linux/power_supply.h>

int g_typeC_update_flag = 0;

static struct class *typec_class;
static struct device *typec_dev;
static struct power_supply *usb_psy;
static struct power_supply typec_psy;

/* to get the Type-C Current mode */
static ssize_t current_detect_show(struct device *pdev,
                                   struct device_attribute *attr, char *buf)
{
        struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
        enum typec_current_mode current_mode = typec_ops->current_detect();
        return snprintf(buf, PAGE_SIZE, "%d\n", current_mode);
}

/* to get the attached state and determine what was attached */
static ssize_t attached_state_show(struct device *pdev,
                                   struct device_attribute *attr, char *buf)
{
        struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
        enum typec_attached_state attached_state =
            typec_ops->attached_state_detect();
        return snprintf(buf, PAGE_SIZE, "%d\n", attached_state);
}

/* to get the current advertisement in DFP or DRP modes */
static ssize_t current_advertise_show(struct device *pdev,
                                      struct device_attribute *attr, char *buf)
{
        struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
        enum typec_current_mode current_mode =
            typec_ops->current_advertise_get();
        return snprintf(buf, PAGE_SIZE, "%d\n", current_mode);
        return 0;
}

/* to set the current advertisement in DFP or DRP modes */
static ssize_t current_advertise_store(struct device *pdev,
                                       struct device_attribute *attr,
                                       const char *buff, size_t size)
{
        struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
        int current_mode;

        if (sscanf(buff, "%d", &current_mode) != 1)
                return -EINVAL;

        if (current_mode >= TYPEC_CURRENT_MODE_UNSPPORTED)
                return -EINVAL;

        if (typec_ops->current_advertise_set((enum typec_current_mode)
                                             current_mode))
                return -1;

        return size;
}

/* to get the port mode (UFP, DFP or DRP) */
static ssize_t port_mode_ctrl_show(struct device *pdev,
                                   struct device_attribute *attr, char *buf)
{
        struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
        enum typec_port_mode port_mode = typec_ops->port_mode_get();
        return snprintf(buf, PAGE_SIZE, "%d\n", port_mode);
        return 0;
}

/* to set the port mode (UFP, DFP or DRP), the chip will operate according the mode */
static ssize_t port_mode_ctrl_store(struct device *pdev,
                                    struct device_attribute *attr,
                                    const char *buff, size_t size)
{
        struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
        int port_mode;

        if (sscanf(buff, "%d", &port_mode) != 1)
                return -EINVAL;

        if (port_mode > TYPEC_DRP_MODE)
                return -EINVAL;

        if (typec_ops->port_mode_set((enum typec_port_mode)port_mode))
                return -1;

        return size;
}

/* to get all the register value */
static ssize_t dump_regs_show(struct device *pdev,
                              struct device_attribute *attr, char *buf)
{
        struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
        return typec_ops->dump_regs(buf);
}

static ssize_t show_i2c_status(struct device *pdev,
                struct device_attribute *attr, char *buf)
{
        struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
        return sprintf(buf, "%d\n",
                       (typec_ops->i2c_status && typec_ops->i2c_status()));
}

static ssize_t show_cc_status(struct device *pdev,
                                struct device_attribute *attr, char *buf)
{
        struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
        int cc = -1;
	
	pr_info("%s show cc status\n", __func__);
       	
        if (typec_ops->cc_status){
		pr_info("%s get cc status\n", __func__);	
                cc = typec_ops->cc_status();
		pr_info("%s finish cc status cc=%d\n", __func__, cc);
        }else
                pr_err("%s callback not found\n", __func__);
        return sprintf(buf, "%d\n", cc);
}

static DEVICE_ATTR(current_detect, S_IRUGO, current_detect_show, NULL);
static DEVICE_ATTR(attached_state, S_IRUGO, attached_state_show, NULL);
static DEVICE_ATTR(current_advertise, S_IRUGO | S_IWUSR, current_advertise_show,
                   current_advertise_store);
static DEVICE_ATTR(port_mode_ctrl, S_IRUGO | S_IWUSR, port_mode_ctrl_show,
                   port_mode_ctrl_store);
static DEVICE_ATTR(dump_regs, S_IRUGO, dump_regs_show, NULL);
static DEVICE_ATTR(i2c_status, S_IRUGO, show_i2c_status, NULL);
static DEVICE_ATTR(cc_status, S_IRUGO, show_cc_status, NULL);

static struct device_attribute *typec_attributes[] = {
        &dev_attr_current_detect,
        &dev_attr_attached_state,
        &dev_attr_current_advertise,
        &dev_attr_port_mode_ctrl,
        &dev_attr_dump_regs,
        &dev_attr_i2c_status,
        &dev_attr_cc_status,
        NULL
};

static enum power_supply_property typec_properties[] = {
        POWER_SUPPLY_PROP_ONLINE, /* attached as UFP: 1, detatched: 0 */
        POWER_SUPPLY_PROP_VOLTAGE_MAX,
        POWER_SUPPLY_PROP_VOLTAGE_MIN,
        POWER_SUPPLY_PROP_CURRENT_MAX,
        POWER_SUPPLY_PROP_TYPEC_MODE, /* DFP, UFP or none */
};

static int typec_get_property(struct power_supply *psy,
                              enum power_supply_property prop,
                              union power_supply_propval *val)
{
        struct typec_device_ops *typec_ops;
        enum typec_attached_state attached_state;
        enum typec_current_mode current_mode;
        enum typec_vbus_state vbus_state;

        if (!typec_dev) {
                pr_err("%s: no typec device registered\n", __func__);
                return -EINVAL;
        }
        typec_ops = dev_get_drvdata(typec_dev);
        if (typec_ops == NULL) {
                pr_err("%s: no typec operations registered\n", __func__);
                return -EINVAL;
        }
        /* TYPEC_NOT_ATTACHED
         * TYPEC_ATTACHED_AS_UFP
         * TYPEC_ATTACHED_AS_DFP
         * TYPEC_ATTACHED_TO_ACCESSORY
         */
        attached_state = typec_ops->attached_state_detect ?
                typec_ops->attached_state_detect() : TYPEC_NOT_ATTACHED;

        /* TYPEC_CURRENT_MODE_DEFAULT
         * TYPEC_CURRENT_MODE_MID
         * TYPEC_CURRENT_MODE_HIGH
         * TYPEC_CURRENT_MODE_UNSPPORTED
         */
        current_mode = typec_ops->current_detect ?
                typec_ops->current_detect() : TYPEC_CURRENT_MODE_UNSPPORTED;

        /* TYPEC_VBUS_NONE
           TYPEC_VBUS_SRC_0V
           TYPEC_VBUS_SRC_5V
           TYPEC_VBUS_SINK_0V
           TYPEC_VBUS_SINK_5V
        */
        vbus_state = typec_ops->get_vbus_state ?
                typec_ops->get_vbus_state() : TYPEC_VBUS_NONE;

        switch (prop) {
        case POWER_SUPPLY_PROP_TYPEC_MODE:
                if (attached_state == TYPEC_ATTACHED_AS_UFP)
                        val->intval = POWER_SUPPLY_TYPEC_MODE_UFP;
                else if (attached_state == TYPEC_ATTACHED_AS_DFP)
                        val->intval = POWER_SUPPLY_TYPEC_MODE_DFP;
                else
                        val->intval =  POWER_SUPPLY_TYPEC_MODE_UNKNOWN;
                break;
        case POWER_SUPPLY_PROP_VOLTAGE_MAX:
        case POWER_SUPPLY_PROP_VOLTAGE_MIN:
                if (attached_state != TYPEC_ATTACHED_AS_UFP) {
                        val->intval = 0;
                        break;
                }
                val->intval = 5000*1000; /* 5V */
                break;
        case POWER_SUPPLY_PROP_CURRENT_MAX:
                if (attached_state != TYPEC_ATTACHED_AS_UFP) {
                        val->intval = 0;
                        break;
                }
                if (current_mode == TYPEC_CURRENT_MODE_DEFAULT)
                        val->intval = 500*1000; /* 500mA */
                else if (current_mode == TYPEC_CURRENT_MODE_MID)
                        val->intval = 1500*1000; /* 1500mA */
                else if (current_mode == TYPEC_CURRENT_MODE_HIGH)
                        val->intval = 3000*1000; /* 3000mA */
                else
                        val->intval = 0;
                break;
        case POWER_SUPPLY_PROP_ONLINE:
                if (attached_state == TYPEC_ATTACHED_AS_UFP &&
                        vbus_state == TYPEC_VBUS_SINK_5V) {
                        val->intval = 1;
                } else
                        val->intval = 0;
                break;
        default:
                return -EINVAL;
        }
                g_typeC_update_flag = 1;
        return 0;
}

int add_typec_device(struct device *parent, struct typec_device_ops *typec_ops)
{
        struct device *dev;
        struct device_attribute **attrs = typec_attributes;
        struct device_attribute *attr;
        int err;

        if (!typec_ops || !typec_ops->current_detect
            || !typec_ops->attached_state_detect
            || !typec_ops->current_advertise_get
            || !typec_ops->current_advertise_set || !typec_ops->port_mode_get
            || !typec_ops->port_mode_set || !typec_ops->dump_regs) {
                pr_err("%s: ops is NULL\n", __func__);
                return -1;
        }

        dev = device_create(typec_class, NULL, MKDEV(0, 0), typec_ops,
                            "typec_device");
        if (IS_ERR(dev)) {
                pr_err("%s: device_create fail\n", __func__);
                return -1;
        }

        while ((attr = *attrs++)) {
                err = device_create_file(dev, attr);
                if (err) {
                        pr_err("%s: device_create_file fail\n", __func__);
                        device_destroy(typec_class, dev->devt);
                        return -1;
                }
        }

        typec_dev = dev;

        usb_psy = power_supply_get_by_name("usb");
        if (!usb_psy) {
                pr_err("%s USB supply not found\n", __func__);
        }

        /* Always has TypeC power supply */
        typec_psy.name            = "typec";
        typec_psy.type            = POWER_SUPPLY_TYPE_USB_TYPE_C;
        typec_psy.get_property    = typec_get_property;
        typec_psy.properties      = typec_properties;
        typec_psy.num_properties  = ARRAY_SIZE(typec_properties);
//        typec_psy.supplied_to     = pd_pm_power_supplied_to;
//        typec_psy.num_supplicants = ARRAY_SIZE(pd_pm_power_supplied_to);

        err = power_supply_register(dev, &typec_psy);
        if (err < 0) {
                pr_err("Unable to register typec_psy err=%d\n", err);
                return err;
        }

	return 0;
}

static int __init typec_init(void)
{
        typec_class = class_create(THIS_MODULE, "typec");
        if (IS_ERR(typec_class)) {
                pr_err("failed to create typec class --> %ld\n",
                       PTR_ERR(typec_class));
                return PTR_ERR(typec_class);
        }
//        init_completion(&pd_status.charger_prerequisite_completion);
        return 0;
}

subsys_initcall(typec_init);

static void __exit typec_exit(void)
{
        class_destroy(typec_class);
}

module_exit(typec_exit);

MODULE_AUTHOR("ASUS");
MODULE_DESCRIPTION("Type-C connector Framework");
MODULE_LICENSE("GPL v2");
