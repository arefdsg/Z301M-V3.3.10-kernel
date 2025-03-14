/*
 * tps51632-regulator.c -- TI ads1013
 *
 * Regulator driver for ads1013 3-2-1 Phase D-Cap Step Down Driverless
 * Controller with serial VID control and DVFS.
 *
 * Copyright (c) 2012, NVIDIA Corporation.
 *
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <linux/regulator/driver.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <asm-generic/unaligned.h>


/**********************************************************
 *
 *      [I2C Slave Setting]
 *
 *********************************************************/
#define ads1013_SLAVE_ADDR_READ        0x73
static struct i2c_client *new_client;

/**********************************************************
 *
 *[Global Variable]
 *
 *********************************************************/
extern int Read_HW_ID(void);
extern int Read_PROJ_ID(void);
int HW_ID;
static unsigned int ADC_SW_EN;
static unsigned int ADCPWREN_PMI_GP1;
int ADCPWR_enable(bool bEnable);
static DEFINE_MUTEX(ads1013_i2c_access);
struct qpnp_vadc_map_pt {
	int32_t x;
	int32_t y;
};
static const struct qpnp_vadc_map_pt ads1013_100k_adc[] = {
	{	0,	0x50B	},
	{	1,	0x4FB	},
	{	2,	0x4EB	},
	{	3,	0x4DA	},
	{	4,	0x4C9	},
	{	5,	0x4B8	},
	{	6,	0x4A7	},
	{	7,	0x495	},
	{	8,	0x483	},
	{	9,	0x471	},
	{	10,	0x45D	},
	{	11,	0x44C	},
	{	12,	0x439	},
	{	13,	0x424	},
	{	14,	0x411	},
	{	15,	0x3FE	},
	{	16,	0x3EA	},
	{	17,	0x3D5	},
	{	18,	0x3C2	},
	{	19,	0x3AD	},
	{	20,	0x39B	},
	{	21,	0x387	},
	{	22,	0x372	},
	{	23,	0x360	},
	{	24,	0x349	},
	{	25,	0x338	},
	{	26,	0x323	},
	{	27,	0x30D	},
	{	28,	0x2FA	},
	{	29,	0x2E7	},
	{	30,	0x2D8	},
	{	31,	0x2C3	},
	{	32,	0x2B2	},
	{	33,	0x29B	},
	{	34,	0x28A	},
	{	35,	0x27D	},
	{	36,	0x26A	},
	{	37,	0x257	},
	{	38,	0x249	},
	{	39,	0x234	},
	{	40,	0x226	},
	{	41,	0x217	},
	{	42,	0x207	},
	{	43,	0x1F8	},
	{	44,	0x1E8	},
	{	45,	0x1D7	},
	{	46,	0x1CE	},
	{	47,	0x1BD	},
	{	48,	0x1B4	},
	{	49,	0x1A2	},
	{	50,	0x199	},
	{	51,	0x186	},
	{	52,	0x17C	},
	{	53,	0x176	},
	{	54,	0x16A	},
	{	55,	0x160	},
	{	56,	0x155	},
	{	57,	0x14B	},
	{	58,	0x140	},
	{	59,	0x137	},
	{	60,	0x12D	},
	{	61,	0x11E	},
	{	62,	0x113	},
	{	63,	0x111	},
	{	64,	0x109	},
	{	65,	0x101	},
	{	66,	0xF9	},
	{	67,	0xF2	},
	{	68,	0xE9	},
	{	69,	0xE3	},
	{	70,	0xDC	},
	{	71,	0xD5	},
	{	72,	0xCF	},
	{	73,	0xC8	},
	{	74,	0xC2	},
	{	75,	0xBC	},
	{	76,	0xB6	},
	{	77,	0xB1	},
	{	78,	0xAB	},
	{	79,	0xA6	},
};

/**********************************************************
 *
 *      [I2C Function For Read/Write ads1013]
 *
 *********************************************************/
int ads1013_read_byte(unsigned char cmd, unsigned short int *returnData)
{
        int      ret = 0;
        struct i2c_msg msg[2];
        struct i2c_adapter *adap = new_client->adapter;
	unsigned char data[2];

//	printk(KERN_ERR "[%s]\n", __func__);
        mutex_lock(&ads1013_i2c_access);
        msg[0].addr = new_client->addr;
        msg[0].flags = 0;
        msg[0].len = 1;
        msg[0].buf = &cmd;

        msg[1].addr = new_client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = 2;
        msg[1].buf = data;

        ret = i2c_transfer(adap, msg, 2);
        if (ret < 0) {
		printk(KERN_ERR "[%s]i2c_transfer ret < 0\n", __func__);
                mutex_unlock(&ads1013_i2c_access);
                return 0;
        }
//    	printk(KERN_ERR "[%s] data0 = %x\n", __func__, data[0]);
//    	printk(KERN_ERR "[%s] data1 = %x\n", __func__, data[1]);

        *returnData = data[0]<<8 | data[1];
        //*returnData = data[0];

        mutex_unlock(&ads1013_i2c_access);
//	printk(KERN_ERR "[%s] 16 bit = %x\n", __func__, *returnData);
        return 1;
}
EXPORT_SYMBOL(ads1013_read_byte);
int ads1013_write_byte(int addr, int value)
{
    struct i2c_adapter *adap = new_client->adapter;
    struct i2c_msg msg;
    unsigned char data[3];
    int err;

    if (!new_client || !new_client->adapter)
        return -ENODEV;

    memset(data, 0, sizeof(data));
    mutex_lock(&ads1013_i2c_access);
    data[0] = addr;
    data[1] = (value & 0xFF00) >> 8;
    data[2] = value & 0x00FF;

    msg.addr = new_client->addr;
    msg.flags = 0;
    msg.len = 3;
    msg.buf = data;

    err = i2c_transfer(adap, &msg, 1);
    mutex_unlock(&ads1013_i2c_access);

//    printk(KERN_ERR "[%s] data0 = %x\n", __func__, data[0]);
//    printk(KERN_ERR "[%s] data1 = %x\n", __func__, data[1]);
//    printk(KERN_ERR "[%s] data2 = %x\n", __func__, data[2]);
    if (err < 0) {
         return -EIO;
    }

    return 0;
}
EXPORT_SYMBOL(ads1013_write_byte);
/**********************************************************
 *
 *	[platform_driver API]
 *
 *********************************************************/
static ssize_t show_ads1013_dump (struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	char *p = buf;
	unsigned char RegNum;
	unsigned short int ads1013_reg = 0;
	
	p += sprintf(p,"[ads1013_dump]\n");
	RegNum = 0x00;
	ret = ads1013_read_byte(RegNum, &ads1013_reg);
    p += sprintf(p, "Reg[%x]=0x%x\n",RegNum , ads1013_reg);

	RegNum = 0x01;
	ret = ads1013_read_byte(RegNum, &ads1013_reg);
    p += sprintf(p, "Reg[%x]=0x%x\n",RegNum , ads1013_reg);

	RegNum = 0x02;
	ret = ads1013_read_byte(RegNum, &ads1013_reg);
    p += sprintf(p, "Reg[%x]=0x%x\n",RegNum , ads1013_reg);

	RegNum = 0x03;
	ret = ads1013_read_byte(RegNum, &ads1013_reg);
    p += sprintf(p, "Reg[%x]=0x%x\n",RegNum , ads1013_reg);

	ret = p - buf;		
	return ret;
}
static ssize_t store_ads1013_access(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk(KERN_ERR "[store_ads1013_access]\n ");
	return size;
}
static DEVICE_ATTR(ads1013_dump, 0664, show_ads1013_dump, store_ads1013_access);

static ssize_t show_gpiot (struct device *dev, struct device_attribute *attr, char *buf)
{
        char *p = buf;
        int ret = 0;

        p += sprintf(p,"ADC_SW_EN= %d pull=%d\n", ADC_SW_EN, gpio_get_value(ADC_SW_EN));
        p += sprintf(p,"ADCPWREN_PMI_GP1= %d pull=%d\n", ADCPWREN_PMI_GP1, gpio_get_value(ADCPWREN_PMI_GP1));

        ret = p- buf;
        return ret;
}


static ssize_t store_gpiot(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        int ret = 0;
        char *pvalue = NULL, *gpio;
        unsigned int value = 0;

        pvalue = (char *)buf;

	gpio = strsep(&pvalue, " ");
	if (gpio == NULL)
		return size;
        ret = kstrtou32(gpio, 16, (unsigned int *)&value);
        gpio_direction_output(ADCPWREN_PMI_GP1, value);
        printk(KERN_ERR "value=%d\n", value);
        printk(KERN_ERR "ADCPWREN_PMI_GP1 = %d pull=%d\n", ADCPWREN_PMI_GP1, gpio_get_value(ADCPWREN_PMI_GP1));

	gpio = strsep(&pvalue, " ");
	if (gpio == NULL)
		return size;
        ret = kstrtou32(gpio, 16, (unsigned int *)&value);
        gpio_direction_output(ADC_SW_EN, value);
        printk(KERN_ERR "value=%d\n", value);
        printk(KERN_ERR "ADC_SW_EN = %d pull=%d\n",ADC_SW_EN,gpio_get_value(ADC_SW_EN));

        return size;

}
static DEVICE_ATTR(gpiot, 0664, show_gpiot, store_gpiot); 

int thermal_ads1013_adc_temp(int mode)
{
	int ret = 0, table_size=0, i=0;
	unsigned short int ads1013_reg = 0;
	unsigned short int ads1013_regtest = 0;
	static int temp_therm=0;
//	unsigned short int mask = 0x100;	//bit(8)

	if(mode == 1){
		ret = ads1013_write_byte( 0x1, 0xD283);		//Reg01 [14:9]=101001, Reg01 [8]= 0
	} else if (mode ==2){
		//ret = ads1013_write_byte(0xC283);		//Reg01 [14:9]=100001, Reg01 [8]= 0
		ads1013_write_byte(0x1,0x8283);
//ads1013_write_byte(0x0,0);
//		ret = ads1013_read_byte( 0x01, &ads1013_reg);	//Reg00[15:4]
//		ads1013_reg &= ~mask;		//bit(8) = 0
		 
//		ret = ads1013_write_byte( 0x1, );		//default
	}

	msleep(500);

	ret = ads1013_read_byte( 0x00, &ads1013_reg);	//Reg00[15:4]
	msleep(300);
	ret = ads1013_read_byte( 0x01, &ads1013_regtest);	//Reg00[15:4]
	ads1013_reg = ads1013_reg >>4;
	printk(KERN_ERR "[%s] Reg00 = 0x%x\n", __func__, ads1013_reg);
	printk(KERN_ERR "[%s] Reg01 = 0x%x\n", __func__, ads1013_regtest);

	if (mode == 2){
		return ads1013_reg;
	} else {
		if (ads1013_reg >= 0 && ads1013_reg < 2048){
			table_size = ARRAY_SIZE(ads1013_100k_adc);
			i = 0;
			while ( i < table_size ) {
				if (ads1013_100k_adc[i].y < ads1013_reg){
					break;
				} else {
					i++;
				}
	
				if (i == 0) {
					temp_therm = ads1013_100k_adc[0].x;
				} else if (i == table_size) {
					temp_therm = ads1013_100k_adc[table_size-1].x;
				} else {
					temp_therm = (((int32_t) ((ads1013_100k_adc[i].x - ads1013_100k_adc[i-1].x)*(ads1013_reg - ads1013_100k_adc[i-1].y))/(ads1013_100k_adc[i].y - ads1013_100k_adc[i-1].y))+ads1013_100k_adc[i-1].x);
				}
			}
		} else {
			printk(KERN_ERR "Read i2c Fail!!! Keep Temp = %d\n", temp_therm);
		}
		printk(KERN_ERR "ADC to Temp = %d, index = %d\n", temp_therm, i);
		return temp_therm;
	}

	return 0;
}
EXPORT_SYMBOL(thermal_ads1013_adc_temp);

extern int voltage2temp(void);
static ssize_t show_ads1013_thermal_check_proc(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = voltage2temp();
	//ret = thermal_ads1013_adc_temp(1);

	return sprintf(buf, "%d\n", ret);
}

static ssize_t store_ads1013_thermal_check_proc(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}
static DEVICE_ATTR(thermal_check, 0664, show_ads1013_thermal_check_proc, store_ads1013_thermal_check_proc);

static ssize_t show_ads1013_adapter_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ADCPWR_enable(1);
	ret = thermal_ads1013_adc_temp(2);
	ADCPWR_enable(0);
	if (ret>1550)
		return sprintf(buf, "%s\n", "PASS");
	else
		return sprintf(buf, "%s\n", "FALSE");
}
static ssize_t store_ads1013_adapter_id(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}
static DEVICE_ATTR(adapter_id, 0664, show_ads1013_adapter_id, store_ads1013_adapter_id);

static ssize_t show_ads1013_adc_id_hi(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ADCPWR_enable(1);
	ret = thermal_ads1013_adc_temp(2);
	return sprintf(buf, "%d\n", ret);
}
static ssize_t store_ads1013_adc_id_hi(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}
static DEVICE_ATTR(adc_id_hi, 0664, show_ads1013_adc_id_hi, store_ads1013_adc_id_hi);

static ssize_t show_ads1013_adc_id_lo(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ADCPWR_enable(0);
	ret = thermal_ads1013_adc_temp(2);

	return sprintf(buf, "%d\n", ret);
}
static ssize_t store_ads1013_adc_id_lo(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}
static DEVICE_ATTR(adc_id_lo, 0664, show_ads1013_adc_id_lo, store_ads1013_adc_id_lo);

static ssize_t show_ads1013_rw(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *p = buf;
	unsigned short int ads1013_reg = 0;
	int ret = 0;

	ads1013_read_byte(0x0, &ads1013_reg);
	p += sprintf(p,"REG[00] = 0x%x\n", ads1013_reg);
	msleep(1000);
	ads1013_read_byte(0x1, &ads1013_reg);
	p += sprintf(p,"REG[01] = 0x%x\n", ads1013_reg);
	ret = p-buf;

	return ret;
}
static ssize_t store_ads1013_rw(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL;
	unsigned int value = 0;

	printk(KERN_ERR "%s\n", __func__);

	pvalue = (char *)buf;

	ret = kstrtou32(pvalue, 16, (unsigned int *)&value);
	printk(KERN_ERR "set REG[01] = 0x%x \n", value);

	ads1013_write_byte(0x1, value);

	return size;
}
static DEVICE_ATTR(ads1013_rw, 0664, show_ads1013_rw, store_ads1013_rw);

 /**********************************************************
  *
  *   [External Function]
  *
  *********************************************************/
int ads1013_set_gpio(int gpio, int pull)
{
	int err =0;
	printk(KERN_ERR "gpio = %d, pull %s\n",gpio, pull==1?"hi":"lo");
	err = gpio_direction_output(gpio, pull);
	if (err<0)
		printk(KERN_ERR "pull %s fail, err = %d\n",pull==1?"up":"down", err);
	else
		printk(KERN_ERR "pull %s success\n", pull==1?"up":"down");

	return err;
}
EXPORT_SYMBOL(ads1013_set_gpio);
// Reg = 0x3, 0x4, 0x5, 0xB2
unsigned char ads1013_reg_val(int Reg){
	int err = 0;
	unsigned short int val = 0;
	err = ads1013_read_byte(Reg, &val);
	val = val>>4;
	printk(KERN_ERR "ads1013 Reg[%x]=0x%x\n",Reg ,val);
	return val;
}
EXPORT_SYMBOL(ads1013_reg_val);

int ads1013_enable(bool bEnable)
{
	int rc = 0;

	pr_info("%s: bEnable=%d\n", __func__, bEnable);
	if(bEnable)
	{
		// Enable ads1013
		gpio_direction_output(ADCPWREN_PMI_GP1, 1);
		gpio_direction_output(ADC_SW_EN, 1);
	}
	else
	{
		// Disable ads1013
		gpio_direction_output(ADCPWREN_PMI_GP1, 0);
		gpio_direction_output(ADC_SW_EN, 0);
	}
	
	return rc;
}
EXPORT_SYMBOL(ads1013_enable);

int ADCPWR_enable(bool bEnable)
{
	int rc = 0;
	
	pr_info("%s: bEnable=%d\n", __func__, bEnable);
	if(bEnable)
	{
		// Enable ads1013 ADC , default actie high(Without DCP IN)
		gpio_direction_output(ADCPWREN_PMI_GP1, 1);
	}
	else
	{
		// Disable ads1013 ADC
		gpio_direction_output(ADCPWREN_PMI_GP1, 0);
	}
		
	return rc;
}
EXPORT_SYMBOL(ADCPWR_enable);

int DPM_SW_enable(bool bEnable)
{
	int rc = 0;
	
	pr_info("%s: bEnable=%d\n", __func__, bEnable);
	if(bEnable)
	{
		// Switch D+/D- to ADC channel		
		gpio_direction_output(ADC_SW_EN, 1);
	}
	else
	{
		// Switch D+/D- to USB port		
		gpio_direction_output(ADC_SW_EN, 0);
	}
		
	return rc;
}
EXPORT_SYMBOL(DPM_SW_enable);

 /**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
void ads1013_get_gpio_info(void)
{
	static struct device_node *node=NULL;
	int err = 0;
	
	HW_ID = Read_HW_ID();
	node = of_find_compatible_node(NULL, NULL, "ti,ads1013");
	if (!HW_ID)
		ADC_SW_EN = of_get_named_gpio(node, "sr_adc_sw_en", 0);
	else
		ADC_SW_EN = of_get_named_gpio(node, "er_adc_sw_en", 0);

	printk(KERN_ERR "ADC_SW_EN = %d\n",ADC_SW_EN);
	err = gpio_request(ADC_SW_EN,"ADC_SW_EN");
	if (err < 0)
		printk(KERN_ERR "%s: gpio_request failed for gpio %d\n", __func__, ADC_SW_EN);
	else
		printk(KERN_ERR "%s: gpio_request success for gpio %d\n", __func__, ADC_SW_EN);
	gpio_direction_output(ADC_SW_EN, 0);
	
	ADCPWREN_PMI_GP1 = of_get_named_gpio(node, "adcpwren_pmi_gp1",0);
	printk(KERN_ERR "ADCPWREN_PMI_GP1 = %d\n", ADCPWREN_PMI_GP1);

	err = gpio_request(ADCPWREN_PMI_GP1,"ADCPWREN_PMI_GP1");
        if (err < 0)
                printk(KERN_ERR "%s: gpio_request failed for gpio %d, err = %d\n", __func__, ADCPWREN_PMI_GP1,err);
        else
                printk(KERN_ERR "%s: gpio_request success for gpio %d\n", __func__, ADCPWREN_PMI_GP1);
	gpio_direction_output(ADCPWREN_PMI_GP1, 0);

}  
  
static int ads1013_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	printk(KERN_ERR "[ads1013_probe] \n");
	new_client = client;

	ads1013_write_byte(0x1, 0x483);

	return 0;
}

static int ads1013_remove(struct i2c_client *client)
{
	return 0;
}
struct platform_device ads1013_user_space_device = {
                .name   = "ads1013",
                .id     = -1,
};

static int ads1013_user_space_probe(struct platform_device *dev)
{
    int ret_device_file = 0;
	printk(KERN_ERR "******** ads1013_user_space_probe!! ********\n");
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ads1013_dump);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_gpiot);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_thermal_check);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_adapter_id);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_adc_id_hi);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_adc_id_lo);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ads1013_rw);
	
	return 0;
}

static struct platform_driver ads1013_user_space_driver = {
                .probe      = ads1013_user_space_probe,
                .driver     = {
                                .name = "ads1013",
                },
};


static const struct i2c_device_id ads1013_i2c_id[] = { {"ads1013", 0}, {} };

static struct i2c_driver ads1013_i2c_driver = {
	.driver = {
		.name = "ads1013",
		.owner = THIS_MODULE,
	},
	.probe = ads1013_probe,
	.remove = ads1013_remove,
	.id_table = ads1013_i2c_id,
};

static int __init ads1013_init(void)
{
	short int ads1013_reg = 0;
	int ret = 0;
	unsigned char RegNum;

	if (Read_PROJ_ID() != 3)
		return 0;

	if(i2c_add_driver(&ads1013_i2c_driver) != 0)
		printk(KERN_ERR "[ads1013_init] failed to register ads1013 i2c driver\n");
	else
		printk(KERN_ERR "[ads1013_init] Success to register ads1013 i2c driver\n");
	
	ret = platform_device_register(&ads1013_user_space_device);
	if (ret){
		printk(KERN_ERR "[ads1013_init] Unable to device register\n");
	}

	ret = platform_driver_register(&ads1013_user_space_driver);
	if (ret){
		printk(KERN_ERR "[ads1013_init] Unable to register driver\n");
	}

	ads1013_get_gpio_info();

	RegNum = 0x00;
	ret = ads1013_read_byte(RegNum, &ads1013_reg);
	printk(KERN_ERR  "[ads1013_init] Reg[%x]=0x%x\n",RegNum , ads1013_reg);

	RegNum = 0x01;
	ret = ads1013_read_byte(RegNum, &ads1013_reg);
	printk(KERN_ERR  "[ads1013_init] Reg[%x]=0x%x\n",RegNum , ads1013_reg);

	RegNum = 0x02;
	ret = ads1013_read_byte(RegNum, &ads1013_reg);
	printk(KERN_ERR  "[ads1013_init] Reg[%x]=0x%x\n",RegNum , ads1013_reg);

	RegNum = 0x3;
	ret = ads1013_read_byte(RegNum, &ads1013_reg);
	printk(KERN_ERR  "[ads1013_init] Reg[%x]=0x%x\n",RegNum , ads1013_reg);

	
	return 0;
	
}
late_initcall(ads1013_init);

static void __exit ads1013_cleanup(void)
{
	i2c_del_driver(&ads1013_i2c_driver);
}
module_exit(ads1013_cleanup);

MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_DESCRIPTION("ads1013 voltage regulator driver");
MODULE_LICENSE("GPL v2");
