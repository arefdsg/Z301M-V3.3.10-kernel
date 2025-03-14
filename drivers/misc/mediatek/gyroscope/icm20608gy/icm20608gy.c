/* ICM20608 motion sensor driver
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

#include <cust_gyro.h>
#include "icm20608.h"
#include <gyroscope.h>
#include <hwmsensor.h>
#include <mt_boot_common.h>

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

#define INV_GYRO_AUTO_CALI  1

/*----------------------------------------------------------------------------*/
#define ICM20608_DEFAULT_FS		    GYRO_FS_1000DPS
#define ICM20608_DEFAULT_LSB		ICM20608_FS_1000_LSB
/*---------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
#define CONFIG_ICM20608_LOWPASS	/*apply low pass filter on output */
/*----------------------------------------------------------------------------*/
#define ICM20608_AXIS_X          0
#define ICM20608_AXIS_Y          1
#define ICM20608_AXIS_Z          2
#define ICM20608_AXES_NUM        3
#define ICM20608_DATA_LEN        6
#define ICM20608_DEV_NAME        "ICM20608GY"	/* name must different with gsensor icm20608 */
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id icm20608_i2c_id[] = {{ICM20608_DEV_NAME, 0}, {} };

int packet_thresh = 75;		/* 600 ms / 8ms/sample */

/* Maintain  cust info here */
struct gyro_hw gyro_cust;
static struct gyro_hw *hw = &gyro_cust;
struct platform_device *gyroPltFmDev;
/* For  driver get cust info */
struct gyro_hw *get_cust_gyro(void)
{
	return &gyro_cust;
}

/*----------------------------------------------------------------------------*/
static int icm20608_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int icm20608_i2c_remove(struct i2c_client *client);
static int icm20608_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#if !defined(CONFIG_HAS_EARLYSUSPEND)
static int icm20608_suspend(struct i2c_client *client, pm_message_t msg);
static int icm20608_resume(struct i2c_client *client);
#endif
static int icm20608_local_init(struct platform_device *pdev);
static int  icm20608_remove(void);
static int icm20608_init_flag = -1; /* 0<==>OK -1 <==> fail */
static struct gyro_init_info icm20608_init_info = {
		.name = "icm20608GY",
		.init = icm20608_local_init,
		.uninit = icm20608_remove,
};

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
static int gyroscope_setup_irq(void);
static DEFINE_MUTEX(gyroscope_scp_en_mutex);
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
/*----------------------------------------------------------------------------*/
enum {
	GYRO_TRC_FILTER = 0x01,
	GYRO_TRC_RAWDATA = 0x02,
	GYRO_TRC_IOCTL = 0x04,
	GYRO_TRC_CALI = 0X08,
	GYRO_TRC_INFO = 0X10,
	GYRO_TRC_DATA = 0X20,
};
/*----------------------------------------------------------------------------*/
struct scale_factor {
	u8 whole;
	u8 fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
	struct scale_factor scalefactor;
	int sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
	s16 raw[C_MAX_FIR_LENGTH][ICM20608_AXES_NUM];
	int sum[ICM20608_AXES_NUM];
	int num;
	int idx;
};
/*----------------------------------------------------------------------------*/
struct icm20608_i2c_data {
	struct i2c_client *client;
	struct gyro_hw *hw;
	struct hwmsen_convert cvt;
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	struct work_struct irq_work;
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

	/*misc */
	struct data_resolution *reso;
	atomic_t trace;
	atomic_t suspend;
	atomic_t selftest;
	atomic_t filter;
	s16 cali_sw[ICM20608_AXES_NUM + 1];

	/*data */
	s8 offset[ICM20608_AXES_NUM + 1];	/*+1: for 4-byte alignment */
	s16 data[ICM20608_AXES_NUM + 1];
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	int SCP_init_done;
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

#if defined(CONFIG_ICM20608_LOWPASS)
	atomic_t firlen;
	atomic_t fir_en;
	struct data_filter fir;
#endif
	/*early suspend */
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_drv;
#endif
#if INV_GYRO_AUTO_CALI == 1
	s16 inv_cali_raw[ICM20608_AXES_NUM + 1];
	s16 temperature;
	struct mutex temperature_mutex;	/* for temperature protection */
	struct mutex raw_data_mutex;	/* for inv_cali_raw[] protection */
#endif
};
#ifdef CONFIG_OF
static const struct of_device_id gyro_of_match[] = {
	{.compatible = "mediatek,gyroscope"},
	{},
};
#endif
/*----------------------------------------------------------------------------*/
static struct i2c_driver icm20608_i2c_driver = {
	.driver = {
		   .name = ICM20608_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = gyro_of_match,
#endif
		   },
	.probe = icm20608_i2c_probe,
	.remove = icm20608_i2c_remove,
	.detect = icm20608_i2c_detect,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = icm20608_suspend,
	.resume = icm20608_resume,
#endif
	.id_table = icm20608_i2c_id,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *icm20608_i2c_client;
static struct icm20608_i2c_data *obj_i2c_data;
static bool sensor_power;

/*----------------------------------------------------------------------------*/
//#define GYRO_FUN(f)               pr_debug(GYRO_TAG"%s\n", __func__)
/*----------------------------------------------------------------------------*/


static unsigned int power_on;
#if INV_GYRO_AUTO_CALI == 1
/*
devpath : "/sys/devices/virtual/invensense_daemon_class/invensense_daemon_device
class : "/sys/class/invensense_daemon_class"
inv_mpl_motion --
sysfs : "/sys/class/invensense_daemon_class/invensense_daemon_device/inv_mpl_motion", 1:motion 0:no motion
	   "/sys/devices/virtual/invensense_daemon_class/invensense_daemon_device/inv_mpl_motion", 1:motion 0:no motion
inv_gyro_data_ready --
sysfs : "/sys/class/invensense_daemon_class/invensense_daemon_device/inv_gyro_data_ready"
	   "/sys/devices/virtual/invensense_daemon_class/invensense_daemon_device/inv_gyro_data_ready"
inv_gyro_power_state --
sysfs : "/sys/class/invensense_daemon_class/invensense_daemon_device/inv_gyro_power_state"
	   "/sys/devices/virtual/invensense_daemon_class/invensense_daemon_device/inv_gyro_power_state"
*/

#define INV_DAEMON_CLASS_NAME  "invensense_daemon_class"
#define INV_DAEMON_DEVICE_NAME  "invensense_daemon_device"

static struct class *inv_daemon_class;
static struct device *inv_daemon_device;
static int inv_mpl_motion_state;	/* default is 0: no motion */
static int inv_gyro_power_state;
static ssize_t inv_mpl_motion_store(struct device *dev,
				    struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result;
	unsigned long data;

	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;

	/* if (inv_mpl_motion_state != data) */
	{
		char *envp[2];

		if (data)
			envp[0] = "STATUS=MOTION";
		else
			envp[0] = "STATUS=NOMOTION";
		envp[1] = NULL;
		result = kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);

		inv_mpl_motion_state = data;
	}

	return count;
}

static ssize_t inv_mpl_motion_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", inv_mpl_motion_state);
}

static ssize_t inv_gyro_data_ready_store(struct device *dev,
					 struct device_attribute *attr, const char *buf,
					 size_t count)
{
	sysfs_notify(&dev->kobj, NULL, "inv_gyro_data_ready");
	return count;
}

static ssize_t inv_gyro_data_ready_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "1\n");
}

static ssize_t inv_gyro_power_state_store(struct device *dev,
					  struct device_attribute *attr, const char *buf,
					  size_t count)
{
	unsigned int result;
	unsigned long data;

	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;

	inv_gyro_power_state = data;

	sysfs_notify(&dev->kobj, NULL, "inv_gyro_power_state");
	return count;
}

static ssize_t inv_gyro_power_state_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", inv_gyro_power_state);
}

static DEVICE_ATTR(inv_mpl_motion, S_IRUGO | S_IWUSR, inv_mpl_motion_show, inv_mpl_motion_store);
static DEVICE_ATTR(inv_gyro_data_ready, S_IRUGO | S_IWUSR, inv_gyro_data_ready_show, inv_gyro_data_ready_store);
static DEVICE_ATTR(inv_gyro_power_state, S_IRUGO | S_IWUSR, inv_gyro_power_state_show, inv_gyro_power_state_store);

static struct device_attribute *inv_daemon_dev_attributes[] = {
	&dev_attr_inv_mpl_motion,
	&dev_attr_inv_gyro_data_ready,
	&dev_attr_inv_gyro_power_state,
};
#endif				/* #if INV_GYRO_AUTO_CALI == 1 */


int ICM20608_gyro_power(void)
{
	return power_on;
}
EXPORT_SYMBOL(ICM20608_gyro_power);

int ICM20608_gyro_mode(void)
{
	return sensor_power;
}
EXPORT_SYMBOL(ICM20608_gyro_mode);

/*--------------------gyroscopy power control function----------------------------------*/
static void ICM20608_power(struct gyro_hw *hw, unsigned int on)
{
}
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static int ICM20608_write_rel_calibration(struct icm20608_i2c_data *obj, int dat[ICM20608_AXES_NUM])
{
	obj->cali_sw[ICM20608_AXIS_X] = obj->cvt.sign[ICM20608_AXIS_X]*dat[obj->cvt.map[ICM20608_AXIS_X]];
	obj->cali_sw[ICM20608_AXIS_Y] = obj->cvt.sign[ICM20608_AXIS_Y]*dat[obj->cvt.map[ICM20608_AXIS_Y]];
	obj->cali_sw[ICM20608_AXIS_Z] = obj->cvt.sign[ICM20608_AXIS_Z]*dat[obj->cvt.map[ICM20608_AXIS_Z]];
#if DEBUG
	if (atomic_read(&obj->trace) & GYRO_TRC_CALI) {
		GYRO_LOG("test  (%5d, %5d, %5d) ->(%5d, %5d, %5d)->(%5d, %5d, %5d))\n",
		 obj->cvt.sign[ICM20608_AXIS_X], obj->cvt.sign[ICM20608_AXIS_Y], obj->cvt.sign[ICM20608_AXIS_Z],
		 dat[ICM20608_AXIS_X], dat[ICM20608_AXIS_Y], dat[ICM20608_AXIS_Z],
		 obj->cvt.map[ICM20608_AXIS_X], obj->cvt.map[ICM20608_AXIS_Y], obj->cvt.map[ICM20608_AXIS_Z]);
		GYRO_LOG("write gyro calibration data  (%5d, %5d, %5d)\n",
		 obj->cali_sw[ICM20608_AXIS_X], obj->cali_sw[ICM20608_AXIS_Y], obj->cali_sw[ICM20608_AXIS_Z]);
	}
#endif
	return 0;
}


/*----------------------------------------------------------------------------*/
static int ICM20608_ResetCalibration(struct i2c_client *client)
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);
	/*there is not any application in sensorhub, so only get raw data from sensorhub.
	 Do not need reset calibration in sensorhub*/
/*#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA data;
	union ICM20608GY_CUST_DATA *pCustData;
	unsigned int len;

	if (0 != obj->SCP_init_done) {
		pCustData = (union ICM20608GY_CUST_DATA *) &data.set_cust_req.custData;

		data.set_cust_req.sensorType = ID_GYROSCOPE;
		data.set_cust_req.action = SENSOR_HUB_SET_CUST;
		pCustData->resetCali.action = ICM20608GY_CUST_ACTION_RESET_CALI;
		len =
			offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->resetCali);
		SCP_sensorHub_req_send(&data, &len, 1);
	}
#endif*/				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	return 0;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_ReadCalibration(struct i2c_client *client, int dat[ICM20608_AXES_NUM])
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);

	dat[obj->cvt.map[ICM20608_AXIS_X]] = obj->cvt.sign[ICM20608_AXIS_X]*obj->cali_sw[ICM20608_AXIS_X];
	dat[obj->cvt.map[ICM20608_AXIS_Y]] = obj->cvt.sign[ICM20608_AXIS_Y]*obj->cali_sw[ICM20608_AXIS_Y];
	dat[obj->cvt.map[ICM20608_AXIS_Z]] = obj->cvt.sign[ICM20608_AXIS_Z]*obj->cali_sw[ICM20608_AXIS_Z];

#if DEBUG
	if (atomic_read(&obj->trace) & GYRO_TRC_CALI) {
		GYRO_LOG("Read gyro calibration data  (%5d, %5d, %5d)\n",
			dat[ICM20608_AXIS_X], dat[ICM20608_AXIS_Y], dat[ICM20608_AXIS_Z]);
	}
#endif

	return 0;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int ICM20608_WriteCalibration(struct i2c_client *client, int dat[ICM20608_AXES_NUM])
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[ICM20608_AXES_NUM];

	/*there is not any application in sensorhub, so only get raw data from sensorhub.
	 Do not need write calibration data to sensorhub*/
/*#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA data;
	union ICM20608GY_CUST_DATA *pCustData;
	unsigned int len;
#endif*/				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

	GYRO_FUN();
	if (!obj || !dat) {
		GYRO_ERR("null ptr!!\n");
		return -EINVAL;
	}
/*#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	pCustData = (union ICM20608GY_CUST_DATA *) data.set_cust_req.custData;
	data.set_cust_req.sensorType = ID_GYROSCOPE;
	data.set_cust_req.action = SENSOR_HUB_SET_CUST;
	pCustData->setCali.action = ICM20608GY_CUST_ACTION_SET_CALI;
	pCustData->setCali.data[ICM20608_AXIS_X] = dat[ICM20608_AXIS_X];
	pCustData->setCali.data[ICM20608_AXIS_Y] = dat[ICM20608_AXIS_Y];
	pCustData->setCali.data[ICM20608_AXIS_Z] = dat[ICM20608_AXIS_Z];
	len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->setCali);
	SCP_sensorHub_req_send(&data, &len, 1);
#endif*/				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

	cali[obj->cvt.map[ICM20608_AXIS_X]] = obj->cvt.sign[ICM20608_AXIS_X]*obj->cali_sw[ICM20608_AXIS_X];
	cali[obj->cvt.map[ICM20608_AXIS_Y]] = obj->cvt.sign[ICM20608_AXIS_Y]*obj->cali_sw[ICM20608_AXIS_Y];
	cali[obj->cvt.map[ICM20608_AXIS_Z]] = obj->cvt.sign[ICM20608_AXIS_Z]*obj->cali_sw[ICM20608_AXIS_Z];
		cali[ICM20608_AXIS_X] += dat[ICM20608_AXIS_X];
		cali[ICM20608_AXIS_Y] += dat[ICM20608_AXIS_Y];
		cali[ICM20608_AXIS_Z] += dat[ICM20608_AXIS_Z];
#if DEBUG
		if (atomic_read(&obj->trace) & GYRO_TRC_CALI) {
			GYRO_LOG("write gyro calibration data  (%5d, %5d, %5d)-->(%5d, %5d, %5d)\n",
				 dat[ICM20608_AXIS_X], dat[ICM20608_AXIS_Y], dat[ICM20608_AXIS_Z],
				 cali[ICM20608_AXIS_X], cali[ICM20608_AXIS_Y], cali[ICM20608_AXIS_Z]);
		}
#endif
		return ICM20608_write_rel_calibration(obj, cali);

	return err;
}

/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static int ICM20608_ReadStart(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	GYRO_FUN();

	if (enable) {
		/* enable xyz gyro in FIFO */
	databuf[0] = BIT_GYRO_FIFO_EN;
	} else {
		/* disable xyz gyro in FIFO */
	databuf[0] = 0;
	}

#ifdef ICM20608_ACCESS_BY_GSE_I2C
	res = ICM20608_hwmsen_write_block(ICM20608_REG_FIFO_EN, databuf, 0x1);
#else

	databuf[1] = databuf[0];
	databuf[0] = ICM20608_REG_FIFO_EN;
	res = i2c_master_send(client, databuf, 0x2);
#endif
	if (res <= 0) {
		GYRO_ERR(" enable xyz gyro in FIFO error,enable: 0x%x!\n", databuf[0]);
		return ICM20608_ERR_I2C;
	}
	GYRO_LOG("ICM20608_ReadStart: enable xyz gyro in FIFO: 0x%x\n", databuf[0]);
	return ICM20608_SUCCESS;
}

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
int ICM20608gy_SCP_SetPowerMode(bool enable, int sensorType)
{
	static bool gyroscope_scp_en_status;
	static unsigned int gyroscope_scp_en_map;
	SCP_SENSOR_HUB_DATA req;
	int len;
	int err = 0;

	if (enable == sensor_power) {
		GYRO_LOG("Sensor power status is newest!\n");
		return ICM20608_SUCCESS;
	}

	mutex_lock(&gyroscope_scp_en_mutex);
	if (sensorType >= 32) {
		GYRO_ERR("Out of index!\n");
		return -1;
	}
	if (true == enable)
		gyroscope_scp_en_map |= (1 << sensorType);
	else
		gyroscope_scp_en_map &= ~(1 << sensorType);

	if (0 == gyroscope_scp_en_map)
		enable = false;
	else
		enable = true;

	if (gyroscope_scp_en_status != enable) {
		gyroscope_scp_en_status = enable;

		req.activate_req.sensorType = ID_GYROSCOPE;
		req.activate_req.action = SENSOR_HUB_ACTIVATE;
		req.activate_req.enable = enable;
		len = sizeof(req.activate_req);
		err = SCP_sensorHub_req_send(&req, &len, 1);
		if (err)
			GYRO_ERR("SCP_sensorHub_req_send fail\n");
	}

	mutex_unlock(&gyroscope_scp_en_mutex);

	if (enable == true)
		msleep(50);
	sensor_power = enable;
	return err;
}
EXPORT_SYMBOL(ICM20608gy_SCP_SetPowerMode);
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
/* ----------------------------------------------------------------------------// */
static int ICM20608_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	if (enable == sensor_power) {
		GYRO_LOG("Sensor power status is newest!\n");
		return ICM20608_SUCCESS;
	}

#if 0
#ifdef ICM20608_ACCESS_BY_GSE_I2C
	if (ICM20608_hwmsen_read_block(ICM20608_REG_PWR_MGMT_1, databuf, 0x01)) {
#else
	if (hwmsen_read_byte(client, ICM20608_REG_PWR_MGMT_1, databuf)) {
#endif
		GYRO_ERR("read power ctl register err!\n");
		return ICM20608_ERR_I2C;
	}
#endif

    databuf[0] = BIT_CLK_PLL;
#ifdef ICM20608_ACCESS_BY_GSE_I2C
    res = ICM20608_hwmsen_write_block(ICM20608_REG_PWR_MGMT_1, databuf, 0x1);
#else
    databuf[1] = databuf[0];
    databuf[0] = ICM20608_REG_PWR_MGMT_1;
    res = i2c_master_send(client, databuf, 0x2);
#endif
    if (res <= 0) {
        GYRO_LOG("set power mode failed!\n");
        return ICM20608_ERR_I2C;
    }

    //set ICM20608_REG_PWR_MGMT_2
    databuf[0] = 0;
	if (ICM20608_gse_mode() == false)
        databuf[0] |= BIT_STBY_A;
	if (enable == false)
        databuf[0] |= BIT_STBY_G;
#ifdef ICM20608_ACCESS_BY_GSE_I2C
    res = ICM20608_hwmsen_write_block(ICM20608_REG_PWR_MGMT_2, databuf, 0x1);
#else
    databuf[1] = databuf[0];
    databuf[0] = ICM20608_REG_PWR_MGMT_2;
    res = i2c_master_send(client, databuf, 0x2);
#endif
    if (res <= 0) {
        GYRO_LOG("set power mode failed!\n");
        return ICM20608_ERR_I2C;
    }


    //set ICM20608_REG_PWR_MGMT_1
	databuf[0] = BIT_CLK_PLL;
	if (enable == false) {
		if (ICM20608_gse_mode() == false)
			databuf[0] |= BIT_SLEEP;
	} else {
		/* do nothing */
	}
#ifdef ICM20608_ACCESS_BY_GSE_I2C
	res = ICM20608_hwmsen_write_block(ICM20608_REG_PWR_MGMT_1, databuf, 0x1);
#else
	databuf[1] = databuf[0];
	databuf[0] = ICM20608_REG_PWR_MGMT_1;
	res = i2c_master_send(client, databuf, 0x2);
#endif

	if (res <= 0) {
		GYRO_LOG("set power mode failed!\n");
		return ICM20608_ERR_I2C;
	}
	GYRO_LOG("set power mode ok %d!\n", enable);


    if (enable == true)
    {
        msleep(50);
    }

	sensor_power = enable;

	return ICM20608_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	GYRO_FUN();

#ifdef ICM20608_ACCESS_BY_GSE_I2C
	databuf[0] = dataformat;
	res = ICM20608_hwmsen_write_block(ICM20608_REG_CONFIG, databuf, 0x1);
#else
	databuf[0] = ICM20608_REG_CONFIG;
	databuf[1] = dataformat;
	res = i2c_master_send(client, databuf, 0x2);
#endif
	if (res <= 0)
		return ICM20608_ERR_I2C;

	/* read sample rate after written for test */
	udelay(500);
#ifdef ICM20608_ACCESS_BY_GSE_I2C
	if (ICM20608_hwmsen_read_block(ICM20608_REG_CONFIG, databuf, 0x01)) {
#else
	if (hwmsen_read_byte(client, ICM20608_REG_CONFIG, databuf)) {
#endif
		GYRO_ERR("read data format register err!\n");
		return ICM20608_ERR_I2C;
	}
	GYRO_LOG("read  data format: 0x%x\n", databuf[0]);

	return ICM20608_SUCCESS;
}

static int ICM20608_SetFullScale(struct i2c_client *client, u8 dataformat)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	GYRO_FUN();

#ifdef ICM20608_ACCESS_BY_GSE_I2C
	databuf[0] = dataformat;
	res = ICM20608_hwmsen_write_block(ICM20608_REG_GYRO_CONFIG, databuf, 0x1);
#else
	databuf[0] = ICM20608_REG_GYRO_CONFIG;
	databuf[1] = dataformat;
	res = i2c_master_send(client, databuf, 0x2);
#endif
	if (res <= 0)
		return ICM20608_ERR_I2C;

	/* read sample rate after written for test */
	udelay(500);
#ifdef ICM20608_ACCESS_BY_GSE_I2C
	if (ICM20608_hwmsen_read_block(ICM20608_REG_GYRO_CONFIG, databuf, 0x01)) {
#else
	if (hwmsen_read_byte(client, ICM20608_REG_GYRO_CONFIG, databuf)) {
#endif
		GYRO_ERR("read data format register err!\n");
		return ICM20608_ERR_I2C;
	}
	GYRO_LOG("read  data format: 0x%x\n", databuf[0]);


	return ICM20608_SUCCESS;
}


/* set the sample rate */
static int ICM20608_SetSampleRate(struct i2c_client *client, int sample_rate)
{
	u8 databuf[2] = { 0 };
	int rate_div = 0;
	int res = 0;

	GYRO_FUN();
#ifdef ICM20608_ACCESS_BY_GSE_I2C
	if (ICM20608_hwmsen_read_block(ICM20608_REG_CONFIG, databuf, 0x01)) {
#else
	if (hwmsen_read_byte(client, ICM20608_REG_CONFIG, databuf)) {
#endif
		GYRO_ERR("read gyro data format register err!\n");
		return ICM20608_ERR_I2C;
	}
	GYRO_LOG("read  gyro data format register: 0x%x\n", databuf[0]);

	if ((databuf[0] & 0x07) == 0) {	/* Analog sample rate is 8KHz */
		rate_div = 8 * 1024 / sample_rate - 1;
	} else {	/* 1kHz */
		rate_div = 1024 / sample_rate - 1;
	}

	if (rate_div > 255)	{ /* rate_div: 0 to 255; */
		rate_div = 255;
	} else if (rate_div < 0) {
		rate_div = 0;
	}

#ifdef ICM20608_ACCESS_BY_GSE_I2C
	databuf[0] = rate_div;
	res = ICM20608_hwmsen_write_block(ICM20608_REG_SMPLRT_DIV, databuf, 0x1);
#else
	databuf[0] = ICM20608_REG_SMPLRT_DIV;
	databuf[1] = rate_div;
	res = i2c_master_send(client, databuf, 0x2);
#endif
	if (res <= 0) {
		GYRO_ERR("write sample rate register err!\n");
		return ICM20608_ERR_I2C;
	}
	/* read sample div after written for test */
	udelay(500);
#ifdef ICM20608_ACCESS_BY_GSE_I2C
	if (ICM20608_hwmsen_read_block(ICM20608_REG_SMPLRT_DIV, databuf, 0x01)) {
#else
	if (hwmsen_read_byte(client, ICM20608_REG_SMPLRT_DIV, databuf)) {
#endif
		GYRO_ERR("read gyro sample rate register err!\n");
		return ICM20608_ERR_I2C;
	}
	GYRO_LOG("read  gyro sample rate: 0x%x\n", databuf[0]);

	return ICM20608_SUCCESS;
}

static int ICM20608_SetGyroCycleMode(struct i2c_client *client, u8 enable)
{
    u8 databuf[2] = { 0 };
    int gyro_avg = 0; // 1x
    int res = 0;

    GYRO_FUN();
#ifdef ICM20608_ACCESS_BY_GSE_I2C
    if (ICM20608_hwmsen_read_block(ICM20608_REG_LP_MODE_CFG, databuf, 0x01)) {
#else
    if (hwmsen_read_byte(client, ICM20608_REG_LP_MODE_CFG, databuf)) {
#endif
        GYRO_ERR("read LP_MODE_CFG register err!\n");
        return ICM20608_ERR_I2C;
    }
    GYRO_LOG("read LP_MODE_CFG register: 0x%x\n", databuf[0]);

    databuf[0] &= (~(BIT_GYRO_CYCLE | BIT_G_AVGCFG));
    if (enable)
        databuf[0] |= (BIT_GYRO_CYCLE | (gyro_avg << SHIFT_G_AVGCFG));

#ifdef ICM20608_ACCESS_BY_GSE_I2C
    res = ICM20608_hwmsen_write_block(ICM20608_REG_LP_MODE_CFG, databuf, 0x1);
#else
    databuf[1] = databuf[0];
    databuf[0] = ICM20608_REG_LP_MODE_CFG;
    res = i2c_master_send(client, databuf, 0x2);
#endif
    if (res <= 0) {
        GYRO_ERR("write LP_MODE_CFG register err!\n");
        return ICM20608_ERR_I2C;
    }
    
    /* read LP_MODE_CFG after written for test */
    udelay(500);
#ifdef ICM20608_ACCESS_BY_GSE_I2C
    if (ICM20608_hwmsen_read_block(ICM20608_REG_LP_MODE_CFG, databuf, 0x01)) {
#else
    if (hwmsen_read_byte(client, ICM20608_REG_LP_MODE_CFG, databuf)) {
#endif
        GYRO_ERR("read gyro sample rate register err!\n");
        return ICM20608_ERR_I2C;
    }
    GYRO_LOG("read LP_MODE_CFG: 0x%x\n", databuf[0]);

    return ICM20608_SUCCESS;
}


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int ICM20608_FIFOConfig(struct i2c_client *client, u8 clk)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	GYRO_FUN();

	/* use gyro X, Y or Z for clocking */
#ifdef ICM20608_ACCESS_BY_GSE_I2C
	databuf[0] = clk;
	res = ICM20608_hwmsen_write_block(ICM20608_REG_PWR_MGMT_1, databuf, 0x1);
#else
	databuf[0] = ICM20608_REG_PWR_MGMT_1;
	databuf[1] = clk;
	res = i2c_master_send(client, databuf, 0x2);
#endif
	if (res <= 0) {
		GYRO_ERR("write Power CTRL register err!\n");
		return ICM20608_ERR_I2C;
	}
	GYRO_LOG("ICM20608 use gyro X for clocking OK!\n");

	mdelay(50);

	/* enable xyz gyro in FIFO */
#ifdef ICM20608_ACCESS_BY_GSE_I2C
	databuf[0] = BIT_GYRO_FIFO_EN;
	res = ICM20608_hwmsen_write_block(ICM20608_REG_FIFO_EN, databuf, 0x1);
#else
	databuf[0] = ICM20608_REG_FIFO_EN;
	databuf[1] = BIT_GYRO_FIFO_EN;
	res = i2c_master_send(client, databuf, 0x2);
#endif
	if (res <= 0) {
		GYRO_ERR("write Power CTRL register err!\n");
		return ICM20608_ERR_I2C;
	}
	GYRO_LOG("ICM20608 enable xyz gyro in FIFO OK!\n");

	/* enable FIFO and reset FIFO */
#ifdef ICM20608_ACCESS_BY_GSE_I2C
	databuf[0] = (BIT_FIFO_EN | BIT_FIFO_RST);
	res = ICM20608_hwmsen_write_block(ICM20608_REG_USER_CTRL, databuf, 0x1);
#else
	databuf[0] = ICM20608_REG_USER_CTRL;
	databuf[1] = (BIT_FIFO_EN | BIT_FIFO_RST);
	res = i2c_master_send(client, databuf, 0x2);
#endif
	if (res <= 0) {
		GYRO_ERR("write FIFO CTRL register err!\n");
		return ICM20608_ERR_I2C;
	}
	GYRO_LOG("ICM20608_FIFOConfig OK!\n");
	return ICM20608_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_ReadFifoData(struct i2c_client *client, s16 *data, int *datalen)
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);
	u8 buf[ICM20608_DATA_LEN] = {0};
	s16 tmp1[ICM20608_AXES_NUM] = {0};
	s16 tmp2[ICM20608_AXES_NUM] = {0};
	int err = 0;
	u8 tmp = 0;
	int packet_cnt = 0;
	int i;

	GYRO_FUN();

	if (NULL == client)
		return -EINVAL;

	/* stop putting data in FIFO */
	ICM20608_ReadStart(client, false);

	/* read data number of bytes in FIFO */
#ifdef ICM20608_ACCESS_BY_GSE_I2C
	err = ICM20608_hwmsen_read_block(ICM20608_REG_FIFO_COUNT_H, &tmp, 0x01);
#else
	err = hwmsen_read_byte(client, ICM20608_REG_FIFO_COUNT_H, &tmp);
#endif
	if (err) {
		GYRO_ERR("read data high number of bytes error: %d\n", err);
		return -1;
	}
	packet_cnt = tmp << 8;

#ifdef ICM20608_ACCESS_BY_GSE_I2C
	err = ICM20608_hwmsen_read_block(ICM20608_REG_FIFO_COUNT_H+1, &tmp, 0x01);
#else
	err = hwmsen_read_byte(client, ICM20608_REG_FIFO_COUNT_H+1, &tmp);
#endif
	if (err) {
		GYRO_ERR("read data low number of bytes error: %d\n", err);
		return -1;
	}
	packet_cnt = (packet_cnt + tmp) / ICM20608_DATA_LEN;

	GYRO_LOG("ICM20608 Read Data packet number OK: %d\n", packet_cnt);

	*datalen = packet_cnt;

	/* Within +-5% range: timing_tolerance * packet_thresh=0.05*75 */
	if (packet_cnt && (abs(packet_thresh - packet_cnt) < 4)) {
		/* read data in FIFO */
		for (i = 0; i < packet_cnt; i++) {
#ifdef ICM20608_ACCESS_BY_GSE_I2C
			if (ICM20608_hwmsen_read_block(ICM20608_REG_FIFO_R_W, buf, ICM20608_DATA_LEN)) {
#else
			if (hwmsen_read_block(client, ICM20608_REG_FIFO_R_W, buf, ICM20608_DATA_LEN)) {
#endif
				GYRO_ERR("ICM20608 read data from FIFO error: %d\n", err);
				return -2;
	    } else
				GYRO_LOG("ICM20608 read Data of diff address from FIFO OK !\n");

			tmp1[ICM20608_AXIS_X] = (s16)((buf[ICM20608_AXIS_X*2+1]) | (buf[ICM20608_AXIS_X*2] << 8));
			tmp1[ICM20608_AXIS_Y] = (s16)((buf[ICM20608_AXIS_Y*2+1]) | (buf[ICM20608_AXIS_Y*2] << 8));
			tmp1[ICM20608_AXIS_Z] = (s16)((buf[ICM20608_AXIS_Z*2+1]) | (buf[ICM20608_AXIS_Z*2] << 8));

	    /* remap coordinate// */
			tmp2[obj->cvt.map[ICM20608_AXIS_X]] = obj->cvt.sign[ICM20608_AXIS_X]*tmp1[ICM20608_AXIS_X];
			tmp2[obj->cvt.map[ICM20608_AXIS_Y]] = obj->cvt.sign[ICM20608_AXIS_Y]*tmp1[ICM20608_AXIS_Y];
			tmp2[obj->cvt.map[ICM20608_AXIS_Z]] = obj->cvt.sign[ICM20608_AXIS_Z]*tmp1[ICM20608_AXIS_Z];

			data[3 * i + ICM20608_AXIS_X] = tmp2[ICM20608_AXIS_X];
			data[3 * i + ICM20608_AXIS_Y] = tmp2[ICM20608_AXIS_Y];
			data[3 * i + ICM20608_AXIS_Z] = tmp2[ICM20608_AXIS_Z];

			GYRO_LOG("gyro FIFO packet[%d]:[%04X %04X %04X] => [%5d %5d %5d]\n", i,
				data[3*i + ICM20608_AXIS_X], data[3*i + ICM20608_AXIS_Y], data[3*i + ICM20608_AXIS_Z],
				data[3*i + ICM20608_AXIS_X], data[3*i + ICM20608_AXIS_Y], data[3*i + ICM20608_AXIS_Z]);
		}
	} else {
		GYRO_ERR("ICM20608 Incorrect packet count: %d\n", packet_cnt);
		return -3;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_ReadGyroData(struct i2c_client *client, char *buf, int bufsize)
{
	char databuf[6];
	int data[3];
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
	int err = 0;
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);

    if (atomic_read(&obj->suspend))
    {
        return -3;
    }

	if (sensor_power == false) {
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
		ICM20608gy_SCP_SetPowerMode(true, ID_GYROSCOPE);
#else
		ICM20608_SetPowerMode(client, true);
#endif
	}


#if INV_GYRO_AUTO_CALI == 1
#ifdef ICM20608_ACCESS_BY_GSE_I2C
	if (ICM20608_hwmsen_read_block(ICM20608_REG_TEMP_OUT_H, databuf, 2)) {
#else
	if (hwmsen_read_block(client, ICM20608_REG_TEMP_OUT_H, databuf, 2)) {
#endif
		GYRO_ERR("ICM20608 read temperature data  error\n");
		return -2;
	}

	mutex_lock(&obj->temperature_mutex);
	obj->temperature = ((s16)((databuf[1]) | (databuf[0] << 8)));
	mutex_unlock(&obj->temperature_mutex);

#endif
/*just read raw data form sensor hub, calibration data save at AP side*/
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	req.get_data_req.sensorType = ID_GYROSCOPE;
	req.get_data_req.action = SENSOR_HUB_GET_DATA;
	len = sizeof(req.get_data_req);
	err = SCP_sensorHub_req_send(&req, &len, 1);
	if (err) {
		GYRO_ERR("SCP_sensorHub_req_send fail!\n");
		return err;
	}

	if (ID_GYROSCOPE != req.get_data_rsp.sensorType ||
		SENSOR_HUB_GET_DATA != req.get_data_rsp.action || 0 != req.get_data_rsp.errCode) {
		GYRO_ERR("error : %d\n", req.get_data_rsp.errCode);
		return req.get_data_rsp.errCode;
	}

	databuf[0] = (char)req.get_data_rsp.int8_Data[0];
	databuf[1] = (char)req.get_data_rsp.int8_Data[1];
	databuf[2] = (char)req.get_data_rsp.int8_Data[2];
	databuf[3] = (char)req.get_data_rsp.int8_Data[3];
	databuf[4] = (char)req.get_data_rsp.int8_Data[4];
	databuf[5] = (char)req.get_data_rsp.int8_Data[5];
#else
#ifdef ICM20608_ACCESS_BY_GSE_I2C
	if (ICM20608_hwmsen_read_block(ICM20608_REG_GYRO_XOUT_H, databuf, 6)) {
#else
	if (hwmsen_read_block(client, ICM20608_REG_GYRO_XOUT_H, databuf, 6)) {
#endif
		GYRO_ERR("ICM20608 read gyroscope data  error\n");
		return -2;
	}
	/*GYRO_ERR("x = %d, y = %d, z = %d -- 2\n", obj->data[ICM20608_AXIS_X], obj->data[ICM20608_AXIS_Y], obj->data[ICM20608_AXIS_Z]);*/
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */


	obj->data[ICM20608_AXIS_X] = ((s16)((databuf[ICM20608_AXIS_X*2+1]) | (databuf[ICM20608_AXIS_X*2] << 8)));
	obj->data[ICM20608_AXIS_Y] = ((s16)((databuf[ICM20608_AXIS_Y*2+1]) | (databuf[ICM20608_AXIS_Y*2] << 8)));
	obj->data[ICM20608_AXIS_Z] = ((s16)((databuf[ICM20608_AXIS_Z*2+1]) | (databuf[ICM20608_AXIS_Z*2] << 8)));
	/*GYRO_ERR("x = %d, y = %d, z = %d -- 1\n", obj->data[ICM20608_AXIS_X], obj->data[ICM20608_AXIS_Y], obj->data[ICM20608_AXIS_Z]);*/
    
#if DEBUG
	if (atomic_read(&obj->trace) & GYRO_TRC_RAWDATA) {
		GYRO_LOG("read gyro register: %d, %d, %d, %d, %d, %d",
			databuf[0], databuf[1], databuf[2], databuf[3], databuf[4], databuf[5]);
		GYRO_LOG("get gyro raw data (0x%08X, 0x%08X, 0x%08X) -> (%5d, %5d, %5d)\n",
			obj->data[ICM20608_AXIS_X], obj->data[ICM20608_AXIS_Y], obj->data[ICM20608_AXIS_Z],
			obj->data[ICM20608_AXIS_X], obj->data[ICM20608_AXIS_Y], obj->data[ICM20608_AXIS_Z]);
	}
#endif
#if INV_GYRO_AUTO_CALI == 1
	mutex_lock(&obj->raw_data_mutex);
	/*remap coordinate*/
	obj->inv_cali_raw[obj->cvt.map[ICM20608_AXIS_X]] = obj->cvt.sign[ICM20608_AXIS_X]*obj->data[ICM20608_AXIS_X];
	obj->inv_cali_raw[obj->cvt.map[ICM20608_AXIS_Y]] = obj->cvt.sign[ICM20608_AXIS_Y]*obj->data[ICM20608_AXIS_Y];
	obj->inv_cali_raw[obj->cvt.map[ICM20608_AXIS_Z]] = obj->cvt.sign[ICM20608_AXIS_Z]*obj->data[ICM20608_AXIS_Z];
	mutex_unlock(&obj->raw_data_mutex);
#endif
	obj->data[ICM20608_AXIS_X] = obj->data[ICM20608_AXIS_X] + obj->cali_sw[ICM20608_AXIS_X];
	obj->data[ICM20608_AXIS_Y] = obj->data[ICM20608_AXIS_Y] + obj->cali_sw[ICM20608_AXIS_Y];
	obj->data[ICM20608_AXIS_Z] = obj->data[ICM20608_AXIS_Z] + obj->cali_sw[ICM20608_AXIS_Z];

	/*remap coordinate*/
	data[obj->cvt.map[ICM20608_AXIS_X]] = obj->cvt.sign[ICM20608_AXIS_X]*obj->data[ICM20608_AXIS_X];
	data[obj->cvt.map[ICM20608_AXIS_Y]] = obj->cvt.sign[ICM20608_AXIS_Y]*obj->data[ICM20608_AXIS_Y];
	data[obj->cvt.map[ICM20608_AXIS_Z]] = obj->cvt.sign[ICM20608_AXIS_Z]*obj->data[ICM20608_AXIS_Z];

	/* Out put the degree/second(o/s) */
	data[ICM20608_AXIS_X] = data[ICM20608_AXIS_X] * ICM20608_FS_MAX_LSB / ICM20608_DEFAULT_LSB;
	data[ICM20608_AXIS_Y] = data[ICM20608_AXIS_Y] * ICM20608_FS_MAX_LSB / ICM20608_DEFAULT_LSB;
	data[ICM20608_AXIS_Z] = data[ICM20608_AXIS_Z] * ICM20608_FS_MAX_LSB / ICM20608_DEFAULT_LSB;

	sprintf(buf, "%04x %04x %04x", data[ICM20608_AXIS_X], data[ICM20608_AXIS_Y], data[ICM20608_AXIS_Z]);

#if DEBUG
	if (atomic_read(&obj->trace) & GYRO_TRC_DATA)
		GYRO_LOG("get gyro data packet:[%d %d %d]\n", data[0], data[1], data[2]);
#endif

	return 0;
}

/* for factory mode */
static int ICM20608_PROCESS_SMT_DATA(struct i2c_client *client, short *data)
{
	int total_num = 0;
	int retval = 0;
	long xSum = 0;
	long ySum = 0;
	long zSum = 0;
	long xAvg, yAvg, zAvg;
	long xRMS, yRMS, zRMS;
	int i = 0;

	int bias_thresh = 5242; /* 40 dps * 131.072 LSB/dps */
	/* float RMS_thresh = 687.19f; // (.2 dps * 131.072) ^ 2 */
	long RMS_thresh = 68719; /* (.2 dps * 131.072) ^ 2 */

	total_num = data[0];
	retval = data[1];
	GYRO_LOG("ICM20608 read gyro data OK, total number: %d\n", total_num);
	for (i = 0; i < total_num; i++) {
		xSum = xSum + data[ICM20608_AXES_NUM*i + ICM20608_AXIS_X + 2];
		ySum = ySum + data[ICM20608_AXES_NUM*i + ICM20608_AXIS_Y + 2];
		zSum = zSum + data[ICM20608_AXES_NUM*i + ICM20608_AXIS_Z + 2];

		/*
		FLPLOGD("read gyro data OK: packet_num:%d, [X:%5d, Y:%5d, Z:%5d]\n", i,
			data[ICM20608_AXES_NUM*i + ICM20608_AXIS_X +2], data[ICM20608_AXES_NUM*i + ICM20608_AXIS_Y +2],
			data[ICM20608_AXES_NUM*i + ICM20608_AXIS_Z +2]);
			FLPLOGD("ICM20608 xSum: %5d,  ySum: %5d, zSum: %5d\n", xSum, ySum, zSum);
		*/
	}
	GYRO_LOG("ICM20608 xSum: %5ld,  ySum: %5ld, zSum: %5ld\n", xSum, ySum, zSum);

	if (total_num != 0) {
		xAvg = (xSum / total_num);
		yAvg = (ySum / total_num);
		zAvg = (zSum / total_num);
	} else {
		xAvg = xSum;
		yAvg = ySum;
		zAvg = zSum;
	}

	GYRO_LOG("ICM20608 xAvg: %ld,  yAvg: %ld,  zAvg: %ld\n", xAvg, yAvg, zAvg);

	if (abs(xAvg) > bias_thresh) {
		GYRO_LOG("X-Gyro bias exceeded threshold\n");
		retval |= 1 << 3;
	}
	if (abs(yAvg) >  bias_thresh) {
		GYRO_LOG("Y-Gyro bias exceeded threshold\n");
		retval |= 1 << 4;
	}
	if (abs(zAvg) > bias_thresh) {
		GYRO_LOG("Z-Gyro bias exceeded threshold\n");
		retval |= 1 << 5;
	}

	xRMS = 0;
	yRMS = 0;
	zRMS = 0;

	/* Finally, check RMS */
	for (i = 0; i < total_num; i++) {
		xRMS += (data[ICM20608_AXES_NUM*i + ICM20608_AXIS_X+2]-xAvg)*
			(data[ICM20608_AXES_NUM*i + ICM20608_AXIS_X+2]-xAvg);
		yRMS += (data[ICM20608_AXES_NUM*i + ICM20608_AXIS_Y+2]-yAvg)*
			(data[ICM20608_AXES_NUM*i + ICM20608_AXIS_Y+2]-yAvg);
		zRMS += (data[ICM20608_AXES_NUM*i + ICM20608_AXIS_Z+2]-zAvg)*
			(data[ICM20608_AXES_NUM*i + ICM20608_AXIS_Z+2]-zAvg);
	}

	GYRO_LOG("ICM20608 xRMS: %ld,  yRMS: %ld,  zRMS: %ld\n", xRMS, yRMS, zRMS);
	xRMS = 100*xRMS;
	yRMS = 100*yRMS;
	zRMS = 100*zRMS;

	if (FACTORY_BOOT == get_boot_mode())
		return retval;
	if (xRMS > RMS_thresh * total_num) {
		GYRO_LOG("X-Gyro RMS exceeded threshold, RMS_thresh: %ld\n", RMS_thresh * total_num);
		retval |= 1 << 6;
	}
	if (yRMS > RMS_thresh * total_num) {
		GYRO_LOG("Y-Gyro RMS exceeded threshold, RMS_thresh: %ld\n", RMS_thresh * total_num);
		retval |= 1 << 7;
	}
	if (zRMS > RMS_thresh * total_num) {
		GYRO_LOG("Z-Gyro RMS exceeded threshold, RMS_thresh: %ld\n", RMS_thresh * total_num);
		retval |= 1 << 8;
	}
	if (xRMS == 0 || yRMS == 0 || zRMS == 0)
		/* If any of the RMS noise value returns zero, then we might have dead gyro or FIFO/register failure */
		retval |= 1 << 9;

	GYRO_LOG("retval %d\n", retval);
	return retval;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_SMTReadSensorData(struct i2c_client *client, s16 *buf, int bufsize)
{
/* S16 gyro[ICM20608_AXES_NUM*ICM20608_FIFOSIZE]; */
	int res = 0;
	int i;
	int datalen, total_num = 0;

	GYRO_FUN();

	if (sensor_power == false)
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
		ICM20608gy_SCP_SetPowerMode(true, ID_GYROSCOPE);
#else
		ICM20608_SetPowerMode(client, true);
#endif

	if (NULL == buf)
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	for (i = 0; i < ICM20608_AXES_NUM; i++) {
		res = ICM20608_FIFOConfig(client, (i+1));
		if (res) {
			GYRO_ERR("ICM20608_FIFOConfig error:%d!\n", res);
			return -3;
		}

		/* putting data in FIFO during the delayed 600ms */
		mdelay(600);

		res = ICM20608_ReadFifoData(client, &(buf[total_num+2]), &datalen);
		if (res) {
			if (res == (-3))
				buf[1] = (1 << i);
	    else {
				GYRO_ERR("ICM20608_ReadData error:%d!\n", res);
				return -3;
	    }
		} else {
			buf[0] = datalen;
			total_num += datalen*ICM20608_AXES_NUM;
		}
	}

	GYRO_LOG("gyroscope read data OK, total packet: %d", buf[0]);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];

	memset(databuf, 0, sizeof(u8)*10);

	if ((NULL == buf) || (bufsize <= 30))
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	sprintf(buf, "ICM20608 Chip");
	return 0;
}

#if INV_GYRO_AUTO_CALI == 1
/*----------------------------------------------------------------------------*/
static int ICM20608_ReadGyroDataRaw(struct i2c_client *client, char *buf, int bufsize)
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);

	mutex_lock(&obj->raw_data_mutex);
	/* return gyro raw LSB in device orientation */
	sprintf(buf, "%x %x %x", obj->inv_cali_raw[ICM20608_AXIS_X],
		obj->inv_cali_raw[ICM20608_AXIS_Y], obj->inv_cali_raw[ICM20608_AXIS_Z]);

#if DEBUG
	if (atomic_read(&obj->trace) & GYRO_TRC_DATA)
		GYRO_LOG("get gyro raw data packet:[%d %d %d]\n", obj->inv_cali_raw[0],
			obj->inv_cali_raw[1], obj->inv_cali_raw[2]);
#endif
	mutex_unlock(&obj->raw_data_mutex);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_ReadTemperature(struct i2c_client *client, char *buf, int bufsize)
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);

	mutex_lock(&obj->temperature_mutex);
	sprintf(buf, "%x", obj->temperature);

#if DEBUG
	if (atomic_read(&obj->trace) & GYRO_TRC_DATA)
		GYRO_LOG("get gyro temperature:[%d]\n", obj->temperature);
#endif
	mutex_unlock(&obj->temperature_mutex);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_ReadPowerStatus(struct i2c_client *client, char *buf, int bufsize)
{
#if DEBUG
	GYRO_LOG("get gyro PowerStatus:[%d]\n", sensor_power);
#endif

	sprintf(buf, "%x", sensor_power);

	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = icm20608_i2c_client;
	char strbuf[ICM20608_BUFSIZE];

	if (NULL == client) {
		GYRO_ERR("i2c client is null!!\n");
		return 0;
	}

	ICM20608_ReadChipInfo(client, strbuf, ICM20608_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = icm20608_i2c_client;
	char strbuf[ICM20608_BUFSIZE];

	if (NULL == client) {
		GYRO_ERR("i2c client is null!!\n");
		return 0;
	}

	ICM20608_ReadGyroData(client, strbuf, ICM20608_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct icm20608_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GYRO_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct icm20608_i2c_data *obj = obj_i2c_data;
	int trace;

	if (obj == NULL) {
		GYRO_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&obj->trace, trace);
	else
		GYRO_ERR("invalid content: '%s', length = %zu\n", buf, count);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct icm20608_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GYRO_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (obj->hw)
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
			obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);
	else
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");

	return len;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,             S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *ICM20608_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
};

/*----------------------------------------------------------------------------*/
static int icm20608_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(ICM20608_attr_list) / sizeof(ICM20608_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;


	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, ICM20608_attr_list[idx]);
		if (0 != err) {
			GYRO_ERR("driver_create_file (%s) = %d\n",
				 ICM20608_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int icm20608_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(ICM20608_attr_list) / sizeof(ICM20608_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;



	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, ICM20608_attr_list[idx]);



	return err;
}

/*----------------------------------------------------------------------------*/
static int icm20608_gpio_config(void)
{
	/* because we donot use EINT ,to support low power */
	/* config to GPIO input mode + PD */
	/* set   GPIO_MSE_EINT_PIN */
	/*
	mt_set_gpio_mode(GPIO_GYRO_EINT_PIN, GPIO_GYRO_EINT_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_GYRO_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_GYRO_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_GYRO_EINT_PIN, GPIO_PULL_DOWN);
	*/
	return 0;
}
static int icm20608_init_client(struct i2c_client *client, bool enable)
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
    bool sensor_power_org;

	GYRO_FUN();
	icm20608_gpio_config();

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	res = gyroscope_setup_irq();
	if (res != ICM20608_SUCCESS)
		return res;
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

  sensor_power_org = sensor_power;
	res = ICM20608_SetPowerMode(client, true);
	if (res != ICM20608_SUCCESS)
		return res;



	/* The range should at least be 17.45 rad/s (ie: ~1000 deg/s). */
	res = ICM20608_SetDataFormat(client, ICM20608_G_BW_177HZ);

	res = ICM20608_SetFullScale(client, ICM20608_DEFAULT_FS);
	if (res != ICM20608_SUCCESS)
		return res;

	/* Set 125HZ sample rate */
	res = ICM20608_SetSampleRate(client, 200);
	if (res != ICM20608_SUCCESS)
		return res;

    /* put icm20608 into gyro cycel mode for power save */
    res = ICM20608_SetGyroCycleMode(client, 1);
    if (res != ICM20608_SUCCESS)
        return res;

	res = ICM20608_SetPowerMode(client, sensor_power_org);
	if (res != ICM20608_SUCCESS)
		return res;


	GYRO_LOG("icm20608_init_client OK!\n");

#ifdef CONFIG_ICM20608_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif

	return ICM20608_SUCCESS;
}

/*----------------------------------------------------------------------------*/
int icm20608_operate(void *self, uint32_t command, void *buff_in, int size_in,
		    void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct icm20608_i2c_data *priv = (struct icm20608_i2c_data *)self;
	struct hwm_sensor_data *gyro_data;
	char buff[ICM20608_BUFSIZE];

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			GYRO_ERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {

		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			GYRO_ERR("Enable gyroscope parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (((value == 0) && (sensor_power == false))
			    || ((value == 1) && (sensor_power == true))) {
				GYRO_LOG("gyroscope device have updated!\n");
			} else {
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
				err = ICM20608gy_SCP_SetPowerMode(!sensor_power, ID_GYROSCOPE);
#else
				err = ICM20608_SetPowerMode(priv->client, !sensor_power);
#endif
			}
#if INV_GYRO_AUTO_CALI == 1
			inv_gyro_power_state = sensor_power;
			/* put this in where gyro power is changed, waking up mpu daemon */
			sysfs_notify(&inv_daemon_device->kobj, NULL, "inv_gyro_power_state");
#endif
		}
		break;

	case SENSOR_GET_DATA:
	if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			GYRO_ERR("get gyroscope data parameter error!\n");
			err = -EINVAL;
		} else {
			gyro_data = (struct hwm_sensor_data *) buff_out;
			err = ICM20608_ReadGyroData(priv->client, buff, ICM20608_BUFSIZE);
			if (!err) {
				err = sscanf(buff, "%x %x %x", &gyro_data->values[0],
					&gyro_data->values[1], &gyro_data->values[2]);
				gyro_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				gyro_data->value_divide = DEGREE_TO_RAD;
#if INV_GYRO_AUTO_CALI == 1
				/* put this in where gyro data is ready to report to hal, waking up mpu daemon */
				sysfs_notify(&inv_daemon_device->kobj, NULL, "inv_gyro_data_ready");
#endif
			}
		}
		break;
	default:
		GYRO_ERR("gyroscope operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
static void gyroscope_irq_work(struct work_struct *work)
{
	struct icm20608_i2c_data *obj = obj_i2c_data;
	struct scp_gyro_hw scp_hw;
	union ICM20608GY_CUST_DATA *p_cust_data;
	SCP_SENSOR_HUB_DATA data;
	int max_cust_data_size_per_packet;
	int i;
	uint sizeOfCustData;
	uint len;
	char *p = (char *)&scp_hw;

	GYRO_FUN();

	scp_hw.i2c_num = obj->hw->i2c_num;
	scp_hw.direction = obj->hw->direction;
	scp_hw.power_id = obj->hw->power_id;
	scp_hw.power_vol = obj->hw->power_vol;
	scp_hw.firlen = obj->hw->firlen;
	memcpy(scp_hw.i2c_addr, obj->hw->i2c_addr, sizeof(obj->hw->i2c_addr));
	scp_hw.power_vio_id = obj->hw->power_vio_id;
	scp_hw.power_vio_vol = obj->hw->power_vio_vol;
	scp_hw.is_batch_supported = obj->hw->is_batch_supported;

	p_cust_data = (union ICM20608GY_CUST_DATA *) data.set_cust_req.custData;
	sizeOfCustData = sizeof(scp_hw);
	max_cust_data_size_per_packet =
		sizeof(data.set_cust_req.custData) - offsetof(struct ICM20608GY_SET_CUST, data);

	/*GYRO_ERR("sizeOfCustData = %d, max_cust_data_size_per_packet = %d\n", sizeOfCustData,
		max_cust_data_size_per_packet);
	GYRO_ERR("offset %lu\n", offsetof(struct ICM20608GY_SET_CUST, data));*/

	for (i = 0; sizeOfCustData > 0; i++) {
		data.set_cust_req.sensorType = ID_GYROSCOPE;
		data.set_cust_req.action = SENSOR_HUB_SET_CUST;
		p_cust_data->setCust.action = ICM20608GY_CUST_ACTION_SET_CUST;
		p_cust_data->setCust.part = i;
		if (sizeOfCustData > max_cust_data_size_per_packet)
			len = max_cust_data_size_per_packet;
		else
			len = sizeOfCustData;

		memcpy(p_cust_data->setCust.data, p, len);
		sizeOfCustData -= len;
		p += len;

		/*GYRO_ERR("i= %d, sizeOfCustData = %d, len = %d\n", i, sizeOfCustData, len);*/
		len +=
			offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + offsetof(struct ICM20608GY_SET_CUST,
										   data);
		/*GYRO_ERR("data.set_cust_req.sensorType= %d\n", data.set_cust_req.sensorType);*/
		SCP_sensorHub_req_send(&data, &len, 1);

	}
	p_cust_data = (union ICM20608GY_CUST_DATA *) &data.set_cust_req.custData;
	data.set_cust_req.sensorType = ID_GYROSCOPE;
	data.set_cust_req.action = SENSOR_HUB_SET_CUST;
	p_cust_data->resetCali.action = ICM20608GY_CUST_ACTION_RESET_CALI;
	len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(p_cust_data->resetCali);
	SCP_sensorHub_req_send(&data, &len, 1);
	obj->SCP_init_done = 1;
}

/*----------------------------------------------------------------------------*/
static int gyroscope_irq_handler(void *data, uint len)
{
	struct icm20608_i2c_data *obj = obj_i2c_data;
	SCP_SENSOR_HUB_DATA_P rsp = (SCP_SENSOR_HUB_DATA_P) data;

	/*GYRO_ERR("gsensor_irq_handler len = %d, type = %d, action = %d, errCode = %d\n", len,
		rsp->rsp.sensorType, rsp->rsp.action, rsp->rsp.errCode);*/
	if (!obj)
		return -1;

	switch (rsp->rsp.action) {
	case SENSOR_HUB_NOTIFY:
		switch (rsp->notify_rsp.event) {
		case SCP_INIT_DONE:
			schedule_work(&obj->irq_work);
			/*GYRO_ERR("OK sensor hub notify\n");*/
			break;
		default:
			GYRO_ERR("Error sensor hub notify\n");
			break;
		}
		break;
	default:
		GYRO_ERR("Error sensor hub action\n");
		break;
	}

	return 0;
}

static int gyroscope_setup_irq(void)
{
	int err = 0;

	err = SCP_sensorHub_rsp_registration(ID_GYROSCOPE, gyroscope_irq_handler);

	return err;
}
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
/******************************************************************************
 * Function Configuration
******************************************************************************/
static int icm20608_open(struct inode *inode, struct file *file)
{
	file->private_data = icm20608_i2c_client;

	if (file->private_data == NULL) {
		GYRO_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int icm20608_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static long icm20608_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	char strbuf[ICM20608_BUFSIZE] = { 0 };
	s16 *SMTdata;
	void __user *data;
	long err = 0;
	int copy_cnt = 0;
	struct SENSOR_DATA sensor_data;
	int cali[3];
	int smtRes = 0;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));


	if (err) {
		GYRO_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case GYROSCOPE_IOCTL_INIT:
		icm20608_init_client(client, false);
		break;

	case GYROSCOPE_IOCTL_SMT_DATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		SMTdata = kzalloc(sizeof(*SMTdata) * 800, GFP_KERNEL);
		if (SMTdata == NULL) {
			err = -ENOMEM;
			break;
		}
		memset(SMTdata, 0, sizeof(*SMTdata) * 800);
		ICM20608_SMTReadSensorData(client, SMTdata, 800);

		GYRO_LOG("gyroscope read data from kernel OK: SMTdata[0]:%d, copied packet:%zd!\n",
			 SMTdata[0], ((SMTdata[0] * ICM20608_AXES_NUM + 2) * sizeof(s16) + 1));

		smtRes = ICM20608_PROCESS_SMT_DATA(client, SMTdata);
		GYRO_LOG("ioctl smtRes: %d!\n", smtRes);
		copy_cnt = copy_to_user(data, &smtRes, sizeof(smtRes));
		kfree(SMTdata);
		if (copy_cnt) {
			err = -EFAULT;
			GYRO_ERR("copy gyro data to user failed!\n");
		}
		GYRO_LOG("copy gyro data to user OK: %d!\n", copy_cnt);
		break;

	case GYROSCOPE_IOCTL_READ_SENSORDATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		ICM20608_ReadGyroData(client, strbuf, ICM20608_BUFSIZE);
		if (copy_to_user(data, strbuf, sizeof(strbuf))) {
			err = -EFAULT;
			break;
		}
		break;

	case GYROSCOPE_IOCTL_SET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}

		else {
	    cali[ICM20608_AXIS_X] = sensor_data.x * ICM20608_DEFAULT_LSB / ICM20608_FS_MAX_LSB;
	    cali[ICM20608_AXIS_Y] = sensor_data.y * ICM20608_DEFAULT_LSB / ICM20608_FS_MAX_LSB;
	    cali[ICM20608_AXIS_Z] = sensor_data.z * ICM20608_DEFAULT_LSB / ICM20608_FS_MAX_LSB;
			GYRO_LOG("gyro set cali:[%5d %5d %5d]\n",
			 cali[ICM20608_AXIS_X], cali[ICM20608_AXIS_Y], cali[ICM20608_AXIS_Z]);
	    err = ICM20608_WriteCalibration(client, cali);
		}
		break;

	case GYROSCOPE_IOCTL_CLR_CALI:
		err = ICM20608_ResetCalibration(client);
		break;

	case GYROSCOPE_IOCTL_GET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		err = ICM20608_ReadCalibration(client, cali);
		if (err)
			break;


		sensor_data.x = cali[ICM20608_AXIS_X] * ICM20608_FS_MAX_LSB / ICM20608_DEFAULT_LSB;
		sensor_data.y = cali[ICM20608_AXIS_Y] * ICM20608_FS_MAX_LSB / ICM20608_DEFAULT_LSB;
		sensor_data.z = cali[ICM20608_AXIS_Z] * ICM20608_FS_MAX_LSB / ICM20608_DEFAULT_LSB;
		if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
		break;

#if INV_GYRO_AUTO_CALI == 1
	case GYROSCOPE_IOCTL_READ_SENSORDATA_RAW:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		ICM20608_ReadGyroDataRaw(client, strbuf, ICM20608_BUFSIZE);
		if (copy_to_user(data, strbuf, sizeof(strbuf))) {
			err = -EFAULT;
			break;
		}
		break;

	case GYROSCOPE_IOCTL_READ_TEMPERATURE:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		ICM20608_ReadTemperature(client, strbuf, ICM20608_BUFSIZE);
		if (copy_to_user(data, strbuf, sizeof(strbuf))) {
			err = -EFAULT;
			break;
		}
		break;

	case GYROSCOPE_IOCTL_GET_POWER_STATUS:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		ICM20608_ReadPowerStatus(client, strbuf, ICM20608_BUFSIZE);
		if (copy_to_user(data, strbuf, sizeof(strbuf))) {
			err = -EFAULT;
			break;
		}
		break;
#endif

	default:
		GYRO_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}
	return err;
}

#ifdef CONFIG_COMPAT
static long icm20608_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret;

	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

    /* printk("akm8963_compat_ioctl arg: 0x%lx, arg32: 0x%p\n",arg, arg32); */

	switch (cmd) {
	case COMPAT_GYROSCOPE_IOCTL_INIT:
		/* printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_WRITE\n"); */
		if (arg32 == NULL) {
			GYRO_ERR("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_INIT,
			(unsigned long)arg32);
		if (ret) {
			GYRO_ERR("GYROSCOPE_IOCTL_INIT unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_GYROSCOPE_IOCTL_SET_CALI:
		if (arg32 == NULL) {
			GYRO_ERR("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_SET_CALI,
			(unsigned long)arg32);
		if (ret) {
			GYRO_ERR("GYROSCOPE_IOCTL_SET_CALI unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_GYROSCOPE_IOCTL_CLR_CALI:
		if (arg32 == NULL) {
			GYRO_ERR("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_CLR_CALI,
			(unsigned long)arg32);
		if (ret) {
			GYRO_ERR("GYROSCOPE_IOCTL_CLR_CALI unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_GYROSCOPE_IOCTL_GET_CALI:
		if (arg32 == NULL) {
			GYRO_ERR("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_GET_CALI,
			(unsigned long)arg32);
		if (ret) {
			GYRO_ERR("GYROSCOPE_IOCTL_GET_CALI unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_GYROSCOPE_IOCTL_READ_SENSORDATA:
		if (arg32 == NULL) {
			GYRO_ERR("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_READ_SENSORDATA,
			(unsigned long)arg32);
		if (ret) {
			GYRO_ERR("GYROSCOPE_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
			return ret;
		}

		break;

	default:
		pr_debug("%s not supported = 0x%04x", __func__, cmd);
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}
#endif
/*----------------------------------------------------------------------------*/
static const struct file_operations icm20608_fops = {
	.open = icm20608_open,
	.release = icm20608_release,
	.unlocked_ioctl = icm20608_unlocked_ioctl,
#ifdef CONFIG_COMPAT
			.compat_ioctl = icm20608_compat_ioctl,
#endif
};

/*----------------------------------------------------------------------------*/
static struct miscdevice icm20608_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gyroscope",
	.fops = &icm20608_fops,
};

/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int icm20608_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	GYRO_FUN();

	if (msg.event == PM_EVENT_SUSPEND) {
		if (obj == NULL) {
			GYRO_ERR("null pointer!!\n");
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
		err = ICM20608gy_SCP_SetPowerMode(false, ID_GYROSCOPE);
#else
		err = ICM20608_SetPowerMode(client, false);
#endif
		if (err <= 0)
			return err;

#if INV_GYRO_AUTO_CALI == 1
		inv_gyro_power_state = sensor_power;
		/* inv_gyro_power_state = 0; */
		/* put this in where gyro power is changed, waking up mpu daemon */
		sysfs_notify(&inv_daemon_device->kobj, NULL, "inv_gyro_power_state");
#endif

	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int icm20608_resume(struct i2c_client *client)
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);
	int err;

	GYRO_FUN();

	if (obj == NULL) {
		GYRO_ERR("null pointer!!\n");
		return -EINVAL;
	}

#ifndef CONFIG_CUSTOM_KERNEL_SENSORHUB
	ICM20608_power(obj->hw, 1);
	err = icm20608_init_client(client, false);
#else
	err = ICM20608_SetPowerMode(client, inv_gyro_power_state);
#endif
	if (err) {
		GYRO_ERR("initialize client fail!!\n");
		return err;
	}
	atomic_set(&obj->suspend, 0);

	return 0;
}

/*----------------------------------------------------------------------------*/
#else				/*CONFIG_HAS_EARLY_SUSPEND is defined */
/*----------------------------------------------------------------------------*/
static void icm20608_early_suspend(struct early_suspend *h)
{
	struct icm20608_i2c_data *obj = container_of(h, struct icm20608_i2c_data, early_drv);
	int err;
	/* u8 databuf[2]; */

	GYRO_FUN();

	if (obj == NULL) {
		GYRO_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1);
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	err = ICM20608gy_SCP_SetPowerMode(false, ID_GYROSCOPE);
#else
	err = ICM20608_SetPowerMode(obj->client, false);
#endif
	if (err) {
		GYRO_ERR("write power control fail!!\n");
		return;
	}

	sensor_power = false;

	ICM20608_power(obj->hw, 0);

#if INV_GYRO_AUTO_CALI == 1
	inv_gyro_power_state = sensor_power;
	/* inv_gyro_power_state = 0; */
	/* put this in where gyro power is changed, waking up mpu daemon */
	sysfs_notify(&inv_daemon_device->kobj, NULL, "inv_gyro_power_state");
#endif

}

/*----------------------------------------------------------------------------*/
static void icm20608_late_resume(struct early_suspend *h)
{
	struct icm20608_i2c_data *obj = container_of(h, struct icm20608_i2c_data, early_drv);
	int err;

	GYRO_FUN();

	if (obj == NULL) {
		GYRO_ERR("null pointer!!\n");
		return;
	}

	ICM20608_power(obj->hw, 1);
	err = icm20608_init_client(obj->client, false);
	if (err) {
		GYRO_ERR("initialize client fail! err code %d!\n", err);
		return;
	}
	atomic_set(&obj->suspend, 0);

}

/*----------------------------------------------------------------------------*/
#endif				/*CONFIG_HAS_EARLYSUSPEND */
/*----------------------------------------------------------------------------*/
static int icm20608_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, ICM20608_DEV_NAME);
	return 0;
}


/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int icm20608_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */

static int icm20608_enable_nodata(int en)
{
	int res = 0;
	int retry = 0;
	bool power = false;

	if (1 == en)
		power = true;

	if (0 == en)
		power = false;


	for (retry = 0; retry < 3; retry++) {
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
		res = ICM20608gy_SCP_SetPowerMode(power, ID_GYROSCOPE);
#else
		res = ICM20608_SetPowerMode(obj_i2c_data->client, power);
#endif
		if (res == 0) {
			GYRO_LOG("ICM20608_SetPowerMode done\n");
			break;
		}
		GYRO_LOG("ICM20608_SetPowerMode fail\n");
	}


	if (res != ICM20608_SUCCESS) {
		GYRO_LOG("ICM20608_SetPowerMode fail!\n");
		return -1;
	}
	GYRO_LOG("icm20608_enable_nodata OK!\n");
	return 0;

}

static int icm20608_set_delay(u64 ns)
{
	/*original function body is empty, so do nothing in sensorhub*/

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	int err = 0;
	int value;
	SCP_SENSOR_HUB_DATA req;
	int len;
	value = (int)ns/1000/1000;
	req.set_delay_req.sensorType = ID_GYROSCOPE;
	req.set_delay_req.action = SENSOR_HUB_SET_DELAY;
	req.set_delay_req.delay = value;
	len = sizeof(req.activate_req);
	err = SCP_sensorHub_req_send(&req, &len, 1);
	if (err) {
		GYRO_ERR("SCP_sensorHub_req_send!\n");
		return err;
	}
#else
#endif			/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

	return 0;
}

static int icm20608_get_data(int *x , int *y, int *z, int *status)
{
	char buff[ICM20608_BUFSIZE];
	int ret;

	ret = ICM20608_ReadGyroData(obj_i2c_data->client, buff, ICM20608_BUFSIZE);
	if (ret) {
		GYRO_ERR("ICM20608_ReadGyroData fail!\n");
		return ret;
	}
	ret = sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}


/*----------------------------------------------------------------------------*/
static int icm20608_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct icm20608_i2c_data *obj;
	int err = 0;
	struct gyro_control_path ctl = {0};
	struct gyro_data_path data = {0};

	GYRO_FUN();

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!(obj)) {
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(struct icm20608_i2c_data));

	obj->hw = hw;
	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
	if (err) {
		GYRO_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	GYRO_LOG("gyro_default_i2c_addr: %x\n", client->addr);
#ifdef ICM20608_ACCESS_BY_GSE_I2C
	obj->hw->addr = ICM20608_I2C_SLAVE_ADDR;	/* mtk i2c not allow to probe two same address */
#endif

	GYRO_LOG("gyro_custom_i2c_addr: %x\n", obj->hw->addr);
	if (0 != obj->hw->addr) {
		client->addr = obj->hw->addr >> 1;
		GYRO_LOG("gyro_use_i2c_addr: %x\n", client->addr);
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);


#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	INIT_WORK(&obj->irq_work, gyroscope_irq_work);
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
	icm20608_i2c_client = new_client;
	err = icm20608_init_client(new_client, false);
	if (err)
		goto exit_init_failed;

	err = misc_register(&icm20608_device);
	if (err) {
		GYRO_ERR("icm20608_device misc register failed!\n");
		goto exit_misc_device_register_failed;
	}
	ctl.is_use_common_factory = false;

	err = icm20608_create_attr(&(icm20608_init_info.platform_diver_addr->driver));
	if (err) {
		GYRO_ERR("icm20608 create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}


	ctl.open_report_data = icm20608_open_report_data;
	ctl.enable_nodata = icm20608_enable_nodata;
	ctl.set_delay  = icm20608_set_delay;
	ctl.is_report_input_direct = false;
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	ctl.is_support_batch = obj->hw->is_batch_supported;
#else
	ctl.is_support_batch = false;
#endif

	err = gyro_register_control_path(&ctl);
	if (err) {
		GYRO_ERR("register gyro control path err\n");
		goto exit_kfree;
	}

	data.get_data = icm20608_get_data;
	data.vender_div = DEGREE_TO_RAD;
	err = gyro_register_data_path(&data);
	if (err) {
		GYRO_ERR("gyro_register_data_path fail = %d\n", err);
		goto exit_kfree;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	obj->early_drv.suspend = icm20608_early_suspend,
	obj->early_drv.resume = icm20608_late_resume, register_early_suspend(&obj->early_drv);
#endif

#if INV_GYRO_AUTO_CALI == 1
	mutex_init(&obj->temperature_mutex);
	mutex_init(&obj->raw_data_mutex);
	{
		int i;
		int result;

		/* create a class to avoid event drop by uevent_ops->filter function (dev_uevent_filter()) */
		inv_daemon_class = class_create(THIS_MODULE, INV_DAEMON_CLASS_NAME);
		if (IS_ERR(inv_daemon_class)) {
			GYRO_ERR("cannot create inv daemon class, %s\n", INV_DAEMON_CLASS_NAME);
			goto exit_class_create_failed;
		}
#if 0
		inv_daemon_device = device_create(inv_daemon_class, NULL,
						  MKDEV(MISC_MAJOR, MISC_DYNAMIC_MINOR), NULL,
						  INV_DAEMON_DEVICE_NAME);
		if (IS_ERR(inv_daemon_device)) {
			GYRO_ERR("cannot create inv daemon device, %s\n", INV_DAEMON_DEVICE_NAME);
			goto exit_inv_device_create_failed;
		}
#endif

		inv_daemon_device = kzalloc(sizeof(struct device), GFP_KERNEL);
		if (!inv_daemon_device) {
			GYRO_ERR("cannot allocate inv daemon device, %s\n", INV_DAEMON_DEVICE_NAME);
			goto exit_device_register_failed;
		}
		inv_daemon_device->init_name = INV_DAEMON_DEVICE_NAME;
		inv_daemon_device->class = inv_daemon_class;
		inv_daemon_device->release = (void (*)(struct device *))kfree;
		result = device_register(inv_daemon_device);
		if (result) {
			GYRO_ERR("cannot register inv daemon device, %s\n", INV_DAEMON_DEVICE_NAME);
			goto exit_device_register_failed;
		}

		result = 0;
		for (i = 0; i < ARRAY_SIZE(inv_daemon_dev_attributes); i++) {
			result =
			    device_create_file(inv_daemon_device, inv_daemon_dev_attributes[i]);
			if (result)
				break;
		}
		if (result) {
			while (--i >= 0)
				device_remove_file(inv_daemon_device, inv_daemon_dev_attributes[i]);
			GYRO_ERR("cannot create inv daemon dev attr.\n");
			goto exit_create_file_failed;
		}
	}
#endif
	icm20608_init_flag = 0;

	GYRO_LOG("%s: OK\n", __func__);
	return 0;

#if INV_GYRO_AUTO_CALI == 1
exit_create_file_failed:
	device_unregister(inv_daemon_device);
exit_device_register_failed:
	class_destroy(inv_daemon_class);
exit_class_create_failed:
	hwmsen_detach(ID_GYROSCOPE);
#endif
exit_create_attr_failed:
	misc_deregister(&icm20608_device);
exit_misc_device_register_failed:
exit_init_failed:
	/* i2c_detach_client(new_client); */
exit_kfree:
	kfree(obj);
exit:
	icm20608_init_flag =  -1;
	GYRO_ERR("%s: err = %d\n", __func__, err);
	return err;
}

/*----------------------------------------------------------------------------*/
static int icm20608_i2c_remove(struct i2c_client *client)
{
	int err = 0;

#if INV_GYRO_AUTO_CALI == 1
	{
		int i;

		for (i = 0; i < ARRAY_SIZE(inv_daemon_dev_attributes); i++)
			device_remove_file(inv_daemon_device, inv_daemon_dev_attributes[i]);

		device_unregister(inv_daemon_device);
		class_destroy(inv_daemon_class);
	}
#endif

	err = icm20608_delete_attr(&(icm20608_init_info.platform_diver_addr->driver));
	if (err)
		GYRO_ERR("icm20608_delete_attr fail: %d\n", err);


	err = misc_deregister(&icm20608_device);
	if (err)
		GYRO_ERR("misc_deregister fail: %d\n", err);


	err = hwmsen_detach(ID_GYROSCOPE);
	if (err)
		GYRO_ERR("hwmsen_detach fail: %d\n", err);

	icm20608_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int icm20608_remove(void)
{
	GYRO_FUN();
	ICM20608_power(hw, 0);
	i2c_del_driver(&icm20608_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int icm20608_local_init(struct platform_device *pdev)
{
	ICM20608_power(hw, 1);
	GYRO_ERR("alp.D : icm20608_local_init ++\n");
	if (i2c_add_driver(&icm20608_i2c_driver)) {
		GYRO_ERR("add driver error\n");
		return -1;
	}
	if (-1 == icm20608_init_flag)
		return -1;
	GYRO_ERR("alp.D : icm20608_local_init --\n");

	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init icm20608_init(void)
{
	const char *name = "mediatek,icm20608gy";

	hw = get_gyro_dts_func(name, hw);
	if (!hw)
		GYRO_ERR("get dts info fail\n");

	gyro_driver_add(&icm20608_init_info);
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit icm20608_exit(void)
{
	GYRO_FUN();
}

/*----------------------------------------------------------------------------*/
module_init(icm20608_init);
module_exit(icm20608_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ICM20608 gyroscope driver");
MODULE_AUTHOR("Yucong.Xiong@mediatek.com");

