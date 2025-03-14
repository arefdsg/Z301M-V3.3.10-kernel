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



#include <cust_acc.h>
#include "icm20608.h"
#include <accel.h>
#include <hwmsensor.h>

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

static DEFINE_MUTEX(icm20608_i2c_mutex);
/* Maintain  cust info here */
struct acc_hw accel_cust;
static struct acc_hw *hw = &accel_cust;

/* For  driver get cust info */
struct acc_hw *get_cust_acc(void)
{
	return &accel_cust;
}
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
#define CONFIG_ICM20608_LOWPASS	/*apply low pass filter on output */
#define SW_CALIBRATION
/*----------------------------------------------------------------------------*/
#define ICM20608_AXIS_X          0
#define ICM20608_AXIS_Y          1
#define ICM20608_AXIS_Z          2
#define ICM20608_AXES_NUM        3
#define ICM20608_DATA_LEN        6
#define ICM20608_DEV_NAME        "ICM20608G"	/* name must different with gyro icm20608 */
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id icm20608_i2c_id[] = { {ICM20608_DEV_NAME, 0}, {} };

/*----------------------------------------------------------------------------*/
static int icm20608_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int icm20608_i2c_remove(struct i2c_client *client);
static int icm20608_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#ifndef USE_EARLY_SUSPEND
static int icm20608_suspend(struct i2c_client *client, pm_message_t msg);
static int icm20608_resume(struct i2c_client *client);
#endif

static int icm20608_local_init(void);
static int icm20608_remove(void);
static int icm20608_init_flag = -1; /*0<==>OK -1 <==> fail*/

static struct acc_init_info icm20608_init_info = {
		.name = "icm20608g",
		.init = icm20608_local_init,
		.uninit = icm20608_remove,
};
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
static int gsensor_setup_irq(void);
static DEFINE_MUTEX(gsensor_scp_en_mutex);
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
/*----------------------------------------------------------------------------*/
enum {
	ICM20608_TRC_FILTER = 0x01,
	ICM20608_TRC_RAWDATA = 0x02,
	ICM20608_TRC_IOCTL = 0x04,
	ICM20608_TRC_CALI = 0X08,
	ICM20608_TRC_INFO = 0X10,
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
	struct acc_hw *hw;
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
#if defined(USE_EARLY_SUSPEND)
	struct early_suspend early_drv;
#endif
	u8 bandwidth;
#ifdef ICM20608_DELAY_CALIBRATION
	struct delayed_work delayworkcalibration; 
	atomic_t delaycalibration;   
#endif

};
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
	{.compatible = "mediatek,gsensor"},
	{},
};
#endif
static struct i2c_driver icm20608_i2c_driver = {
	.driver = {
		.name = ICM20608_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = accel_of_match,
#endif
	},
	.probe = icm20608_i2c_probe,
	.remove = icm20608_i2c_remove,
	.detect = icm20608_i2c_detect,
#if !defined(USE_EARLY_SUSPEND)
	.suspend = icm20608_suspend,
	.resume = icm20608_resume,
#endif
	.id_table = icm20608_i2c_id,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *icm20608_i2c_client;
static struct icm20608_i2c_data *obj_i2c_data;
static bool sensor_power;
static struct GSENSOR_VECTOR3D gsensor_gain;
static char selftestRes[8] = { 0 };

/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               pr_debug(GSE_TAG"%s\n", __func__)
#define GSE_ERR(fmt, args...)    pr_err(GSE_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    pr_info(GSE_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/
static struct data_resolution icm20608_data_resolution[] = {
	/*8 combination by {FULL_RES,RANGE} */
	{{0, 6}, 16384},	/*+/-2g  in 16-bit resolution:  0.06 mg/LSB */
	{{0, 12}, 8192},	/*+/-4g  in 16-bit resolution:  0.12 mg/LSB */
	{{0, 24}, 4096},	/*+/-8g  in 16-bit resolution:  0.24 mg/LSB */
	{{0, 5}, 2048},		/*+/-16g in 16-bit resolution:  0.49 mg/LSB */
};
/*----------------------------------------------------------------------------*/
static struct data_resolution icm20608_offset_resolution = { {0, 5}, 2048 };

static unsigned int power_on;
static int calibration_thres = 30;


int ICM20608_gse_power(void)
{
	return power_on;
}
EXPORT_SYMBOL(ICM20608_gse_power);

int ICM20608_gse_mode(void)
{
	return sensor_power;
}
EXPORT_SYMBOL(ICM20608_gse_mode);
/*----------------------------------------------------------------------------*/

// for calibration file added 
#define ICM20608_NVRAM_LENGTH     14
#define GS_CALI_PATH "/persist/gs_cali.ini"
#define PROJ_ID_Z301M 	5				// Z301M
#define HW_ID_ER 		1				// ER device
mm_segment_t old_gs_fs;

/*===========================================================================

 FUNCTION:   read_PROJID

===========================================================================*/
/*
  @brief Add by Tom Cheng for Read PROJECT ID
  @return project ID
*/
/*=========================================================================*/
void read_PROJ_ID(char *proj_id)
{
	   int ilen = 0;
	   struct file *fp = NULL;
	   old_gs_fs = get_fs();
	   set_fs(KERNEL_DS);

	   fp = filp_open("/sys/module/main/parameters/project_id",O_RDONLY, 0);
	   if(IS_ERR(fp)){
		   GSE_ERR("filp_open fail [/sys/module/main/parameters/project_id] \n");
		   return;
	   }

	   ilen = fp->f_op->read(fp,proj_id, 8 ,&fp->f_pos);
	   proj_id[ilen] = 0;
	   set_fs(old_gs_fs);
	   filp_close(fp,NULL);

}
/*===========================================================================

 FUNCTION:   read_HW_ID

==========================================================================*/
/*
 @brief Add by Tom Cheng for Read HW ID
 @return HW ID

=========================================================================*/
void read_HW_ID(char *hw_id)
{
	  int ilen = 0;
	   struct file *fp = NULL;
	   old_gs_fs = get_fs();
	   set_fs(KERNEL_DS);

	   fp = filp_open("/sys/module/main/parameters/hardware_id",O_RDONLY, 0);
	   if(IS_ERR(fp)){
		   GSE_ERR("filp_open fail [/sys/module/main/parameters/hardware_id] \n");
		   return;
	   }

	   ilen = fp->f_op->read(fp,hw_id, 8 ,&fp->f_pos);
	   hw_id[ilen] = 0;
	   set_fs(old_gs_fs);
	   filp_close(fp,NULL);

}

/*=========================================================================*/

static void icm20608_read_from_califile(s16 *calidata); 
static int icm20608_write_to_califile(int raw_x,int raw_y,int raw_z);

static void icm20608_read_from_califile(s16 *calidata) 
{
	int ilen = 0;
	char tmp_data[ICM20608_NVRAM_LENGTH] = {0};
	unsigned int long_raw_x=0, long_raw_y=0, long_raw_z=0;
	int raw_x = 0, raw_y = 0, raw_z = 0, factor=0;
	struct file *fp = NULL;
	struct icm20608_i2c_data *obj = obj_i2c_data;
	/*for ER calibrate error - declare variable*/
	char proj_id_str[7], hw_id_str[7];
	int proj_id, hw_id;
	char *buff;
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA data;
    union ICM20608G_CUST_DATA *pCustData;
    unsigned int len;
#endif

	factor = 65536/(obj->reso->sensitivity);

	/*for ER calibrate error - read project id & hw id*/
	read_PROJ_ID(proj_id_str);
	proj_id = (int)simple_strtol(proj_id_str,&buff,10);
	read_HW_ID(hw_id_str);
	hw_id = (int)simple_strtol(hw_id_str,&buff,10);

	old_gs_fs = get_fs();
	set_fs(KERNEL_DS);
	fp=filp_open(GS_CALI_PATH,O_RDWR, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR(fp)){
		GSE_LOG("filp_open fail [%s] \n", GS_CALI_PATH);
		return;
	}

	ilen = fp->f_op->read(fp,tmp_data,ICM20608_NVRAM_LENGTH,&fp->f_pos);
	if(ilen == ICM20608_NVRAM_LENGTH) {
		long_raw_x = tmp_data[0] |(tmp_data[1]<<8) | (tmp_data[2]<<16) |(tmp_data[3]<<24) ;   
		raw_x = (int)long_raw_x;
		long_raw_y = tmp_data[4] |(tmp_data[5]<<8) | (tmp_data[6]<<16) |(tmp_data[7]<<24) ;   
		raw_y = (int)long_raw_y;
		long_raw_z = tmp_data[8] |(tmp_data[9]<<8) | (tmp_data[10]<<16) |(tmp_data[11]<<24) ;   
		raw_z = (int)long_raw_z;

		/*for ER calibrate error - exchange x y  k-value*/
		if (proj_id == PROJ_ID_Z301M && hw_id <= HW_ID_ER) {
			GSE_LOG("Project ID = %d, HW ID = %d, exchange x,y calib value", proj_id, hw_id);
			calidata[0] =(s16) raw_y/factor;
			calidata[1] =(s16) raw_x/factor;
		}
		else {
			calidata[0] =(s16) raw_x/factor;
			calidata[1] =(s16) raw_y/factor;
		}
		calidata[2] =(s16) raw_z/factor;
		//GSE_LOG("raw_x [%d], raw_y[%d], raw_z[%d] ;  long_raw_x[%d], long_raw_z[%d], long_raw_z[%d] \n", raw_x, raw_y, raw_z, long_raw_x, long_raw_y, long_raw_z);
		GSE_ERR("Load Calibration Data X[%d] , Y[%d] , Z[%d] \n", calidata[0], calidata[1], calidata[2]);

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
		/*just write the calibration data to MD32 at same time*/
		pCustData = (union ICM20608G_CUST_DATA *) data.set_cust_req.custData;
		data.set_cust_req.sensorType = ID_ACCELEROMETER;
		data.set_cust_req.action = SENSOR_HUB_SET_CUST;
		pCustData->setCali.action = ICM20608G_CUST_ACTION_SET_CALI;
		pCustData->setCali.data[ICM20608_AXIS_X] = calidata[ICM20608_AXIS_X];
		pCustData->setCali.data[ICM20608_AXIS_Y] = calidata[ICM20608_AXIS_Y];
		pCustData->setCali.data[ICM20608_AXIS_Z] = calidata[ICM20608_AXIS_Z];
		len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->setCali);
		SCP_sensorHub_req_send(&data, &len, 1);
#endif

	} else {
		calidata[0] = 0 ;
		calidata[1] = 0 ;                                                                                                                                 
		calidata[2] = 0 ;
		GSE_LOG("ilen[%d] != 14 \n", ilen); 
		GSE_LOG("Use Defalut Calibration Data X[%d] , Y[%d] , Z[%d] \n", calidata[0], calidata[1], calidata[2]);
	}

	set_fs(old_gs_fs);
	filp_close(fp,NULL);

}
static int icm20608_write_to_califile(int raw_x,int raw_y,int raw_z)
{
	int retval = 0, ilen=0, iloop=0;
	int tmp_data[ICM20608_NVRAM_LENGTH]={0};
	char nvram_data[ICM20608_NVRAM_LENGTH]={0};
	unsigned int long_x=0,long_y=0,long_z=0;
	int ck=0, flag=0;   //for checknumber
	struct file *fp=NULL;
	int factor = 0;
	struct icm20608_i2c_data *obj = obj_i2c_data;
	factor = 65536/(obj->reso->sensitivity);

	old_gs_fs = get_fs();
	set_fs(KERNEL_DS);
	fp=filp_open(GS_CALI_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR(fp)){
		GSE_LOG("GS filp_open fail\n");
		return retval;
	}

	// long_data = raw_data * (1/2048) *9.80065 * 65536 / 9.80065 ==> long_data = raw_data*32
	// *1/512 -> Normaliate into ? g (for range 4G) ; *1/1024 -> Normaliate into ? g (for range 2G)
	// *9.80065 -> 1 g = 9.80065
	// *65536/9.80065 Normaliate into NVRam 

	long_x = (unsigned int) (raw_x * factor);
	long_y = (unsigned int) (raw_y * factor);
	long_z = (unsigned int) (raw_z * factor);
	//for X into NVRam
	for(iloop=0;iloop<4;iloop++){
		tmp_data[iloop]=long_x%256;
	    long_x = long_x/256;
	}
	//for Y into NVRam
	for(iloop=0;iloop<4;iloop++){
	tmp_data[iloop+4]=long_y%256;
	long_y = long_y/256;
	}
	//for Z into NVRam
	for(iloop=0;iloop<4;iloop++){
	    tmp_data[iloop+8]=long_z%256;
	    long_z = long_z/256;
	}
	tmp_data[12]=0xAA;
	//for checknumber
	for(iloop=0;iloop<12;iloop++){
	    if(flag){
			ck ^= tmp_data[iloop];
	        flag = 0;
        }else{
			ck += tmp_data[iloop];
			flag = 1;
		}
	}
	tmp_data[13]=ck;
	GSE_LOG("check number = %d\n",ck);
	GSE_LOG("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %3x %3x ", tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3], tmp_data[4], tmp_data[5], tmp_data[6], tmp_data[7], tmp_data[8], tmp_data[9], tmp_data[10], tmp_data[11], tmp_data[12], tmp_data[13]);

    //transfor into nvram char
    for(iloop=0;iloop<ICM20608_NVRAM_LENGTH;iloop++){
                nvram_data[iloop] = tmp_data[iloop];
    }
                    
	//sprintf(nvram_data, "%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x", tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3], tmp_data[4], tmp_data[5], tmp_da			ta[6], tmp_data[7], tmp_data[8], tmp_data[9], tmp_data[10], tmp_data[11], tmp_data[12], tmp_data[13]);                                                    
	ilen = fp->f_op->write(fp,nvram_data,ICM20608_NVRAM_LENGTH,&fp->f_pos);
	GSE_LOG("GS write file len = %d\n",ilen);
	set_fs(old_gs_fs);
	filp_close(fp,NULL);

	return retval;

}

/*-----------------------------------------------------------------------------*/

static ssize_t atd_cali(struct device_driver *ddri, char *buf)                                                               
{
	s16 calivalue[3]={0};
	GSE_LOG("atd_cali !!\n");
	icm20608_read_from_califile(calivalue);
	return sprintf(buf, "%d %d %d\n", calivalue[0], calivalue[1], calivalue[2]);
}

/*-----------------------------------------------------------------------------*/


static void icm20608_delay_calibration_func(struct work_struct *work)
{
	struct icm20608_i2c_data *obj = obj_i2c_data;	
	//GSE_LOG("Before read persist_data x[%d] y[%d] z[%d]", obj->cali_sw[0], obj->cali_sw[1], obj->cali_sw[2]);
	icm20608_read_from_califile(obj->cali_sw);
	//printk(KERN_DEBUG "[Gsensor][ICM20608]Read persist_data x[%d] y[%d] z[%d]\n", obj->cali_sw[0], obj->cali_sw[1], obj->cali_sw[2]);
	return ;
}

#define MPU_I2C_RETRY
#define MPU_I2C_RETRY_COUNT		3

/*----------------------------------------------------------------------------*/
static int mpu_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err;
	u8 beg = addr;
	struct i2c_msg msgs[2] = { {0}, {0} };
#ifdef MPU_I2C_RETRY
    int i=0;
#endif

	mutex_lock(&icm20608_i2c_mutex);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;


	if (!client) {
		mutex_unlock(&icm20608_i2c_mutex);
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		mutex_unlock(&icm20608_i2c_mutex);
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

#ifdef MPU_I2C_RETRY
    for (i=0; i<MPU_I2C_RETRY_COUNT; i++) {
        err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
        if (err != 2) {
            GSE_ERR("alp.D : i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
            GSE_ERR("alp.D : i2c_transfer error: retry (%d)\n", i);
            err = -EIO;
            mdelay(10);
        } else {
            err = 0;
            break;
        }
    }
#else
	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) {
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n",
			addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
#endif

	mutex_unlock(&icm20608_i2c_mutex);
	return err;

}

static int mpu_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   /*because address also occupies one byte, the maximum length for write is 7 bytes*/
	int err, idx, num;
	char buf[C_I2C_FIFO_SIZE];
#ifdef MPU_I2C_RETRY
    int i=0;
#endif

	err = 0;
	mutex_lock(&icm20608_i2c_mutex);
	if (!client) {
		mutex_unlock(&icm20608_i2c_mutex);
		return -EINVAL;
	} else if (len >= C_I2C_FIFO_SIZE) {
		mutex_unlock(&icm20608_i2c_mutex);
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

#ifdef MPU_I2C_RETRY
    for (i=0; i<MPU_I2C_RETRY_COUNT; i++) {
        err = i2c_master_send(client, buf, num);
        if (err < 0) {
            GSE_ERR("alp.D : i2c send command error!!\n");
            GSE_ERR("alp.D : i2c_send command: retry (%d)\n", i);
            err = -EFAULT;
            mdelay(10);
        } else {
            //err = 0;
            break;
        }
    }
#else
	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		mutex_unlock(&icm20608_i2c_mutex);
		GSE_ERR("send command error!!\n");
		return -EFAULT;
	}
#endif
	mutex_unlock(&icm20608_i2c_mutex);
	return err;
}

int ICM20608_hwmsen_read_block(u8 addr, u8 *buf, u8 len)
{
	if (NULL == icm20608_i2c_client) {
		GSE_ERR("ICM20608_hwmsen_read_block null ptr!!\n");
		return ICM20608_ERR_I2C;
	}
	return mpu_i2c_read_block(icm20608_i2c_client, addr, buf, len);
}
EXPORT_SYMBOL(ICM20608_hwmsen_read_block);

int ICM20608_hwmsen_write_block(u8 addr, u8 *buf, u8 len)
{
	if (NULL == icm20608_i2c_client) {
		GSE_ERR("ICM20608_hwmsen_write_block null ptr!!\n");
		return ICM20608_ERR_I2C;
	}
	return mpu_i2c_write_block(icm20608_i2c_client, addr, buf, len);
}
EXPORT_SYMBOL(ICM20608_hwmsen_write_block);


/*--------------------icm20608 power control function----------------------------------*/
static void ICM20608_power(struct acc_hw *hw, unsigned int on)
{
}
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
int ICM20608G_SCP_SetPowerMode(bool enable, int sensorType)
{
	static bool gsensor_scp_en_status;
	static unsigned int gsensor_scp_en_map;
	SCP_SENSOR_HUB_DATA req;
	int len;
	int err = 0;


	if (enable == sensor_power) {
		GSE_LOG("Sensor power status is newest!\n");
		return ICM20608_SUCCESS;
	}
	mutex_lock(&gsensor_scp_en_mutex);

	if (sensorType >= 32) {
		GSE_ERR("Out of index!\n");
		return -1;
	}

	if (true == enable)
		gsensor_scp_en_map |= (1 << sensorType);
	else
		gsensor_scp_en_map &= ~(1 << sensorType);

	if (0 == gsensor_scp_en_map)
		enable = false;
	else
		enable = true;

	if (gsensor_scp_en_status != enable) {
		gsensor_scp_en_status = enable;

		req.activate_req.sensorType = ID_ACCELEROMETER;
		req.activate_req.action = SENSOR_HUB_ACTIVATE;
		req.activate_req.enable = enable;
		len = sizeof(req.activate_req);
		err = SCP_sensorHub_req_send(&req, &len, 1);
		if (err)
			GSE_ERR("SCP_sensorHub_req_send fail\n");
	}

	mutex_unlock(&gsensor_scp_en_mutex);

    if (enable == true)
		msleep(50);

    sensor_power = enable;
    return err;
}
EXPORT_SYMBOL(ICM20608G_SCP_SetPowerMode);
#endif
/*----------------------------------------------------------------------------*/
static int ICM20608_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];
	int res = 0;
	/* u8 addr = ICM20608_REG_PWR_MGMT_1; */
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);


	if (enable == sensor_power) {
		GSE_LOG("Sensor power status is newest!\n");
		return ICM20608_SUCCESS;
	}

#if 0
	res = mpu_i2c_read_block(client, ICM20608_REG_PWR_MGMT_1, databuf, 0x1);
	if (res < 0)
		return ICM20608_ERR_I2C;
#endif

    databuf[0] = BIT_CLK_PLL;
    res = mpu_i2c_write_block(client, ICM20608_REG_PWR_MGMT_1, databuf, 0x1);
    if (res < 0) {
        GSE_ERR("set power mode failed!\n");
        return ICM20608_ERR_I2C;
    }


    //set ICM20608_REG_PWR_MGMT_2
    databuf[0] = 0;
//    if (ICM20608_gyro_mode() == false)
//        databuf[0] |= BIT_STBY_G;
    if (enable == false)
        databuf[0] |= BIT_STBY_A;
    res = mpu_i2c_write_block(client, ICM20608_REG_PWR_MGMT_2, databuf, 0x1);
    if (res < 0) {
        GSE_ERR("set power mode failed!\n");
        return ICM20608_ERR_I2C;
    }


    //set ICM20608_REG_PWR_MGMT_1
	databuf[0] = BIT_CLK_PLL;
	if (enable == false) {
//		if (ICM20608_gyro_mode() == false)
//			databuf[0] |= BIT_SLEEP;
	} else {
		/* do nothing */
	}
	res = mpu_i2c_write_block(client, ICM20608_REG_PWR_MGMT_1, databuf, 0x1);
	if (res < 0) {
		GSE_ERR("set power mode failed!\n");
		return ICM20608_ERR_I2C;
	} else if (atomic_read(&obj->trace) & ICM20608_TRC_INFO)
		GSE_ERR("set power mode ok %d!\n", databuf[0]);

	if (enable == true)
		msleep(50);

	sensor_power = enable;

	return ICM20608_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_SetDataResolution(struct icm20608_i2c_data *obj)
{
	int err;
	u8 dat, reso;

	err = mpu_i2c_read_block(obj->client, ICM20608_REG_ACCEL_CONFIG, &dat, 1);
	if (err) {
		GSE_ERR("write data format fail!!\n");
		return err;
	}

	/*the data_reso is combined by 3 bits: {FULL_RES, DATA_RANGE} */
	reso = 0x00;
	reso = (dat & ACCEL_FS_16G) >> SHIFT_ACCEL_FS_SEL;

	if (reso < sizeof(icm20608_data_resolution) / sizeof(icm20608_data_resolution[0])) {
		obj->reso = &icm20608_data_resolution[reso];
		return 0;
	} else {
		return -EINVAL;
	}
}

/*----------------------------------------------------------------------------*/
static int ICM20608_ReadData(struct i2c_client *client, s16 data[ICM20608_AXES_NUM])
{
	struct icm20608_i2c_data *priv = i2c_get_clientdata(client);
	u8 buf[ICM20608_DATA_LEN] = { 0 };
	int err = 0;


	if (NULL == client)
		return -EINVAL;


	{
		/* write then burst read */
		mpu_i2c_read_block(client, ICM20608_REG_ACCEL_XOUT_H, buf, ICM20608_DATA_LEN);

		data[ICM20608_AXIS_X] = (s16) ((buf[ICM20608_AXIS_X * 2] << 8) |
					      (buf[ICM20608_AXIS_X * 2 + 1]));
		data[ICM20608_AXIS_Y] = (s16) ((buf[ICM20608_AXIS_Y * 2] << 8) |
					      (buf[ICM20608_AXIS_Y * 2 + 1]));
		data[ICM20608_AXIS_Z] = (s16) ((buf[ICM20608_AXIS_Z * 2] << 8) |
					      (buf[ICM20608_AXIS_Z * 2 + 1]));

		if (atomic_read(&priv->trace) & ICM20608_TRC_RAWDATA) {
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[ICM20608_AXIS_X],
				data[ICM20608_AXIS_Y], data[ICM20608_AXIS_Z], data[ICM20608_AXIS_X],
				data[ICM20608_AXIS_Y], data[ICM20608_AXIS_Z]);
		}
#ifdef CONFIG_ICM20608_LOWPASS
		if (atomic_read(&priv->filter)) {
			if (atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend)) {
				int idx, firlen = atomic_read(&priv->firlen);

				if (priv->fir.num < firlen) {
					priv->fir.raw[priv->fir.num][ICM20608_AXIS_X] = data[ICM20608_AXIS_X];
					priv->fir.raw[priv->fir.num][ICM20608_AXIS_Y] = data[ICM20608_AXIS_Y];
					priv->fir.raw[priv->fir.num][ICM20608_AXIS_Z] = data[ICM20608_AXIS_Z];
					priv->fir.sum[ICM20608_AXIS_X] += data[ICM20608_AXIS_X];
					priv->fir.sum[ICM20608_AXIS_Y] += data[ICM20608_AXIS_Y];
					priv->fir.sum[ICM20608_AXIS_Z] += data[ICM20608_AXIS_Z];
					if (atomic_read(&priv->trace) & ICM20608_TRC_FILTER) {
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
						priv->fir.raw[priv->fir.num][ICM20608_AXIS_X],
						priv->fir.raw[priv->fir.num][ICM20608_AXIS_Y],
						priv->fir.raw[priv->fir.num][ICM20608_AXIS_Z],
						priv->fir.sum[ICM20608_AXIS_X], priv->fir.sum[ICM20608_AXIS_Y],
						priv->fir.sum[ICM20608_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				} else {
					idx = priv->fir.idx % firlen;
					priv->fir.sum[ICM20608_AXIS_X] -= priv->fir.raw[idx][ICM20608_AXIS_X];
					priv->fir.sum[ICM20608_AXIS_Y] -= priv->fir.raw[idx][ICM20608_AXIS_Y];
					priv->fir.sum[ICM20608_AXIS_Z] -= priv->fir.raw[idx][ICM20608_AXIS_Z];
					priv->fir.raw[idx][ICM20608_AXIS_X] = data[ICM20608_AXIS_X];
					priv->fir.raw[idx][ICM20608_AXIS_Y] = data[ICM20608_AXIS_Y];
					priv->fir.raw[idx][ICM20608_AXIS_Z] = data[ICM20608_AXIS_Z];
					priv->fir.sum[ICM20608_AXIS_X] += data[ICM20608_AXIS_X];
					priv->fir.sum[ICM20608_AXIS_Y] += data[ICM20608_AXIS_Y];
					priv->fir.sum[ICM20608_AXIS_Z] += data[ICM20608_AXIS_Z];
					priv->fir.idx++;
					data[ICM20608_AXIS_X] = priv->fir.sum[ICM20608_AXIS_X]/firlen;
					data[ICM20608_AXIS_Y] = priv->fir.sum[ICM20608_AXIS_Y]/firlen;
					data[ICM20608_AXIS_Z] = priv->fir.sum[ICM20608_AXIS_Z]/firlen;
					if (atomic_read(&priv->trace) & ICM20608_TRC_FILTER)
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n",
						idx,
						priv->fir.raw[idx][ICM20608_AXIS_X], priv->fir.raw[idx][ICM20608_AXIS_Y],
						priv->fir.raw[idx][ICM20608_AXIS_Z], priv->fir.sum[ICM20608_AXIS_X],
						priv->fir.sum[ICM20608_AXIS_Y], priv->fir.sum[ICM20608_AXIS_Z],
						data[ICM20608_AXIS_X], data[ICM20608_AXIS_Y], data[ICM20608_AXIS_Z]);
				}
			}
		}
#endif
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_ReadOffset(struct i2c_client *client, s8 ofs[ICM20608_AXES_NUM])
{
	int err = 0;
#ifdef SW_CALIBRATION
	ofs[0] = ofs[1] = ofs[2] = 0x0;
#else
//	err = mpu_i2c_read_block(client, ICM20608_REG_OFSX, ofs, ICM20608_AXES_NUM);
//	if (err)
//		GSE_ERR("error: %d\n", err);
#endif
	/* GSE_LOG("offesx=%x, y=%x, z=%x",ofs[0],ofs[1],ofs[2]); */

	return err;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_ResetCalibration(struct i2c_client *client)
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);
#ifndef SW_CALIBRATION
	s8 ofs[ICM20608_AXES_NUM] = { 0x00, 0x00, 0x00 };
#endif
	int err = 0;
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA data;
	union ICM20608G_CUST_DATA *pCustData;
	unsigned int len;
#endif

#ifdef SW_CALIBRATION
	/* do not thing */
	/*just clear calibration buffer in MD32*/
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	if (0 != obj->SCP_init_done) {
		pCustData = (union ICM20608G_CUST_DATA *) &data.set_cust_req.custData;
		data.set_cust_req.sensorType = ID_ACCELEROMETER;
		data.set_cust_req.action = SENSOR_HUB_SET_CUST;
		pCustData->resetCali.action = ICM20608G_CUST_ACTION_RESET_CALI;
		len =
			offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->resetCali);
		SCP_sensorHub_req_send(&data, &len, 1);
	}
#endif
#else
//	err = hwmsen_write_block(client, ICM20608_REG_OFSX, ofs, ICM20608_AXES_NUM);
//	if (err)
//		GSE_ERR("error: %d\n", err);
#endif

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));

	return err;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_ReadCalibration(struct i2c_client *client, int dat[ICM20608_AXES_NUM])
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);
#ifdef SW_CALIBRATION
	int mul;
#else
	int err;
#endif
#ifdef SW_CALIBRATION
	mul = 0;		/* only SW Calibration, disable HW Calibration */
#else

	err = ICM20608_ReadOffset(client, obj->offset);
	if (err) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
	mul = obj->reso->sensitivity / icm20608_offset_resolution.sensitivity;
#endif
#ifdef DEBUG	
	GSE_ERR("icm20608 Read Calibration\n");
#endif
	dat[obj->cvt.map[ICM20608_AXIS_X]] =
		obj->cvt.sign[ICM20608_AXIS_X]*(obj->offset[ICM20608_AXIS_X]*mul + obj->cali_sw[ICM20608_AXIS_X]);
	dat[obj->cvt.map[ICM20608_AXIS_Y]] =
		obj->cvt.sign[ICM20608_AXIS_Y]*(obj->offset[ICM20608_AXIS_Y]*mul + obj->cali_sw[ICM20608_AXIS_Y]);
	dat[obj->cvt.map[ICM20608_AXIS_Z]] =
		obj->cvt.sign[ICM20608_AXIS_Z]*(obj->offset[ICM20608_AXIS_Z]*mul + obj->cali_sw[ICM20608_AXIS_Z]);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int ICM20608_ReadCalibrationEx(struct i2c_client *client, int act[ICM20608_AXES_NUM],
				     int raw[ICM20608_AXES_NUM])
{
	/*raw: the raw calibration data; act: the actual calibration data */
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);
#ifdef SW_CALIBRATION
	int mul;
#else
	int err;
#endif
#ifdef SW_CALIBRATION
	mul = 0;		/* only SW Calibration, disable HW Calibration */
#else

	err = ICM20608_ReadOffset(client, obj->offset);
	if (err) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
	mul = obj->reso->sensitivity / icm20608_offset_resolution.sensitivity;
#endif

	raw[ICM20608_AXIS_X] = obj->offset[ICM20608_AXIS_X]*mul + obj->cali_sw[ICM20608_AXIS_X];
	raw[ICM20608_AXIS_Y] = obj->offset[ICM20608_AXIS_Y]*mul + obj->cali_sw[ICM20608_AXIS_Y];
	raw[ICM20608_AXIS_Z] = obj->offset[ICM20608_AXIS_Z]*mul + obj->cali_sw[ICM20608_AXIS_Z];

	act[obj->cvt.map[ICM20608_AXIS_X]] = /*obj->cvt.sign[ICM20608_AXIS_X] * */raw[ICM20608_AXIS_X];
	act[obj->cvt.map[ICM20608_AXIS_Y]] = /*obj->cvt.sign[ICM20608_AXIS_Y] * */raw[ICM20608_AXIS_Y];
	act[obj->cvt.map[ICM20608_AXIS_Z]] = /*obj->cvt.sign[ICM20608_AXIS_Z] * */raw[ICM20608_AXIS_Z];

	return 0;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_WriteCalibration(struct i2c_client *client, int dat[ICM20608_AXES_NUM])
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int cali[ICM20608_AXES_NUM], raw[ICM20608_AXES_NUM];
#ifndef SW_CALIBRATION
	int lsb = icm20608_offset_resolution.sensitivity;
	int divisor = obj->reso->sensitivity / lsb;
#endif
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA data;
	union ICM20608G_CUST_DATA *pCustData;
	unsigned int len;
#endif

	err = ICM20608_ReadCalibrationEx(client, cali, raw);
	if (err) {	/*offset will be updated in obj->offset */
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
		raw[ICM20608_AXIS_X], raw[ICM20608_AXIS_Y], raw[ICM20608_AXIS_Z],
		obj->offset[ICM20608_AXIS_X], obj->offset[ICM20608_AXIS_Y], obj->offset[ICM20608_AXIS_Z],
		obj->cali_sw[ICM20608_AXIS_X], obj->cali_sw[ICM20608_AXIS_Y], obj->cali_sw[ICM20608_AXIS_Z]);

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	/*just write the calibration data to MD32 at same time*/
	pCustData = (union ICM20608G_CUST_DATA *) data.set_cust_req.custData;
	data.set_cust_req.sensorType = ID_ACCELEROMETER;
	data.set_cust_req.action = SENSOR_HUB_SET_CUST;
	pCustData->setCali.action = ICM20608G_CUST_ACTION_SET_CALI;
	pCustData->setCali.data[ICM20608_AXIS_X] = dat[ICM20608_AXIS_X];
	pCustData->setCali.data[ICM20608_AXIS_Y] = dat[ICM20608_AXIS_Y];
	pCustData->setCali.data[ICM20608_AXIS_Z] = dat[ICM20608_AXIS_Z];
	len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->setCali);
	SCP_sensorHub_req_send(&data, &len, 1);
#endif

	/*calculate the real offset expected by caller */
	cali[ICM20608_AXIS_X] += dat[ICM20608_AXIS_X];
	cali[ICM20608_AXIS_Y] += dat[ICM20608_AXIS_Y];
	cali[ICM20608_AXIS_Z] += dat[ICM20608_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n",
		dat[ICM20608_AXIS_X], dat[ICM20608_AXIS_Y], dat[ICM20608_AXIS_Z]);
#ifdef SW_CALIBRATION
	obj->cali_sw[ICM20608_AXIS_X] = /*obj->cvt.sign[ICM20608_AXIS_X]* */(cali[obj->cvt.map[ICM20608_AXIS_X]]);
	obj->cali_sw[ICM20608_AXIS_Y] = /*obj->cvt.sign[ICM20608_AXIS_Y]* */(cali[obj->cvt.map[ICM20608_AXIS_Y]]);
	obj->cali_sw[ICM20608_AXIS_Z] = /*obj->cvt.sign[ICM20608_AXIS_Z]* */(cali[obj->cvt.map[ICM20608_AXIS_Z]]);
	printk(KERN_DEBUG "[Gsensor][ICM20608]WRITE NEW CAlI: (%+3d %+3d %+3d)\n",obj->cali_sw[ICM20608_AXIS_X], obj->cali_sw[ICM20608_AXIS_Y], obj->cali_sw[ICM20608_AXIS_Z]);
#else
	obj->offset[ICM20608_AXIS_X] =
		(s8)(obj->cvt.sign[ICM20608_AXIS_X]*(cali[obj->cvt.map[ICM20608_AXIS_X]])/(divisor));
	obj->offset[ICM20608_AXIS_Y] =
		(s8)(obj->cvt.sign[ICM20608_AXIS_Y]*(cali[obj->cvt.map[ICM20608_AXIS_Y]])/(divisor));
	obj->offset[ICM20608_AXIS_Z] =
		(s8)(obj->cvt.sign[ICM20608_AXIS_Z]*(cali[obj->cvt.map[ICM20608_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[ICM20608_AXIS_X] = obj->cvt.sign[ICM20608_AXIS_X]*(cali[obj->cvt.map[ICM20608_AXIS_X]])%(divisor);
	obj->cali_sw[ICM20608_AXIS_Y] = obj->cvt.sign[ICM20608_AXIS_Y]*(cali[obj->cvt.map[ICM20608_AXIS_Y]])%(divisor);
	obj->cali_sw[ICM20608_AXIS_Z] = obj->cvt.sign[ICM20608_AXIS_Z]*(cali[obj->cvt.map[ICM20608_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
		obj->offset[ICM20608_AXIS_X]*divisor + obj->cali_sw[ICM20608_AXIS_X],
		obj->offset[ICM20608_AXIS_Y]*divisor + obj->cali_sw[ICM20608_AXIS_Y],
		obj->offset[ICM20608_AXIS_Z]*divisor + obj->cali_sw[ICM20608_AXIS_Z],
		obj->offset[ICM20608_AXIS_X], obj->offset[ICM20608_AXIS_Y], obj->offset[ICM20608_AXIS_Z],
		obj->cali_sw[ICM20608_AXIS_X], obj->cali_sw[ICM20608_AXIS_Y], obj->cali_sw[ICM20608_AXIS_Z]);

//	err = hwmsen_write_block(obj->client, ICM20608_REG_OFSX, obj->offset, ICM20608_AXES_NUM);
//	if (err) {
//		GSE_ERR("write offset fail: %d\n", err);
//		return err;
//	}
#endif

	return err;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);
	res = mpu_i2c_read_block(client, ICM20608_REG_WHO_AM_I, databuf, 0x1);
	if (res < 0)
		goto exit_ICM20608_CheckDeviceID;

	GSE_LOG("ICM20608_CheckDeviceID 0x%x\n", databuf[0]);
exit_ICM20608_CheckDeviceID:
	if (res < 0)
		return ICM20608_ERR_I2C;

	return ICM20608_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*2);
	res = mpu_i2c_read_block(client, ICM20608_REG_ACCEL_CONFIG, databuf, 0x1);
	if (res < 0)
		return ICM20608_ERR_I2C;

	/* write */
	databuf[0] = databuf[0] | dataformat;
	res = mpu_i2c_write_block(client, ICM20608_REG_ACCEL_CONFIG, databuf, 0x1);

	if (res < 0)
		return ICM20608_ERR_I2C;
	return ICM20608_SetDataResolution(obj);
}

/*----------------------------------------------------------------------------*/
static int ICM20608_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];
	int res = 0;

	if ((obj->bandwidth != bwrate) || (atomic_read(&obj->suspend))) {
		memset(databuf, 0, sizeof(u8)*10);

		/* read */
		res = mpu_i2c_read_block(client, ICM20608_REG_ACCEL_CONFIG2, databuf, 0x1);
		if (res < 0)
			return ICM20608_ERR_I2C;

		/* write */
		databuf[0] = databuf[0] | bwrate;
		res = mpu_i2c_write_block(client, ICM20608_REG_ACCEL_CONFIG2, databuf, 0x1);

		if (res < 0)
			return ICM20608_ERR_I2C;

		obj->bandwidth = bwrate;
	}

	return ICM20608_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_Dev_Reset(struct i2c_client *client)
{
	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);

	/* read */
	res = mpu_i2c_read_block(client, ICM20608_REG_PWR_MGMT_1, databuf, 0x1);
	if (res < 0)
		return ICM20608_ERR_I2C;

	/* write */
	databuf[0] = databuf[0] | BIT_DEVICE_RESET;
	res = mpu_i2c_write_block(client, ICM20608_REG_PWR_MGMT_1, databuf, 0x1);

	if (res < 0)
		return ICM20608_ERR_I2C;

	do {
		res = mpu_i2c_read_block(client, ICM20608_REG_PWR_MGMT_1, databuf, 0x1);
		if (res < 0)
			return ICM20608_ERR_I2C;
		GSE_LOG("[Gsensor] check reset bit");
	} while ((databuf[0]&BIT_DEVICE_RESET) != 0);

	msleep(50);
	return ICM20608_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_Reset(struct i2c_client *client)
{
	return ICM20608_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	u8 databuf[2];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*2);
	databuf[0] = intenable;
	res = mpu_i2c_write_block(client, ICM20608_REG_INT_ENABLE, databuf, 0x1);

	if (res < 0)
		return ICM20608_ERR_I2C;

	return ICM20608_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int icm20608_gpio_config(void)
{
	return 0;
}

static int icm20608_init_client(struct i2c_client *client, int reset_cali)
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0 , iloop=0;
    bool sensor_power_org;

	icm20608_gpio_config();
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	res = gsensor_setup_irq();
	if (res != ICM20608_SUCCESS)
		return res;
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
	sensor_power_org = sensor_power;

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	if (1 != reset_cali) {
		/*power on MD32 other time */
		res = ICM20608G_SCP_SetPowerMode(true, ID_ACCELEROMETER);
		if (res) {
			GSE_ERR("Power on icm20608 MD32 error\n");
		    return res;
		}
	} else 
#endif
	{
        for(iloop=0;iloop<5;iloop++){
			/*power on AP only probe */
		    res = ICM20608_SetPowerMode(client, true);
		    if (res != ICM20608_SUCCESS) {
			    GSE_ERR("alp.D : set power error (%d)\n",iloop);
//			    return res;
		    }else{
				GSE_ERR("alp.D : set power success!!\n");
			    break;
			}
			mdelay(10);
		}
		if (res != ICM20608_SUCCESS){
		    GSE_ERR("alp.D : set power return fail\n");
			return res;
		}
	}
	
	res = ICM20608_CheckDeviceID(client);
	if (res != ICM20608_SUCCESS) {
		GSE_ERR("Check ID error\n");
		return res;
	}

	res = ICM20608_SetBWRate(client, ICM20608_A_BW_235HZ);
	if (res != ICM20608_SUCCESS)	{ /* 0x2C->BW=100Hz */
		GSE_ERR("set power error\n");
		return res;
	}

	res = ICM20608_SetDataFormat(client, ACCEL_FS_16G);
	if (res != ICM20608_SUCCESS)	{ /* 0x2C->BW=100Hz */
		GSE_ERR("set data format error\n");
		return res;
	}

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	res = ICM20608_SetIntEnable(client, 0x00);	/* disable INT */
	if (res != ICM20608_SUCCESS) {
		GSE_ERR("icm20608_SetIntEnable error\n");
		return res;
	}

	if (0 != reset_cali) {
		/*reset calibration only in power on */
		res = ICM20608_ResetCalibration(client);
		if (res != ICM20608_SUCCESS)
			return res;
	}

//    res = MPU6050_SetPowerMode(client, sensor_power_org);
//    if (res != MPU6050_SUCCESS)
//    {
//        GSE_ERR("set power error\n");
//        return res;
//    }

#ifdef CONFIG_ICM20608_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif

	return ICM20608_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_ReadAllReg(struct i2c_client *client, char *buf, int bufsize)
{
	u8 total_len = 0x5C;	/* (0x75-0x19); */

	u8 addr = 0x19;
	u8 buff[total_len + 1];
	int err = 0;
	int i;


	if (sensor_power == false) {
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
		err = ICM20608G_SCP_SetPowerMode(true, ID_ACCELEROMETER);
#else
		err = ICM20608_SetPowerMode(client, true);
#endif
		if (err)
			GSE_ERR("Power on icm20608 error %d!\n", err);
	}

	mpu_i2c_read_block(client, addr, buff, total_len);

	for (i = 0; i <= total_len; i++)
		GSE_LOG("ICM20608 reg=0x%x, data=0x%x\n", (addr + i), buff[i]);


	return 0;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];

	memset(databuf, 0, sizeof(u8) * 10);

	if ((NULL == buf) || (bufsize <= 30))
		return -1;


	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	sprintf(buf, "ICM20608 Chip");
	return 0;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct icm20608_i2c_data *obj = obj_i2c_data;	/* (struct icm20608_i2c_data*)i2c_get_clientdata(client); */
	int acc[ICM20608_AXES_NUM];
	int res = 0;

	client = obj->client;

	if (atomic_read(&obj->suspend))
		return -3;


	if (NULL == buf)
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	if (sensor_power == false) {
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
		res = ICM20608G_SCP_SetPowerMode(true, ID_ACCELEROMETER);
#else
		res = ICM20608_SetPowerMode(client, true);
#endif
		if (res)
			GSE_ERR("Power on icm20608 error %d!\n", res);
	}

	res = ICM20608_ReadData(client, obj->data);
	if (res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
/*
	obj->data[ICM20608_AXIS_X] += obj->cali_sw[ICM20608_AXIS_X];
	obj->data[ICM20608_AXIS_Y] += obj->cali_sw[ICM20608_AXIS_Y];
	obj->data[ICM20608_AXIS_Z] += obj->cali_sw[ICM20608_AXIS_Z];
*/
	/*remap coordinate*/
	acc[obj->cvt.map[ICM20608_AXIS_X]] = obj->cvt.sign[ICM20608_AXIS_X]*obj->data[ICM20608_AXIS_X];
	acc[obj->cvt.map[ICM20608_AXIS_Y]] = obj->cvt.sign[ICM20608_AXIS_Y]*obj->data[ICM20608_AXIS_Y];
	acc[obj->cvt.map[ICM20608_AXIS_Z]] = obj->cvt.sign[ICM20608_AXIS_Z]*obj->data[ICM20608_AXIS_Z];

	//GSE_ERR("ReadSensorData : x,y,z  data = [%d, %d, %d]\n",acc[ICM20608_AXIS_X], acc[ICM20608_AXIS_Y], acc[ICM20608_AXIS_Z]);
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	acc[ICM20608_AXIS_X] += obj->cali_sw[ICM20608_AXIS_X];
	acc[ICM20608_AXIS_Y] += obj->cali_sw[ICM20608_AXIS_Y];
	acc[ICM20608_AXIS_Z] += obj->cali_sw[ICM20608_AXIS_Z];
#endif
	//GSE_ERR("ReadSensorData : x,y,z  cali = [%d, %d, %d]\n", obj->cali_sw[ICM20608_AXIS_X], obj->cali_sw[ICM20608_AXIS_Y], obj->cali_sw[ICM20608_AXIS_Z]);
	
	/* Out put the mg */
	acc[ICM20608_AXIS_X] = acc[ICM20608_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	acc[ICM20608_AXIS_Y] = acc[ICM20608_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	acc[ICM20608_AXIS_Z] = acc[ICM20608_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;

	sprintf(buf, "%04x %04x %04x", acc[ICM20608_AXIS_X], acc[ICM20608_AXIS_Y], acc[ICM20608_AXIS_Z]);
	if (atomic_read(&obj->trace) & ICM20608_TRC_IOCTL)
		GSE_LOG("gsensor data: %s!\n", buf);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_ReadRawData(struct i2c_client *client, char *buf)
{
	struct icm20608_i2c_data *obj = (struct icm20608_i2c_data *)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
		return -EINVAL;

	if (atomic_read(&obj->suspend))
		return -EIO;

	res = ICM20608_ReadData(client, obj->data);
	if (res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -EIO;
	}
	sprintf(buf, "%04x %04x %04x", obj->data[ICM20608_AXIS_X],
	obj->data[ICM20608_AXIS_Y], obj->data[ICM20608_AXIS_Z]);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_InitSelfTest(struct i2c_client *client)
{
	int res = 0;
	u8 data;

//    res = MPU6050_SetPowerMode(client, true);
//    if (res != MPU6050_SUCCESS)
//    {
//        GSE_ERR("set power error\n");
//        return res;
//    }

	res = ICM20608_SetBWRate(client, ICM20608_A_BW_235HZ);
	if (res != ICM20608_SUCCESS)	{ /* 0x2C->BW=100Hz */
		return res;
	}

	res = mpu_i2c_read_block(client, ICM20608_REG_ACCEL_CONFIG, &data, 1);

	if (res != ICM20608_SUCCESS)
		return res;


	return ICM20608_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int ICM20608_JudgeTestResult(struct i2c_client *client, s32 prv[ICM20608_AXES_NUM], s32 nxt[ICM20608_AXES_NUM])
{
	struct criteria {
		int min;
		int max;
	};

	struct criteria self[4][3] = {
		{{0, 540}, {0, 540}, {0, 875} },
		{{0, 270}, {0, 270}, {0, 438} },
		{{0, 135}, {0, 135}, {0, 219} },
		{{0, 67}, {0, 67}, {0, 110} },
	};
	struct criteria (*ptr)[3] = NULL;
	u8 format;
	int res;

	res = mpu_i2c_read_block(client, ICM20608_REG_ACCEL_CONFIG, &format, 1);
	if (res)
		return res;

	format = format & ACCEL_FS_16G;

	switch (format) {
	case ACCEL_FS_2G:
		GSE_LOG("format use self[0]\n");
		ptr = &self[0];
		break;

	case ACCEL_FS_4G:
		GSE_LOG("format use self[1]\n");
		ptr = &self[1];
		break;

	case ACCEL_FS_8G:
		GSE_LOG("format use self[2]\n");
		ptr = &self[2];
		break;

	case ACCEL_FS_16G:
		GSE_LOG("format use self[3]\n");
		ptr = &self[3];
		break;

	default:
		GSE_LOG("format unknown use\n");
		break;
	}

	if (!ptr) {
		GSE_ERR("null pointer\n");
		return -EINVAL;
	}
	GSE_LOG("format=0x%x\n", format);

	GSE_LOG("X diff is %ld\n", abs(nxt[ICM20608_AXIS_X] - prv[ICM20608_AXIS_X]));
	GSE_LOG("Y diff is %ld\n", abs(nxt[ICM20608_AXIS_Y] - prv[ICM20608_AXIS_Y]));
	GSE_LOG("Z diff is %ld\n", abs(nxt[ICM20608_AXIS_Z] - prv[ICM20608_AXIS_Z]));


	if ((abs(nxt[ICM20608_AXIS_X] - prv[ICM20608_AXIS_X]) > (*ptr)[ICM20608_AXIS_X].max) ||
	    (abs(nxt[ICM20608_AXIS_X] - prv[ICM20608_AXIS_X]) < (*ptr)[ICM20608_AXIS_X].min)) {
		GSE_ERR("X is over range\n");
		res = -EINVAL;
	}
	if ((abs(nxt[ICM20608_AXIS_Y] - prv[ICM20608_AXIS_Y]) > (*ptr)[ICM20608_AXIS_Y].max) ||
	    (abs(nxt[ICM20608_AXIS_Y] - prv[ICM20608_AXIS_Y]) < (*ptr)[ICM20608_AXIS_Y].min)) {
		GSE_ERR("Y is over range\n");
		res = -EINVAL;
	}
	if ((abs(nxt[ICM20608_AXIS_Z] - prv[ICM20608_AXIS_Z]) > (*ptr)[ICM20608_AXIS_Z].max) ||
	    (abs(nxt[ICM20608_AXIS_Z] - prv[ICM20608_AXIS_Z]) < (*ptr)[ICM20608_AXIS_Z].min)) {
		GSE_ERR("Z is over range\n");
		res = -EINVAL;
	}
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = icm20608_i2c_client;
	char strbuf[ICM20608_BUFSIZE];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	if (sensor_power == false)
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
		ICM20608G_SCP_SetPowerMode(true, ID_ACCELEROMETER);
#else
		ICM20608_SetPowerMode(client, true);
#endif

	ICM20608_ReadAllReg(client, strbuf, ICM20608_BUFSIZE);

	ICM20608_ReadChipInfo(client, strbuf, ICM20608_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = icm20608_i2c_client;
	char strbuf[ICM20608_BUFSIZE];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	ICM20608_ReadSensorData(client, strbuf, ICM20608_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = icm20608_i2c_client;
	struct icm20608_i2c_data *obj;
	int err, len = 0, mul;
	int tmp[ICM20608_AXES_NUM];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);
	err = ICM20608_ReadOffset(client, obj->offset);
	if (err)
		return -EINVAL;
	err = ICM20608_ReadCalibration(client, tmp);
	if (err)
		return -EINVAL;

	mul = obj->reso->sensitivity/icm20608_offset_resolution.sensitivity;
	len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,
		obj->offset[ICM20608_AXIS_X], obj->offset[ICM20608_AXIS_Y], obj->offset[ICM20608_AXIS_Z],
		obj->offset[ICM20608_AXIS_X], obj->offset[ICM20608_AXIS_Y], obj->offset[ICM20608_AXIS_Z]);
	len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
		obj->cali_sw[ICM20608_AXIS_X], obj->cali_sw[ICM20608_AXIS_Y], obj->cali_sw[ICM20608_AXIS_Z]);

	len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
		obj->offset[ICM20608_AXIS_X] * mul + obj->cali_sw[ICM20608_AXIS_X],
		obj->offset[ICM20608_AXIS_Y] * mul + obj->cali_sw[ICM20608_AXIS_Y],
		obj->offset[ICM20608_AXIS_Z] * mul + obj->cali_sw[ICM20608_AXIS_Z],
		tmp[ICM20608_AXIS_X], tmp[ICM20608_AXIS_Y], tmp[ICM20608_AXIS_Z]);
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = icm20608_i2c_client;
	int err, x, y, z;
	int dat[ICM20608_AXES_NUM];

	if (!strncmp(buf, "rst", 3)) {
		err = ICM20608_ResetCalibration(client);
		if (err)
			GSE_ERR("reset offset err = %d\n", err);

	} else if (3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z)) {
		dat[ICM20608_AXIS_X] = x;
		dat[ICM20608_AXIS_Y] = y;
		dat[ICM20608_AXIS_Z] = z;
		err = ICM20608_WriteCalibration(client, dat);
		if (err)
			GSE_ERR("write calibration err = %d\n", err);

	} else {
		GSE_ERR("invalid format\n");
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_self_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = icm20608_i2c_client;

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	return snprintf(buf, 8, "%s\n", selftestRes);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_self_value(struct device_driver *ddri, const char *buf, size_t count)
{				/*write anything to this register will trigger the process */
	struct item {
		s16 raw[ICM20608_AXES_NUM];
	};

	struct i2c_client *client = icm20608_i2c_client;
	int idx, res, num;
	long prv_len, nxt_len;
	struct item *prv = NULL, *nxt = NULL;
	s32 avg_prv[ICM20608_AXES_NUM] = { 0, 0, 0 };
	s32 avg_nxt[ICM20608_AXES_NUM] = { 0, 0, 0 };


	res = kstrtoint(buf, 10, &num);
	if (res != 0) {
		GSE_ERR("parse number fail\n");
		return count;
	} else if (num == 0) {
		GSE_ERR("invalid data count\n");
		return count;
	}
	prv_len = sizeof(*prv) * num;
	nxt_len = sizeof(*nxt) * num;
	prv = kzalloc(prv_len, GFP_KERNEL);
	nxt = kzalloc(prv_len, GFP_KERNEL);
	if (!prv || !nxt)
		goto exit;



	GSE_LOG("NORMAL:\n");
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	ICM20608G_SCP_SetPowerMode(true, ID_ACCELEROMETER);
#else
	ICM20608_SetPowerMode(client, true);
#endif

	for (idx = 0; idx < num; idx++) {
		res = ICM20608_ReadData(client, prv[idx].raw);
		if (res) {
			GSE_ERR("read data fail: %d\n", res);
			goto exit;
		}

		avg_prv[ICM20608_AXIS_X] += prv[idx].raw[ICM20608_AXIS_X];
		avg_prv[ICM20608_AXIS_Y] += prv[idx].raw[ICM20608_AXIS_Y];
		avg_prv[ICM20608_AXIS_Z] += prv[idx].raw[ICM20608_AXIS_Z];
		GSE_LOG("[%5d %5d %5d]\n", prv[idx].raw[ICM20608_AXIS_X],
			prv[idx].raw[ICM20608_AXIS_Y], prv[idx].raw[ICM20608_AXIS_Z]);
	}

	avg_prv[ICM20608_AXIS_X] /= num;
	avg_prv[ICM20608_AXIS_Y] /= num;
	avg_prv[ICM20608_AXIS_Z] /= num;

	/*initial setting for self test */
	GSE_LOG("SELFTEST:\n");
	for (idx = 0; idx < num; idx++) {
		res = ICM20608_ReadData(client, nxt[idx].raw);
		if (res) {
			GSE_ERR("read data fail: %d\n", res);
			goto exit;
		}
		avg_nxt[ICM20608_AXIS_X] += nxt[idx].raw[ICM20608_AXIS_X];
		avg_nxt[ICM20608_AXIS_Y] += nxt[idx].raw[ICM20608_AXIS_Y];
		avg_nxt[ICM20608_AXIS_Z] += nxt[idx].raw[ICM20608_AXIS_Z];
		GSE_LOG("[%5d %5d %5d]\n", nxt[idx].raw[ICM20608_AXIS_X],
			nxt[idx].raw[ICM20608_AXIS_Y], nxt[idx].raw[ICM20608_AXIS_Z]);
	}

	avg_nxt[ICM20608_AXIS_X] /= num;
	avg_nxt[ICM20608_AXIS_Y] /= num;
	avg_nxt[ICM20608_AXIS_Z] /= num;

	GSE_LOG("X: %5d - %5d = %5d\n", avg_nxt[ICM20608_AXIS_X], avg_prv[ICM20608_AXIS_X],
		avg_nxt[ICM20608_AXIS_X] - avg_prv[ICM20608_AXIS_X]);
	GSE_LOG("Y: %5d - %5d = %5d\n", avg_nxt[ICM20608_AXIS_Y], avg_prv[ICM20608_AXIS_Y],
		avg_nxt[ICM20608_AXIS_Y] - avg_prv[ICM20608_AXIS_Y]);
	GSE_LOG("Z: %5d - %5d = %5d\n", avg_nxt[ICM20608_AXIS_Z], avg_prv[ICM20608_AXIS_Z],
		avg_nxt[ICM20608_AXIS_Z] - avg_prv[ICM20608_AXIS_Z]);

	if (!ICM20608_JudgeTestResult(client, avg_prv, avg_nxt)) {
		GSE_LOG("SELFTEST : PASS\n");
		strcpy(selftestRes, "y");
	} else {
		GSE_LOG("SELFTEST : FAIL\n");
		strcpy(selftestRes, "n");
	}

exit:
	/*restore the setting */
	icm20608_init_client(client, 0);
	kfree(prv);
	kfree(nxt);
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = icm20608_i2c_client;
	struct icm20608_i2c_data *obj;

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->selftest));
}

/*----------------------------------------------------------------------------*/
static ssize_t store_selftest_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct icm20608_i2c_data *obj = obj_i2c_data;
	int tmp;

	if (NULL == obj) {
		GSE_ERR("i2c data obj is null!!\n");
		return 0;
	}


	if (0 == kstrtoint(buf, 10, &tmp)) {
		if (atomic_read(&obj->selftest) && !tmp) {
			/*enable -> disable */
			icm20608_init_client(obj->client, 0);
		} else if (!atomic_read(&obj->selftest) && tmp) {
			/*disable -> enable */
			ICM20608_InitSelfTest(obj->client);
		}

		GSE_LOG("selftest: %d => %d\n", atomic_read(&obj->selftest), tmp);
		atomic_set(&obj->selftest, tmp);
	} else {
		GSE_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_ICM20608_LOWPASS
	struct i2c_client *client = icm20608_i2c_client;
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);

	if (atomic_read(&obj->firlen)) {
		int idx, len = atomic_read(&obj->firlen);

		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for (idx = 0; idx < len; idx++)
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][ICM20608_AXIS_X],
				obj->fir.raw[idx][ICM20608_AXIS_Y], obj->fir.raw[idx][ICM20608_AXIS_Z]);

		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[ICM20608_AXIS_X],
			obj->fir.sum[ICM20608_AXIS_Y], obj->fir.sum[ICM20608_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[ICM20608_AXIS_X]/len,
			obj->fir.sum[ICM20608_AXIS_Y]/len, obj->fir.sum[ICM20608_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}

/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_ICM20608_LOWPASS
	struct i2c_client *client = icm20608_i2c_client;
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if (0 != kstrtoint(buf, 10, &firlen)) {
		GSE_ERR("invallid format\n");
	} else if (firlen > C_MAX_FIR_LENGTH) {
		GSE_ERR("exceeds maximum filter length\n");
	} else {
		atomic_set(&obj->firlen, firlen);
		if (0 == firlen) {
			atomic_set(&obj->fir_en, 0);
		} else {
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct icm20608_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
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
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (0 == kstrtoint(buf, 16, &trace))
		atomic_set(&obj->trace, trace);
	else
		GSE_ERR("invalid content: '%s', length = %zu\n", buf, count);


	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct icm20608_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (obj->hw)
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
			obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);
	else
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");

	return len;
}

/*----------------------------------------------------------------------------*/
/*----------------------------Add For BMMI Test-------------------------------*/
static ssize_t gsensor_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int res = 0;
	u8 databuf[10];
	struct i2c_client *client = icm20608_i2c_client;

	memset(databuf, 0, sizeof(u8)*10);
	res = mpu_i2c_read_block(client, ICM20608_REG_WHO_AM_I, databuf, 0x1);
	if (res < 0)
		return ICM20608_ERR_I2C;
	GSE_LOG("ICM20608_CheckDeviceID 0x%x\n", databuf[0]);
	if(databuf[0] == 175)
		len += snprintf(buf + len, PAGE_SIZE - len, "1\n");

	return len;
}

/*----------------------------------------------------------------------------*/
/*----------------------------Add For Calibration-----------------------------*/

static ssize_t gsensor_cali(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = icm20608_i2c_client;
	struct icm20608_i2c_data *obj = (struct icm20608_i2c_data*)i2c_get_clientdata(client);
	int acc[ICM20608_AXES_NUM]={0};
	int cali[ICM20608_AXES_NUM]={0};
	int sens = obj->reso->sensitivity;
	int res = 0, index=0,iloop=10,pre_enable=1;
	s16 databuf[ICM20608_AXES_NUM];
	char strbuf[ICM20608_BUFSIZE];
	int POS_CALI_THRS = (sens*calibration_thres)/100;
	int NEG_CALI_THRS = (-1)*(sens*calibration_thres)/100;

	GSE_LOG("gsensor_do_cali !!\n");
	if(sensor_power==false){
		pre_enable = 0;
		ICM20608_SetPowerMode(client, true);
		mdelay(100);
	}
	for(index=0;index<iloop;index++){
		GSE_LOG("gsensor_do_cali %s !!\n", strbuf);                                                                                    
		res = ICM20608_ReadData(client, databuf);

		acc[obj->cvt.map[ICM20608_AXIS_X]] = obj->cvt.sign[ICM20608_AXIS_X]*databuf[ICM20608_AXIS_X];
		acc[obj->cvt.map[ICM20608_AXIS_Y]] = obj->cvt.sign[ICM20608_AXIS_Y]*databuf[ICM20608_AXIS_Y];
		acc[obj->cvt.map[ICM20608_AXIS_Z]] = obj->cvt.sign[ICM20608_AXIS_Z]*databuf[ICM20608_AXIS_Z];

		cali[ICM20608_AXIS_X]=cali[ICM20608_AXIS_X]+acc[obj->cvt.map[ICM20608_AXIS_X]] ;
		cali[ICM20608_AXIS_Y]=cali[ICM20608_AXIS_Y]+acc[obj->cvt.map[ICM20608_AXIS_Y]] ;
		cali[ICM20608_AXIS_Z]=cali[ICM20608_AXIS_Z]+acc[obj->cvt.map[ICM20608_AXIS_Z]] ;
	}
	GSE_LOG("sensitivity:%d, GRAVITY_EARTH_1000:%d \n",sens ,GRAVITY_EARTH_1000);
	GSE_LOG("+ calibration data x:%d, y:%d, z:%d  !!\n",cali[ICM20608_AXIS_X],cali[ICM20608_AXIS_Y],cali[ICM20608_AXIS_Z]);
	cali[ICM20608_AXIS_X]=(0-cali[ICM20608_AXIS_X]/iloop);
	cali[ICM20608_AXIS_Y]=(0-cali[ICM20608_AXIS_Y]/iloop);
	cali[ICM20608_AXIS_Z]=(sens - cali[ICM20608_AXIS_Z]/iloop);
	GSE_LOG("- calibration data x:%d, y:%d, z:%d  !!\n",cali[ICM20608_AXIS_X],cali[ICM20608_AXIS_Y],cali[ICM20608_AXIS_Z]);

	if(pre_enable==0){
		ICM20608_SetPowerMode(client, false);
	}
	if( (cali[ICM20608_AXIS_X]> POS_CALI_THRS )||(cali[ICM20608_AXIS_X]< NEG_CALI_THRS) ){
		GSE_ERR("sensitivity X out of range !!!\n");
		return sprintf(buf, "%d\n", 0);
	}
	else if( (cali[ICM20608_AXIS_Y]> POS_CALI_THRS )||(cali[ICM20608_AXIS_Y]< NEG_CALI_THRS) ){
		GSE_ERR("sensitivity Y out of range !!!\n");
		return sprintf(buf, "%d\n", 0);
	}
	else if( (cali[ICM20608_AXIS_Z]> POS_CALI_THRS )||(cali[ICM20608_AXIS_Z]< NEG_CALI_THRS) ){
		GSE_ERR("sensitivity Z out of range !!!\n");
		return sprintf(buf, "%d\n", 0);
	}else{      
		res = ICM20608_ResetCalibration(client);
		if (res != ICM20608_SUCCESS)
			return res;
		res = ICM20608_WriteCalibration(icm20608_i2c_client,cali);
		if (res)
			GSE_ERR("write calibration err = %d\n", res);

		icm20608_write_to_califile(cali[ICM20608_AXIS_X],cali[ICM20608_AXIS_Y],cali[ICM20608_AXIS_Z]);
	}
	return sprintf(buf, "%d\n", 1);

}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(cali, S_IWUSR | S_IRUGO, show_cali_value, store_cali_value);
static DRIVER_ATTR(self, S_IWUSR | S_IRUGO, show_selftest_value, store_selftest_value);
static DRIVER_ATTR(selftest, S_IWUSR | S_IRUGO, show_self_value, store_self_value);
static DRIVER_ATTR(firlen, S_IWUSR | S_IRUGO, show_firlen_value, store_firlen_value);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value, store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(gsensor_status, S_IRUGO, gsensor_status, NULL);
static DRIVER_ATTR(GsensorCali, S_IRUGO, gsensor_cali , NULL);
static DRIVER_ATTR(read_cali, S_IRUGO, atd_cali, NULL);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *icm20608_attr_list[] = {
	&driver_attr_chipinfo,		/*chip information */
	&driver_attr_sensordata,	/*dump sensor data */
	&driver_attr_cali,			/*show calibration data */
	&driver_attr_self,			/*self test demo */
	&driver_attr_selftest,		/*self control: 0: disable, 1: enable */
	&driver_attr_firlen,		/*filter length: 0: disable, others: enable */
	&driver_attr_trace,			/*trace log */
	&driver_attr_status,
	&driver_attr_gsensor_status,/*For BMMI Test*/
	&driver_attr_GsensorCali,	/*For SMMI Calibration*/
	&driver_attr_read_cali,	/*For SMMI Debug*/
};

/*----------------------------------------------------------------------------*/
static int icm20608_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(icm20608_attr_list) / sizeof(icm20608_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;


	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, icm20608_attr_list[idx]);
		if (0 != err) {
			GSE_ERR("driver_create_file (%s) = %d\n", icm20608_attr_list[idx]->attr.name,
				err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int icm20608_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(icm20608_attr_list) / sizeof(icm20608_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;


	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, icm20608_attr_list[idx]);


	return err;
}

/*----------------------------------------------------------------------------*/
int gsensor_operate(void *self, uint32_t command, void *buff_in, int size_in,
		    void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value, sample_delay;
	struct icm20608_i2c_data *priv = (struct icm20608_i2c_data *)self;
	struct hwm_sensor_data *gsensor_data;
	char buff[ICM20608_BUFSIZE];


	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			GSE_ERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;

			if (value <= 5)
				sample_delay = ICM20608_A_BW_235HZ;
			else if (value <= 10)
				sample_delay = ICM20608_A_BW_121HZ;
			else
				sample_delay = ICM20608_A_BW_61HZ;

			GSE_LOG("Set delay parameter value:%d\n", value);


			err = ICM20608_SetBWRate(priv->client, sample_delay);
			if (err != ICM20608_SUCCESS)	{ /* 0x2C->BW=100Hz */
				GSE_ERR("Set delay parameter error!\n");
			}

			if (value >= 50) {
				atomic_set(&priv->filter, 0);
			} else {
#if defined(CONFIG_ICM20608_LOWPASS)
				priv->fir.num = 0;
				priv->fir.idx = 0;
				priv->fir.sum[ICM20608_AXIS_X] = 0;
				priv->fir.sum[ICM20608_AXIS_Y] = 0;
				priv->fir.sum[ICM20608_AXIS_Z] = 0;
#endif
				atomic_set(&priv->filter, 1);
			}
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			GSE_ERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (((value == 0) && (sensor_power == false)) || ((value == 1) && (sensor_power == true)))
				GSE_LOG("Gsensor device have updated!\n");
			else
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
				err = ICM20608G_SCP_SetPowerMode(!sensor_power, ID_ACCELEROMETER);
#else
				err = ICM20608_SetPowerMode(priv->client, !sensor_power);
#endif
		}
		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			GSE_ERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			gsensor_data = (struct hwm_sensor_data *) buff_out;
			err = ICM20608_ReadSensorData(priv->client, buff, ICM20608_BUFSIZE);
			if (!err) {
				err = sscanf(buff, "%x %x %x", &gsensor_data->values[0],
					&gsensor_data->values[1], &gsensor_data->values[2]);
				if (3 == err) {
					gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
					gsensor_data->value_divide = 1000;
				} else
					GSE_ERR("gsensor operate function sscanf invaild parameter !\n");
			}
		}
		break;
	default:
		GSE_ERR("gsensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
static void gsensor_irq_work(struct work_struct *work)
{
	struct icm20608_i2c_data *obj = obj_i2c_data;
	struct scp_acc_hw scp_hw;
	union ICM20608G_CUST_DATA *p_cust_data;
	SCP_SENSOR_HUB_DATA data;
	int max_cust_data_size_per_packet;
	int i;
	uint sizeOfCustData;
	uint len;
	char *p = (char *)&scp_hw;

	GSE_FUN();

	scp_hw.i2c_num = obj->hw->i2c_num;
	scp_hw.direction = obj->hw->direction;
	scp_hw.power_id = obj->hw->power_id;
	scp_hw.power_vol = obj->hw->power_vol;
	scp_hw.firlen = obj->hw->firlen;
	memcpy(scp_hw.i2c_addr, obj->hw->i2c_addr, sizeof(obj->hw->i2c_addr));
	scp_hw.power_vio_id = obj->hw->power_vio_id;
	scp_hw.power_vio_vol = obj->hw->power_vio_vol;
	scp_hw.is_batch_supported = obj->hw->is_batch_supported;

	p_cust_data = (union ICM20608G_CUST_DATA *) data.set_cust_req.custData;
	sizeOfCustData = sizeof(scp_hw);
	max_cust_data_size_per_packet =
		sizeof(data.set_cust_req.custData) - offsetof(struct ICM20608G_SET_CUST, data);

	/*GSE_ERR("sizeOfCustData = %d, max_cust_data_size_per_packet = %d\n", sizeOfCustData,
		max_cust_data_size_per_packet);
	GSE_ERR("offset %lu\n", offsetof(struct ICM20608G_SET_CUST, data));*/

	for (i = 0; sizeOfCustData > 0; i++) {
		data.set_cust_req.sensorType = ID_ACCELEROMETER;
		data.set_cust_req.action = SENSOR_HUB_SET_CUST;
		p_cust_data->setCust.action = ICM20608G_CUST_ACTION_SET_CUST;
		p_cust_data->setCust.part = i;
		if (sizeOfCustData > max_cust_data_size_per_packet)
			len = max_cust_data_size_per_packet;
		else
			len = sizeOfCustData;

		memcpy(p_cust_data->setCust.data, p, len);
		sizeOfCustData -= len;
		p += len;

		/*GSE_ERR("i= %d, sizeOfCustData = %d, len = %d\n", i, sizeOfCustData, len);*/
		len +=
			offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + offsetof(struct ICM20608G_SET_CUST,
										   data);
		/*GSE_ERR("data.set_cust_req.sensorType= %d\n", data.set_cust_req.sensorType);*/
		SCP_sensorHub_req_send(&data, &len, 1);

	}
	p_cust_data = (union ICM20608G_CUST_DATA *) &data.set_cust_req.custData;
	data.set_cust_req.sensorType = ID_ACCELEROMETER;
	data.set_cust_req.action = SENSOR_HUB_SET_CUST;
	p_cust_data->resetCali.action = ICM20608G_CUST_ACTION_RESET_CALI;
	len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(p_cust_data->resetCali);
	SCP_sensorHub_req_send(&data, &len, 1);
	obj->SCP_init_done = 1;
}

/*----------------------------------------------------------------------------*/
static int gsensor_irq_handler(void *data, uint len)
{
	struct icm20608_i2c_data *obj = obj_i2c_data;
	SCP_SENSOR_HUB_DATA_P rsp = (SCP_SENSOR_HUB_DATA_P) data;

	/*GSE_ERR("gsensor_irq_handler len = %d, type = %d, action = %d, errCode = %d\n", len,
		rsp->rsp.sensorType, rsp->rsp.action, rsp->rsp.errCode);*/
	if (!obj)
		return -1;

	switch (rsp->rsp.action) {
	case SENSOR_HUB_NOTIFY:
		switch (rsp->notify_rsp.event) {
		case SCP_INIT_DONE:
			schedule_work(&obj->irq_work);
			/*GSE_ERR("OK sensor hub notify\n");*/
			break;
		default:
			GSE_ERR("Error sensor hub notify\n");
			break;
		}
		break;
	default:
		GSE_ERR("Error sensor hub action\n");
		break;
	}

	return 0;
}

static int gsensor_setup_irq(void)
{
	int err = 0;



	err = SCP_sensorHub_rsp_registration(ID_ACCELEROMETER, gsensor_irq_handler);

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
		GSE_ERR("null pointer!!\n");
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
#ifdef CONFIG_COMPAT
static long icm20608_compat_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	long err = 0;
	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
			return err;
		}
		break;
	case COMPAT_GSENSOR_IOCTL_SET_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
			return err;
		}
		break;
	case COMPAT_GSENSOR_IOCTL_GET_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
			return err;
		}
		break;
	case COMPAT_GSENSOR_IOCTL_CLR_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
			return err;
		}
		break;
	default:
		GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}

	return err;
}
#endif

static long icm20608_unlocked_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct icm20608_i2c_data *obj = (struct icm20608_i2c_data *)i2c_get_clientdata(client);
	char strbuf[ICM20608_BUFSIZE];
	void __user *data;
	struct SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));


	if (err) {
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case GSENSOR_IOCTL_INIT:
		icm20608_init_client(client, 0);
		break;

	case GSENSOR_IOCTL_READ_CHIPINFO:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		ICM20608_ReadChipInfo(client, strbuf, ICM20608_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_SENSORDATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		ICM20608_ReadSensorData(client, strbuf, ICM20608_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_GAIN:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		if (copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D))) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_RAW_DATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		if (atomic_read(&obj->suspend)) {
			err = -EINVAL;
		} else {
			ICM20608_ReadRawData(client, strbuf);
			if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
				err = -EFAULT;
				break;
			}
		}
		break;

	case GSENSOR_IOCTL_SET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
		if (atomic_read(&obj->suspend)) {
			GSE_ERR("Perform calibration in suspend state!!\n");
			err = -EINVAL;
		} else {
			cali[ICM20608_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
			cali[ICM20608_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
			cali[ICM20608_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;
			err = ICM20608_WriteCalibration(client, cali);
		}
		break;

	case GSENSOR_IOCTL_CLR_CALI:
		err = ICM20608_ResetCalibration(client);
		break;

	case GSENSOR_IOCTL_GET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		err = ICM20608_ReadCalibration(client, cali);
		if (err)
			break;

		sensor_data.x = cali[ICM20608_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		sensor_data.y = cali[ICM20608_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		sensor_data.z = cali[ICM20608_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
		break;


	default:
		GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;

	}

	return err;
}


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
	.name = "gsensor",
	.fops = &icm20608_fops,
};
/*----------------------------------------------------------------------------*/
#ifndef USE_EARLY_SUSPEND
/*----------------------------------------------------------------------------*/
static int icm20608_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	GSE_FUN();

	if (msg.event == PM_EVENT_SUSPEND) {
		if (obj == NULL) {
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
		err = ICM20608G_SCP_SetPowerMode(false, ID_ACCELEROMETER);
#else
		err = ICM20608_SetPowerMode(obj->client, false);
#endif
		if (err) {
			GSE_ERR("write power control fail!!\n");
			return err;
		}
		ICM20608_power(obj->hw, 0);
		GSE_LOG("icm20608_suspend ok\n");
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int icm20608_resume(struct i2c_client *client)
{
	struct icm20608_i2c_data *obj = i2c_get_clientdata(client);
	int err;

	GSE_FUN();

	if (obj == NULL) {
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	ICM20608_power(obj->hw, 1);

	err = icm20608_init_client(client, 0);
	if (err) {
		GSE_ERR("initialize client fail!!\n");
		return err;
	}
	atomic_set(&obj->suspend, 0);
	GSE_LOG("icm20608_resume ok\n");

	return 0;
}

/*----------------------------------------------------------------------------*/
#else				/*CONFIG_HAS_EARLY_SUSPEND is defined */
/*----------------------------------------------------------------------------*/
static void icm20608_early_suspend(struct early_suspend *h)
{
	struct icm20608_i2c_data *obj = container_of(h, struct icm20608_i2c_data, early_drv);
	int err;

	GSE_FUN();

	if (obj == NULL) {
		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1);

	err = ICM20608_SetPowerMode(obj->client, false);
	if (err) {
		GSE_ERR("write power control fail!!\n");
		return;
	}
    
    /*
	if (ICM20608_gyro_mode() == false) {
		ICM20608_Dev_Reset(obj->client);
		ICM20608_Reset(obj->client);
	}
        */

	obj->bandwidth = 0;

	sensor_power = false;

	ICM20608_power(obj->hw, 0);
}

/*----------------------------------------------------------------------------*/
static void icm20608_late_resume(struct early_suspend *h)
{
	struct icm20608_i2c_data *obj = container_of(h, struct icm20608_i2c_data, early_drv);
	int err;

	GSE_FUN();

	if (obj == NULL) {
		GSE_ERR("null pointer!!\n");
		return;
	}

	ICM20608_power(obj->hw, 1);

	err = icm20608_init_client(obj->client, 0);
	if (err) {
		GSE_ERR("initialize client fail!!\n");
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



/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL*/
static int icm20608_open_report_data(int open)
{
	/*should queuq work to report event if  is_report_input_direct=true*/
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL*/

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
		res = ICM20608G_SCP_SetPowerMode(power, ID_ACCELEROMETER);
#else
		res = ICM20608_SetPowerMode(obj_i2c_data->client, power);
#endif
		if (res == 0) {
			GSE_LOG("ICM20608_SetPowerMode done\n");
			break;
		}
		GSE_LOG("ICM20608_SetPowerMode fail\n");
	}

	if (res != ICM20608_SUCCESS) {
		GSE_LOG("ICM20608_SetPowerMode fail!\n");
		return -1;
	}
	GSE_LOG("icm20608_enable_nodata OK!\n");
	return 0;
}

static int icm20608_set_delay(u64 ns)
{
	int value = 0;
	int err;
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#else				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
	int sample_delay;
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

	value = (int)ns/1000/1000;
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	req.set_delay_req.sensorType = ID_ACCELEROMETER;
	req.set_delay_req.action = SENSOR_HUB_SET_DELAY;
	req.set_delay_req.delay = value;
	len = sizeof(req.activate_req);
	err = SCP_sensorHub_req_send(&req, &len, 1);
	if (err) {
		GSE_ERR("SCP_sensorHub_req_send!\n");
		return err;
	}
#else				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
	if (value <= 5)
		sample_delay = ICM20608_A_BW_235HZ;
	else if (value <= 10)
		sample_delay = ICM20608_A_BW_121HZ;
	else
		sample_delay = ICM20608_A_BW_61HZ;

	err = ICM20608_SetBWRate(obj_i2c_data->client, sample_delay);
	if (err != ICM20608_SUCCESS) {
		GSE_ERR("icm20608_set_delay Set delay parameter error!\n");
		return -1;
	}
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
	GSE_LOG("icm20608_set_delay (%d)\n", value);
	return 0;
}

static int icm20608_get_data(int *x, int *y, int *z, int *status)
{
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
	int err = 0;
#else
	char buff[ICM20608_BUFSIZE];
	int err;
#endif

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	if (sensor_power == false) {
		err = ICM20608G_SCP_SetPowerMode(true, ID_ACCELEROMETER);
		if (err)
			GSE_ERR("Power on ICM20608G error %d!\n", err);
	}
	req.get_data_req.sensorType = ID_ACCELEROMETER;
	req.get_data_req.action = SENSOR_HUB_GET_DATA;
	len = sizeof(req.get_data_req);
	err = SCP_sensorHub_req_send(&req, &len, 1);
	if (err) {
		GSE_ERR("SCP_sensorHub_req_send!\n");
		return err;
	}

	if (ID_ACCELEROMETER != req.get_data_rsp.sensorType ||
		SENSOR_HUB_GET_DATA != req.get_data_rsp.action || 0 != req.get_data_rsp.errCode) {
		GSE_ERR("error : %d\n", req.get_data_rsp.errCode);
		return req.get_data_rsp.errCode;
	}

	*x = (int)req.get_data_rsp.int16_Data[0] * GRAVITY_EARTH_1000 / 1000;
	*y = (int)req.get_data_rsp.int16_Data[1] * GRAVITY_EARTH_1000 / 1000;
	*z = (int)req.get_data_rsp.int16_Data[2] * GRAVITY_EARTH_1000 / 1000;
	/*GSE_ERR("x = %d, y = %d, z = %d\n", *x, *y, *z);*/

	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
#else

	ICM20608_ReadSensorData(obj_i2c_data->client, buff, ICM20608_BUFSIZE);
	err = sscanf(buff, "%x %x %x", x, y, z);
	if (err == 3)
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	else
		GSE_ERR("gsensor operate function sscanf invaild parameter !\n");
#endif

	return 0;
}


/*----------------------------------------------------------------------------*/
static int icm20608_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct icm20608_i2c_data *obj;
	int err = 0;
	struct acc_control_path ctl = {0};
	struct acc_data_path data = {0};

	GSE_FUN();

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!(obj)) {
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(struct icm20608_i2c_data));

	obj->hw = hw;

	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
	if (err) {
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
	/* obj->client->timing = 400; */

	new_client = obj->client;
	i2c_set_clientdata(new_client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	INIT_WORK(&obj->irq_work, gsensor_irq_work);
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
#ifdef CONFIG_ICM20608_LOWPASS
	if (obj->hw->firlen > C_MAX_FIR_LENGTH)
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	else
		atomic_set(&obj->firlen, obj->hw->firlen);


	if (atomic_read(&obj->firlen) > 0)
		atomic_set(&obj->fir_en, 1);

#endif

	icm20608_i2c_client = new_client;
	ICM20608_Dev_Reset(new_client);
	ICM20608_Reset(new_client);

	err = icm20608_init_client(new_client, 1);
	if (err)
		goto exit_init_failed;


	err = misc_register(&icm20608_device);
	if (err) {
		GSE_ERR("icm20608_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	ctl.is_use_common_factory = false;
	err = icm20608_create_attr(&(icm20608_init_info.platform_diver_addr->driver));
	if (err) {
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	ctl.open_report_data = icm20608_open_report_data;
	ctl.enable_nodata = icm20608_enable_nodata;
	ctl.set_delay  = icm20608_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = obj->hw->is_batch_supported;

	err = acc_register_control_path(&ctl);
	if (err) {
		GSE_ERR("register acc control path err\n");
		goto exit_kfree;
	}

	data.get_data = icm20608_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if (err) {
		GSE_ERR("register acc data path err= %d\n", err);
		goto exit_kfree;
	}

	//Add By KoHsiang to READ /persist/gsensor_cali
#ifdef ICM20608_DELAY_CALIBRATION
	atomic_set(&obj->delaycalibration, 12000);  // 20000 ms
	INIT_DELAYED_WORK(&obj->delayworkcalibration, icm20608_delay_calibration_func);
	schedule_delayed_work(&obj->delayworkcalibration, msecs_to_jiffies(atomic_read(&obj->delaycalibration)));
#endif

#ifdef USE_EARLY_SUSPEND
	obj->early_drv.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	obj->early_drv.suspend = icm20608_early_suspend,
	obj->early_drv.resume = icm20608_late_resume, register_early_suspend(&obj->early_drv);
#endif
	icm20608_init_flag = 0;

	GSE_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
	misc_deregister(&icm20608_device);
exit_misc_device_register_failed:
exit_init_failed:
	/* i2c_detach_client(new_client); */
exit_kfree:
	kfree(obj);
exit:
	GSE_ERR("%s: err = %d\n", __func__, err);
	icm20608_init_flag = -1;
	return err;
}

/*----------------------------------------------------------------------------*/
static int icm20608_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	err =  icm20608_delete_attr(&(icm20608_init_info.platform_diver_addr->driver));
	if (err)
		GSE_ERR("icm20608_delete_attr fail: %d\n", err);

	err = misc_deregister(&icm20608_device);
	if (err)
		GSE_ERR("misc_deregister fail: %d\n", err);

	err = hwmsen_detach(ID_ACCELEROMETER);
	if (err)
		GSE_ERR("hwmsen_detach fail: %d\n", err);


	icm20608_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int icm20608_remove(void)
{

	GSE_FUN();
	ICM20608_power(hw, 0);
	i2c_del_driver(&icm20608_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/

static int  icm20608_local_init(void)
{
	ICM20608_power(hw, 1);
	if (i2c_add_driver(&icm20608_i2c_driver)) {
		GSE_ERR("add driver error\n");
		return -1;
	}
	if (-1 == icm20608_init_flag)
		return -1;

	return 0;
}


/*----------------------------------------------------------------------------*/
static int __init icm20608gse_init(void)
{
	const char *name = "mediatek,icm20608g";
	GSE_ERR("alp.D : icm20608gse_init ++\n");
	hw = get_accel_dts_func(name, hw);
	if (!hw)
		GSE_ERR("get dts info fail\n");

	acc_driver_add(&icm20608_init_info);
	GSE_ERR("alp.D : icm20608gse_init --\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit icm20608gse_exit(void)
{
	GSE_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(icm20608gse_init);
module_exit(icm20608gse_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ICM20608 gse driver");
MODULE_AUTHOR("Yucong.Xiong@mediatek.com");

