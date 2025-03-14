/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)，All Rights Reserved.
*
* File Name: Test_lib.c
*
* Author: Software Development Team, AE
*
* Created: 2015-07-14
*
* Abstract: test entry for all IC
*
************************************************************************/
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/slab.h>

#include "test_lib.h"
#include "Global.h"
//#include "Config_FT8606.h"
//#include "Test_FT8606.h"
//#include "Config_FT5X46.h"
//#include "Test_FT5X46.h"
#include "Config_FT5822.h"
#include "Test_FT5822.h"

#define FTS_DRIVER_LIB_INFO  "Test_Lib_Version  V1.1.0 2015-07-30"


///////////////////////about test
int set_param_data(char *TestParamData);//loadextern int fts_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);

//FTS_I2C_READ_FUNCTION fts_i2c_read;
//FTS_I2C_WRITE_FUNCTION fts_i2c_write;

char *g_testparamstring = NULL;

/////////////////////IIC communication
/*
int init_i2c_read_func(FTS_I2C_READ_FUNCTION fpI2C_Read)
{
	fts_i2c_read = fpI2C_Read;
	return 0;
}

int init_i2c_write_func(FTS_I2C_WRITE_FUNCTION fpI2C_Write)
{
	fts_i2c_write = fpI2C_Write;
	return 0;
}
*/

/************************************************************************
* Name: set_param_data
* Brief:  load Config. Set IC series, init test items, init basic threshold, int detailThreshold, and set order of test items
* Input: TestParamData, from ini file.
* Output: none
* Return: 0. No sense, just according to the old format.
***********************************************************************/
int set_param_data(char * TestParamData)
{
	//int time_use = 0;//ms
	//struct timeval time_start;
	//struct timeval time_end;

	//gettimeofday(&time_start, NULL);//Start time
	
	printk("%s: set_param_data START\n", __func__);
	g_testparamstring = TestParamData;//get param of ini file
	
	//从配置读取所选芯片类�
	//Set g_ScreenSetParam.iSelectedIC
	OnInit_InterfaceCfg(g_testparamstring);

	/*Get IC Name*/
	get_ic_name(g_ScreenSetParam.iSelectedIC, g_strIcName);

	#if 0
	g_strIcName[0] = 'F';
	g_strIcName[1] = 'T';
	g_strIcName[2] = '5';	
	g_strIcName[3] = '7';
	g_strIcName[4] = '2';
	g_strIcName[5] = '6';
	#endif

	//测试项配置
	/*
	if(IC_FT5X46>>4 == g_ScreenSetParam.iSelectedIC>>4)
	{
		OnInit_FT5X22_TestItem(g_testparamstring);
		OnInit_FT5X22_BasicThreshold(g_testparamstring);
		OnInit_MCap_DetailThreshold(g_testparamstring);//测试项详细配置 
		SetTestItem_FT5X22();
	}
	else if(IC_FT8606>>4 == g_ScreenSetParam.iSelectedIC>>4)
	{
		OnInit_FT8606_TestItem(g_testparamstring);
		OnInit_FT8606_BasicThreshold(g_testparamstring);
		OnInit_MCap_DetailThreshold(g_testparamstring);
		SetTestItem_FT8606();
	}
	*/
	//else if(IC_FT5822>>4 == g_ScreenSetParam.iSelectedIC>>4)
	if(IC_FT5822>>4 == g_ScreenSetParam.iSelectedIC>>4) // modify by leo
	{
		OnInit_FT5822_TestItem(g_testparamstring);
		OnInit_FT5822_BasicThreshold(g_testparamstring);
		OnInit_MCap_DetailThreshold(g_testparamstring);
		SetTestItem_FT5822();
	}
	
	/*gettimeofday(&time_end, NULL);//End time
	time_use = (time_end.tv_sec - time_start.tv_sec)*1000 + (time_end.tv_usec - time_start.tv_usec)/1000;
	printk("Load Config, use time = %d ms \n", time_use);
	*/
	return 0;
}

/************************************************************************
* Name: start_test_tp
* Brief:  Test entry. Select test items based on IC series
* Input: none
* Output: none
* Return: Test Result, PASS or FAIL
***********************************************************************/

boolean start_test_tp(void)
{
	boolean bTestResult = false;
	printk("[focal] %s \n", FTS_DRIVER_LIB_INFO);	//show lib version
	printk("[focal] %s START \n", __func__);
	printk("focal] IC_%s Test\n", g_strIcName);
	
	switch(g_ScreenSetParam.iSelectedIC>>4)
		{
		/*
		case IC_FT8606>>4:
			bTestResult = FT8606_StartTest();
			break;	
		case IC_FT5X46>>4:
			bTestResult = FT5X46_StartTest();
			break;
		*/
		case IC_FT5822>>4:
			bTestResult = FT5822_StartTest();
			break;	
		default:
			printk("[focal]  Error IC, IC Name: %s, IC Code:  %d\n", g_strIcName, g_ScreenSetParam.iSelectedIC);
			bTestResult = FT5822_StartTest();
			break;
		}


	return bTestResult;
}
/************************************************************************
* Name: get_test_data
* Brief:  Get test data based on IC series
* Input: none
* Output: pTestData, External application for memory, buff size >= 1024*8
* Return: the length of test data. if length > 0, got data;else ERR.
***********************************************************************/
int get_test_data(char *pTestData)
{
	int iLen = 0;
	printk("[focal] %s start \n", __func__);	
	switch(g_ScreenSetParam.iSelectedIC>>4)
		{
		/*
		case IC_FT8606>>4:
			iLen = FT8606_get_test_data(pTestData);
			break;
		case IC_FT5X46>>4:
			iLen = FT5X46_get_test_data(pTestData);
			break;
		*/
		case IC_FT5822>>4:
			iLen = FT5822_get_test_data(pTestData);
			break;
		default:
			printk("[focal]  Error IC, IC Name: %s, IC Code:  %d\n", g_strIcName, g_ScreenSetParam.iSelectedIC);
			break;
		}


	return iLen;	
}
/************************************************************************
* Name: free_test_param_data
* Brief:  release printer memory
* Input: none
* Output: none
* Return: none. 
***********************************************************************/
void free_test_param_data(void)
{
	if(NULL != g_testparamstring)
		kfree(g_testparamstring);

	g_testparamstring = NULL;
}

/************************************************************************
* Name: show_lib_ver
* Brief:  get lib version
* Input: none
* Output: pLibVer
* Return: the length of lib version. 
***********************************************************************/
int show_lib_ver(char *pLibVer)
{
	int num_read_chars = 0;
	
	num_read_chars = snprintf(pLibVer, 128,"%s \n", FTS_DRIVER_LIB_INFO);

	return num_read_chars;
}


