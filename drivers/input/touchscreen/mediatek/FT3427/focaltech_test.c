/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2016, FocalTech Systems, Ltd., all rights reserved.
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

/************************************************************************
*
* File Name: focaltech_test.c
*
* Author:     Software Department, FocalTech
*
* Created: 2016-08-01
*
* Modify:
*
* Abstract: create char device and proc node for  the comm between APK and TP
*
************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include "focaltech_test.h"
#include "focaltech_core.h"
#include "tpd.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define RAWDATA_TEST_RESULT_LEN  4     //pass or fail

#define FTS_REG_DATA_TYPE       0x01
#define REG_TX_NUM          0x02
#define REG_RX_NUM          0x03

#define REG_TX_NOMAPPING_NUM            0x55
#define REG_RX_NOMAPPING_NUM            0x56

#define REG_MAPPING_SWITCH      0x54
#define BEFOR_MAPPING       0x01
#define REG_CB_ADDR_H           0x18
#define REG_CB_ADDR_L           0x19

#define REG_RAW_BUF0            0x6A
#define REG_CB_BUF0         0x6E

#define REG_3427_RAW_BUF0  0x36
#define REG_3427_CB_BUF0  0x4E
#define REG_SCWORK_MODE  0x44
#define REG_SCCBADDR   0x45

#define REG_CLB             0x04
#define REG_DATA_TYPE           0x06
#define DATA_TYPE_RAW_DATA      0xAA
#define SCAP_RAW_WATERPROOF     0xAC
#define SCAP_RAW_NO_WATERPROOF  0xAB
#define DEBUG_DETA_DIFF_DATA        0x01
#define DEBUG_DATA_RAW_DATA     0x00

#define FTS_CALIBRATION_DISABLE_REG     0xEE

#define REG_3427_ADC_SCAN           0x07

#define REG_3427_ADC_SCAN_STATUS    0x07

#define REG_3427_IC_VERSION             0xB1
#define REG_3427_FIR_EN             0xFB
#define REG_3427_CHANG_FRE              0x0A

#define REG_3427_ADC_DATA_ADDR      0xF4
#define MAX_ADC_NUM                 2047

#define REG_GIP                         0x20
#define MAX_CB_VALUE                200
#define TP_TEST_FAILED_REASON_LEN   20

#define ERROR_CODE_OK               0x00
#define DELAY_TIME_OF_CALIBRATION   30

#define MAX_FREQUENCY               0x81

#define SCAPRAW_TYPE                0x16
#define OVERALL_NORMALIZE           0
#define FT_SELFTEST_NAME                "tp_capacitance_data"

static char tp_test_failed_reason[TP_TEST_FAILED_REASON_LEN] = { "-software_reason" };
static int  tp_cap_test_status =TEST_SUCCESS;
static struct proc_dir_entry *fts_proc_test;
extern int focaltech_set_app_info(struct i2c_client *client);

static int focal_start_scan(void);
static int focal_read_raw_data(u8 data_type, u8 *raw_data, size_t size);

static int focal_get_channel_num(u8 *chl_x, u8 *chl_y);

static int focal_read_channel_x(u8 *chl_x);
static int focal_read_channel_y(u8 *chl_y);


static int focal_get_nomapping_channel_num(u8 *chl_x, u8 *chl_y);

static int focal_read_nomapping_channel_x(u8 *chl_x);
static int focal_read_nomapping_channel_y(u8 *chl_y);
static int focal_start_test_tp(struct focal_test_params *params,
	struct focal_rawdata_info *info);
static int focal_get_Scap_cb_data_3427(u8 *data,size_t data_size);
static int focal_get_scap_raw_data_format_3427(int raw_data_type,int *data, size_t size);
static int focal_get_adc_data(int *data, size_t size,unsigned int chl_x,unsigned int chl_y);
static int focal_get_raw_data_format(unsigned int chl_x,unsigned int chl_y,int *data, size_t size);

static void focal_print_test_data(char *msg, int row, int col, int max,
	int min, int value);

static void focal_put_test_result(struct focal_test_params *params,
	struct focal_rawdata_info *info, struct focal_test_result *test_results[],
	int size);

static int focal_alloc_test_container(struct focal_test_result **result,
	size_t data_size);
static void focal_free_test_container(struct focal_test_result *result);

static int focal_get_channel_form_ic(struct focal_test_params *params);
static int focal_init_test_prm(struct focal_test_params *params,
	struct focal_rawdata_info *info);
static int focal_parse_cap_test_config(
	struct focal_test_params *params);
static int focal_scap_raw_data_test_3427(struct focal_test_params *params,
	struct focal_test_result **result,char test_num);
static int focal_raw_data_test(struct focal_test_params *params,
	struct focal_test_result **result,char test_num);
static int focal_cb_test(struct focal_test_params *params,
	struct focal_test_result **result,char test_num);
static int focal_short_circuit_test(struct focal_test_params *params,
	struct focal_test_result **result,char test_num);
static int focal_enter_work(void);
static int focal_enter_factory(void);


static int focal_enter_work(void)
{
	int i = 0;
	int ret = 0;
	u8 ret_val = 0;

	TPD_DEBUG("%s: enter work\n", __func__);

	for (i = 0; i < MAX_RETRY_TIMES; i++) {
		ret = focal_read_reg(DEVICE_MODE_ADDR, &ret_val);
		if (ret<0) {
			TPD_DMESG("%s: read DEVICE_MODE_ADDR reg failed\n",
				__func__);
			return ret;
		}
		if (((ret_val >> TPD_DEVICE_MODE_SHIFT) & TPD_DEVICE_MODE_MASK) != DEVICE_MODE_WORK) {
			ret = focal_write_reg(DEVICE_MODE_ADDR, DEVICE_MODE_WORK);
			if (ret>0) {
				msleep(FTS_ENTER_FACTORY_DELAY);
				continue;
			} else {
				TPD_DMESG("%s: change work mode failed\n",
					__func__);
				break;
			}
		} else {
			TPD_DEBUG("%s: change work mode success\n", __func__);
			break;
		}
	}

	return ret;
}

static int focal_enter_factory(void)
{
	int i = 0;
	int ret = 0;
	u8 ret_val = 0;

	TPD_DEBUG("%s: start change factory mode\n", __func__);
	for (i = 0; i < MAX_RETRY_TIMES; i++) {
		ret = focal_read_reg(DEVICE_MODE_ADDR, &ret_val);
		if (ret<0) {
			TPD_DMESG("%s: read DEVICE_MODE_ADDR error\n",
				__func__);
			return ret;
		}

		if ((ret_val & FTS_REG_WORKMODE_FACTORY_MASK) == FTS_REG_WORKMODE_FACTORY_VALUE) {
			TPD_DEBUG("%s: change factory success\n", __func__);
			return 0;
		} else {
			ret = focal_write_reg(DEVICE_MODE_ADDR, DEVICE_MODE_FACTORY);
			if (ret>0) {
				msleep(FTS_ENTER_FACTORY_DELAY);
				continue;
			} else {
				TPD_DMESG("%s: write reg failed\n",
					__func__);
				return ret;
			}
		}
	}

	TPD_DEBUG("%s: end change factory mode\n", __func__);
	return ret;
}

static int focal_get_int_average(int *p, size_t len)
{
	long long sum = 0;
	size_t i = 0;

	if (!p)
		return 0;

	for (i = 0; i < len; i++)
		sum += p[i];

	if (len != 0)
		return (int)div_s64(sum, len);
	else
		return 0;
}

static int focal_get_int_min(int *p, size_t len)
{
	int min = 0;
	size_t i = 0;

	if (!p || len <= 0)
		return 0;

	min = p[0];
	for (i = 0; i < len; i++)
		min = min > p[i] ? p[i] : min;

	return min;
}

static int focal_get_int_max(int *p, size_t len)
{
	int max = 0;
	size_t i = 0;

	if (!p || len <= 0)
		return 0;

	max = p[0];
	for (i = 0; i < len; i++)
		max = max < p[i] ? p[i] : max;

	return max;
}

static int  focal_start_test_tp(
	struct focal_test_params *params,
	struct focal_rawdata_info *info)
{
	int i = 0;
	int ret = 0;
	char cap_test_num = 0;
	struct focal_test_result *test_results[FTS_MAX_CAP_TEST_NUM] = {0};

	TPD_DEBUG("%s: start test tp\n",  __func__);
	if((!params )|| !(info)){
		TPD_DMESG("%s: parameter error\n",  __func__);
		return -EINVAL;
	}
	//first test is i2c test which is completed in  focal_init_test_prm,so there count from 1;
	/*  raw data test */
	ret = focal_raw_data_test(params, &test_results[0],cap_test_num+1);
	if(ret<0){
		tp_cap_test_status = SOFTWARE_REASON;
		strncpy(tp_test_failed_reason, "-software reason",TP_TEST_FAILED_REASON_LEN);
	}
	cap_test_num++;
	/*  cb data test */
	ret = focal_cb_test(params, &test_results[1],cap_test_num+1);
	if(ret<0){
		tp_cap_test_status = SOFTWARE_REASON;
		strncpy(tp_test_failed_reason, "-software reason",TP_TEST_FAILED_REASON_LEN);
	}
	cap_test_num++;
	/*  scap rawdata test */
	ret = focal_scap_raw_data_test_3427(params, &test_results[2], cap_test_num+1);
	if(ret<0){
		tp_cap_test_status = SOFTWARE_REASON;
		strncpy(tp_test_failed_reason, "-software reason",TP_TEST_FAILED_REASON_LEN);
	}
	cap_test_num++;

	/*  short test */
	ret = focal_short_circuit_test(params, &test_results[3],cap_test_num+1);
	if(ret<0){
		tp_cap_test_status = SOFTWARE_REASON;
		strncpy(tp_test_failed_reason, "-software reason",TP_TEST_FAILED_REASON_LEN);
	}
	cap_test_num++;

	focal_put_test_result(params, info, test_results, cap_test_num);

	for (i = 0; i < cap_test_num; i++) {
		focal_free_test_container(test_results[i]);
		test_results[i] = NULL;
	}

	return ret;
}

static int focal_start_scan(void)
{
	int i = 0;
	int ret = 0;
	int query_delay = 0;
	int max_query_times = 0;

	u8 reg_val = 0x00;

	ret = focal_read_reg(DEVICE_MODE_ADDR, &reg_val);
	if (ret<0) {
		TPD_DMESG("%s: read device mode fail, ret=%d\n",
			__func__, ret);
		return ret;
	}

	reg_val |= 0x80; //scan bit,bit[7]
	ret = focal_write_reg(DEVICE_MODE_ADDR, reg_val);
	if (ret<0) {
		TPD_DMESG("%s:set device mode fail, ret=%d\n", __func__, ret);
		return ret;
	}

	query_delay = FTS_3427_SCAN_QUERY_DELAY;
	max_query_times = FTS_3427_SCAN_MAX_TIME / query_delay;

	msleep(query_delay);
	for (i = 0; i < max_query_times; i++) {
		ret = focal_read_reg(DEVICE_MODE_ADDR, &reg_val);
		if (ret<0) {
			TPD_DMESG("%s: read device model fail, ret=%d\n",
				__func__, ret);
			msleep(query_delay);
			continue;
		}

		if ((reg_val & 0x80) != 0) {   //scan bit,bit[7]
			TPD_DEBUG("%s:device is scan, retry=%d\n",
				__func__, i);
			msleep(query_delay);
		} else {
			TPD_DEBUG("%s:device scan finished, retry=%d\n",
				__func__, i);
			return 0;
		}
	}

	TPD_DMESG("%s:device scan timeout\n", __func__);
	return -ETIMEDOUT;
}

static int focal_read_raw_data(u8 data_type, u8 *raw_data, size_t size)
{
	int i = 0;
	int ret = 0;
	int pkg_size = 0;
	int pkg_count = 0;
	int readed_count = 0;

	u8 cmd = 0;

	pkg_count = size / BYTES_PER_TIME;
	if (size % BYTES_PER_TIME != 0)
		pkg_count += 1;
	//reg FTS_REG_DATA_TYPE:rawdata type
	ret = focal_write_reg(FTS_REG_DATA_TYPE, data_type);
	if (ret<0) {
		TPD_DMESG("%s:write data type to ret, ret=%d\n",
			__func__, ret);
		return ret;
	}
	msleep(FTS_RAWDATA_TYPE_SWITCH_DELAY);

	cmd = REG_3427_RAW_BUF0;   //rawdata reg
	for (i = 0; i < pkg_count; i++) {
		/*
		 * compute pkg_size value
		 * if the last package is not equal to BYTES_PER_TIME,
		 * the package size will be set to (size % BYTES_PER_TIME)
		 */
		if ((size % BYTES_PER_TIME != 0) && (i + 1 == pkg_count))
			pkg_size = size % BYTES_PER_TIME;
		else
			pkg_size = BYTES_PER_TIME;

		/* first package shuold write cmd to ic */
		if (i == 0)
			ret = focal_read(&cmd, FT_WRITE_CHAR_NUM_1, raw_data, pkg_size);
		else
			ret = focal_read_default(raw_data + readed_count,
				pkg_size);

		if (ret < 0) {
			TPD_DMESG("%s:read raw data fail, ret=%d\n",
				__func__, ret);
			return ret;
		}

		readed_count += pkg_size;
	}

	return 0;
}

static int focal_get_Scap_cb_data_3427(u8 *data, size_t data_size)
{
	int i = 0;
	int ret = 0;
	int pkg_size = 0;
	int pkg_count = 0;
	int readed_count = 0;
	u8 cmd[3] = {0};

	pkg_count = data_size / BYTES_PER_TIME;
	if (0 != (data_size % BYTES_PER_TIME))
		pkg_count += 1;
	ret = focal_start_scan();
	if (ret<0) {
		TPD_DMESG("%s:start scan fail, ret=%d\n",
			__func__, ret);
		return ret;
	}
	/*waterproof*/
	ret = focal_write_reg(REG_SCWORK_MODE,1);
	if (ret<0) {
		TPD_DMESG("%s:write work mode addr fail, ret=%d\n",
			__func__, ret);
		return ret;
	}
	ret = focal_write_reg(REG_SCCBADDR,0);
	if (ret<0) {
		TPD_DMESG("%s:write cb addr fail, ret=%d\n",
			__func__, ret);
		return ret;
	}
	cmd[0] = REG_3427_CB_BUF0;
	if(data_size <BYTES_PER_TIME ){
		TPD_DMESG("cb data read size:%d\n",data_size);
		ret = focal_read(cmd, FT_WRITE_CHAR_NUM_1, data, data_size);
		if (ret<0) {
			TPD_DMESG("%s:read cb data fail, ret=%d\n",
				__func__, ret);
			return ret;
		}
	}
	else{
		for (i = 0; i < pkg_count; i++) {

			if ((i + 1 == pkg_count) && (data_size % BYTES_PER_TIME != 0)){
				pkg_size = data_size % BYTES_PER_TIME;
			}
			else{
				pkg_size = BYTES_PER_TIME;
			}
			if (i == 0)
				ret = focal_read(cmd, FT_WRITE_CHAR_NUM_1, data , pkg_size);
			else
				ret = focal_read_default( data + readed_count,
				pkg_size);
			if (ret<0) {
				TPD_DMESG("%s:read cb data fail, ret=%d\n",
					__func__, ret);
				return ret;
			}

			readed_count += pkg_size;
		}
	}
	/*non-waterproof*/
	ret = focal_write_reg(REG_SCWORK_MODE,0);
	if (ret<0) {
		TPD_DMESG("%s:write work mode addr fail, ret=%d\n",
			__func__, ret);
		return ret;
	}
	ret = focal_write_reg(REG_SCCBADDR,0);
	if (ret<0) {
		TPD_DMESG("%s:write cb addr fail, ret=%d\n",
			__func__, ret);
		return ret;
	}
	if(data_size <BYTES_PER_TIME ){
		TPD_DMESG("cb data read size:%d\n",data_size);
		ret = focal_read(cmd, 1, data+data_size, data_size);
		if (ret<0) {
			TPD_DMESG("%s:read cb data fail, ret=%d\n",
				__func__, ret);
			return ret;
		}
	}
	else{
		for (i = 0; i < pkg_count; i++) {

			if ((i + 1 == pkg_count) && (data_size % BYTES_PER_TIME != 0))
				pkg_size = data_size % BYTES_PER_TIME;
			else
				pkg_size = BYTES_PER_TIME;
			if(i==0)
				ret = focal_read(cmd, 1, data + readed_count, pkg_size);
			else
				ret = focal_read_default(data + readed_count,pkg_size );
			if (ret<0) {
				TPD_DMESG("%s:read cb data fail, ret=%d\n",
					__func__, ret);
				return ret;
			}

			readed_count += pkg_size;
		}
	}
	return 0;
}

/* get panelrows */
static int focal_read_channel_x(u8 *channel_x)
{
	return focal_read_reg(REG_TX_NUM, channel_x);
}

/* get panelcols */
static int focal_read_channel_y(u8 *channel_y)
{
	return focal_read_reg(REG_RX_NUM, channel_y);
}

/* get panelrows */
static int focal_read_nomapping_channel_x(u8 *channel_x)
{
	return focal_read_reg(REG_TX_NOMAPPING_NUM, channel_x);
}

/* get panelcols */
static int focal_read_nomapping_channel_y(u8 *channel_y)
{
	return focal_read_reg(REG_RX_NOMAPPING_NUM, channel_y);
}

/************************************************************************
* name: focal_get_nomapping_channel_num
* brief:  get num of ch_x, ch_y and key
* input: none
* output: none
* return: comm code. code = 0x00 is ok, else fail.
***********************************************************************/
static int focal_get_nomapping_channel_num(u8 *channel_x, u8 *channel_y)
{
	int ret = 0;
	u8 chl_x = 0;
	u8 chl_y = 0;

	/* get channel x num */
	ret = focal_read_nomapping_channel_x(&chl_x);
	if (ret<0) {
		TPD_DMESG("%s:get chennel x from ic fail, ret=%d\n",
			__func__, ret);
		return ret;
	}

	/* get channel y num */
	ret = focal_read_nomapping_channel_y(&chl_y);
	if (ret<0) {
		TPD_DMESG("%s:get chennel y from ic fail, ret=%d\n",
			__func__, ret);
		return ret;
	}

	if (chl_x <= 0 || chl_x > TX_NUM_MAX) {
		TPD_DMESG("%s:channel x value out of range, value = %d\n",
			__func__, chl_x);
		return -EINVAL;
	}

	if (chl_y <= 0 || chl_y > RX_NUM_MAX) {
		TPD_DMESG("%s:channel y value out of range, value = %d\n",
			__func__, chl_y);
		return -EINVAL;
	}

	*channel_x = chl_x;
	*channel_y = chl_y;

	return 0;
}
/************************************************************************
* name: focal_get_channel_num
* brief:  get num of ch_x, ch_y and key
* input: none
* output: none
* return: comm code. code = 0x00 is ok, else fail.
***********************************************************************/
static int focal_get_channel_num(u8 *channel_x, u8 *channel_y)
{
	int ret = 0;
	u8 chl_x = 0;
	u8 chl_y = 0;

	/* get channel x num */
	ret = focal_read_channel_x(&chl_x);
	if (ret<0) {
		TPD_DMESG("%s:get chennel x from ic fail, ret=%d\n",
			__func__, ret);
		return ret;
	}

	/* get channel y num */
	ret = focal_read_channel_y(&chl_y);
	if (ret<0) {
		TPD_DMESG("%s:get chennel y from ic fail, ret=%d\n",
			__func__, ret);
		return ret;
	}

	if (chl_x <= 0 || chl_x > TX_NUM_MAX) {
		TPD_DMESG("%s:channel x value out of range, value = %d\n",
			__func__, chl_x);
		return -EINVAL;
	}

	if (chl_y <= 0 || chl_y > RX_NUM_MAX) {
		TPD_DMESG("%s:channel y value out of range, value = %d\n",
			__func__, chl_y);
		return -EINVAL;
	}

	*channel_x = chl_x;
	*channel_y = chl_y;

	return 0;
}

static int focal_get_channel_form_ic(struct focal_test_params *params)
{
	int ret = 0;
	u8 chl_x = 0;
	u8 chl_y = 0;
	u8 strSwitch = 0;

	ret = focal_enter_factory();
	if (ret<0) {
		TPD_DMESG("%s:enter factory model fail, ret=%d\n",
			__func__, ret);
		return ret;
	}

	ret = focal_get_channel_num(&chl_x, &chl_y);
	if (ret<0) {
		TPD_DMESG("%s:get channel num fail, ret=%d\n", __func__, ret);
		return ret;
	} else {
		params->channel_x_num = chl_x;
		params->channel_y_num = chl_y;
		params->key_num = 0;//visual key num
		TPD_DEBUG("%s:channel_x=%d, channel_y=%d\n,params->key_num=%d",
			__func__, chl_x, chl_y,params->key_num);
	}

	ret = focal_read_reg(REG_MAPPING_SWITCH, &strSwitch);
	if (ret<0) {
		TPD_DMESG("%s:read mapping type fail, ret=%d\n", __func__, ret);
		return ret;
	}

	ret = focal_write_reg(REG_MAPPING_SWITCH, BEFOR_MAPPING);
	if (ret<0) {
		TPD_DMESG("%s:switch mapping type fail, ret=%d\n", __func__, ret);
		return ret;
	}

	ret = focal_get_nomapping_channel_num(&chl_x, &chl_y);
	if (ret<0) {
		TPD_DMESG("%s:get nomapping channel num fail, ret=%d\n", __func__, ret);
		return ret;
	} else {
		params->nomapping_channel_x_num = chl_x;
		params->nomapping_channel_y_num = chl_y;
		params->key_num = 0;//visual key num
		TPD_DEBUG("%s:nomapping_channel_x=%d, nomapping_channel_y=%d\n,params->key_num=%d",
			__func__, chl_x, chl_y,params->key_num);
	}

	ret = focal_write_reg(REG_MAPPING_SWITCH, strSwitch);
	if (ret<0) {
		TPD_DMESG("%s:switch mapping type fail, ret=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}
static void focal_of_property_read_u32_default(
	struct device_node *np,
	char *prop_name,
	u32 *out_value,
	u32 default_value)
{
	int ret = 0;

	ret = of_property_read_u32(np, prop_name, out_value);
	if (ret) {
		TPD_DMESG("%s:%s not set in dts, use default\n",
			__func__, prop_name);
		*out_value = default_value;
	}
}


static int focal_parse_test_threshold(
	struct device_node *np,
	struct focal_test_threshold *threshold)
{
	int ret = 0;
	/* rawdata_test */
	ret = of_property_read_u32_array(np, DTS_RAW_DATA_MIN,
			threshold->raw_data_min,TX_MAX*RX_MAX);
	if (ret){
		TPD_DMESG("threshold,raw_data_min parse err=%d \n", ret);
		return -EINVAL;
	}
	ret = of_property_read_u32_array(np, DTS_RAW_DATA_MAX,
			threshold->raw_data_max,TX_MAX*RX_MAX);
	if (ret){
		TPD_DMESG("threshold,raw_data_max parse err=%d \n", ret);
		return -EINVAL;
	}

	/* scap_cb_test */
	focal_of_property_read_u32_default(np, DTS_CB_TEST_MIN,
		&threshold->cb_test_min, 0);

	focal_of_property_read_u32_default(np, DTS_CB_TEST_MAX,
		&threshold->cb_test_max, 0);

	/* scap_raw_test */
	focal_of_property_read_u32_default(np, DTS_SCAP_RAW_DATA_MIN,
		&threshold->scap_raw_data_min, 0);
	focal_of_property_read_u32_default(np, DTS_SCAP_RAW_DATA_MAX,
		&threshold->scap_raw_data_max, 0);

	/* short_circuit_test */
	focal_of_property_read_u32_default(np, DTS_SHORT_CIRCUIT_RES_MIN,
		&threshold->short_circuit_min, 0);

	TPD_DMESG("%s:%s:%s=%d, %s=%d, %s=%d, %s=%d, %s=%d\n",
		__func__, "factory test thresholds",
		"raw_data_min", threshold->raw_data_min[0],
		"raw_data_max", threshold->raw_data_max[0],
		"cb_test_min",  threshold->cb_test_min,
		"cb_test_max",  threshold->cb_test_max,
		"short_circuit_min", threshold->short_circuit_min);
	return 0;
}


static int focal_parse_cap_test_config(
	struct focal_test_params *params)
{

	struct device_node *np = NULL;
	int ret = 0;

	np = of_find_compatible_node(NULL, NULL, FTS_DTS_COMCOPATIBLE);
	if (np<0) {
		TPD_DMESG("%s:find dev node faile\n",__func__);
		return -ENODEV;
	}

	ret = focal_parse_test_threshold(np, &params->threshold);
	if(ret < 0)
	{
		TPD_DMESG("%s: parse test threshold err=%d \n",__func__, ret);
		return -EINVAL;
	}

	return 0;
}


static int focal_alloc_test_container(
	struct focal_test_result **result,
	size_t data_size)
{

	*result = kzalloc(sizeof(struct focal_test_result), GFP_KERNEL);
	if (!*result)
		return -ENOMEM;

	(*result)->size = data_size;
	(*result)->values = kzalloc(data_size * sizeof(int), GFP_KERNEL);
	if (!((*result)->values)) {
		kfree(*result);
		*result = NULL;
		return -ENOMEM;
	}

	return 0;
}

static void focal_free_test_container(struct focal_test_result *result)
{

	if (result){
		if(result->values)
			kfree(result->values);
		kfree(result);
	}
}

static int focal_get_scap_raw_data_format_3427(int raw_data_type,int *data, size_t size)
{
	unsigned int i = 0;
	unsigned int Count = 0;
	int ret = 0;
	int raw_data_size = 0;
	short raw_data_value = 0;
	u8 *scap_raw_data = NULL;
	if(!data||(0 == size)){
		TPD_DMESG("%s: parameter error\n",  __func__);
		return -EINVAL;
	}
	raw_data_size = size*2 ;
	scap_raw_data = kzalloc(raw_data_size*sizeof(int), GFP_KERNEL);
	if (!scap_raw_data) {
		TPD_DMESG("%s:alloc mem fail\n", __func__);
		return -ENOMEM;
	}
	ret = focal_read_raw_data(raw_data_type, scap_raw_data,
		raw_data_size);
	if (ret<0) {
		TPD_DMESG("%s:read raw data fail, ret=%d\n", __func__, ret);
		goto exit;
	}

	for (i = 0; i < size; i++) {
		Count = i * 2;
		raw_data_value = (scap_raw_data[Count] << 8);
		Count ++;
		raw_data_value |= scap_raw_data[Count];

		data[i] = raw_data_value;
	}

exit:
	if(NULL!=scap_raw_data)
	{
		kfree(scap_raw_data);
		scap_raw_data = NULL;
	}
	return ret;
}

static int focal_get_raw_data_format(unsigned int chl_x ,unsigned int chl_y,int *data, size_t size)
{
	unsigned int i = 0;
	unsigned int CurTx = 0;
	unsigned int CurRx = 0;
	int ret = 0;
	int raw_data_size = 0;

	short   raw_data_value = 0;
	u8 *original_raw_data = NULL;

	if(!data||(0 == size) ||(0 == chl_x) ||(0 == chl_y)){
		TPD_DMESG("%s: parameter error\n",  __func__);
		return -EINVAL;
	}
	raw_data_size = size * 2;//one data need 2 byte

	original_raw_data = kzalloc(raw_data_size, GFP_KERNEL);
	if (!original_raw_data) {
		TPD_DMESG("%s:alloc mem fail\n", __func__);
		return -ENOMEM;
	}

	ret = focal_read_raw_data(DATA_TYPE_RAW_DATA, original_raw_data,raw_data_size);
	if (ret<0) {
		TPD_DMESG("%s:read raw data fail, ret=%d\n", __func__, ret);
		goto exit;
	}

	for (i = 0; i < size; i++) {
		raw_data_value = (original_raw_data[ i * 2]<<8);
		raw_data_value |= original_raw_data[i * 2 + 1];
		CurTx = i%chl_y;
		CurRx = i /chl_y;
		data[CurTx*chl_x + CurRx] = raw_data_value;
		//TPD_DEBUG("%s:raw_data_value[%d][%d]= %d  \n",__func__, CurTx,CurRx,raw_data_value);
	}

exit:
	kfree(original_raw_data);
	original_raw_data = NULL;
	return ret;
}

static int focal_scap_raw_data_test_3427(
	struct focal_test_params *params,
	struct focal_test_result **result,char test_num)
{
	int i = 0,size=0;
	int ret = 0;
	int chl_x = 0;
	int chl_y = 0;
	int raw_data_min = 0;
	int raw_data_max = 0;
	int raw_data_size = 0;
	int retry = 3;
	char result_code[FTS_RESULT_CODE_LEN] = {0};
	struct focal_test_result *test_result = NULL;

	TPD_DMESG("%s: test item raw data,%d\n", __func__,test_num);
	if(!params || !result || !(params->channel_x_num)|| !(params->channel_y_num)){
		TPD_DMESG("%s: parameters invalid !\n", __func__);
		return -EINVAL;
	}
	chl_x = params->nomapping_channel_x_num;
	chl_y = params->nomapping_channel_y_num;
	raw_data_min = params->threshold.scap_raw_data_min;
	raw_data_max = params->threshold.scap_raw_data_max;
	raw_data_size = chl_x +chl_y;

	result_code[0] = test_num+'0';
	result_code[1] = 'F'; //default result_code is failed
	result_code[2] = '\0';
	size=raw_data_size*2;
	ret = focal_alloc_test_container(&test_result, size);
	if (ret) {
		TPD_DMESG("%s:alloc raw data container fail, ret=%d\n",
			__func__, ret);
		*result = NULL;
		return ret;
	}

	test_result->result = true;
	strncpy(test_result->test_name, "raw_data_test", FTS_TEST_NAME_LEN - 1);

	ret = focal_start_scan();
	if (ret<0) {
		TPD_DMESG("%s:scan fail, ret=%d\n", __func__, ret);
		test_result->result = false;
		goto test_finish;
	}
	/* get scap rawdata with water proof on&off,0xAC:Sc water,0xAB:Sc normal*/
	for (i = 0; i < retry; i++) {
		ret = focal_get_scap_raw_data_format_3427(SCAP_RAW_WATERPROOF,test_result->values,
			raw_data_size);
		if (ret<0) {
			TPD_DMESG("%s:get raw data fail, ret=%d\n",
				__func__, ret);

			test_result->result = false;
			goto test_finish;
		}
		ret = focal_get_scap_raw_data_format_3427(SCAP_RAW_NO_WATERPROOF,test_result->values+raw_data_size,
			raw_data_size);
		if (ret<0) {
			TPD_DMESG("%s:get raw data fail, ret=%d\n",
				__func__, ret);

			test_result->result = false;
			goto test_finish;
		}
	}
	/* compare raw data with shreshold */
	for (i = 0; i < (raw_data_size*2-chl_x); i++) {
		if(i>=chl_y && i < (chl_x+chl_y))
				continue;
		if ((test_result->values[i] < raw_data_min)
			|| (test_result->values[i] > raw_data_max)) {

			test_result->result = false;
			strncpy(tp_test_failed_reason, "-panel_reason",TP_TEST_FAILED_REASON_LEN);
			focal_print_test_data("raw data test fail",
				i / chl_y,
				i % chl_y,
				raw_data_max,
				raw_data_min,
				test_result->values[i]);
		}
	}

test_finish:
	if (test_result->result) {
		result_code[1] = 'P'; //test pass
		TPD_DEBUG("%s:raw data test pass\n", __func__);
	} else {
		TPD_DMESG("%s:raw data test ng\n", __func__);
	}
	tp_cap_test_status =(tp_cap_test_status && test_result->result);
	strncpy(test_result->result_code, result_code, FTS_RESULT_CODE_LEN - 1);
	*result = test_result;

	return ret;
}

static int focal_raw_data_test(
	struct focal_test_params *params,
	struct focal_test_result **result,char test_num)
{
	int i = 0;
	int ret = 0;
	int chl_x = 0;
	int chl_y = 0;
	int *raw_data_min = NULL;
	int *raw_data_max = NULL;
	int raw_data_size = 0;
	unsigned char orignal_value = 0;

	char result_code[FTS_RESULT_CODE_LEN] = {0};
	struct focal_test_result *test_result = NULL;

	TPD_DEBUG("%s: test item raw data,%d\n", __func__,test_num);
	if(!params || !result || !(params->channel_x_num)|| !(params->channel_y_num)){
		TPD_DMESG("%s: parameters invalid !\n", __func__);
		return -EINVAL;
	}
	chl_x = params->channel_x_num;
	chl_y = params->channel_y_num;
	raw_data_min = params->threshold.raw_data_min;
	raw_data_max = params->threshold.raw_data_max;
	raw_data_size = chl_x * chl_y;
	if(raw_data_size > (TX_MAX*RX_MAX))
		raw_data_size = TX_MAX*RX_MAX;

	result_code[0] = test_num+'0';
	result_code[1] = 'F'; //default result_code is failed
	result_code[2] = '\0';
	ret = focal_alloc_test_container(&test_result, raw_data_size);
	if (ret) {
		TPD_DMESG("%s:alloc raw data container fail, ret=%d\n",
			__func__, ret);
		*result = NULL;
		return ret;
	}

	test_result->result = true;
	strncpy(test_result->test_name, "raw_data_test", FTS_TEST_NAME_LEN - 1);

	ret = focal_write_reg(REG_3427_CHANG_FRE, MAX_FREQUENCY);
	if(ret<0){
		TPD_DMESG("%s:write_charnge_fre fail, ret=%d\n", __func__, ret);
		test_result->result = false;
		goto test_finish;
	}

	ret = focal_write_reg(REG_3427_FIR_EN, DISABLE);
	if(ret<0){
		TPD_DMESG("%s:write_charnge_fre fail, ret=%d\n", __func__, ret);
		test_result->result = false;
		goto test_finish;
	}
	ret = focal_read_reg(SCAPRAW_TYPE, &orignal_value);
	if(ret<0){
		TPD_DMESG("%s:read scapraw_type failed , ret=%d\n", __func__, ret);
		test_result->result = false;
		goto test_finish;
	}
	ret = focal_write_reg(SCAPRAW_TYPE,OVERALL_NORMALIZE);
	if(ret<0){
		TPD_DMESG("%s:writes capraw_type fail, ret=%d\n", __func__, ret);
		test_result->result = false;
		goto test_finish;
	}

	for (i = 0; i < FTS_RAWDATA_GET_TIMES; i++) {
		/* start scanning */
		ret = focal_start_scan();
		if (ret<0) {
			TPD_DEBUG("%s:scan fail, ret=%d\n", __func__, ret);
			test_result->result = false;
			goto test_finish;
		}

		ret = focal_get_raw_data_format(chl_x,chl_y,test_result->values,test_result->size);
		if (ret<0) {
			TPD_DMESG("%s:get raw data fail, ret=%d\n",
				__func__, ret);

			test_result->result = false;
			goto test_finish;
		}
	}

	/* compare raw data with shreshold */
	for (i = 0; i < raw_data_size; i++) {
		if ((test_result->values[i] < raw_data_min[i])
			|| (test_result->values[i] > raw_data_max[i])) {

			test_result->result = false;
			tp_cap_test_status = PANEL_REASON;
			strncpy(tp_test_failed_reason, "-panel_reason",TP_TEST_FAILED_REASON_LEN);
			focal_print_test_data("raw data test fail",
				i / chl_x,
				i % chl_x,
				raw_data_max[i],
				raw_data_min[i],
				test_result->values[i]);
		}
	}

	ret = focal_write_reg(0x16, orignal_value);
	if(ret<0){
		TPD_DMESG("%s:write_charnge_fre fail, ret=%d\n", __func__, ret);
		test_result->result = false;
		goto test_finish;
	}

test_finish:
	if (test_result->result) {
		result_code[1] = 'P'; //test pass
		TPD_DEBUG("%s:raw data test pass\n", __func__);
	} else {
		TPD_DMESG("%s:raw data test ng\n", __func__);
	}
	strncpy(test_result->result_code, result_code, FTS_RESULT_CODE_LEN - 1);
	*result = test_result;
	return ret;
}

static void focal_print_test_data(
	char *msg,
	int row,
	int col,
	int max,
	int min,
	int value)
{

	TPD_DMESG("%s:%s,data[%d, %d]=%d, range=[%d, %d]\n",
		__func__, msg, row, col, value, min, max);
}

static int focal_get_cb_data_format(int *data, size_t size,unsigned int chl_x,unsigned int chl_y)
{
	unsigned int i = 0;
	int ret = 0;
	size_t read_size = 0;
	unsigned int CurTx = 0;
	unsigned int CurRx = 0;
	unsigned int Count = 0;
	char *cb_data = NULL;
	if(!data||(0 == size) ||(0 == chl_x) ||(0 == chl_y)){
		TPD_DMESG("%s: parameters invalid !\n", __func__);
		return -EINVAL;
	}
	read_size = size;
	cb_data = kzalloc(read_size, GFP_KERNEL);
	if (!cb_data) {
		TPD_DMESG("%s:alloc mem fail\n", __func__);
		return -ENOMEM;
	}

	ret = focal_get_Scap_cb_data_3427( cb_data, read_size/2);

	if (ret < 0) {
		TPD_DEBUG("%s: get cb data failed\n", __func__);
		goto free_cb_data;
	}

	memset(data, 0, size);

	for (i = 0; i < size; i++){
			CurTx = i / chl_y;
			CurRx = i % chl_y;
			Count = CurTx*chl_y+ CurRx;
			data[Count] = cb_data[i];
			//TPD_DEBUG("%s:Scapcb_value[%d][%d]= %d\n",__func__, CurTx,CurRx,data[Count]);
	}


free_cb_data:

	kfree(cb_data);
	cb_data = NULL;

	return ret;
}

static int focal_cb_test(
	struct focal_test_params *params,
	struct focal_test_result **result,char test_num)
{
	int i = 0;
	int ret = 0;
	int chl_x = 0;
	int chl_y = 0;
	int cb_max = 0;
	int cb_min = 0;
	int cb_data_size = 0;

	int *cb_data = NULL;
	char result_code[FTS_RESULT_CODE_LEN] = {0};
	struct focal_test_result *test_result = NULL;

	TPD_DEBUG("%s: cb test start\n", __func__);
	if(!params || !result || !(params->channel_x_num) || !(params->channel_y_num)){
		TPD_DMESG("%s: parameters invalid !\n", __func__);
		return -EINVAL;
	}
	cb_min = params->threshold.cb_test_min;
	cb_max = params->threshold.cb_test_max;
	chl_x = params->nomapping_channel_x_num;
	chl_y = params->nomapping_channel_y_num;
	cb_data_size = (chl_x + chl_y)*2;

	result_code[0] = test_num +'0';
	result_code[1] = 'F'; //default result_code is failed
	result_code[2] = '\0';
	ret = focal_alloc_test_container(&test_result, cb_data_size);
	if (ret) {
		TPD_DEBUG("%s:alloc mem for cb test result fail\n", __func__);
		*result = NULL;
		return ret;
	}

	test_result->result = true;
	strncpy(test_result->test_name, "cb_test", FTS_TEST_NAME_LEN - 1);

	ret = focal_get_cb_data_format(test_result->values, cb_data_size,chl_x,chl_y);
	if (ret<0) {
		TPD_DEBUG("%s:get cb data fail, ret=%d\n", __func__, ret);
		test_result->result = false;
		goto test_finish;
	}

	TPD_DMESG("x= %d, y= %d \n", chl_x, chl_y);
	cb_data_size = cb_data_size - chl_x ;
	cb_data = test_result->values;
	for (i = 0; i < cb_data_size; i++) {
		if((i>=chl_y) && (i <(chl_x + chl_y)) )
			continue;
		if (cb_data[i] < cb_min || cb_data[i] > cb_max) {

			test_result->result = false;
			tp_cap_test_status  = PANEL_REASON;
			strncpy(tp_test_failed_reason, "-panel_reason",TP_TEST_FAILED_REASON_LEN);
			focal_print_test_data("cb test failed",
				i / chl_x,
				i % chl_x,
				cb_max,
				cb_min,
				cb_data[i]);
		}
	}

test_finish:
	if (test_result->result) {
		result_code[1] = 'P'; //test pass
		TPD_DEBUG("%s:cb test pass\n", __func__);
	} else {
		TPD_DMESG("%s:cb test ng!\n", __func__);
	}

	strncpy(test_result->result_code, result_code, FTS_RESULT_CODE_LEN);
	*result = test_result;
	return ret;
}

static int focal_get_short_circuit_data_3427(int *data, size_t size,unsigned int chl_x,unsigned int chl_y)
{
	int i = 0;
	int ret = 0;
	int retry = 5;
	int adc_data_tmp = 0;
	int read_size;
	int *read_data = NULL;
	int data_offset = 0, clbdata_ground = 0,clbdata_mutual = 0;
	int clbChannelNum = 0;
	int cg_data = 0,cc_data = 0,data_cal = 0,mutual = 0;
	int short_data_pass = 2000;
	int RSense = 57;
	int ground_read_data_count = 0;
	int channel_read_data_count = 0;
	int channel_data_count = 0;
	unsigned char IcValue = 0;

	if(!data||(0 == size) ||(0 == chl_x) ||(0 == chl_y)){
		TPD_DMESG("%s: parameters invalid !\n", __func__);
		return -EINVAL;
	}

	clbChannelNum = size/2;
	read_size = size+3;//79
	read_data = kzalloc(read_size*sizeof(int), GFP_KERNEL);
	if (!read_data) {
		TPD_DMESG("%s:alloc mem fail\n", __func__);
		return -ENOMEM;
	}
	ret = focal_enter_work();
	if (ret<0) {
		TPD_DMESG("%s:enter_work fail, ret=%d\n", __func__, ret);
		goto free_mem;
	}
	ret = focal_read_reg(REG_3427_IC_VERSION, &IcValue);
	if (ret<0) {
		TPD_DMESG("%s:get IC_VERSION fail, ret=%d\n", __func__, ret);
		goto free_mem;
	}
	ret = focal_enter_factory();
	if (ret<0) {
		TPD_DMESG("%s:get IC_VERSION fail, ret=%d\n", __func__, ret);
		goto free_mem;
	}
	for(i = 0; i < retry; i++){
	ret = focal_get_adc_data(read_data, read_size,chl_x,chl_y);
		if (ret < 0)
			continue;
		else
			break;
	}
	if (i == retry)
	{
		TPD_DMESG("%s:get adc data fail, ret=%d\n", __func__, ret);
		goto free_mem;
	}


	/* data check */
	data_offset = read_data[0]-FTS_ADC_HALF_RANGE;
	clbdata_ground = read_data[1];
	clbdata_mutual = read_data[2+size/2];
	data_cal = clbdata_mutual > (data_offset+116) ? clbdata_mutual : (data_offset+116);

	/* data exchange */
	for (i = 0; i < clbChannelNum; i++) {
		/* Channel and Ground */
		ground_read_data_count = i+2;
		cg_data = read_data[ground_read_data_count];
		if((2047 + data_offset) - cg_data <= 0)
			adc_data_tmp = short_data_pass;
		else{
				if (IcValue <= 0x05 || IcValue == 0xff){
					adc_data_tmp = (cg_data -data_offset +410)* 25/( FTS_ADC_FULL_RANGE + data_offset - cg_data ) -3;
				}
			else{
				if (clbdata_ground-cg_data<=0)
					adc_data_tmp = short_data_pass;
				else
					adc_data_tmp = (cg_data -data_offset +384)/(clbdata_ground-cg_data)*57-1;
			}
		}
		if(adc_data_tmp<0)
			adc_data_tmp = 0;

		data[i] = adc_data_tmp;

		/* Channel and Channel */

		channel_read_data_count = i+clbChannelNum+3;
		cc_data = read_data[channel_read_data_count];//i+3+38

		if (IcValue <= 0x05 || IcValue == 0xff){
			if(cc_data - clbdata_mutual < 0){
				adc_data_tmp = short_data_pass;
			}
			else{
				mutual = cc_data - data_cal;//2+38
				mutual = mutual ? mutual : 1;
				adc_data_tmp =( (2047 + data_offset - data_cal)*24/mutual -27) -6;
			}
		}
		else{
			if ( clbdata_mutual  - cc_data<= 0){
				adc_data_tmp = short_data_pass;
				}
			else
				adc_data_tmp = ( cc_data - data_offset - 123 ) * RSense / (clbdata_mutual - cc_data /*temp*/ ) - 2;
		}
		if((adc_data_tmp <0) && (adc_data_tmp >=(-240)))
			adc_data_tmp = 0;
		else if(adc_data_tmp <(-240)){
			adc_data_tmp = short_data_pass;
		}
		channel_data_count = i+clbChannelNum;
		data[channel_data_count] = adc_data_tmp;
	}
free_mem:
	if(NULL!=read_data)
	{
		kfree(read_data);
	}
	return ret;
}

static int focal_short_circuit_test(
	struct focal_test_params *params,
	struct focal_test_result **result,char test_num)
{
	int i = 0;
	int ret = 0;
	int chl_x = 0;
	int chl_y = 0;
	int adc_data_size = 0;
	int short_data_min = 0;

	char result_code[FTS_RESULT_CODE_LEN] = {0};
	struct focal_test_result *test_result = NULL;

	TPD_DEBUG("%s:short circuit test start\n", __func__);
	if(!params || !result || !(params->channel_x_num) || !(params->channel_y_num)){
		TPD_DMESG("%s: parameters invalid !\n", __func__);
		return -EINVAL;
	}
	chl_x = params->nomapping_channel_x_num;
	chl_y = params->nomapping_channel_y_num;
	adc_data_size = (chl_x + chl_y)*2;

	result_code[0] = test_num + '0';
	result_code[1] = 'F'; //default result_code is failed
	result_code[2] = '\0';
	ret = focal_alloc_test_container(&test_result, adc_data_size);
	if (ret) {
		TPD_DMESG("%s:alloc mem fail\n", __func__);
		*result = NULL;
		return ret;
	}

	test_result->result = true;
	strncpy(test_result->test_name,
		"short_circuit_test", FTS_TEST_NAME_LEN - 1);

	ret = focal_get_short_circuit_data_3427(test_result->values,adc_data_size,chl_x,chl_y);

	if (ret<0) {
		TPD_DMESG("%s:get short circuit data fail, ret=%d\n",
			__func__, ret);
		test_result->result = false;
		goto test_finish;
	}

	/* data compare */
	short_data_min = params->threshold.short_circuit_min;

	for (i = 0; i < adc_data_size; i++) {
		if (short_data_min > test_result->values[i]) {

			test_result->result = false;
			tp_cap_test_status = PANEL_REASON;
			strncpy(tp_test_failed_reason, "-panel_reason",TP_TEST_FAILED_REASON_LEN);
			focal_print_test_data("short circuit test fail:",
				i / chl_x,
				i % chl_x,
				0,
				short_data_min,
				test_result->values[i]);
		}
	}

test_finish:
	if (test_result->result) {
		result_code[1] = 'P'; //test pass
		TPD_DEBUG("%s:short circuit test pass\n", __func__);
	} else {
		TPD_DMESG("%s:short circuit test ng\n", __func__);
	}

	strncpy(test_result->result_code, result_code, FTS_RESULT_CODE_LEN);
	*result = test_result;

	return ret;
}


static int focal_adc_scan(void)
{
	int i = 0;
	int ret = 0;
	int scan_query_times = 50;

	u8 reg_val = 0;

	/* start adc sample */
	ret = focal_write_reg(REG_3427_ADC_SCAN, ENABLE); //enable adc short detect
	if (ret<0) {
		TPD_DMESG("%s:adc scan fail, ret=%d\n", __func__, ret);
		return ret;
	}
	msleep(FTS_ADC_SAMPLE_DELAY);

	for (i = 0; i < scan_query_times; i++) {
		ret = focal_read_reg(REG_3427_ADC_SCAN_STATUS, &reg_val);
		if (ret>0) {
			if (reg_val == 0) {
				TPD_DEBUG("%s:adc scan success\n", __func__);
				return 0;
			} else {
				TPD_DEBUG("%s:adc scan status:0x%02X\n",
					__func__, reg_val);
				msleep(FTS_ADC_SAMPLE_DELAY);
			}
		} else {
			return ret;
		}
	}

	TPD_DMESG("%s:adc scan timeout\n", __func__);
	return -ETIMEDOUT;
}

static int focal_get_adc_data(int *data, size_t size,unsigned int chl_x,unsigned int chl_y)
{
	unsigned int i = 0;
	int ret = 0;
	int pkg_size = 0;
	int pkg_count = 0;
	int readed_count = 0;
	int adc_data_size = 0;
	int  temp_data= 0;
	u8 cmd[2] = {0};
	u8 *adc_data = NULL;
	unsigned int CurTx = 0;
	unsigned int CurRx = 0;
	unsigned int Count = 0;
	if(!data||(0 == size) ||(0 == chl_x) ||(0 == chl_y)){
		TPD_DMESG("%s: parameters invalid !\n", __func__);
		return -EINVAL;
	}
	adc_data_size = size * 2;
	pkg_count = adc_data_size / BYTES_PER_TIME;
	if (adc_data_size % BYTES_PER_TIME != 0)
		pkg_count += 1;

	adc_data = kzalloc(adc_data_size, GFP_KERNEL);
	if (!adc_data) {
		TPD_DMESG("%s:alloc mem fail\n", __func__);
		return -ENOMEM;
	}

	focal_start_scan();
	ret = focal_adc_scan();
	if (ret<0) {
		TPD_DMESG("%s:adc scan fail, ret=%d\n", __func__, ret);
		goto free_mem;
	}
	msleep(FTS_ADC_READ_DELAY);
	cmd[0] = REG_3427_ADC_DATA_ADDR;

	for (i = 0; i < pkg_count; i++) {

		/* if the last package is not full, set package size */
		if ((i + 1 == pkg_count)
			&& (adc_data_size % BYTES_PER_TIME != 0)) {

			pkg_size = adc_data_size % BYTES_PER_TIME;
		} else {
			pkg_size = BYTES_PER_TIME;
		}

		/* the first package should send a command and read */
		if (i == 0) {
			ret = focal_read(cmd, 1, adc_data, pkg_size);
		} else {
			ret = focal_read_default(adc_data + readed_count, pkg_size);
		}

		if (ret<0) {
			TPD_DMESG("%s:read adc data fail, ret=%d\n",
				__func__, ret);
			goto free_mem;
		}

		readed_count += pkg_size;
	}

	TPD_DEBUG("%s:readed cout=%d, size=%d\n", __func__,
		readed_count, size);

	for (i = 0; i < size; i++){
		Count = i * 2;
		temp_data =( adc_data[Count] << 8);
		Count ++;
		temp_data |=  adc_data[Count];
		CurTx = i / chl_y;
		CurRx = i % chl_y;
		Count = CurTx*chl_y + CurRx;
		data[Count] = temp_data;
		TPD_DEBUG("%s:CurTx= %d , Rx =%d ,i =%d adc = %d \n",__func__, CurTx,CurRx,i, data[Count]);
	}

	TPD_DEBUG("%s:copy data to buff finished\n", __func__);

free_mem:
	kfree(adc_data);
	adc_data = NULL;

	return ret;
}

static int focal_get_statistics_data(
	int *data, size_t data_size, char *result, size_t res_size)
{
	int avg = 0;
	int min = 0;
	int max = 0;

	if (!data) {
		TPD_DMESG("%s:data is null\n", __func__);
		return -ENODATA;
	}

	if (!result) {
		TPD_DMESG("%s:result is null\n", __func__);
		return -ENODATA;
	}

	if (data_size <= 0 || res_size <= 0) {
		TPD_DMESG("%s:%s, data_size=%d, res_size=%d\n", __func__,
			"input parameter is illega",
			data_size, res_size);
		return -EINVAL;
	}

	avg = focal_get_int_average(data, data_size);
	min = focal_get_int_min(data, data_size);
	max = focal_get_int_max(data, data_size);

	memset(result, 0, res_size);
	return snprintf(result, res_size, "[%4d,%4d,%4d]", avg, max, min);
}

static void focal_put_test_result(
	struct focal_test_params *params,
	struct focal_rawdata_info *info,
	struct focal_test_result *test_results[],
	int size)
{
	int i = 0;
	int j = 0;
	int buff_index = 0;
	char statistics_data[FTS_STATISTICS_DATA_LEN] = {0};

	/* put test result */
	for (i = 0; i < size; i++) {

		if (!test_results[i]) {
			TPD_DEBUG("%s:test result is null, index=%d\n",
				__func__, i);
			focal_strncat(info->result, "FF", TS_RAWDATA_RESULT_MAX);

			continue;
		}

		focal_strncat(info->result, test_results[i]->result_code,
			TS_RAWDATA_RESULT_MAX);
		if (i != size - 1)
			focal_strncat(info->result, "-", TS_RAWDATA_RESULT_MAX);
	}

	/* put statistics data */
	for (i = 0; i < size; i++) {
		if (!test_results[i]) {
			TPD_DEBUG("%s:test result is null, index=%d\n",
				__func__, i);
			continue;
		}

		focal_get_statistics_data(test_results[i]->values,
			test_results[i]->size,
			statistics_data,
			FTS_STATISTICS_DATA_LEN);

		focal_strncat(info->result, statistics_data,
			TS_RAWDATA_RESULT_MAX);
	}

	info->result[TEST_RESULT_LEN_MAX] = '\0';
	/* put test failed reason */
	if(tp_cap_test_status)//TEST_SUCCESS = 0
	{
		focal_strncat(info->result, tp_test_failed_reason,TP_TEST_FAILED_REASON_LEN);
	}
	/* put test data */
	memset(info->buff, 0, TS_RAWDATA_BUFF_MAX);
	info->buff[buff_index++] = params->channel_x_num;
	info->buff[buff_index++] = params->channel_y_num;
	for (i = 0; i < size; i++) {

		if (!test_results[i]) {
			TPD_DEBUG("%s:test result is null, index=%d\n",
				__func__, i);
			continue;
		}

		for (j = 0; j < test_results[i]->size; j++) {

			if (buff_index >= TS_RAWDATA_BUFF_MAX) {
				TPD_DEBUG("%s:buff is full, len=%d\n",
					__func__, buff_index);
				break;
			}

			info->buff[buff_index++] = test_results[i]->values[j];
		}
	}

	info->used_size = buff_index;

}


 int focal_get_cb_debug_data_( int *data, size_t size,int chl_x,int chl_y)
{
	int ret = 0;
	TPD_DEBUG("%s: cb debug  test start\n", __func__);
	if(!data || (0 == chl_x) || ( 0 == chl_y)){
		TPD_DEBUG("%s: parameters is invalid .\n", __func__);
		return -EINVAL;
	}
	ret = focal_enter_factory();
	if (ret<0) {
		TPD_DMESG("%s:enter factory mode fail, ret=%d\n",__func__, ret);
		goto test_finish;
	}

	ret = focal_get_cb_data_format(data, size,chl_x,chl_y);
	if (ret<0) {
		TPD_DEBUG("%s:get cb data fail, ret=%d\n", __func__, ret);
		goto test_finish;
	}
test_finish:
	return ret;
}
static int focal_init_test_prm(
	struct focal_test_params *params,
	struct focal_rawdata_info *info)
{
	int ret = 0;

	TPD_DEBUG("%s: set param data called\n", __func__);
	if( !(params) || !(info)){
		TPD_DEBUG("%s: parameters invalid!\n", __func__);
		return -EINVAL;
	}
	ret = focal_get_channel_form_ic(params);
	if (ret < 0) {
		TPD_DMESG("%s:get channel num fail,I2C communication error! ret=%d\n", __func__, ret);
		focal_strncat(info->result, "0F-1F-2F-3F-4F-FOCAL-FT3427-BG2", TS_RAWDATA_RESULT_MAX);
		focal_strncat(info->result, "i2c communication error ", TS_RAWDATA_RESULT_MAX);
		return ret;
	}
	else{
		TPD_DEBUG("%s: i2c test pass.ret =%d\n", __func__,ret);
		focal_strncat(info->result, "0P-", TS_RAWDATA_RESULT_MAX);
	}

	TPD_DEBUG("%s: set param data success\n", __func__);

	return 0;
}


static void focal_rawdata_proc_printf(struct seq_file *s, struct focal_rawdata_info *info,
					int range_size, int row_size ,int nomapping_range_size,int nomapping_row_size)
{
	int index = 0;
	int index1 = 0;
	 int count = 0;
	int tmp=0;
	int i =0;
	seq_printf(s,"rawdata begin\n");
	TPD_DMESG("rawdata begin\n");

	for(index=0; index < range_size; index++){
		for(index1=0; index1 < row_size; index1++ ){
			seq_printf(s,"%5d,", info->buff[2 + count]);
			printk("%5d,", info->buff[2 + count]);
			count++;
		}
		seq_printf(s, "\n");
		TPD_DMESG("\n");
	}
	seq_printf(s,"rawdata end\n");
	TPD_DMESG("rawdata end\n");

	seq_printf(s,"cb begin\n");
	TPD_DMESG("cb begin\n");
	for(i=0; i<4; i++)
	{
		if(tmp %2 == 0){
			for(index=0; index < nomapping_row_size; index++ ){
				seq_printf(s, "%3d,", info->buff[2 + count]);
				printk("%3d,", info->buff[2 + count]);
				count++;
			}
			seq_printf(s, "\n");
			printk("\n");
			tmp = 1;
		}
		else{
			for(index=0; index < nomapping_range_size; index++ ){
				count++;
			}
			tmp =2;
		}
	}
	seq_printf(s,"cb end\n");
	TPD_DMESG("cb end\n");

	seq_printf(s,"scapraw begin\n");
	TPD_DMESG("scapraw begin\n");
	for(i=0; i<4; i++)
	{
		if(tmp %2 == 0){
			for(index=0; index < nomapping_row_size; index++ ){
				seq_printf(s, "%5d,", info->buff[2 + count]);
				printk("%5d,", info->buff[2 + count]);
				count++;
			}
			seq_printf(s, "\n");
			printk("\n");
			tmp = 1;
		}
		else{
			for(index=0; index < nomapping_range_size; index++ ){
				count++;
			}
			tmp =2;
		}
	}
	seq_printf(s,"scapraw end\n");
	TPD_DMESG("scapraw end\n");

	seq_printf(s,"short begin\n");
	TPD_DMESG("short begin\n");

	seq_printf(s,"Channel and Ground\n");
	TPD_DMESG("Channel and Ground\n");
	for(index1=0; index1 < nomapping_row_size; index1++ ){
			seq_printf(s,"%5d,", info->buff[2 + count]);
			printk("%5d,", info->buff[2 + count]);
			count++;

	}
	seq_printf(s, "\n");
	TPD_DMESG("\n");

	for(index=0; index < nomapping_range_size; index++ ){
			seq_printf(s,"%5d,", info->buff[2 + count]);
			printk("%5d,", info->buff[2 + count]);
			count++;
	}
	seq_printf(s, "\n");
	TPD_DMESG("\n");

	seq_printf(s,"Channel and channel\n");
	TPD_DMESG("Channel and channel\n");
	for(index1=0; index1 < nomapping_row_size; index1++ ){
			seq_printf(s,"%5d,", info->buff[2 + count]);
			printk("%5d,", info->buff[2 + count]);
			count++;

	}
	seq_printf(s, "\n");
	TPD_DMESG("\n");

	for(index=0; index < nomapping_range_size; index++ ){
			seq_printf(s,"%5d,", info->buff[2 + count]);
			printk("%5d,", info->buff[2 + count]);
			count++;
	}
	seq_printf(s, "\n");
	TPD_DMESG("\n");

	seq_printf(s,"short end\n");
	TPD_DMESG("short end\n");

}


/************************************************************************
* Name: fts_test_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
int focal_test_show(struct seq_file *s, void *unused)
{
	int ret = 0;
	short row_size = 0;
	int range_size = 0;
	short nomapping_row_size = 0;
	int nomapping_range_size = 0;
	u8 fw_ver = 0;
	struct focal_rawdata_info *info = NULL;

	TPD_DEBUG("%s:focal get rawdata called\n", __func__);

	if(FTS_SELF_TEST_TRUE == ft3427_data.rawdata_mode)
	{
		TPD_DMESG("%s:self test is ongoing\n", __func__);
		return -EIO;
	}
	if (NULL == s) {
		TPD_DMESG("%s:seq point is NULL\n", __func__);
		return -EINVAL;
	}

	wake_lock(&(ft3427_data.tp_ft3427_selftest_lock));
	mutex_lock(&ft3427_data.input_dev->mutex);
	disable_irq(ft3427_data.client->irq);
	memset(tp_test_failed_reason, 0, TP_TEST_FAILED_REASON_LEN);
	ft3427_data.rawdata_mode = FTS_SELF_TEST_TRUE;
#ifdef FTS_ESD_CHECK
	focal_esdcheck_switch(FTS_ESD_DISABLE);
#endif
	info = kzalloc(sizeof(struct focal_rawdata_info), GFP_KERNEL);
	if (!info) {
		TPD_DMESG("%s:alloc mem for info fail\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_mem1;
	}

	ret = focal_init_test_prm( ft3427_data.params, info);
	if (ret < 0) {
		TPD_DMESG("%s:get param from dts fail, ret=%d", __func__, ret);
		ret  = 0;//use for printf I2c error information
		seq_printf(s,"0F-1F-2F-3F-4F-FOCAL-FT3427-BG2 \n");
		TPD_DMESG("0F-1F-2F-3F-4F-FOCAL-FT3427-BG2 \n");
		goto free_mem;
	}

	ret = focal_enter_factory();
	if (ret<0) {
		TPD_DMESG("%s:enter factory mode fail, ret=%d\n",
			__func__, ret);
		goto free_mem;
	}

	if ( focal_start_test_tp(ft3427_data.params, info) > 0 )
		TPD_DMESG("%s:tp test pass\n", __func__);
	else
		TPD_DMESG("%s:tp test fail, ret=%d\n", __func__, ret);

	ret = focal_enter_work();
	if (ret < 0){
		TPD_DMESG("%s:enter work model fail, ret=%d\n", __func__, ret);
		goto free_mem;
	}

	ret = focal_read_reg(FTS_REG_FW_VER, &fw_ver);
	if (ret < 0){
		TPD_DMESG("%s:read firmware version fail, ret=%d\n", __func__, ret);
		goto free_mem;
	}

	seq_printf(s, "Result : %s-%s0x%x \n", info->result, "FOCAL-FT3427-BG2-FW_VERSION:", fw_ver);
	TPD_DMESG("Result : %s-%s0x%x \n", info->result, "FOCAL-FT3427-BG2-FW_VERSION:", fw_ver);
	seq_printf(s, "*************touch data*************\n");
	TPD_DMESG("*************touch data*************\n");

	range_size = info->buff[0];
	row_size = info->buff[1];

	nomapping_range_size  = ft3427_data.params->nomapping_channel_x_num;
	nomapping_row_size = ft3427_data.params->nomapping_channel_y_num;

	seq_printf(s,"rx: %d, tx : %d\n", row_size, range_size);
	TPD_DMESG("rx: %d, tx : %d\n", row_size, range_size);

	focal_rawdata_proc_printf(s, info, range_size, row_size,nomapping_range_size,nomapping_row_size  );

free_mem:
	if(info)
	{
		kfree(info);
		info = NULL;
	}
err_alloc_mem1:
#ifdef FTS_ESD_CHECK
	focal_esdcheck_switch(FTS_ESD_ENABLE);
#endif
	ft3427_data.rawdata_mode = FTS_SELF_TEST_FALSE;
	enable_irq(ft3427_data.client->irq);
	mutex_unlock(&ft3427_data.input_dev->mutex);
	wake_unlock(&(ft3427_data.tp_ft3427_selftest_lock));
	focal_sw_reset();
	return 0;
}


 int focal_test_open(struct inode *inode, struct file *file)
{
	return single_open(file, focal_test_show, inode->i_private);
}

 const struct file_operations fts_test_fops = {
	.open = focal_test_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

int focal_test_init(struct i2c_client *client)
{
	int ret = 0;
	if(!ft3427_data.proc_root)
	{
		ft3427_data.proc_root = proc_mkdir(NAME_TOUCHSCREEN_PROC_DIR, NULL);
		if (!ft3427_data.proc_root)
		{
			TPD_DMESG("unable to create /proc/%s entry\n", NAME_TOUCHSCREEN_PROC_DIR);
			return -EINVAL;
		}
	}

	fts_proc_test = proc_create(FT_SELFTEST_NAME, S_IRUSR | S_IRGRP | S_IROTH, ft3427_data.proc_root, &fts_test_fops);
	if (NULL == fts_proc_test)
	{
		TPD_DMESG("Couldn't create proc upgrade entry!\n");
		return -EINVAL;
	}
	else
	{
		TPD_DMESG("Create proc upgrade entry success!\n");
	}

	ft3427_data.params = kzalloc(sizeof(struct focal_test_params), GFP_KERNEL);
	if (!ft3427_data.params) {
		TPD_DMESG("%s:alloc mem for params fail\n", __func__);
		return -ENOMEM;
	}
	ret = focal_parse_cap_test_config( ft3427_data.params);
	if (ret < 0) {
		TPD_DMESG("%s: analysis tp test data failed\n", __func__);
		return -EIO;
	}

	return 0;
}

int fts_test_exit(struct i2c_client *client)
{

	remove_proc_entry(FT_SELFTEST_NAME, ft3427_data.proc_root);
	return 0;
}

