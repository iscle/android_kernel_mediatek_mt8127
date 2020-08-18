/*******************************************************************
		Copyright 2008 - 2013, Huawei Tech. Co., Ltd.
		ALL RIGHTS RESERVED

Filename      : focaltech_test.h
Author        : wWX231245
Creation time : 2016/7/22
Description   :

Version       : 1.0
********************************************************************/

#ifndef _FOCALTECH_TEST_H_
#define _FOCALTECH_TEST_H_

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <focaltech_core.h>

#define bool  unsigned char
#define BYTE  unsigned char
#define false 0
#define true  1
#define ENABLE                          1
#define DISABLE                         0

#define TX_NUM_MAX          30
#define RX_NUM_MAX          30
#define DEVICE_MODE_ADDR        0x00
#define DEVICE_MODE_WORK        0x00
#define DEVICE_MODE_FACTORY     0x40

#define FTS_SCAN_MAX_TIME       160
#define FTS_SCAN_QUERY_DELAY        8

#define FTS_3427_SCAN_MAX_TIME          4000
#define FTS_3427_SCAN_QUERY_DELAY      16

#define BYTES_PER_TIME          31
#define MAX_RETRY_TIMES         3

#define FTS_RESULT_CODE_LEN     4
#define FTS_TEST_NAME_LEN       16
#define FTS_STATISTICS_DATA_LEN     32
#define FTS_DATA_TYPE_DIFF      1
#define FTS_DATA_TYPE_RAW       0
#define FTS_LCD_NOISE_TEST_FRAME_COUNT  50

#define FTS_LCD_PRE_FRAME_TIME      16
#define FTS_MAX_CAP_TEST_NUM        5

#define TEST_SUCCESS   0
#define SOFTWARE_REASON  1
#define PANEL_REASON   2
#define   TEST_RESULT_LEN_MAX               14
#define FTS_VENDOR_COMP_NAME_LEN        32
#define TS_RAWDATA_BUFF_MAX             2400
#define TS_RAWDATA_RESULT_MAX           100
#define DTS_RAW_DATA_MIN                "threshold,raw_data_min"
#define DTS_RAW_DATA_MAX                "threshold,raw_data_max"
#define DTS_CB_TEST_MIN                 "threshold,cb_test_min"
#define DTS_CB_TEST_MAX                 "threshold,cb_test_max"
#define DTS_SHORT_CIRCUIT_RES_MIN       "threshold,short_circuit_min"
#define DTS_LCD_NOISE_MAX               "threshold,lcd_noise_max"
#define DTS_OPEN_TEST_CB_MIN            "threshold,open_test_cb_min"
#define DTS_SCAP_RAW_DATA_MIN               "threshold,scap_raw_data_min"
#define DTS_SCAP_RAW_DATA_MAX               "threshold,scap_raw_data_max"

#define TX_MAX               14
#define RX_MAX               28

struct focal_test_threshold {
	int raw_data_min[TX_MAX*RX_MAX];
	int raw_data_max[TX_MAX*RX_MAX];

	int cb_test_min;
	int cb_test_max;

	int scap_raw_data_min;
	int scap_raw_data_max;
	int short_circuit_min;
};

struct focal_test_params {
	struct focal_test_threshold threshold;

	u8 channel_x_num;
	u8 channel_y_num;
	u8 nomapping_channel_x_num;
	u8 nomapping_channel_y_num;
	u8 key_num;
};

struct focal_test_result {
	int result;
	int *values;
	size_t size;
	char test_name[FTS_TEST_NAME_LEN];
	char result_code[FTS_RESULT_CODE_LEN];
};




struct focal_rawdata_info
{
	int used_size; // fill in rawdata size
	int buff[TS_RAWDATA_BUFF_MAX];
	char result[TS_RAWDATA_RESULT_MAX];
};


#endif
