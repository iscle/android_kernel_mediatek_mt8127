/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
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

//#ifndef __LINUX_FTXXXX_H__
//#define __LINUX_FTXXXX_H__
#ifndef __FOCALTECH_CORE_H__
#define __FOCALTECH_CORE_H__
 /*******************************************************************************
*
* File Name: focaltech_core.h
*
*    Author: Xu YongFeng
*
*   Created: 2015-01-29
*
*  Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/

#include "tpd.h"
#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/syscalls.h>
#include <linux/byteorder/generic.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>
#include <asm/unistd.h>
#include "mt_boot_common.h"
#include <linux/of_irq.h>
#include <linux/jiffies.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/mount.h>
#include <linux/unistd.h>
#include <linux/proc_fs.h>
#include <linux/netdevice.h>
#include <../fs/proc/internal.h>
#include <linux/wakelock.h>
#include <linux/firmware.h>

#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#include <linux/gpio.h>
#include <dsm/dsm_pub.h>
#endif

#ifdef CONFIG_GET_HARDWARE_INFO
#include <asm/hardware_info.h>
#define TP_HARDWARE_INFO "focaltech_ft3427_"
#endif

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#ifdef CONFIG_HUAWEI_DSM
#define FT3427_DSM_TYPE_FW_LEN_ERR 0
#define FT3427_DSM_TYPE_CHECK_SUM_ERR 1
#define FT3427_DSM_TYPE_PARSE_FW_ERR 2
#define FT3427_DSM_TYPE_UPDATE_FW_ERR 3
#define FT3427_DSM_TYPE_I2C_RW_ERR 4

#define TPD_FLAG_SLEEP 0
#define TPD_FLAG_WAKE 1

#define CLIENT_NAME_TOUCH_FT3427    "dsm_tp"

#define FT3427_GPIO_HW_INFO  "i2c sda num:%d,status:%d;i2c scl num:%d,status:%d;irq gpio num:%d,status:%d;reset gpio num:%d,status:%d,"
#define FT3427_MSG_FW_LEN_ERR "firmware length error\n"
#define FT3427_MSG_FW_CHKSUM_ERR  "firmware checksum error\n"
#define FT3427_MSG_FW_PARSE_ERR  "firmware parse error\n"
#define FT3427_MSG_I2C_RW_ERR  "i2c write/read error\n"
#define FT3427_MSG_UPDATE_FW_ERR  "fw update process error\n"
#define FT3427_MSG_UNKNOW_ERR  "unknow error\n"
#define FT3427_GPIO_OFFSET 881
#define FT3427_GPIO_I2C_SDA 57
#define FT3427_GPIO_I2C_SCL 58
#define FT3427_GPIO_IRQ 29
#define FT3427_GPIO_RST 45
#define FT3427_GET_GPIO_VALUE(x)  gpio_get_value(x)
#define FT3427_MSG_MAX_LEN 256

#define MTK_TP_RST_GPIO_NUM    0
#define MTK_TP_INT_GPIO_NUM    1

#define NAME_TOUCHSCREEN_PROC_DIR "touchscreen"
#define FT3427_SUB_MSG_MAX_LEN(x) (FT3427_MSG_MAX_LEN-sizeof(x))
#define FT3427_SHOW_GPIO_INFO  \
		FT3427_GPIO_I2C_SDA,FT3427_GET_GPIO_VALUE((FT3427_GPIO_I2C_SDA+FT3427_GPIO_OFFSET)),\
		FT3427_GPIO_I2C_SCL,FT3427_GET_GPIO_VALUE((FT3427_GPIO_I2C_SCL+FT3427_GPIO_OFFSET)),\
		FT3427_GPIO_IRQ,FT3427_GET_GPIO_VALUE(FT3427_GPIO_IRQ+FT3427_GPIO_OFFSET),\
		FT3427_GPIO_RST,FT3427_GET_GPIO_VALUE(FT3427_GPIO_RST+FT3427_GPIO_OFFSET)
#endif


#define FT_READ_CHAR_NUM_1   1
#define FT_READ_CHAR_NUM_2   2
#define FT_READ_CHAR_NUM_3   3
#define FT_READ_CHAR_NUM_4   4

#define FT_WRITE_CHAR_NUM_1   1
#define FT_WRITE_CHAR_NUM_2   2
#define FT_WRITE_CHAR_NUM_3   3
#define FT_WRITE_CHAR_NUM_4   4


/* report related define macro */
#define TPD_FINGER_NUM_MASK   0x0f
#define TPD_DEVICE_MODE_MASK   0x07
#define TPD_DEVICE_MODE_SHIFT   4
#define CFG_MAX_TOUCH_POINTS        5
#define TPD_SUPPORT_POINTS    5
#define REG_TOUCH_INFO       0x00
#define FTS_RAWDATA_GET_TIMES    3
#define FTS_DTS_COMCOPATIBLE   "mediatek,FT3427"
/* 0:R-touch, 1: Cap-touch */
#define FTS_TOUCH_TYPE_CAP   1

#define FTS_MAX_ID                          0x0F
#define FTS_TOUCH_STEP                      6
#define FTS_TOUCH_X_H_POS                   (3 - REG_TOUCH_INFO)
#define FTS_TOUCH_X_L_POS                   (4 - REG_TOUCH_INFO)
#define FTS_TOUCH_Y_H_POS                   (5 - REG_TOUCH_INFO)
#define FTS_TOUCH_Y_L_POS                   (6 - REG_TOUCH_INFO)
#define FTS_TOUCH_EVENT_POS                 (3 - REG_TOUCH_INFO)
#define FTS_TOUCH_ID_POS                    (5 - REG_TOUCH_INFO)
#define FT_TOUCH_POINT_NUM                  (2 - REG_TOUCH_INFO)
#define FTS_TOUCH_XY_POS                    (7 - REG_TOUCH_INFO)
#define FTS_TOUCH_MISC                      (8 - REG_TOUCH_INFO)
#define POINT_READ_BUF_LEN                  (3 - REG_TOUCH_INFO + FTS_TOUCH_STEP * TPD_SUPPORT_POINTS)
#define FT_FW_NAME_MAX_LEN              50
#define FTS_ID_SHIFT_BIT        4
#define FTS_ID_REG_NUM       2

#define GPIO_OUTPUT_HIGH_LEVEL  1
#define GPIO_OUTPUT_LOW_LEVEL   0
#define APK_DEBUG_ON            1
#define APK_DEBUG_OFF           0
#define ESD_SWITCH_ON           1
#define ESD_SWITCH_OFF          0
#define ESD_SUSPEND_SWITCH_ON           1
#define ESD_SUSPEND_SWITCH_OFF          0
#define FTS_STATUS_HALT         0
#define FTS_STATUS_NOTHALT      1
#define FTS_LOADING_FW_TRUE     1
#define FTS_LOADING_FW_FALSE    0
#define FTS_SELF_TEST_TRUE      1
#define FTS_SELF_TEST_FALSE     0
#define FTS_ESD_ENABLE          1
#define FTS_ESD_DISABLE         0
#define FTS_INT_WORK_STATUS     1
#define FTS_INT_IDLE_STATUS     0



#define FT3427_CHIP_ID 0x54
#define FTS_POWER_LEVEL 2800000
#define FTS_I2C_ADDR 0x38
#define TPD_I2C_TEST_COUNT 3
#define TPD_DEVICE_NAME_LEN 8
#define TPD_I2C_TEST_ADDR  0x00
#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_MAX_RESET_COUNT 3
#define BYTES_PER_TIME          31

#define FIREWARE_LEN_MIN            8
#define FIREWARE_LEN_MAX            (64 * 1024)
#define UPGREDE_FILE_PATH_LENGTH    128
#define TOUCH_INFO_LEN              40
#define FT_MODULE_NAME_LEN          10
#define FTS_RESET_LOW_DELAY         10
#define FTS_POWERON_TRTP_DELAY         2
#define FTS_RESET_DELAY             200
#define FTS_ENTER_FACTORY_DELAY     50
#define FTS_HID_TO_STD_DELAY        50
#define FTS_ADC_SAMPLE_DELAY        50
#define FTS_ADC_READ_DELAY          300
#define FTS_RAWDATA_TYPE_SWITCH_DELAY  10
#define FTS_WAIT_TP_LOAD_SUCCESS  500
#define FTS_ESDCHECK_WAIT_TIME      1000
#define FTS_ESD_MAX_TIMES           3
#define FTS_ADC_FULL_RANGE          2047
#define FTS_ADC_HALF_RANGE          1024


/*register address*/
#define FTS_REG_WORKMODE                0x00
#define FTS_REG_WORKMODE_FACTORY_VALUE  0x40
#define FTS_REG_WORKMODE_WORK_VALUE     0x00
#define FTS_REG_WORKMODE_FACTORY_MASK   0x70

#define FTS_REG_CHIP_ID                 0xA3                // chip ID
#define FTS_REG_FW_VER                  0xA6            // FW  version
#define FTS_REG_VENDOR_ID               0xA8            // TP vendor ID
#define TPD_MAX_POINTS_2                2
#define TPD_MAX_POINTS_5                5
#define TPD_MAX_POINTS_10               10
#define AUTO_CLB_NEED                   1
#define AUTO_CLB_NONEED                 0

/*touch event info*/
struct ts_event
{
	u16 au16_x[CFG_MAX_TOUCH_POINTS];               /* x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];               /* y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];       /* touch event: 0 -- down; 1-- up; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];         /* touch ID */
	u16 pressure[CFG_MAX_TOUCH_POINTS];
	u16 area[CFG_MAX_TOUCH_POINTS];
	u8 touch_point;
	int touchs;
	u8 touch_point_num;
};

struct fts_esdcheck_st {
	u8 active:1;                /* 1- esd check active, need check esd 0- no esd check */
	u8 suspend:1;
	u8 proc_debug:1;            /* apk or adb is accessing I2C */
	u8 intr:1;                  /* 1- Interrupt trigger */
	u8 unused:4;
	u8 flow_work_hold_cnt;
	u8 flow_work_cnt_last;
	u32 hardware_reset_cnt;
	u32 i2c_nack_cnt;
	u32 i2c_dataerror_cnt;
};

struct fts_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	const struct ftxxxx_ts_platform_data *pdata;
	struct work_struct  touch_event_work;
	struct workqueue_struct *ts_workqueue;
	struct delayed_work esdcheck_work;
	struct workqueue_struct *esdcheck_workqueue;
	struct fts_esdcheck_st esdcheck_data;
	struct task_struct *thread_tpd;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	char fw_name[FT_FW_NAME_MAX_LEN];
	bool loading_fw;
	bool rawdata_mode;
	u8 family_id;
	struct dentry *dir;
	u16 addr;
	bool suspended;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 fw_ver[3];
	unsigned int irq;
	u8 fw_vendor_id;
	int apk_debug_flag;
	struct focal_test_params *params;
	struct proc_dir_entry *proc_root;
	struct proc_dir_entry *fts_proc_upgrade;
	struct proc_dir_entry *fts_proc_entry;
	struct wake_lock tp_ft3427_suspend_lock;
	struct wake_lock tp_ft3427_selftest_lock;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
	int tpd_flag;
};

/*******************************************************************************
* Customer function define
*
* Function Switchs: define to open,  comment to close
*
*******************************************************************************/
#define TPD_POWER_SOURCE_CUSTOM
#define TPD_AUTO_UPGRADE
#define FTS_APK_DEBUG
#define SYSFS_DEBUG
#define FTS_ESD_CHECK

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
extern struct fts_ts_data ft3427_data;
extern struct tpd_device *tpd;
extern int focal_test_init(struct i2c_client *client);
extern int ft3427_dsm_report_err(int errno,int type);
extern void focal_sw_reset(void);
extern int focal_write(u8 *writebuf, u16 writelen);
extern int focal_write_default(u8 value);
extern int focal_write_reg(u8 regaddr, u8 regvalue);
extern int focal_read(u8 *writebuf, u16 writelen, u8 *readbuf, u16 readlen);
extern int focal_read_default(u8 *readbuf, u16 readlen);
extern int focal_read_reg(u8 regaddr, u8 *regvalue);

extern int focal_ctpm_auto_upgrade(void *unused);
extern int focal_firmware_manual_update(const char *fw_name);

/* Apk and ADB functions */
extern int focal_create_sysfs(struct i2c_client *client);
extern void focal_remove_sysfs(struct i2c_client *client);
extern int focal_create_apk_debug_channel(void);
extern void focal_remove_apk_debug_channel(void);

char *focal_strncat(char *dest, char *src, size_t dest_size);
void focal_esdcheck_switch(bool enable);

#endif
