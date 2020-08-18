/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include "focaltech_core.h"
#include "tpd.h"

/*******************************************************************************
*  Static functions
*******************************************************************************/
static irqreturn_t focal_eint_interrupt_handler(int irq, void *dev_id);
static int focal_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int focal_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int focal_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static void focal_resume(struct device *h);
static void focal_suspend(struct device *h);

/*******************************************************************************
* variables
*******************************************************************************/
struct fts_ts_data ft3427_data;

static int tpd_rst_gpio_number = MTK_TP_RST_GPIO_NUM;
static int tpd_int_gpio_number = MTK_TP_INT_GPIO_NUM;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_rw_access);

struct touch_info {
	int y[TPD_SUPPORT_POINTS];
	int x[TPD_SUPPORT_POINTS];
	int p[TPD_SUPPORT_POINTS];
	int id[TPD_SUPPORT_POINTS];
	int count;
};

static const struct i2c_device_id ft3427_tpd_id[] = {{"ft3427", 0}, {} };
static const struct of_device_id ft3427_dt_match[] = {
	{.compatible = FTS_DTS_COMCOPATIBLE},
	{},
};
MODULE_DEVICE_TABLE(of, ft3427_dt_match);

static struct i2c_driver focal_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ft3427_dt_match),
		.name = "ft3247",
	},
	.probe = focal_probe,
	.remove = focal_remove,
	.id_table = ft3427_tpd_id,
	.detect = focal_i2c_detect,
};

struct dsm_client *ft3427_touch_dclient = NULL;
static struct dsm_dev dsm_touch_ft3427 = {
	.name       = CLIENT_NAME_TOUCH_FT3427,
	.fops       = NULL,
	.buff_size  = DSM_SENSOR_BUF_MAX,
};

char *focal_strncat(char *dest, char *src, size_t dest_size)
{
	size_t dest_len = 0;
	char *start_index = NULL;

	if((!dest) || (!src))
	{
		TPD_DMESG("%s: dest or src is NULL\n", __func__);
		return NULL;
	}

	dest_len = strnlen(dest, dest_size);
	start_index = dest + dest_len;

	return strncat(&dest[dest_len], src, dest_size - dest_len - 1);
}

#ifdef CONFIG_HUAWEI_DSM
int ft3427_dsm_report_err(int errno, int type)
{
	int size = 0;
	char *msg = NULL;

	if(!ft3427_touch_dclient)
	{
		TPD_DMESG("%s: ft3427_touch_dclient is null\n", __func__);
		return -EFAULT;
	}

	msg = (char *)kmalloc(FT3427_MSG_MAX_LEN, GFP_KERNEL);
	if (NULL == msg)
	{
		TPD_DMESG("%s: kmalloc failed\n", __func__);
		return -ENOMEM;
	}

	if(dsm_client_ocuppy(ft3427_touch_dclient))
	{
		/* buffer is busy */
		TPD_DMESG("%s: buffer is busy!, errno = %d\n", __func__,errno);
		kfree(msg);
		msg = NULL;
		return -EBUSY;
	}

	memset(msg, 0, FT3427_MSG_MAX_LEN);
	snprintf(msg, FT3427_MSG_MAX_LEN, FT3427_GPIO_HW_INFO, FT3427_SHOW_GPIO_INFO);
	if(FT3427_DSM_TYPE_FW_LEN_ERR == type)
	{
		strncat(msg, FT3427_MSG_FW_LEN_ERR, FT3427_SUB_MSG_MAX_LEN(msg));
	}
	else if(FT3427_DSM_TYPE_CHECK_SUM_ERR == type)
	{
		strncat(msg, FT3427_MSG_FW_CHKSUM_ERR, FT3427_SUB_MSG_MAX_LEN(msg));
	}
	else if(FT3427_DSM_TYPE_PARSE_FW_ERR == type)
	{
		strncat(msg, FT3427_MSG_FW_PARSE_ERR, FT3427_SUB_MSG_MAX_LEN(msg));
	}
	else if(FT3427_DSM_TYPE_UPDATE_FW_ERR == type)
	{
		strncat(msg, FT3427_MSG_UPDATE_FW_ERR, FT3427_SUB_MSG_MAX_LEN(msg));
	}
	else if(FT3427_DSM_TYPE_I2C_RW_ERR == type)
	{
		strncat(msg, FT3427_MSG_I2C_RW_ERR, FT3427_SUB_MSG_MAX_LEN(msg));
	}
	else
	{
		strncat(msg, FT3427_MSG_UNKNOW_ERR, FT3427_SUB_MSG_MAX_LEN(msg));
	}

	size = dsm_client_record(ft3427_touch_dclient, msg);
	dsm_client_notify(ft3427_touch_dclient, errno);

	kfree(msg);
	msg = NULL;
	return size;
}
#endif

static int of_get_ft3427_platform_data(struct device *dev)
{
	if(NULL == dev)
	{
		TPD_DMESG("%s:Error: No device match found\n", __func__);
		return -EINVAL;
	}
	if (dev->of_node) {
		const struct of_device_id *match;
		match = of_match_device(of_match_ptr(ft3427_dt_match), dev);
		if (!match) {
			TPD_DMESG("Error: No device match found\n");
			return -ENODEV;
		}
	}
	return 0;
}

void focal_sw_reset(void)
{
	tpd_gpio_output(tpd_rst_gpio_number, GPIO_OUTPUT_HIGH_LEVEL);
	msleep(FTS_RESET_LOW_DELAY);
	tpd_gpio_output(tpd_rst_gpio_number, GPIO_OUTPUT_LOW_LEVEL);
	msleep(FTS_RESET_LOW_DELAY);
	tpd_gpio_output(tpd_rst_gpio_number, GPIO_OUTPUT_HIGH_LEVEL);
	msleep(FTS_RESET_DELAY);
}

static void focal_power_on(void)
{
#ifdef TPD_POWER_SOURCE_CUSTOM
	int ret = 0;
#endif
	tpd_gpio_output(tpd_rst_gpio_number, GPIO_OUTPUT_LOW_LEVEL);
	msleep(FTS_POWERON_TRTP_DELAY);
#ifdef TPD_POWER_SOURCE_CUSTOM
	ret = regulator_enable(tpd->reg);
	if (ret != 0)
		TPD_DMESG("failed to enable touch voltage: %d\n", ret);
#endif
	msleep(FTS_RESET_LOW_DELAY);
	tpd_gpio_output(tpd_rst_gpio_number, GPIO_OUTPUT_HIGH_LEVEL);
	msleep(FTS_RESET_DELAY);
}

#ifdef FTS_ESD_CHECK
void focal_esdcheck_switch(bool enable)
{
	if (enable == ESD_SWITCH_ON) {
		if (ft3427_data.esdcheck_data.active == false) {
			TPD_DEBUG("[ESD]: ESD check start!!");
			ft3427_data.esdcheck_data.active = true;
			queue_delayed_work(ft3427_data.esdcheck_workqueue,
			&ft3427_data.esdcheck_work, msecs_to_jiffies(FTS_ESDCHECK_WAIT_TIME));
		}
	}
	else {
		if (ft3427_data.esdcheck_data.active == true) {
			TPD_DEBUG("[ESD]: ESD check stop!!");
			ft3427_data.esdcheck_data.active = false;
			cancel_delayed_work_sync(&ft3427_data.esdcheck_work);
		}
	}
}

static int focal_esdcheck_chip_id(void)
{
	int ret = 0;
	int i = 0;
	u8 reg_value = 0;
	u8 cmd = 0;

	for (i = 0; i < FTS_ESD_MAX_TIMES; i++){
		cmd = FTS_REG_CHIP_ID;
		ret = focal_read(&cmd, FT_WRITE_CHAR_NUM_1, &reg_value, FT_READ_CHAR_NUM_1);
		if ( ret < 0 ){
			TPD_DMESG("%s:[ESD] Read Reg 0xA3 failed ret = %d \n", __func__, ret);
			continue;
		}

		if (reg_value == FT3427_CHIP_ID){
			TPD_DEBUG("%s:chip id read success, chip id:0x%X, i=%d\n",__func__, reg_value,i);
			break;
		}
	}

	if(FTS_ESD_MAX_TIMES == i){
		TPD_DMESG("%s:chip id read fail, reg_value=%d, i=%d, chipid_high=%d, \n",__func__, reg_value, i, FT3427_CHIP_ID);
		return -EIO;
	}

	return 0;
}


static int focal_esdcheck_algorithm(void)
{
	int ret = 0;
	u8 reg_value = 0;
	u8 cmd = 0;
	int hardware_reset = 0;

	TPD_DMESG("%s Enter\n", __func__);

	/* 1. esdcheck is interrupt, then return */
	if (FTS_INT_WORK_STATUS == ft3427_data.esdcheck_data.intr){
		TPD_DMESG("%s:[ESD]: In interrupt state, not check esd, return immediately! \n", __func__);
		return 0;
	}

	/* 2. check power state, if suspend, no need check esd */
	if (ESD_SUSPEND_SWITCH_ON == ft3427_data.esdcheck_data.suspend){
		TPD_DMESG("%s:[ESD]: In suspend, not check esd, return immediately!\n", __func__);
		/* because in suspend state, adb can be used, when upgrade FW, will active ESD check(active = 1)
		*  But in suspend, then will don't queue_delayed_work, when resume, don't check ESD again
		*/
		ft3427_data.esdcheck_data.active = false;
		return 0;
	}

	/* 3. In boot upgrade mode , can't check esd */
	if (true == ft3427_data.esdcheck_data.proc_debug){
		TPD_DMESG("%s:[ESD]: In boot upgrade state, not check esd, return immediately! \n", __func__);
		return 0;
	}

	/* 4. In factory mode, can't check esd */
	cmd = FTS_REG_WORKMODE;
	ret= focal_read(&cmd, 1, &reg_value, FT_READ_CHAR_NUM_1);
	if ( ret < 0 ){
		TPD_DMESG("%s : focal read FTS_REG_WORKMODE error\n", __func__);
	}
	else if ( (reg_value & FTS_REG_WORKMODE_FACTORY_MASK) ==  FTS_REG_WORKMODE_FACTORY_VALUE){
		TPD_DMESG("%s: [ESD]: In factory mode, not check esd, return immediately!!\n", __func__);
		return 0;
	}

	/* 5. Get Chip ID */
	hardware_reset = focal_esdcheck_chip_id();

	/* 6. If need hardware reset, then handle it here */
	TPD_DEBUG("%s esd reset begin hardware_reset = %d \n", __func__, hardware_reset);
	if (hardware_reset < 0){
		focal_sw_reset();
	}

	TPD_DMESG("%s esd reset end\n", __func__);
	return 0;
}

static void focal_esdcheck_func(struct work_struct *work)
{
	focal_esdcheck_algorithm();

	if (ft3427_data.esdcheck_data.suspend == ESD_SUSPEND_SWITCH_OFF) {
		queue_delayed_work(ft3427_data.esdcheck_workqueue,
			&ft3427_data.esdcheck_work, msecs_to_jiffies(FTS_ESDCHECK_WAIT_TIME));
	}
}

static int focal_esdcheck_init(void)
{
	INIT_DELAYED_WORK(&ft3427_data.esdcheck_work, focal_esdcheck_func);
	ft3427_data.esdcheck_workqueue = create_workqueue("focal_esdcheck_wq");
	if (ft3427_data.esdcheck_workqueue == NULL) {
		TPD_DMESG("%s failed to create esd work queue", __func__);
		return -ENOMEM;
	}

	focal_esdcheck_switch(FTS_ESD_ENABLE);
	return 0;
}
#endif

static void focal_down(int x, int y, int p, int id)
{
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(tpd->dev);
}

static void focal_up(int x, int y)
{
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);
}

static int focal_read_point(u8 *writebuf, u16 writelen, u8 *readbuf, u16 readlen)
{
	int i = 0;
	int ret = 0;
	int pkg_size = 0;
	int pkg_count = 0;
	int readed_count = 0;

	pkg_count = readlen / BYTES_PER_TIME;
	if (readlen % BYTES_PER_TIME != 0)
		pkg_count += 1;

	for (i = 0; i < pkg_count; i++) {
		/*
		 * compute pkg_size value
		 * if the last package is not equal to BYTES_PER_TIME,
		 * the package size will be set to (size % BYTES_PER_TIME)
		 */
		if ((readlen % BYTES_PER_TIME != 0) && (i + 1 == pkg_count))
			pkg_size = readlen % BYTES_PER_TIME;
		else
			pkg_size = BYTES_PER_TIME;

		/* first package shuold write cmd to ic */
		if (i == 0)
			ret = focal_read(writebuf, writelen, readbuf, pkg_size);
		else
			ret = focal_read_default(readbuf + readed_count, pkg_size);

		if (ret < 0) {
			TPD_DMESG("%s:read fail, ret=%d\n",__func__, ret);
			return ret;
		}

		readed_count += pkg_size;
	}

	return ret;
}

/*
 * return : return 0 if read touch info success, otherwize return error code
 */
static int focal_touchinfo(struct touch_info *cinfo)
{
	int i = 0;
	u8 data[POINT_READ_BUF_LEN] = {0};
	int ret = 0;
	u32 offset = 0;
	u8 point_addr = REG_TOUCH_INFO;

	if(NULL == cinfo) {
		TPD_DMESG("cinfo is NULL, line: %d\n", __LINE__);
		return -EINVAL;
	}

	ret = focal_read_point(&point_addr, 1, data, POINT_READ_BUF_LEN);
	if(ret < 0)
	{
		TPD_DMESG("I2C read touchinfo error, line: %d\n", __LINE__);
		return -EIO;
	}
	/* Device bit[6:4](Mode[2:0]) == 0 :Normal operating Mode*/
	if ((data[0] & 0x70) != 0)
		return -EIO;

	memset(cinfo, 0, sizeof(struct touch_info));
	for (i = 0; i < TPD_SUPPORT_POINTS; i++)
		cinfo->p[i] = 1;    /* Put up */

	/*get the number of the touch points*/
	cinfo->count = data[FTS_ID_REG_NUM] & TPD_FINGER_NUM_MASK;

	TPD_DEBUG("Number of touch points = %d\n", cinfo->count);

	for (i = 0; i < cinfo->count; i++) {
		offset = FTS_TOUCH_STEP * i;

		cinfo->id[i] = data[FTS_TOUCH_ID_POS + offset] >> FTS_ID_SHIFT_BIT;
		cinfo->p[i] = data[FTS_TOUCH_XY_POS + offset];
		/*get the X coordinate, 2 bytes*/
		cinfo->x[i] = (u16)(data[FTS_TOUCH_X_H_POS + offset] & 0x0F) << 8
			| (u16)(data[FTS_TOUCH_X_L_POS + offset]);

		/*get the Y coordinate, 2 bytes*/
		cinfo->y[i] = (u16)(data[FTS_TOUCH_Y_H_POS + offset] & 0x0F) << 8
			| (u16)(data[FTS_TOUCH_Y_L_POS + offset]);

		TPD_DEBUG(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i,
		cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
	}

	return 0;
}

int focal_i2c_read(struct i2c_client *client, u8 *writebuf,
			u16 writelen, u8 *readbuf, u16 readlen)
{
	int ret = 0;

	if( NULL == client || NULL == readbuf) {
		TPD_DMESG("%s i2c_client/readbuf is NULL", __func__);
		return -EINVAL;
	}

	if(writelen > 0 && NULL == writebuf) {
		TPD_DMESG("%s writebuf is NULL", __func__);
		return -EINVAL;
	}

	mutex_lock(&i2c_rw_access);

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};

		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0){
#ifdef CONFIG_HUAWEI_DSM
			ft3427_dsm_report_err(DSM_TP_I2C_RW_ERROR_NO,FT3427_DSM_TYPE_I2C_RW_ERR);
#endif
			TPD_DMESG("%s: i2c read error.\n",__func__);
		}
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};

		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0){
#ifdef CONFIG_HUAWEI_DSM
			ft3427_dsm_report_err(DSM_TP_I2C_RW_ERROR_NO,FT3427_DSM_TYPE_I2C_RW_ERR);
#endif
			TPD_DMESG("%s:i2c read error.\n", __func__);
		}
	}

	mutex_unlock(&i2c_rw_access);
	return ret;
}

/*write data by i2c*/
int focal_i2c_write(struct i2c_client *client, u8 *writebuf, u16 writelen)
{
	int ret = 0;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		},
	};

	if( NULL == client || NULL == writebuf) {
		TPD_DMESG("%s i2c_client/writebuf is NULL", __func__);
		return -EINVAL;
	}

	mutex_lock(&i2c_rw_access);

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
	{
#ifdef CONFIG_HUAWEI_DSM
		ft3427_dsm_report_err(DSM_TP_I2C_RW_ERROR_NO,FT3427_DSM_TYPE_I2C_RW_ERR);
#endif
		TPD_DMESG("%s i2c write error.\n", __func__);
	}

	mutex_unlock(&i2c_rw_access);

	return ret;
}

int focal_write(u8 *writebuf, u16 writelen)
{
	return focal_i2c_write(ft3427_data.client, writebuf, writelen);
}

int focal_write_default(u8 value)
{
	return focal_write(&value, FT_WRITE_CHAR_NUM_1);
}

int focal_write_reg(u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	int ret = 0;
	buf[0] = regaddr;
	buf[1] = regvalue;

	ret = focal_write(buf, sizeof(buf));
	if (ret < 0) {
#ifdef CONFIG_HUAWEI_DSM
		ft3427_dsm_report_err(DSM_TP_I2C_RW_ERROR_NO,FT3427_DSM_TYPE_I2C_RW_ERR);
#endif
	}
	return ret;
}
int focal_read(u8 *writebuf, u16 writelen, u8 *readbuf, u16 readlen)
{
	return focal_i2c_read(ft3427_data.client, writebuf, writelen, readbuf, readlen);
}

int focal_read_default(u8 *readbuf, u16 readlen)
{
	return focal_read(NULL, 0, readbuf, readlen);
}

int focal_read_reg(u8 regaddr, u8 *regvalue)
{
	return focal_read(&regaddr, 1, regvalue, 1);
}

static int touch_event_handler(void *unused)
{
	int i = 0;
	int ret = 0;
	struct touch_info cinfo;
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };

	memset(&cinfo,0,sizeof(cinfo));
	sched_setscheduler(current, SCHED_RR, &param);

	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, ft3427_data.tpd_flag != TPD_FLAG_SLEEP);

		ft3427_data.tpd_flag = TPD_FLAG_SLEEP;
		if (ft3427_data.loading_fw) {
			break;
		}
		set_current_state(TASK_RUNNING);
#ifdef FTS_ESD_CHECK
		ft3427_data.esdcheck_data.intr = FTS_INT_WORK_STATUS;
#endif
		ret = focal_touchinfo(&cinfo);
		if (ret < 0) {
			TPD_DMESG("read touch info failed, ret=%d", ret);
		} else {
			if (cinfo.count > 0) {
				for (i = 0; i < cinfo.count; i++)
					focal_down(cinfo.x[i], cinfo.y[i], cinfo.p[i], cinfo.id[i]);
			} else {
				focal_up(cinfo.x[0], cinfo.y[0]);
			}

			input_sync(tpd->dev);
		}
#ifdef FTS_ESD_CHECK
		ft3427_data.esdcheck_data.intr = FTS_INT_IDLE_STATUS;
#endif
	} while (!kthread_should_stop());

	TPD_DEBUG("touch_event_handler exit\n");

	return 0;
}

static int focal_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	if( NULL == info) {
		TPD_DMESG("%s info is NULL", __func__);
		return -EINVAL;
	}

	strncpy(info->type, TPD_DEVICE, TPD_DEVICE_NAME_LEN);
	return 0;
}

static irqreturn_t focal_eint_interrupt_handler(int irq, void *dev_id)
{
	TPD_DEBUG("TPD interrupt has been triggered\n");
	ft3427_data.tpd_flag = TPD_FLAG_WAKE;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

static int focal_irq_ft3427_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;

	node = of_find_compatible_node(NULL, NULL, FTS_DTS_COMCOPATIBLE);
	if (node) {
		ft3427_data.irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(ft3427_data.irq, focal_eint_interrupt_handler,
					IRQF_TRIGGER_FALLING, TPD_DEVICE, NULL);
		if (ret)
		{
			TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			return -EIO;
		}
	} else {
		TPD_DMESG("%s tpd request_irq can not find touch eint device node!.", __func__);
		return -ENODEV;
	}
	return 0;
}


static int focal_init_update_proc(struct i2c_client *client)
{
	struct task_struct *thread = NULL;

	TPD_DEBUG("Ready to run auto update thread");

	thread = kthread_run(focal_ctpm_auto_upgrade, (void *)NULL, "ft3427_update");
	if (IS_ERR(thread)) {
		TPD_DMESG("Failed to create update thread.\n");
		return -ENOMEM;
	}

	return 0;
}

static int focal_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int reset_count = 0;
	u8 data = 0;

	if (!client)
	{
		TPD_DMESG("%s:client is NULL\n", __func__);
		return -EINVAL;
	}

	ft3427_data.client = client;
	ft3427_data.input_dev = tpd->dev;
	if(ft3427_data.client->addr != FTS_I2C_ADDR)
	{
		ft3427_data.client->addr = FTS_I2C_ADDR;
		TPD_DMESG(":i2c_client_FT->addr=%x\n", ft3427_data.client->addr);
	}

	/* configure the gpio pins */
	ret = of_get_ft3427_platform_data(&client->dev);
	if (ret != 0)
	{
		TPD_DMESG("Failed to get ft3427 platform data: %d\n", ret);
		return -ENODEV;
	}

	TPD_DEBUG("mtk_tpd: focal_probe ft3427\n");

	/* Reset CTP */
	tpd_gpio_output(tpd_rst_gpio_number, GPIO_OUTPUT_LOW_LEVEL);
	msleep(FTS_POWERON_TRTP_DELAY);
	ret = regulator_enable(tpd->reg);
	if (ret != 0) {
		TPD_DMESG("Failed to enable touch voltage: %d\n", ret);
		return -EIO;
	}
	msleep(FTS_RESET_LOW_DELAY);
	tpd_gpio_output(tpd_rst_gpio_number, GPIO_OUTPUT_HIGH_LEVEL);
	msleep(FTS_RESET_DELAY);
	/* set INT mode */
	tpd_gpio_as_int(tpd_int_gpio_number);

reset_proc:
	ret = focal_read_reg(TPD_I2C_TEST_ADDR, &data);
	if(ret < 0 || data != 0) //reg0 data running state is 0; other state is not 0
	{
		TPD_DMESG("fts_i2c:err %d,data:%d\n", ret, data);
		/* Reset CTP */
		focal_sw_reset();

#ifdef TPD_RESET_ISSUE_WORKAROUND
		reset_count++;
		if ( reset_count < TPD_MAX_RESET_COUNT )
		{
			goto reset_proc;
		}
#endif

#ifdef TPD_POWER_SOURCE_CUSTOM
		ret = regulator_disable(tpd->reg);
		if(ret)
		{
			TPD_DMESG("focaltech focal_probe regulator_disable() failed:%d\n", ret);
		}
		regulator_put(tpd->reg);
#endif

		return -EIO;
	}

	ret = focal_irq_ft3427_registration();
	if(ret < 0)
	{
		TPD_DMESG("focaltech focal_probe regulator_disable() failed:%d\n", ret);
		goto fts_irq_registration_err;
	}

#ifdef FTS_ESD_CHECK
	ret = focal_esdcheck_init();
	if(ret < 0) {
		TPD_DMESG("focal esdcheck init failed:%d\n", ret);
		goto fts_create_sysfs_err;
	}
#endif

#ifdef SYSFS_DEBUG
	ret = focal_create_sysfs(ft3427_data.client);
	if(ret < 0)
	{
		TPD_DMESG("focaltech focal_create_sysfs failed:%d\n", ret);
		goto fts_create_sysfs_err;
	}
#endif

#ifdef FTS_APK_DEBUG
	ret = focal_create_apk_debug_channel();
	if(ret < 0)
	{
		TPD_DMESG("focaltech create apk debug err:%d\n", ret);
		goto fts_create_sysfs_err;
	}
#endif

	wake_lock_init(&(ft3427_data.tp_ft3427_suspend_lock), WAKE_LOCK_SUSPEND, "tp update wakelock");
	wake_lock_init(&(ft3427_data.tp_ft3427_selftest_lock), WAKE_LOCK_SUSPEND, "tp selftest wakelock");

#ifdef TPD_AUTO_UPGRADE
	ret = get_boot_mode();
	if ((RECOVERY_BOOT == ret) ||(ERECOVERY_BOOT == ret) ||(KERNEL_POWER_OFF_CHARGING_BOOT == ret) ||(LOW_POWER_OFF_CHARGING_BOOT == ret)){
		TPD_DMESG("get_boot_mode=%d, forbid to update TP firmware.",get_boot_mode());
	}else{
		TPD_DMESG("***************Enter CTP Auto Upgrad***********\n");
		ret = focal_init_update_proc(ft3427_data.client);
		if (ret < 0){
			TPD_DMESG("Create update thread error:%d", ret);
		}
	}
#endif

	ft3427_data.thread_tpd = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(ft3427_data.thread_tpd)) {
		ret = PTR_ERR(ft3427_data.thread_tpd);
		TPD_DMESG("failed to create kernel thread_tpd: %d\n", ret);
		goto fts_init_update_err;
	}

#ifdef CONFIG_HUAWEI_DSM
	if (!ft3427_touch_dclient)
	{
		ft3427_touch_dclient = dsm_register_client(&dsm_touch_ft3427);
		if(ft3427_touch_dclient == NULL)
		{
			TPD_DMESG("dsm register client failed.\n");
		}
	}
#endif

	ret = focal_test_init(client);
	if (ret < 0) {
		TPD_DMESG("factory test fail:%d", ret);
		goto fts_huawei_dsm_err;
	}
	TPD_DEBUG("Touch Panel Device Probe %s\n", (ret < 0) ? "FAIL" : "PASS");

	tpd_load_status = true;

	return 0;

fts_huawei_dsm_err:
	if(ft3427_touch_dclient)
	{
		dsm_unregister_client(ft3427_touch_dclient,&dsm_touch_ft3427);
	}

fts_init_update_err:
	wake_lock_destroy(&(ft3427_data.tp_ft3427_suspend_lock));
	wake_lock_destroy(&(ft3427_data.tp_ft3427_selftest_lock));

fts_create_sysfs_err:
	free_irq(ft3427_data.irq, tpd->dev);

fts_irq_registration_err:
	regulator_disable(tpd->reg);
	return ret;
}

static int focal_remove(struct i2c_client *client)
{
	TPD_DEBUG("TPD removed\n");

	wake_lock_destroy(&(ft3427_data.tp_ft3427_suspend_lock));
	wake_lock_destroy(&(ft3427_data.tp_ft3427_selftest_lock));

#ifdef CONFIG_HUAWEI_DSM
	if(ft3427_touch_dclient) {
		dsm_unregister_client(ft3427_touch_dclient,&dsm_touch_ft3427);
		ft3427_touch_dclient = NULL;
	}
#endif

#ifdef SYSFS_DEBUG
	focal_remove_sysfs(ft3427_data.client);
#endif

#ifdef FTS_APK_DEBUG
	focal_remove_apk_debug_channel();
#endif

	return 0;
}

static int focal_local_init(void)
{
	int ret = 0;

#ifdef TPD_POWER_SOURCE_CUSTOM
	ft3427_data.suspended = FTS_STATUS_NOTHALT;
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	ret = regulator_set_voltage(tpd->reg, FTS_POWER_LEVEL, FTS_POWER_LEVEL);
	if (ret != 0) {
		TPD_DMESG("failed to set touch voltage: %d\n", ret);
		return ret;
	}
#endif

	TPD_DEBUG("Focaltech I2C Touchscreen Driver...\n");
	ret = i2c_add_driver(&focal_i2c_driver);
	if (ret != 0) {
		TPD_DMESG("unable to add i2c driver.\n");
		return ret;
	}
	TPD_DEBUG("end %s, %d\n", __func__, __LINE__);
	tpd_type_cap = FTS_TOUCH_TYPE_CAP;

	return 0;
}

static void focal_resume(struct device *h)
{
	if((ft3427_data.loading_fw == FTS_LOADING_FW_TRUE) || (ft3427_data.rawdata_mode == FTS_SELF_TEST_TRUE)){
		return;
	}

	ft3427_data.suspended = FTS_STATUS_NOTHALT;
	TPD_DEBUG("TPD wake up\n");
	focal_power_on();
#ifdef FTS_ESD_CHECK
	focal_esdcheck_switch(FTS_ESD_ENABLE);
	ft3427_data.esdcheck_data.suspend = ESD_SUSPEND_SWITCH_OFF;
#endif

	TPD_DEBUG("resume irq\n");
	enable_irq(ft3427_data.irq);
}

static void focal_suspend(struct device *h)
{
#ifdef TPD_POWER_SOURCE_CUSTOM
	int ret = 0;
#endif

	if((ft3427_data.loading_fw == FTS_LOADING_FW_TRUE) || (ft3427_data.rawdata_mode == FTS_SELF_TEST_TRUE)){
		return;
	}
	TPD_DEBUG("TPD enter sleep\n");

	ft3427_data.suspended = FTS_STATUS_HALT;

	disable_irq(ft3427_data.irq);

#ifdef FTS_ESD_CHECK
	focal_esdcheck_switch(FTS_ESD_DISABLE);
	ft3427_data.esdcheck_data.suspend = ESD_SUSPEND_SWITCH_ON;
#endif

#ifdef TPD_POWER_SOURCE_CUSTOM
	ret = regulator_disable(tpd->reg);
	if (ret != 0)
		TPD_DMESG("Failed to disable touch voltage: %d\n", ret);
	tpd_gpio_output(tpd_rst_gpio_number, GPIO_OUTPUT_LOW_LEVEL);
#endif
}

static struct tpd_driver_t focal_device_driver = {
	.tpd_device_name = "FT3427",
	.tpd_local_init = focal_local_init,
	.suspend = focal_suspend,
	.resume = focal_resume,
};

/* called when loaded into kernel */
static int __init focal_driver_init(void)
{
	TPD_DEBUG("MediaTek Focal touch panel driver init\n");
	tpd_get_dts_info();
	if (tpd_driver_add(&focal_device_driver) < 0)
		TPD_DMESG("add Focal driver failed\n");

	return 0;
}

/* should never be called */
static void __exit focal_driver_exit(void)
{
	TPD_DEBUG("MediaTek Focal touch panel driver exit\n");
	tpd_driver_remove(&focal_device_driver);
}

module_init(focal_driver_init);
module_exit(focal_driver_exit);

