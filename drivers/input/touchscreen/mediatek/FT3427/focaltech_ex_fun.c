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

 /*******************************************************************************
*
* File Name: Focaltech_ex_fun.c
*
* Author: Xu YongFeng
*
* Created: 2015-01-29
*
* Modify by mshl on 2015-03-20
*
* Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_core.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
/*create apk debug channel*/
#define PROC_UPGRADE              0
#define PROC_READ_REGISTER        1
#define PROC_WRITE_REGISTER       2
#define PROC_AUTOCLB              4
#define PROC_UPGRADE_INFO         5
#define PROC_WRITE_DATA           6
#define PROC_READ_DATA            7
#define PROC_SET_TEST_FLAG        8
#define FTS_DEBUG_DIR_NAME        "fts_debug"
#define PROC_NAME                 "ftxxxx-debug"
#define FTS_FIRMWARE_BIN_NAME  "ft3427_fw.bin"
#define WRITE_BUF_SIZE            1016
#define READ_BUF_SIZE             1016
#define PROC_UPGRADE_NUMBUF       13

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/


/*******************************************************************************
* Static variables
*******************************************************************************/
static unsigned char proc_operate_mode = PROC_UPGRADE;

/*******************************************************************************
* Static function prototypes
*******************************************************************************/

/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static ssize_t fts_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned char writebuf[WRITE_BUF_SIZE] = {0};
	int buflen = count;
	int writelen = 0;
	int ret = 0;

	if(buff == NULL)
	{
		TPD_DMESG("%s:buff is NULL", __func__);
		return -EINVAL;
	}

	if(WRITE_BUF_SIZE<buflen){
		buflen = WRITE_BUF_SIZE;
	}
	if (copy_from_user(&writebuf, buff, buflen)) {
		TPD_DMESG("%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	case PROC_SET_TEST_FLAG:
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = focal_write(writebuf + 1, writelen);
		if (ret < 0) {
			TPD_DMESG("%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = focal_write(writebuf + 1, writelen);
		if (ret < 0) {
			TPD_DMESG("%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = count - 1;
		if(writelen>0)
		{
			ret = focal_write(writebuf + 1, writelen);
			if (ret < 0) {
				TPD_DMESG("%s:write iic error\n", __func__);
				return ret;
			}
		}
		break;
	default:
		break;
	}

	return count;
}

/* interface of read proc */
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use
* Output: page point to data
* Return: read char number
***********************************************************************/
static ssize_t fts_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	int ret = 0;
	int num_read_chars = FT_READ_CHAR_NUM_1;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	unsigned char buf[READ_BUF_SIZE]={0};

	if(buff == NULL)
	{
		TPD_DMESG("%s:buff is NULL", __func__);
		return -EINVAL;
	}

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		regaddr = FTS_REG_FW_VER;
		ret = focal_read_reg(regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = focal_read(NULL, 0, buf, readlen);
		if (ret < 0) {
			TPD_DMESG("%s:read iic error\n", __func__);
			return ret;
		}
		num_read_chars = FT_READ_CHAR_NUM_1;
	break;
	case PROC_READ_DATA:
		if(READ_BUF_SIZE > count) {
			readlen = count;
		} else {
			readlen = READ_BUF_SIZE;
		}
		ret = focal_read(NULL, 0, buf, readlen);
		if (ret < 0) {

		TPD_DMESG("%s:read iic error\n", __func__);
		return ret;
		}

		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}

	if (copy_to_user(buff, buf, num_read_chars)) {
		TPD_DMESG("%s:copy to user error\n", __func__);
		return -EFAULT;
	}

	return num_read_chars;
}
static const struct file_operations fts_proc_fops = {
		.owner  = THIS_MODULE,
		.read   = fts_debug_read,
		.write  = fts_debug_write,
};

/************************************************************************
* Name: focal_create_apk_debug_channel
* Brief:  create apk debug channel
* Input:
* Output:
* Return: if success, return 0, else reture error code
***********************************************************************/
int focal_create_apk_debug_channel(void)
{
	ft3427_data.proc_root = proc_mkdir(NAME_TOUCHSCREEN_PROC_DIR, NULL);
	if (NULL == ft3427_data.proc_root)
	{
		TPD_DMESG("unable to create /proc/%s entry\n", NAME_TOUCHSCREEN_PROC_DIR);
		return -ENOMEM;
	}

	ft3427_data.fts_proc_entry = proc_create(PROC_NAME, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, NULL, &fts_proc_fops);
	if (NULL == ft3427_data.fts_proc_entry)
	{
		proc_remove(ft3427_data.proc_root);
		TPD_DMESG("Couldn't create proc entry!\n");
		return -ENOMEM;
	}

	return 0;
}
/************************************************************************
* Name: focal_remove_apk_debug_channel
* Brief:  release apk debug channel
* Input: no
* Output: no
* Return: no
***********************************************************************/
void focal_remove_apk_debug_channel(void)
{
	if (NULL != ft3427_data.fts_proc_entry) {
		proc_remove(ft3427_data.fts_proc_entry);
		ft3427_data.fts_proc_entry = NULL;
	}

	if (NULL != ft3427_data.fts_proc_upgrade) {
		proc_remove(ft3427_data.fts_proc_upgrade);
		ft3427_data.fts_proc_upgrade = NULL;
	}
}

static ssize_t fts_ftsforceupgrade_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return -EPERM;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_ftsforceupgrade_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret=0;
	char data = 0;
	if(ft3427_data.suspended == FTS_STATUS_HALT)
	{
		TPD_DMESG("TP is suspend,cannot upgrade firmware!");
		return -EFAULT;
	}
	if(count < PROC_UPGRADE_NUMBUF)
	{
		TPD_DMESG("err count = %d,write err command,please input :force_upgrade!",count);
	}
	if(strncmp( buf, "force_upgrade", PROC_UPGRADE_NUMBUF )){
		TPD_DMESG("force upgrade failed, the input str must be force_upgrade ");
		return -EFAULT;
	}
test_i2c_ok:
	//test the i2c 3 times
	ret = focal_read_reg(TPD_I2C_TEST_ADDR, &data);
	if( ret<0 )
	{
		if((ret++) < TPD_I2C_TEST_COUNT)
		{
			goto test_i2c_ok;
		}
		else
		{
			TPD_DMESG("%s: i2c test err\n", __func__);
			return -EFAULT;
		}
	}

	wake_lock(&(ft3427_data.tp_ft3427_suspend_lock));
	mutex_lock(&ft3427_data.input_dev->mutex);
	disable_irq(ft3427_data.client->irq);
	focal_firmware_manual_update( FTS_FIRMWARE_BIN_NAME );
	enable_irq(ft3427_data.client->irq);
	mutex_unlock(&ft3427_data.input_dev->mutex);
	wake_unlock(&(ft3427_data.tp_ft3427_suspend_lock));

	return count;
}
/*  upgrade from app.bin
*    example:echo "*_app.bin" > ftsfwupgradeapp
*/
static DEVICE_ATTR(ft3427_upgrade, S_IRUGO|S_IWUSR, fts_ftsforceupgrade_show, fts_ftsforceupgrade_store);

/* add your attr in here*/
static struct attribute *fts_attributes[] = {
	&dev_attr_ft3427_upgrade.attr,
	NULL
};

static struct attribute_group fts_attribute_group = {
	.attrs = fts_attributes
};

/************************************************************************
* Name: focal_create_sysfs
* Brief: create sysfs for debug
* Input: i2c info
* Output:
* Return: if success, return 0, else return error code
***********************************************************************/
int focal_create_sysfs(struct i2c_client *client)
{
	int ret = 0;

	if(client == NULL)
	{
		TPD_DMESG( "%s:client is NULL", __func__);
		return -EINVAL;
	}

	ret = sysfs_create_group(&client->dev.kobj, &fts_attribute_group);
	if (0 != ret)
	{
		TPD_DMESG( "%s sysfs_create_group() failed, ret=%d\n", __func__, ret);
		sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
		return -EIO;
	}

	ret = sysfs_create_link(NULL, &client->dev.kobj, "touchscreen");
	if (ret)
	{
		sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
		TPD_DMESG("%s fail create touchscreen link, ret=%d\n", __func__, ret);
	}
	return ret;
}

/************************************************************************
* Name: focal_remove_sysfs
* Brief: remove sys
* Input: client: i2c info
* Output:
* Return:
***********************************************************************/
void focal_remove_sysfs(struct i2c_client *client)
{
	if(client == NULL)
	{
		TPD_DMESG( "%s:client is NULL", __func__);
		return ;
	}
	sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
}
