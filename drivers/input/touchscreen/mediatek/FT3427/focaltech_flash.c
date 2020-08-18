/*
 *
 * FocalTech fts TouchScreen driver.
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
* File Name: Focaltech_IC_Program.c
*
* Author: Xu YongFeng
*
* Created: 2015-01-29
*
* Modify by mshl on 2015-10-26
*
* Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include <linux/wakelock.h>
#include "focaltech_core.h"
/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FTXXXX_INI_FILEPATH_CONFIG         "/sdcard/"
#define FTS_FW_NAME     "focal/ft3427_fw.bin"
#define FTS_FW_IMG_ADDR_VER_OFFSET_3427     2
#define FTS_RETRY_TIMES                     10
#define FTS_PACKAGE_SIZE    128
#define LEN_FLASH_ECC_MAX               0xFFFE

#define RIGHT_OFFSET_BIT(m, n)  ((m) >> (n))
#define RIGHT_OFFSET_16BIT(m)  (RIGHT_OFFSET_BIT(m, 16))     /*offset 16bits to right*/
#define RIGHT_OFFSET_8BIT(m)  (RIGHT_OFFSET_BIT(m, 8))      /*offset 8bits to right*/
#define CHECK_SUM_I2C_WRITE_BUFF_LEN 6

#define FTS_CHANGE_I2C_HID2STD_ADDR1   0xEB
#define FTS_CHANGE_I2C_HID2STD_ADDR2    0xAA
#define FTS_CHANGE_I2C_HID2STD_ADDR3    0x09
#define FTS_CHANGE_I2C_HID2STD_STATUS   0x08
#define FTS_REG_SOFT_RESET_FC    0xFC
#define FTS_REG_BOOT_CHIP_ID    0x90
#define FTS_UPGRADE_AA      0xAA
#define FTS_UPGRADE_55      0x55
#define FTS_CMD_WRITE_PRAM    0xAE
#define FTS_CMD_WRITE_FLASH    0xBF

#define FTS_CMD_START_APP    0x08
#define FTS_CMD_REBOOT_APP    0x07
#define FTS_CMD_SET_MODE     0x09
#define FTS_CMD_ERASURE_APP   0x61
#define FTS_CMD_GET_STATUS    0x6A
#define FTS_FW_SEND_LENGTH    0xB0
#define FTS_CMD_CALC_CRC      0x64
#define FTS_CMD_SET_CALC_ADDR    0x65
#define FTS_CMD_READ_CRC      0x66
#define FTS_WRITE_FW_CMD_LEN 6

#define FTS_ERASURE_OK_STATUS    0xF0AA
#define FTS_ECC_OK_STATUS    0xF055
#define FTS_FW_WRITE_STATUS_ADDR_START  0x1000

#define FTS_REBOOT_DELAY    200
#define FTS_ERASE_MIN_DELAY    1350
#define FTS_ERASE_QUERY_TIMES    30
#define FTS_ERASE_QUERY_DELAY    50
#define FTS_WRITE_FLASH_QUERY_TIMES   30
#define FTS_READ_ECC_QUERY_TIMES   100
/*DTS2017031706061 shangqiang/ywx422251 20170317 begin */
#define FTS_VENDOR_VERSION_38    0x38
#define FTS_VENDOR_VERSION_F2    0xf2
/*DTS2017031706061 shangqiang/ywx422251 20170317 end */
#ifdef CONFIG_GET_HARDWARE_INFO
int focal_set_app_info(void)
{
	int ret = 0;
	int retry = 0;
	u8 version_info = 0;
	char module_name[FT_MODULE_NAME_LEN] = {0};
	char touch_info[TOUCH_INFO_LEN] = {0};

read_fw_err:
	ret = focal_read_reg(FTS_REG_FW_VER, &version_info);
	if (ret < 0)
	{
		if(retry < TPD_I2C_TEST_COUNT)
		{
			retry++;
			goto read_fw_err;
		}
		TPD_DEBUG("%s: focal_read_reg version failed:%d\n", __func__, ret);
		return ret;
	}

	strncpy(module_name, "boe",sizeof("boe"));
	snprintf(touch_info, TOUCH_INFO_LEN,"%s%s_0x%x",TP_HARDWARE_INFO,module_name,version_info);

	ret = register_hardware_info(CTP,touch_info);
	if( ret < 0) {
		TPD_DEBUG("%s: Get CTP hardware info err:%d\n", __func__, ret);
	}

	return ret;
}
#endif

/*
 * description : get fw data
 *
 * param - fw_name : firmware name
 *
 * param - fw_buf: FW data
 *
 * param - fw_size: FW size
 *
 * return : return buf if read fw size success, otherwize return NULL
 */
static u8 *focal_read_fw(const char *fw_name, u32 *fw_size)
{
	struct file *pfile = NULL;
	struct inode *inode = NULL;
	int ret = 0;
	off_t fsize = 0;
	char filepath[UPGREDE_FILE_PATH_LENGTH]={0};
	loff_t pos = 0;
	mm_segment_t old_fs;
	u8 *fw_buf = NULL;

	if(fw_name == NULL)
	{
		TPD_DMESG("%s:fw name is NULL\n", __func__);
		return NULL;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FTXXXX_INI_FILEPATH_CONFIG, fw_name);
	if (NULL == pfile)
	{
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile))
	{
		TPD_DMESG("error occured while opening file %s.\n", filepath);
		goto open_fw_error;
	}
	inode = pfile->f_dentry->d_inode;
	fsize = inode->i_size;
	*fw_size = (u32)fsize;
	fw_buf = (u8 *)kmalloc(*fw_size, GFP_ATOMIC);
	if(fw_buf == NULL) {
		TPD_DMESG( "%s:alloc FW upgrade buf err\n",__func__);
		goto read_fw_error;
	}

	ret = vfs_read(pfile, fw_buf, fsize, &pos);
	if(ret < 0)
	{
		TPD_DMESG( "%s:alloc FW upgrade buf err\n",__func__);
		goto vfs_read_err;
	}
	filp_close(pfile, NULL);
	set_fs(old_fs);
	return fw_buf;

vfs_read_err:
	if(fw_buf)
	{
		kfree(fw_buf);
		fw_buf = NULL;
	}

read_fw_error:
	filp_close(pfile, NULL);

open_fw_error:
	set_fs(old_fs);
	return fw_buf;
}

/*
 * description : get ic status
 *
 * param - status : buffer to receive ic status
 *
 * return : return 0 if read ic status success, otherwize return error code
 */
static int focal_get_status(u32 *status)
{
	int ret = 0;
	u8 cmd = 0x00;
	u8 reg_val[2] = {0};

	if (NULL == status) {
		TPD_DMESG("%s:input parameter is null\n", __func__);
		return -EINVAL;
	}

	memset(reg_val, 0, sizeof(reg_val));
	cmd = FTS_CMD_GET_STATUS;

	ret = focal_read(&cmd, 1, reg_val, 2);
	if (ret < 0) {
		TPD_DMESG("%s:read status fail, ret=%d\n", __func__, ret);
		return ret;
	}
	*status = (reg_val[0] << 8) | (reg_val[1]);

	return 0;
}

static int focal_get_data_check_sum(
	const u8 *data,
	size_t data_size,
	u8 *check_sum)
{
	int i = 0;

	if (NULL == check_sum || NULL == data) {
		TPD_DMESG("%s:input parameter is null\n", __func__);
		return -EINVAL;
	}

	/* get check sum */
	for (i = 0; i < data_size; i++)
		*check_sum ^= data[i];

	return 0;
}

/*
 * description : switch hid mode to std i2c mode
 *
 * return : if success, return 0, otherwize return error code
 */
static int focal_change_i2c_hid2std(void)
{
	u8 cmd[5] = {0};
	int ret = 0;

	cmd[0] = FTS_CHANGE_I2C_HID2STD_ADDR1;
	cmd[1] = FTS_CHANGE_I2C_HID2STD_ADDR2;
	cmd[2] = FTS_CHANGE_I2C_HID2STD_ADDR3;
	ret = focal_write( cmd, FT_WRITE_CHAR_NUM_3);
	if(ret < 0)
	{
		TPD_DMESG("%s:focal write error ret=%d!!\n", __func__,ret);
		return ret;
	}

	msleep(FTS_HID_TO_STD_DELAY);
	cmd[0] = cmd[1] = cmd[2] = 0;
	ret = focal_read(cmd, 0, cmd, 3);
	if(ret < 0)
	{
		TPD_DMESG("%s:focal read error ret=%d!!\n", __func__,ret);
		return ret;
	}

	if ((FTS_CHANGE_I2C_HID2STD_ADDR1 == cmd[0])
		&& (FTS_CHANGE_I2C_HID2STD_ADDR2 == cmd[1])
		&& (FTS_CHANGE_I2C_HID2STD_STATUS== cmd[2]))
	{
		TPD_DMESG("%s:change to stdi2c successful!!\n", __func__);
		return 0;
	}
	else
	{
		TPD_DMESG("%s:change to stdi2c error!!\n", __func__);
		return -EIO;
	}
}

/*
 * return: if read project id success, return 0, else return error code
 *
 * notice: this function work when ic is in normal model
 */
int focal_enter_rom_update_model_by_software(void)
{
	int ret = 0;

	ret = focal_write_reg(FTS_REG_SOFT_RESET_FC, FTS_UPGRADE_AA);
	if (ret < 0) {
		TPD_DMESG("%s:write 0xAA to soft reset ic fail, ret=%d\n",
			__func__, ret);
		return ret;
	}

	ret = focal_write_reg(FTS_REG_SOFT_RESET_FC, FTS_UPGRADE_55);
	if (ret < 0) {
		TPD_DMESG("%s:write 0x55 to soft reset ic fail, ret=%d\n",
			__func__, ret);
		return ret;
	}
	msleep(FTS_REBOOT_DELAY);

	return 0;
}


/*
 * param - chip_id : buffer to receive chip id
 *
 * return: if read project id success, return 0, else return error code
 */
int focal_read_chip_id_(u32 *chip_id)
{
	int ret = 0;
	u8 cmd[4] = {0};
	u8 reg_val[2] = {0, 0};

	if (NULL == chip_id)
		return -EINVAL;

	cmd[0] = FTS_REG_BOOT_CHIP_ID;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	cmd[3] = 0x00;

	ret = focal_read(cmd, 4, reg_val, 2);
	if (ret < 0) {
		TPD_DMESG("%s:read chip id fail, ret=%d\n", __func__, ret);
		return ret;
	}

	*chip_id = (reg_val[0] << 8) | reg_val[1];
	if((reg_val[0] == 0x0)||(reg_val[1] == 0x0))
	{
		return -ENOMEM;
	}

	return 0;
}

/*
 * description : reset normal mode to upgrade(bootloader)
 *
 * return : return 0 if reset success, otherwize return error code
 */
int focal_software_reset_to_bootloader(void)
{
	int i = 0;
	int ret = 0;
	u32 chip_id = 0;
	u8  cmd[2] = {0};

	focal_change_i2c_hid2std();

	for (i = 0; i < FTS_RETRY_TIMES; i++) {
		ret = focal_enter_rom_update_model_by_software();
		if (ret < 0)
		{
			TPD_DMESG("%s: focal_enter_rom_update_model_by_software fail!!\n",
				__func__);
			continue;
		}

		ret = focal_change_i2c_hid2std();
		if (ret < 0)
		{
			TPD_DMESG("%s: focal_change_i2c_hid2std fail!!\n", __func__);
			continue;
		}

		cmd[0] = FTS_UPGRADE_55;
		cmd[1] = FTS_UPGRADE_AA;
		ret = focal_write(cmd, FT_WRITE_CHAR_NUM_2);
		if (ret < 0)
		{
			TPD_DMESG("%s: failed writing  0x55 and 0xaa!!", __func__);
			continue;
		}

		ret = focal_read_chip_id_(&chip_id);
		if (ret || chip_id == 0) {
			TPD_DMESG("%s:chip id read fail, retry=%d\n",
				__func__, i);
			continue;
		} else {
			TPD_DMESG("%s:chip id read success:%x\n",
			__func__, chip_id);
			return 0;
		}
	}

	return -EINVAL;
}

/*
 * return : return 0 if erase success, otherwize return error code
 */
static int focal_erasure_app_area(void)
{
	int i = 0;
	int ret = 0;
	u32 ic_state = 0;
	u8 cmd = 0x00;

	cmd = FTS_CMD_ERASURE_APP;
	ret = focal_write(&cmd, FT_WRITE_CHAR_NUM_1);
	if (ret < 0) {
		TPD_DMESG("%s:send erase command fail, ret=%d\n",
			__func__, ret);
		return ret;
	}

	msleep(FTS_ERASE_MIN_DELAY);

	for (i = 0; i < FTS_ERASE_QUERY_TIMES; i++) {

		ret = focal_get_status(&ic_state);
		if (ret) {
			TPD_DMESG("%s:get ic status fail, ret=%d\n",
				__func__, ret);
		} else {
			if (FTS_ERASURE_OK_STATUS == ic_state)
				return 0;
		}

		msleep(FTS_ERASE_QUERY_DELAY);
	}

	return -EIO;
}


/*
 * description : check if firmware package have write to ic
 *
 * param - command : firmware write command, such as FTS_CMD_WRITE_PRAM
 *
 * param - start_addr : addr in ic to receive the first firmware package
 *
 * param - start_write_addr : addr in ic to receive current firmware pachage
 *
 * param - data_size : current package data size
 *
 * return : return 0 if read ic status success, otherwize return error code
 */
static int focal_wait_firmware_write_finish(
	u8 command,
	u32 start_addr,
	u32 start_write_addr,
	u32 data_size)
{
	int i = 0;
	int ret = 0;
	u32 ic_status = 0;
	u32 driver_status = 0;

	driver_status = start_addr + start_write_addr / data_size;
	for (i = 0; i < FTS_WRITE_FLASH_QUERY_TIMES; i++) {

		ret = focal_get_status(&ic_status);
		if (ret) {
			TPD_DMESG("%s:get ic status fail, ret=%d\n",
				__func__, ret);
			return ret;
		}

		if (ic_status == driver_status)
			return 0;

		msleep(1);
	}


	TPD_DMESG("%s:%s, ic status=0x%X, driver status=0x%X\n",
			__func__, "time out for check ic status",
			ic_status, driver_status);

	return -ETIMEDOUT;
}


/*
 * description : write firmware data to ic
 *
 * param - fw_data : firmware data to write
 *
 * param - fw_length : firmware data length
 *
 * param - command : firmware write command, like FTS_CMD_WRITE_PRAM
 *
 * param - start_addr : addr in ic that first firmware package to write
 *
 * return : return 0 if read ic status success, otherwize return error code
 */
static int focal_write_firmware_data(
	const u8 *fw_data,
	u32 fw_length,
	u8 command,
	u32 start_addr)
{
	int i = 0;
	int ret = 0;

	u32 data_size = 0;
	u32 package_count = 0;
	u32 copy_offset = 0;
	u32 start_write_addr = 0;

	u8 package_buf[FTS_PACKAGE_SIZE + FTS_WRITE_FW_CMD_LEN] = {0};

	TPD_DEBUG("%s:Write firmware to ic\n", __func__);

	package_count = fw_length / FTS_PACKAGE_SIZE;
	if (fw_length % FTS_PACKAGE_SIZE != 0)
		package_count += 1;

	TPD_DEBUG("%s:fw write:data_length=%u, package_count=%u\n",
		__func__, fw_length, package_count);

	start_write_addr = start_addr;
	package_buf[0] = command;
	for (i = 0; i < package_count; i++) {

		/* the last package to write */
		if ((i + 1 == package_count)
			&& (fw_length % FTS_PACKAGE_SIZE != 0)) {
			data_size = fw_length % FTS_PACKAGE_SIZE;
		} else {
			data_size = FTS_PACKAGE_SIZE;
		}

		/* offset */
		package_buf[1] = (u8)(start_write_addr >> 16);
		package_buf[2] = (u8)(start_write_addr >> 8);
		package_buf[3] = (u8)(start_write_addr);

		/* data size */
		package_buf[4] = (u8)(data_size >> 8);
		package_buf[5] = (u8)(data_size);

		copy_offset = start_write_addr - start_addr;
		memcpy(&package_buf[6], &fw_data[copy_offset], data_size);

		ret = focal_write(package_buf, data_size + FTS_WRITE_FW_CMD_LEN);
		if (ret < 0) {
			TPD_DMESG("%s:write fw data fail, index=%d, ret=%d\n",
				__func__, i, ret);
			return -EIO;
		}

		ret = focal_wait_firmware_write_finish(command,
			FTS_FW_WRITE_STATUS_ADDR_START , start_write_addr, data_size);

		if (ret) {
			TPD_DMESG("%s:%s, ret=%d\n", __func__,
				"wait firmware write finish fail", ret);
			return ret;
		}

		start_write_addr += data_size;
		TPD_DEBUG("%s:%s:index=%d, data_size=%u, writed_size=%u\n",
			__func__, "fw write", i, data_size, start_write_addr);
	}

	return 0;
}

/*
 * param - start_addr : addr in ic to start check sum
 *
 * param - crc_length : length of data to check sum
 *
 * param - check_sum : buffer to receive check sum value
 *
 * return : if success, return 0, otherwize return error code
 */
static int focal_read_check_sum(
	u32 start_addr,
	u32 crc_length,
	u8 *check_sum)
{
	int i = 0;
	int ret = 0;
	u32 ic_status = 0;
	u8 cmd[CHECK_SUM_I2C_WRITE_BUFF_LEN] = {0};
	u8 reg_val = 0;

	if (!check_sum) {
		TPD_DMESG("%s: find a null point !!\n", __func__);
		return -EINVAL;
	}

	if (crc_length > LEN_FLASH_ECC_MAX) {
		TPD_DMESG("%s:%s, crc_length=%u, max=%d\n",
			__func__, "crc length out of range",
			crc_length, LEN_FLASH_ECC_MAX);
		return -EINVAL;
	}

	/* start verify */
	cmd[0] = FTS_CMD_CALC_CRC;
	ret = focal_write(cmd, FT_WRITE_CHAR_NUM_1); /*write one value*/
	if (ret < 0) {
		TPD_DMESG("%s:start verify fail, ret=%d\n",
			__func__, ret);
		return ret;
	}

	cmd[0] = FTS_CMD_SET_CALC_ADDR;

	cmd[1] = (u8)RIGHT_OFFSET_16BIT(start_addr);
	cmd[2] = (u8)RIGHT_OFFSET_8BIT(start_addr);
	cmd[3] = (u8)(start_addr);

	cmd[4] = (u8)RIGHT_OFFSET_8BIT(crc_length);
	cmd[5] = (u8)(crc_length);

	ret = focal_write(cmd, CHECK_SUM_I2C_WRITE_BUFF_LEN);
	if (ret < 0) {
		TPD_DMESG("%s:write verify parameter fail, ret=%d\n",
			__func__, ret);
		return ret;
	}

	msleep(crc_length / 256);/*delay times base on crc_length*/

	cmd[0] = FTS_CMD_GET_STATUS;
	for (i = 0; i < FTS_READ_ECC_QUERY_TIMES; i++) {

		ret = focal_get_status(&ic_status);
		if (ret) {
			TPD_DMESG("%s:get ic status fail, ret=%d\n",
				__func__, ret);
		} else {
			if (FTS_ECC_OK_STATUS == ic_status)
				break;
		}

		if (i == FTS_READ_ECC_QUERY_TIMES - 1) {
			TPD_DMESG("%s:%s, out of max retry times\n",
				__func__, "status check fail");
		}

		msleep(1);
	}

	cmd[0] = FTS_CMD_READ_CRC;
	ret = focal_read(cmd, 1, &reg_val, 1);
	if (ret < 0) {
		TPD_DMESG("%s:read crc fail, ret=%d\n", __func__, ret);
		return ret;
	}

	*check_sum = reg_val;

	return 0;
}

/*
 * return: if reset firmware success, return 0, else return error code
 */
int focal_enter_work_model_form_pram_update(void)
{
	int ret = 0;
	u8 cmd = 0x00;

	cmd = FTS_CMD_REBOOT_APP;
	ret = focal_write(&cmd, FT_WRITE_CHAR_NUM_1);
	if (ret < 0) {
		TPD_DMESG("%s:write reboot app command fail, ret = %d\n",
			__func__, ret);
		return ret;
	}

	msleep(FTS_REBOOT_DELAY);
	return 0;
}

/*
 * param - version : buffer to receive version
 *
 * return : if need to update firmware, return 1, else return 0
 */
int focal_get_ic_firmware_version(u8 *version)
{
	int ret = 0;
	u8 fw_ver = 0;

	if (NULL == version) {
		TPD_DMESG("%s:version is null\n", __func__);
		return -EINVAL;
	}

	ret = focal_read_reg(FTS_REG_FW_VER, &fw_ver);
	if (ret < 0) {
		TPD_DMESG("%s:read firmware version in ic fail, ret=%d\n",
			__func__, ret);
		return ret;
	}

	TPD_DMESG("%s:fw version in ic is:0x%x\n", __func__, fw_ver);

	*version = fw_ver;

	return 0;
}


/*
 * param - fw : const struct firmware *fw
 *
 * param - version : buffer to receive firmware version
 *
 * return : if success, return 0, otherwize return error code
 */
int focal_get_img_file_version(
	const struct firmware *fw,
	u8 *version)
{
	if (NULL == fw || NULL == fw->data || NULL == version) {
		TPD_DMESG("%s:fw or fw data is null.\n", __func__);
		return -EINVAL;
	}

	if(fw->size <= FTS_FW_IMG_ADDR_VER_OFFSET_3427) {
		TPD_DMESG("%s:fw length is too short, len=%d.\n",
			__func__, fw->size);
		return -EINVAL;
	}

	*version = fw->data[fw->size - FTS_FW_IMG_ADDR_VER_OFFSET_3427];

	return 0;
}

/*
 * param - fw : const struct firmware *fw
 *
 * return : if need to update firmware, return 1, else return 0
 */
static int focal_check_firmware_version(
	const struct firmware *fw)
{
	int ret = 0;
	u8 fw_ver_in_ic = 0;
	u8 fw_ver_in_file = 0;

	ret = focal_get_ic_firmware_version(&fw_ver_in_ic);
	/* if get ic firmware version fail, update firmware direct */
	if (ret || fw_ver_in_ic == 0x0) {
		TPD_DMESG("%s:%s, ret=%d, fw_version_in_ic=%d\n", __func__,
			"firmware in ic is damaged, update firmware",
			ret, fw_ver_in_ic);
		return 1;
	}

	TPD_DMESG("%s:firmware version in ic is:0x%x\n",
		__func__, fw_ver_in_ic);

	ret = focal_get_img_file_version(fw, &fw_ver_in_file);
	/* do not update firmware, because firmware file may be damaged */
	if (ret) {
		TPD_DMESG("%s:%s, don't update firmware, ret=%d\n",
			__func__, "firmware file is damaged", ret);
		return 0;
	}

	TPD_DMESG("%s:firmware version in file is:0x%x\n",
		__func__, fw_ver_in_file);

	/* if firmware version is different, update */
	/*DTS2017031706061 shangqiang/ywx422251 20170317 begin */
	if ((fw_ver_in_file > fw_ver_in_ic) || (FTS_VENDOR_VERSION_38 ==fw_ver_in_ic) || (FTS_VENDOR_VERSION_F2 == fw_ver_in_ic)) {
	/*DTS2017031706061 shangqiang/ywx422251 20170317 end */
		TPD_DMESG("%s:%s, update firmware\n",
			__func__, "firmware version is different");
		return 1;
	} else {
		TPD_DMESG("%s:%s, not update firmware\n",
			__func__, "firmware version is same");
	}

	return 0;
}

/*
 * description : give this function the firmware data,
 *               and the lengthe of the firmware data,
 *               this function can finish firmeare update
 *
 * param - fw_data : firmware data to update
 *
 * param - fw_len : the length of firmware data
 *
 * return : if success, return 0, otherwize return error code
 */
static int focal_firmware_update(
	const u8 *fw_data,
	u32 fw_len)
{
	int ret = 0;
	u8 check_sum_of_fw = 0;
	u8 check_sum_in_ic = 0;
	u8 cmd[4] = {0};

	if((fw_len < FIREWARE_LEN_MIN) ||(fw_len > FIREWARE_LEN_MAX)){
#ifdef CONFIG_HUAWEI_DSM
		ft3427_dsm_report_err(DSM_TP_FWUPDATE_ERROR_NO,FT3427_DSM_TYPE_FW_LEN_ERR);
#endif
		TPD_DMESG("%s:firmware len err\n", __func__);
		return -EINVAL;
	}

	ret = focal_get_data_check_sum(fw_data, fw_len, &check_sum_of_fw);
	if(ret < 0)
	{
		TPD_DMESG("%s:get checksum err, ret=%d\n", __func__, ret);
		return ret;
	}

	 /*1. Enter upgrade mode*/
	ret = focal_software_reset_to_bootloader();
	if (ret) {
		TPD_DMESG("%s:enter bootloader model fail, ret=%d\n", __func__, ret);
		goto enter_work_model_from_update_model;
	} else {
		TPD_DMESG("%s:enter bootloader model success\n", __func__);
	}

	/*2. erase app and panel paramenter area*/
	ret = focal_erasure_app_area();
	if (ret) {
		TPD_DMESG("%s:erasure app fail, ret=%d\n", __func__, ret);
		goto enter_work_model_from_update_model;
	} else {
		TPD_DMESG("%s:erasure app success\n", __func__);
	}

	/*3. send firmware(FW) data size*/
	cmd[0] = FTS_FW_SEND_LENGTH;
	cmd[1] = (u8) ((fw_len >> 16) & 0xFF);
	cmd[2] = (u8) ((fw_len >> 8) & 0xFF);
	cmd[3] = (u8) (fw_len & 0xFF);
	ret = focal_write(cmd, FT_WRITE_CHAR_NUM_4);
	if(ret < 0){
		TPD_DMESG("%s:write firmware data fail\n", __func__);
	}
	TPD_DMESG("%s: send fw_len:%d\n", __func__, fw_len);

	/*4. write firmware(FW) to ctpm flash*/
	ret =focal_write_firmware_data(fw_data, fw_len, FTS_CMD_WRITE_FLASH, 0);
	if (ret) {
#ifdef CONFIG_HUAWEI_DSM
		ft3427_dsm_report_err(DSM_TP_FWUPDATE_ERROR_NO,FT3427_DSM_TYPE_UPDATE_FW_ERR);
#endif
		TPD_DMESG("%s:write app data fail, ret=%d\n", __func__, ret);
		goto enter_work_model_from_update_model;
	} else {
		TPD_DMESG("%s:write app data success\n", __func__);
	}

	/*5.read out checksum*/
	ret = focal_read_check_sum(0, fw_len, &check_sum_in_ic);
	if (ret) {
#ifdef CONFIG_HUAWEI_DSM
		ft3427_dsm_report_err(DSM_TP_FWUPDATE_ERROR_NO,FT3427_DSM_TYPE_CHECK_SUM_ERR);
#endif
		TPD_DMESG("%s:read check sum in ic fail, ret=%d\n",
			__func__, ret);

		goto enter_work_model_from_update_model;
	}

	TPD_DMESG("%s:crc_in_ic=0x%02X, crc_of_fw=0x%02X\n",
			__func__, check_sum_in_ic, check_sum_of_fw);

	if (check_sum_in_ic != check_sum_of_fw) {
		TPD_DMESG("%s:check sum check fail\n", __func__);
		ret = -EIO;
	}

enter_work_model_from_update_model:
	if (focal_enter_work_model_form_pram_update()) {
		TPD_DMESG("%s:enter work model fail, ret=%d\n", __func__, ret);
	} else {
		TPD_DMESG("%s:enter work model success\n", __func__);
	}

	return ret;
}

/*
 * param - product_name : any string is ok,
 *                        this string will be used to create firmware file name,
 *                        how to create firmeare file name please see function:
 *                        focal_get_firmware_name
 *
 * return: if read project id success, return 0, else return error code
 */
int focal_ctpm_auto_upgrade(void *unused)
{
	int ret = 0;
	u8 fw_ver = 0;
	const struct firmware *fw = NULL;
	msleep(FTS_WAIT_TP_LOAD_SUCCESS);
	if(tpd_load_status != true)
	{
		TPD_DMESG("%s:driver load err, ret=%d\n",   __func__, ret);
		return -EPERM;
	}
	/* 1. Request Firmware */
	ret = request_firmware(&fw, FTS_FW_NAME, &(ft3427_data.client->dev));
	if (ret != 0) {
		TPD_DMESG("%s:firmware request fail, ret=%d\n", __func__, ret);
		return ret;
	}

	disable_irq(ft3427_data.client->irq);
#ifdef FTS_ESD_CHECK
	focal_esdcheck_switch(FTS_ESD_DISABLE);
#endif
	ft3427_data.loading_fw=FTS_LOADING_FW_TRUE;

	/* 2. compare firmware version */
	ret = focal_check_firmware_version(fw);
	if (!ret) {
		TPD_DMESG("%s:no need to update firmware\n", __func__);
		goto release_fw;
	} else {
		TPD_DMESG("%s:going to update firmware\n", __func__);
	}

	/* 3. firmware update */
	ret = focal_firmware_update(fw->data, fw->size);
	if (ret) {
		TPD_DMESG("%s:firmware update fail, ret=%d\n",  __func__, ret);
		goto release_fw;
	}

	ret = focal_get_ic_firmware_version(&fw_ver);
	if (ret) {
		TPD_DMESG("%s:get firmware version fail, ret=%d\n",
			__func__, ret);
		goto release_fw;
	} else {
		TPD_DMESG("%s:get firmware version:0x%x\n", __func__, fw_ver);
	}

release_fw:
	/* 5. release firmware */
	if(fw != NULL)
	{
		release_firmware(fw);
		fw = NULL;
	}
#ifdef CONFIG_GET_HARDWARE_INFO
	ret = focal_set_app_info();
	if(ret < 0)
	{
		TPD_DMESG("%s: focaltech set app info err\n", __func__);
	}
#endif

	ft3427_data.loading_fw=FTS_LOADING_FW_FALSE;
#ifdef FTS_ESD_CHECK
	focal_esdcheck_switch(FTS_ESD_ENABLE);
#endif
	enable_irq(ft3427_data.client->irq);

	return ret;
}

/*
 * param - fw_name : firmware name, use this function to update firmware,
 *                   you should put the firmware file to /system/etc/firmware/
 *
 * return: if read project id success, return 0, else return error code
 */
int focal_firmware_manual_update(const char *fw_name)
{
	int ret = 0;
	u8 fw_ver = 0;
	u8 *fw_buf = NULL;
	u32 fw_size = 0;

	disable_irq(ft3427_data.client->irq);
#ifdef FTS_ESD_CHECK
	focal_esdcheck_switch(FTS_ESD_DISABLE);
#endif
	ft3427_data.loading_fw = FTS_LOADING_FW_TRUE;

	/* 1. get firmware */
	fw_buf = focal_read_fw(fw_name, &fw_size);
	if (fw_buf == NULL)
	{
#ifdef CONFIG_HUAWEI_DSM
		ft3427_dsm_report_err(DSM_TP_FWUPDATE_ERROR_NO,FT3427_DSM_TYPE_PARSE_FW_ERR);
#endif
		TPD_DMESG( "%s ERROR:Get firmware data failed\n",__func__);
		goto release_fw;
	}
	else {
		TPD_DMESG("%s Read firmware size=%d", __func__, fw_size);
	}

	/* 2. firmware update */
	ret = focal_firmware_update(fw_buf, fw_size);
	if (ret) {
		TPD_DMESG("%s:firmware update fail, ret=%d\n",  __func__, ret);
		goto release_fw;
	}

	ret = focal_get_ic_firmware_version(&fw_ver);
	if (ret) {
		TPD_DMESG("%s:get firmware version fail, ret=%d\n",
			__func__, ret);
		goto release_fw;
	} else {
		TPD_DMESG("%s:get firmware version:0x%x\n", __func__, fw_ver);
	}

release_fw:
	if(fw_buf != NULL) {
		kfree(fw_buf);
		fw_buf = NULL;
	}

	ft3427_data.loading_fw = FTS_LOADING_FW_FALSE;
#ifdef FTS_ESD_CHECK
	focal_esdcheck_switch(FTS_ESD_ENABLE);
#endif
	enable_irq(ft3427_data.client->irq);
	return ret;
}
