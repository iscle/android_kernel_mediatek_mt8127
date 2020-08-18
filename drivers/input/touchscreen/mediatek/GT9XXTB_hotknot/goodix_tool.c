/* drivers/input/touchscreen/mediatek/gt9xx_mtk/goodix_tool.c
 *
 * Copyright  (C)  2010 - 2016 Goodix., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU
 * General Public License for more details.
 *
 * Version: V2.6.0.3
 */

#include "tpd.h"
#include "include/tpd_gt9xx_common.h"
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h> /*proc */

#pragma pack(1)
struct st_cmd_head {
	u8 wr;                     /* write read flag,0:R1:W2:PID 3: */
	u8 flag;                   /* 0:no need flag/int 1: need flag2:need int */
	u8 flag_addr[2];     /* flag address */
	u8 flag_val;            /* flag val */
	u8 flag_relation;    /* flag_val:flag 0:not equal 1:equal 2:> 3:< */
	u16 circle;              /* polling cycle */
	u8 times;                /* plling times */
	u8 retry;                 /* I2C retry times */
	u16 delay;              /* delay before read or after write */
	u16 data_len;         /* data length */
	u8 addr_len;          /* address length */
	u8 addr[2];             /* address */
	u8 res[3];             /* reserved */
	u8 *data;           /* data pointer */
};
#pragma pack()

static struct st_cmd_head cmd_head;
#define TOOL_FLAG_MIN 0
#define TOOL_FLAG_MAX 2
#define FLAG_RELATION_MIN 0
#define FLAG_RELATION_MAX 6
#define TOOL_ADDR_LEN_MIN 0
#define TOOL_ADDR_LEN_MAX 2
#define DATA_LENGTH_UINT 512
#define CMD_HEAD_LENGTH (sizeof(struct st_cmd_head) - sizeof(u8 *))
/*DTS2017032810387  shangqiang/ywx422251 20170331 start */
#define PAGE_MAX_SIZE   4096
#define BUFF_SIZE_MAX    32
#define IC_TYPE_LEN      16
/*DTS2017032810387  shangqiang/ywx422251 20170331 end */
static char procname[20] = { 0 };
static struct i2c_client *gt_client;

static struct proc_dir_entry *goodix_proc_entry;

static ssize_t goodix_tool_write(struct file *, const char __user *, size_t, loff_t *);
static ssize_t goodix_tool_read(struct file *, char __user *, size_t, loff_t *);
static s32 (*tool_i2c_read)(u8*, u16);
static s32 (*tool_i2c_write)(u8*, u16);

static const struct file_operations tool_ops = {
	.owner = THIS_MODULE,
	.read = goodix_tool_read,
	.write = goodix_tool_write,
};

static s32 DATA_LENGTH = 0;
/*DTS2017032810387  shangqiang/ywx422251 20170331 start */
static s8 IC_TYPE[IC_TYPE_LEN] = "GT9XX";
/*DTS2017032810387  shangqiang/ywx422251 20170331 end */
#ifdef CONFIG_HOTKNOT_BLOCK_RW
DECLARE_WAIT_QUEUE_HEAD(bp_waiter);
u8 got_hotknot_state = false;
u8 got_hotknot_extra_state = false;
u8 wait_hotknot_state = false;
u8 force_wake_flag = false;
#endif

static void tool_set_proc_name(char *procname)
{
	sprintf(procname, "hotknot");
}

static s32 tool_i2c_read_no_extra(u8 *buf, u16 len)
{
	s32 ret = -1;

	ret = gtp_i2c_read(gt_client, buf, len + GTP_ADDR_LENGTH);
	return ret;
}

static s32 tool_i2c_write_no_extra(u8 *buf, u16 len)
{
	s32 ret = -1;

	ret = gtp_i2c_write(gt_client, buf, len);
	return ret;
}
/*DTS2017032810387  shangqiang/ywx422251 20170331 start */
static s32 tool_i2c_read_with_extra(u8 *buf, u16 len)
{
	s32 ret = -1;
	u8 pre[2] = { 0x0f, 0xff };
	u8 end[2] = { 0x80, 0x00 };

	ret = tool_i2c_write_no_extra(pre, 2);
	if (ret < 0) {
		GTP_ERROR("%s: tool i2c write pre failed.", __func__);
		return FAIL;
	}
	ret = tool_i2c_read_no_extra(buf, len);
	if (ret < 0) {
		GTP_ERROR("%s: tool i2c read buf failed.", __func__);
		return FAIL;
	}
	ret = tool_i2c_write_no_extra(end, 2);
	if (ret < 0) {
		GTP_ERROR("%s: tool i2c write end failed.", __func__);
		return FAIL;
	}

	return ret;
}

static s32 tool_i2c_write_with_extra(u8 *buf, u16 len)
{
	s32 ret = -1;
	u8 pre[2] = { 0x0f, 0xff };
	u8 end[2] = { 0x80, 0x00 };

	ret = tool_i2c_write_no_extra(pre, 2);
	if (ret < 0) {
		GTP_ERROR("%s tool i2c write pre failed.", __func__);
		return FAIL;
	}
	ret = tool_i2c_write_no_extra(buf, len);
	if (ret < 0) {
		GTP_ERROR("%s tool i2c write buf failed.", __func__);
		return FAIL;
	}
	ret = tool_i2c_write_no_extra(end, 2);
	if (ret < 0) {
		GTP_ERROR("%s: tool i2c write end failed.", __func__);
		return FAIL;
	}

	return ret;
}
/*DTS2017032810387  shangqiang/ywx422251 20170331 end */
static void register_i2c_func(void)
{
	if (strncmp(IC_TYPE, "GT8110", 6) && strncmp(IC_TYPE, "GT8105", 6)
	    && strncmp(IC_TYPE, "GT801", 5) && strncmp(IC_TYPE, "GT800", 5)
	    && strncmp(IC_TYPE, "GT801PLUS", 9) && strncmp(IC_TYPE, "GT811", 5)
	    && strncmp(IC_TYPE, "GTxxx", 5) && strncmp(IC_TYPE, "GT9XX", 5)) {
		tool_i2c_read = tool_i2c_read_with_extra;
		tool_i2c_write = tool_i2c_write_with_extra;
		GTP_DEBUG("I2C function: with pre and end cmd!");
	} else {
		tool_i2c_read = tool_i2c_read_no_extra;
		tool_i2c_write = tool_i2c_write_no_extra;
		GTP_INFO("I2C function: without pre and end cmd!");
	}
}

static void unregister_i2c_func(void)
{
	tool_i2c_read = NULL;
	tool_i2c_write = NULL;
	GTP_INFO("I2C function: unregister i2c transfer function!");
}



s32 init_wr_node(struct i2c_client *client)
{
	s32 i=0;

	gt_client = i2c_client_point;

	memset(&cmd_head, 0, sizeof(cmd_head));
	cmd_head.data = NULL;

	i = 5;

	while ((!cmd_head.data) && i) {
		cmd_head.data = kzalloc(i * DATA_LENGTH_UINT, GFP_KERNEL);

		if (NULL != cmd_head.data)
			break;

		i--;
	}

	if (i) {
		DATA_LENGTH = i * DATA_LENGTH_UINT + GTP_ADDR_LENGTH;
		GTP_INFO("Applied memory size:%d.", DATA_LENGTH);
	} else {
		GTP_ERROR("Apply for memory failed.");
		return FAIL;
	}

	cmd_head.addr_len = GTP_ADDR_LENGTH;
	cmd_head.retry = I2C_BYTES_RETRY_TIMES_MAX;

	register_i2c_func();

	tool_set_proc_name(procname);
	/*DTS2017032810387  shangqiang/ywx422251 20170331 start */
	goodix_proc_entry = proc_create(procname, 0644, NULL, &tool_ops);
	/*DTS2017032810387  shangqiang/ywx422251 20170331 end */

	if (goodix_proc_entry == NULL) {
		GTP_ERROR("Couldn't create proc entry!");
		goto create_err;
	}
	GTP_INFO("Create proc entry success!");

	return SUCCESS;
create_err:
	if (cmd_head.data != NULL) {
		kfree(cmd_head.data);
		cmd_head.data = NULL;
	}

	return FAIL;
}

void uninit_wr_node(void)
{
	if (cmd_head.data != NULL) {
		kfree(cmd_head.data);
		cmd_head.data = NULL;
	}
	unregister_i2c_func();
	remove_proc_entry(procname, NULL);
}

static u8 relation(u8 src, u8 dst, u8 rlt)
{
	u8 ret = 0;

	switch (rlt) {
	case 0:
		ret = (src != dst) ? true : false;
		break;

	case 1:
		ret = (src == dst) ? true : false;
		GTP_DEBUG("equal:src:0x%02x dst:0x%02x ret:%d.", src, dst, (s32) ret);
		break;

	case 2:
		ret = (src > dst) ? true : false;
		break;

	case 3:
		ret = (src < dst) ? true : false;
		break;

	case 4:
		ret = (src & dst) ? true : false;
		break;

	case 5:
		ret = (!(src | dst)) ? true : false;
		break;

	default:
		ret = false;
		break;
	}

	return ret;
}

/*******************************************************
Function:
Comfirm function.
Input:
None.
Output:
Return write length.
********************************************************/
static u8 comfirm(void)
{
	s32 i = 0;
/*DTS2017032810387  shangqiang/ywx422251 20170331 start */
	u8 buf[BUFF_SIZE_MAX] = {0};

	/* memcpy(&buf[GTP_ADDR_LENGTH - cmd_head.addr_len], &cmd_head.flag_addr, cmd_head.addr_len); */
	/* memcpy(buf, &cmd_head.flag_addr, cmd_head.addr_len);//Modified by Scott, 2012-02-17 */
	if (BUFF_SIZE_MAX < cmd_head.addr_len) {
		GTP_ERROR("cmd_head.addr_len is too long!");
		return FAIL;
	}
/*DTS2017032810387  shangqiang/ywx422251 20170331 end */
	memcpy(buf, cmd_head.flag_addr, cmd_head.addr_len);

	for (i = 0; i < cmd_head.times; i++) {
		if (tool_i2c_read(buf, 1) <= 0) {
			GTP_ERROR("Read flag data failed!");
			return FAIL;
		}

		if (true == relation(buf[GTP_ADDR_LENGTH], cmd_head.flag_val, cmd_head.flag_relation)) {
			GTP_DEBUG("value at flag addr:0x%02x.", buf[GTP_ADDR_LENGTH]);
			GTP_DEBUG("flag value:0x%02x.", cmd_head.flag_val);
			break;
		}

		msleep(cmd_head.circle);
	}

	if (i >= cmd_head.times) {
		GTP_ERROR("Didn't get the flag to continue!");
		return FAIL;
	}

	return SUCCESS;
}
/*******************************************************
Function:
Goodix tool write function.
Input:
standard proc write function param.
Output:
Return write length.
********************************************************/
static ssize_t goodix_tool_write(struct file *filp, const char __user *buff, size_t len, loff_t *off)
{
	s32 ret = 0;

	/*DTS2017022407064  shangqiang/ywx422251 20170227 begin */
	/*DTS2017030701693  shangqiang/ywx422251 20170322 begin */
	if( (len < CMD_HEAD_LENGTH) || (buff == NULL) || (filp == NULL) || (off==NULL) || (cmd_head.data == NULL) )
	{
		GTP_ERROR("invalid buffer len or pointer is NULL !");
		return FAIL;
	}
	/*DTS2017030701693  shangqiang/ywx422251 20170322 end */
	if (gtp_resetting == GTP_IS_IN_RESETTING_STATE) {
		GTP_DEBUG("[Write]tpd_halt =1 fail!");
		return FAIL;
	}
	ret = copy_from_user(&cmd_head, buff, CMD_HEAD_LENGTH);
	if (ret)
		GTP_ERROR("copy_from_user failed.");

	if( (cmd_head.flag < TOOL_FLAG_MIN) || (cmd_head.flag > TOOL_FLAG_MAX) )
	{
		GTP_DEBUG("err cmd_head flag=%d.", cmd_head.flag);
		return FAIL;
	}
	if( (cmd_head.flag_relation < FLAG_RELATION_MIN ) || ( cmd_head.flag_relation>FLAG_RELATION_MAX ) )
	{
		GTP_DEBUG("err cmd_head flag_relation=%d.", cmd_head.flag_relation);
		return FAIL;
	}
	/*DTS2017030701693  shangqiang/ywx422251 20170322 begin */
	if( (cmd_head.addr_len < TOOL_ADDR_LEN_MIN) || (cmd_head.addr_len > TOOL_ADDR_LEN_MAX) )
	{
		GTP_DEBUG("err cmd_head flag_relation addr_len=%d.", cmd_head.addr_len);
		return FAIL;
	}
	/*DTS2017030701693  shangqiang/ywx422251 20170322 end */
/*DTS2017032810387  shangqiang/ywx422251 20170331 start */
	if( cmd_head.data_len > (DATA_LENGTH - GTP_ADDR_LENGTH))
/*DTS2017032810387  shangqiang/ywx422251 20170331 end */
	{
		GTP_DEBUG("err cmd_head datalen=%d", cmd_head.data_len);
		return FAIL;
	}
	/*DTS2017022407064  shangqiang/ywx422251 20170227 end */


	GTP_DEBUG("wr:0x%02x.", cmd_head.wr);

	if (GOODID_TOOL_WRITE == cmd_head.wr) {
		if( cmd_head.data_len > (DATA_LENGTH - GTP_ADDR_LENGTH - GTP_ADDR_LENGTH))
		{
			GTP_DEBUG("err cmd_head datalen=%d", cmd_head.data_len);
			return FAIL;
		}
		ret = copy_from_user(&cmd_head.data[GTP_ADDR_LENGTH], &buff[CMD_HEAD_LENGTH], cmd_head.data_len);
		if (ret)
			GTP_ERROR("copy_from_user failed.");

		memcpy(&cmd_head.data[GTP_ADDR_LENGTH - cmd_head.addr_len], cmd_head.addr, cmd_head.addr_len);

		GTP_DEBUG_ARRAY(cmd_head.data, cmd_head.data_len + cmd_head.addr_len);
		GTP_DEBUG_ARRAY((u8 *)&buff[CMD_HEAD_LENGTH], cmd_head.data_len);

		if (GOODID_TOOL_FLAG_CONFIRM == cmd_head.flag) {
			if (FAIL == comfirm()) {
				GTP_ERROR("[WRITE]Comfirm fail!");
				return FAIL;
			}
		} else if (GOODID_TOOL_FLAG_NEED_INTERRUPT == cmd_head.flag) {
			/* Need interrupt! */
		}

		if (tool_i2c_write(&cmd_head.data[GTP_ADDR_LENGTH - cmd_head.addr_len],
				   cmd_head.data_len + cmd_head.addr_len) <= 0) {
			GTP_ERROR("[WRITE]Write data failed!");
			return FAIL;
		}

		GTP_DEBUG_ARRAY(&cmd_head.data[GTP_ADDR_LENGTH - cmd_head.addr_len],
				cmd_head.data_len + cmd_head.addr_len);

		if (cmd_head.delay)
			msleep(cmd_head.delay);

		return cmd_head.data_len + CMD_HEAD_LENGTH;
	} else if (GOODID_TOOL_WRITE_IC_TYPE == cmd_head.wr) {  /* Write ic type */
/*DTS2017032810387  shangqiang/ywx422251 20170331 start */
		if (IC_TYPE_LEN < cmd_head.data_len) {
			GTP_ERROR("write ic type cmd_head.data_len is too long.");
			return FAIL;
		}
/*DTS2017032810387  shangqiang/ywx422251 20170331 end */
		memcpy(IC_TYPE, cmd_head.data, cmd_head.data_len);
		register_i2c_func();

		return cmd_head.data_len + CMD_HEAD_LENGTH;
	} else if (GOODID_TOOL_GET_IC_TYPE == cmd_head.wr) {
		/* memcpy(IC_TYPE, cmd_head.data, cmd_head.data_len); */

		return cmd_head.data_len + CMD_HEAD_LENGTH;
	} else if (GOODID_TOOL_DISABLE_IRQ == cmd_head.wr) { /* disable irq! */
		gtp_irq_disable();
		wake_lock(&tp_gt9293_captest_lock);//DTS2017022106140 shangqiang/swx422215 20170221
#ifdef CONFIG_GTP_ESD_PROTECT
		gtp_esd_switch(i2c_client_point, SWITCH_OFF);
#endif
#ifdef CONFIG_GTP_CHARGER_DETECT
		gtp_charger_switch(CHARGER_DETECT_OFF);
#endif
		return CMD_HEAD_LENGTH;
	} else if (GOODID_TOOL_ENABLE_IRQ == cmd_head.wr) { /* enable irq! */
		gtp_irq_enable();
		wake_unlock(&tp_gt9293_captest_lock);//DTS2017022106140 shangqiang/swx422215 20170221
#ifdef CONFIG_GTP_ESD_PROTECT
		gtp_esd_switch(i2c_client_point, SWITCH_ON);
#endif
#ifdef CONFIG_GTP_CHARGER_DETECT
		gtp_charger_switch(CHARGER_DETECT_ON);
#endif
		return CMD_HEAD_LENGTH;
	} else if (GOODID_TOOL_RAWDIFF == cmd_head.wr) {
		if( cmd_head.data_len > (DATA_LENGTH - GTP_ADDR_LENGTH - GTP_ADDR_LENGTH))
		{
			GTP_DEBUG("err cmd_head datalen=%d", cmd_head.data_len);
			return FAIL;
		}
		ret = copy_from_user(&cmd_head.data[GTP_ADDR_LENGTH], &buff[CMD_HEAD_LENGTH], cmd_head.data_len);
		if (ret)
			GTP_DEBUG("copy_from_user failed.");

		if (cmd_head.data[GTP_ADDR_LENGTH]) {
			GTP_DEBUG("gtp enter rawdiff.");
			gtp_rawdiff_mode = true;
		} else {
			gtp_rawdiff_mode = false;
			GTP_DEBUG("gtp leave rawdiff.");
		}

		return CMD_HEAD_LENGTH;
	}
#ifdef UPDATE_FUNCTIONS
	else if (GOODID_TOOL_ENTER_UPDATE_MODE == cmd_head.wr) {  /* Enter update mode! */
		if (FAIL == gup_enter_update_mode(gt_client)){
			return FAIL;
		}
	} else if (GOODID_TOOL_LEAVE_UPDATE_MODE == cmd_head.wr) {  /* Leave update mode! */
		gup_leave_update_mode();
	} else if (GOODID_TOOL_UPDATE_FIRMWARE == cmd_head.wr) {  /* Update firmware! */
		show_len = 0;
		total_len = 0;
/*DTS2017032810387  shangqiang/ywx422251 20170331 start */
		if ((cmd_head.data == NULL)
			|| (cmd_head.data_len > (len - CMD_HEAD_LENGTH))) {
			GTP_ERROR("copy_from_user data out of range.");
			return -EINVAL;
		}
		if( cmd_head.data_len > (DATA_LENGTH - GTP_ADDR_LENGTH - 1))
		{
			GTP_DEBUG("err cmd_head datalen=%d", cmd_head.data_len);
			return FAIL;
		}
/*DTS2017032810387  shangqiang/ywx422251 20170331 end */
		memset(cmd_head.data, 0, cmd_head.data_len + 1); //fw name end string "\0"
		memcpy(cmd_head.data, &buff[CMD_HEAD_LENGTH], cmd_head.data_len);
		GTP_DEBUG("update firmware, filename: %s", cmd_head.data);
		if (FAIL == gup_update_proc((void *)cmd_head.data))
			return FAIL;

	}
#endif
	return CMD_HEAD_LENGTH;
}
/*******************************************************
Function:
Goodix tool read function.
Input:
standard proc read function param.
Output:
Return read length.
********************************************************/
static ssize_t goodix_tool_read(struct file *flie, char __user *page, size_t size, loff_t *ppos)
{
	s32 ret=0;

	GTP_DEBUG_FUNC();

	if (gtp_resetting == GTP_IS_IN_RESETTING_STATE)
		return FAIL;
/*DTS2017030701693  shangqiang/ywx422251 20170322 begin */
	if ((cmd_head.data == NULL) || (flie == NULL) || (page == NULL)) {
		GTP_ERROR("cmd_head.data or pointer is NULL !");
		return FAIL;
	}
/*DTS2017032810387  shangqiang/ywx422251 20170331 start */
	if( (cmd_head.addr_len < TOOL_ADDR_LEN_MIN) || (cmd_head.addr_len > TOOL_ADDR_LEN_MAX) )
	{
		GTP_DEBUG("err cmd_head.addr_len = %d.", cmd_head.addr_len);
		return FAIL;
	}
	if ((PAGE_MAX_SIZE < size) || (PAGE_MAX_SIZE < cmd_head.data_len)) {
		GTP_ERROR("cmd_head.data_len or size is too long !");
		return FAIL;
	}
/*DTS2017032810387  shangqiang/ywx422251 20170331 end */
/*DTS2017030701693  shangqiang/ywx422251 20170322 end */
	if (*ppos) {
		*ppos = 0;
		return 0;
	}

	if (cmd_head.wr % 2) { //filter command
		GTP_ERROR("[READ] invaild operator fail!");
		return FAIL;
	} else if (!cmd_head.wr) {
		u16 len = 0;
		s16 data_len = 0;
		u16 loc = 0;

		if (GOODID_TOOL_FLAG_CONFIRM == cmd_head.flag) {
			if (FAIL == comfirm()) {
				GTP_ERROR("[READ]Comfirm fail!");
				return FAIL;
			}
		} else if (GOODID_TOOL_FLAG_NEED_INTERRUPT == cmd_head.flag) {
			/* Need interrupt! */
		}

		memcpy(cmd_head.data, cmd_head.addr, cmd_head.addr_len);

		GTP_DEBUG("[CMD HEAD DATA] ADDR:0x%02x%02x.", cmd_head.data[0], cmd_head.data[1]);
		GTP_DEBUG("[CMD HEAD ADDR] ADDR:0x%02x%02x.", cmd_head.addr[0], cmd_head.addr[1]);

		if (cmd_head.delay)
			msleep(cmd_head.delay);

		data_len = cmd_head.data_len;

		while (data_len > 0) {
			if (data_len > DATA_LENGTH)  //DATA_LENGTH depend on alloc size
				len = DATA_LENGTH;
			else
				len = data_len;

			data_len -= len;

			if (tool_i2c_read(cmd_head.data, len) <= 0) {
				GTP_ERROR("[READ]Read data failed!");
				return FAIL;
			}

			/* memcpy(&page[loc], &cmd_head.data[GTP_ADDR_LENGTH], len); */
			ret = simple_read_from_buffer(&page[loc], size, ppos, &cmd_head.data[GTP_ADDR_LENGTH], len);
			if (ret < 0)
				return ret;

			loc += len;

			GTP_DEBUG_ARRAY(&cmd_head.data[GTP_ADDR_LENGTH], len);
			GTP_DEBUG_ARRAY(page, len);
		}
		return cmd_head.data_len;
	} else if (GOODID_TOOL_READ == cmd_head.wr) {
		ret = simple_read_from_buffer(page, size, ppos, IC_TYPE, sizeof(IC_TYPE));
		return ret;
	} else if (GOODID_TOOL_PROCESS_BUF == cmd_head.wr) {
		u8 progress_buf[4];

		progress_buf[0] = show_len >> 8;
		progress_buf[1] = show_len & 0xff;
		progress_buf[2] = total_len >> 8;
		progress_buf[3] = total_len & 0xff;  //update speed judge

		ret = simple_read_from_buffer(page, size, ppos, progress_buf, 4);
		return ret;
	} else if (GOODID_TOOL_READ_ERR_CODE == cmd_head.wr) {
		/* Read error code! */
	} else if (GOODID_TOOL_READ_DRIVER_VERSION == cmd_head.wr) { /* Read driver version */
		ret = simple_read_from_buffer(page, size, ppos, GTP_DRIVER_VERSION, strlen(GTP_DRIVER_VERSION));
		return ret;
	}
	return -EPERM;
}
