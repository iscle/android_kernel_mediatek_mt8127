/* drivers/input/touchscreen/mediatek/gt9xx_mtk/include/tpd_gt9xx_common.h
 *
 * Copyright  (C)  2010 - 2016 Goodix., Ltd.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * Version: V2.6.0.3
 */

#ifndef TPD_CUSTOM_GT9XX_H__
#define TPD_CUSTOM_GT9XX_H__

#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>

#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/jiffies.h>
/*DTS2017012205954 shangqiang/ywx422251 20170122 begin */
#include <linux/wakelock.h>
extern struct wake_lock tp_gt9293_suspend_lock;
/*DTS2017012205954 shangqiang/ywx422251 20170122 end */
/*DTS2017022106140 shangqiang/swx422215 20170221 begin*/
extern struct wake_lock tp_gt9293_captest_lock;
/*DTS2017022106140 shangqiang/swx422215 20170221 end*/
/* Pre-defined definition */
 #define UPDATE_FUNCTIONS

#define GTP_CONFIG_MIN_LENGTH       186
#define GTP_CONFIG_MAX_LENGTH       240

#ifdef CONFIG_GTP_DEBUG_ON
#define GTP_DEBUG_ON          1
#else
#define GTP_DEBUG_ON          0
#endif

#ifdef CONFIG_GTP_DEBUG_ARRAY_ON
#define GTP_DEBUG_ARRAY_ON          1
#else
#define GTP_DEBUG_ARRAY_ON          0
#endif

#ifdef CONFIG_GTP_DEBUG_FUNC_ON
#define GTP_DEBUG_FUNC_ON          1
#else
#define GTP_DEBUG_FUNC_ON          0
#endif

#define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))

#if (defined(CONFIG_GTP_ESD_PROTECT) || defined(CONFIG_GTP_COMPATIBLE_MODE))
extern void force_reset_guitar(void);
#endif
/* STEP_3(optional):Custom set some config by themself,if need. */
/*DTS2017011806572 shangqiang/ywx422251 20170118 begin */
#ifdef CONFIG_GTP_CUSTOM_CFG
#define GTP_MAX_HEIGHT   1024
#define GTP_MAX_WIDTH    600
#define GTP_INT_TRIGGER  1
#else
#define GTP_MAX_HEIGHT   1024
#define GTP_MAX_WIDTH    600
/*DTS2017012304762 shangqiang 20170124 start*/
#define GTP_INT_TRIGGER  0
/*DTS2017012304762 shangqiang 20170124 end*/
#endif
#define GTP_MAX_TOUCH      5
/*DTS2017011806572 shangqiang/ywx422251 20170118 end */

#define VELOCITY_CUSTOM
#define TPD_VELOCITY_CUSTOM_X 15
#define TPD_VELOCITY_CUSTOM_Y 15

#define GTP_RST_GPIO    0
#define GTP_IRQ_GPIO    1

/* STEP_4(optional):If this project have touch key,Set touch key config. */
#ifdef CONFIG_GTP_HAVE_TOUCH_KEY
#define TPD_KEY_COUNT   4
#define TPD_KEYS        {KEY_BACK, KEY_HOME, KEY_MENU, KEY_SEARCH}
#define GTP_KEY_TAB {KEY_MENU, KEY_HOME, KEY_BACK, KEY_SEND}
#endif

/* ***************************PART3:OTHER define********************************* */
#define GTP_DRIVER_VERSION          "V2.6.0.3<2016/10/09>"
#define GTP_I2C_NAME                "Goodix-TS"
#define GT91XX_CONFIG_PROC_FILE     "gt9xx_config"
#define GTP_POLL_TIME               10
#define GTP_ADDR_LENGTH             2
#define GTP_CONFIG_MIN_LENGTH       186
#define GTP_CONFIG_MAX_LENGTH       240
#define FAIL                        0
#define SUCCESS                     1
#define SWITCH_OFF                  0
#define SWITCH_ON                   1

/* ******************** For GT9XXF Start ***********************/
#if defined(CONFIG_GTP_COMPATIBLE_MODE) || defined(CONFIG_GTP_HOTKNOT)
enum chip_type_t {
	CHIP_TYPE_GT9  = 0,
	CHIP_TYPE_GT9F = 1,
};
#endif

#define GTP_REG_MATRIX_DRVNUM           0x8069
#define GTP_REG_MATRIX_SENNUM           0x806A
#define GTP_REG_RQST                    0x8043
#define GTP_REG_BAK_REF                 0x99D0
#define GTP_REG_MAIN_CLK                0x8020
#define GTP_REG_CHIP_TYPE               0x8000
#define GTP_REG_HAVE_KEY                0x804E
#define GTP_REG_HN_STATE                0xAB10

#define GTP_FL_FW_BURN                  0x00
#define GTP_FL_ESD_RECOVERY             0x01
#define GTP_FL_READ_REPAIR              0x02

#define GTP_BAK_REF_SEND                0
#define GTP_BAK_REF_STORE               1
#define CFG_LOC_DRVA_NUM                29
#define CFG_LOC_DRVB_NUM                30
#define CFG_LOC_SENS_NUM                31

#define GTP_CHK_FW_MAX                  1000
#define GTP_CHK_FS_MNT_MAX              300
#define GTP_BAK_REF_PATH                "/data/gtp_ref.bin"
#define GTP_MAIN_CLK_PATH               "/data/gtp_clk.bin"
#define GTP_RQST_CONFIG                 0x01
#define GTP_RQST_BAK_REF                0x02
#define GTP_RQST_RESET                  0x03
#define GTP_RQST_MAIN_CLOCK             0x04
#define GTP_RQST_HOTKNOT_CODE           0x20
#define GTP_RQST_RESPONDED              0x00
#define GTP_RQST_IDLE                   0xFF

#define HN_DEVICE_PAIRED                0x80
#define HN_MASTER_DEPARTED              0x40
#define HN_SLAVE_DEPARTED               0x20
#define HN_MASTER_SEND                  0x10
#define HN_SLAVE_RECEIVED               0x08


/* ******************** For GT9XXF End ***********************/

/* Register define */
#define GTP_READ_COOR_ADDR          0x814E
#define GTP_REG_SLEEP               0x8040
#define GTP_REG_SENSOR_ID           0x814A
#define GTP_REG_CONFIG_DATA         0x8047
#define GTP_REG_VERSION             0x8140
#define GTP_REG_VERSION_LENGTH       6
#define GTP_REG_BOOT_STATUS 0x41E4
#define GTP_REG_BOOT_STATUS_LENGTH  1
#define GTP_REG_HW_INFO             0x4220
#define GTP_REG_REFRESH_RATE        0x8056
#define GTP_REG_ENHANCE_ESD        0x8046

#define GTP_POLLINT_TIME_SLEEP 50
#define GTP_CLK_TICK_CNT_CHARGER 200
#define GTP_CLK_TICK_CNT 200
#define GTP_CFG_MAX_VERSION 90

#define RESOLUTION_LOC              3
#define TRIGGER_LOC                 8


#define GTP_DMA_MAX_TRANSACTION_LENGTH  255   /* for DMA mode */
#define GTP_DMA_MAX_I2C_TRANSFER_SIZE   (GTP_DMA_MAX_TRANSACTION_LENGTH - GTP_ADDR_LENGTH)
#define MAX_TRANSACTION_LENGTH        8
#define TPD_I2C_NUMBER        1
#define I2C_MASTER_CLOCK              300
#define MAX_I2C_TRANSFER_SIZE         (MAX_TRANSACTION_LENGTH - GTP_ADDR_LENGTH)
#define TPD_MAX_RESET_COUNT           3
#define TPD_CALIBRATION_MATRIX        {962, 0, 0, 0, 1600, 0, 0, 0}


#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_HAVE_CALIBRATION
#define TPD_NO_GPIO
#define TPD_RESET_ISSUE_WORKAROUND

#ifdef CONFIG_GTP_WARP_X_ON
#define TPD_WARP_X(x_max, x) (x_max - 1 - x)
#else
#define TPD_WARP_X(x_max, x) x
#endif

#ifdef CONFIG_GTP_WARP_Y_ON
#define TPD_WARP_Y(y_max, y) (y_max - 1 - y)
#else
#define TPD_WARP_Y(y_max, y) y
#endif

/* Log define */
#define GTP_INFO(fmt, arg...)           pr_warn("<<-GTP-INFO->> "fmt"\n", ##arg)
#define GTP_ERROR(fmt, arg...)          pr_err("<<-GTP-ERROR->> "fmt"\n", ##arg)
#define GTP_DEBUG(fmt, arg...)  do {\
	if (GTP_DEBUG_ON)\
		pr_err("<<-GTP-DEBUG->> [%d]"fmt"\n", __LINE__, ##arg);\
} while (0)
#define GTP_DEBUG_ARRAY(array, num)  do {\
	s32 i;\
	u8 *a = array;\
	if (GTP_DEBUG_ARRAY_ON) {\
		pr_debug("<<-GTP-DEBUG-ARRAY->>\n");\
		for (i = 0; i < (num); i++) {\
			pr_debug("%02x   ", (a)[i]);\
			if ((i + 1) % 10 == 0)\
				pr_debug("\n");\
		} \
		pr_debug("\n");\
	} \
} while (0)
#define GTP_DEBUG_FUNC()  do {\
	if (GTP_DEBUG_FUNC_ON)\
		pr_debug("<<-GTP-FUNC->> Func:%s@Line:%d\n", __func__, __LINE__);\
} while (0)
#define GTP_SWAP(x, y)                 do {\
					 typeof(x) z = x;\
					 x = y;\
					 y = z;\
				       } while (0)

/* ****************************PART4:UPDATE define******************************* */
/* Error no */
#define ERROR_NO_FILE           2   /* ENOENT */
#define ERROR_FILE_READ         23  /* ENFILE */
#define ERROR_FILE_TYPE         21  /* EISDIR */
#define ERROR_GPIO_REQUEST      4   /* EINTR */
#define ERROR_I2C_TRANSFER      5   /* EIO */
#define ERROR_NO_RESPONSE       16  /* EBUSY */
#define ERROR_TIMEOUT           110 /* ETIMEDOUT */
#define UPDATE_FUNCTIONS

extern u16 show_len;
extern u16 total_len;
extern volatile u8 gtp_rawdiff_mode;
extern u8 gtp_loading_fw;

extern int tpd_halt;
extern s32 gtp_init_panel(struct i2c_client *client);
extern void gtp_reset_guitar(struct i2c_client *client, s32 ms);
extern void gtp_int_sync(s32 ms);
extern u8 gup_init_update_proc(struct i2c_client *client);
extern u8 gup_init_fw_proc(struct i2c_client *client);
extern s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len);
extern s32 gtp_i2c_write(struct i2c_client *client, u8 *buf, s32 len);
extern int i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len);
extern int i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
extern s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
extern void gtp_gpio_output(int gpio_type, int level);
extern void gtp_gpio_input(int gpio_type);
extern void gtp_irq_enable(void);
extern void gtp_irq_disable(void);
extern void uninit_manual_upgrade_node(void);
/*DTS2017012009653 shangqiang/ywx422251 20170120 begin */
extern int tpd_irq_registration(void);
/*DTS2017012009653 shangqiang/ywx422251 20170120 end */
extern struct i2c_client *i2c_client_point;

#ifdef UPDATE_FUNCTIONS
extern s32 gup_enter_update_mode(struct i2c_client *client);
extern void gup_leave_update_mode(void);
extern s32 gup_update_proc(void *dir);
#endif

extern u8 gtp_updating_fw;

#ifdef CONFIG_GTP_HOTKNOT
extern s32 gup_load_hotknot_fw(void);
extern s32 gup_load_authorization_fw(void);
extern s32 gup_recovery_touch(void);
extern s32 gup_load_touch_fw(char *filepath);
extern u8 gtp_hotknot_enabled;
extern u8 wait_hotknot_state;
extern u8 got_hotknot_state;
extern u8 got_hotknot_extra_state;
extern u8 hotknot_paired_flag;
extern wait_queue_head_t bp_waiter;
#endif

#ifdef CONFIG_GTP_ESD_PROTECT
extern void gtp_esd_switch(struct i2c_client *client, s32 on);
#endif

extern struct tpd_device *tpd;
#ifdef VELOCITY_CUSTOM
extern int tpd_v_magnify_x;
extern int tpd_v_magnify_y;
#endif

#ifdef CONFIG_GTP_SUPPORT_I2C_DMA
s32 i2c_dma_write(struct i2c_client *client, u16 addr, u8 *txbuf, s32 len);
s32 i2c_dma_read(struct i2c_client *client, u16 addr, u8 *rxbuf, s32 len);
#endif
#ifdef CONFIG_GTP_CHARGER_DETECT
extern void gtp_charger_switch(int on);
extern bool upmu_is_chr_det(void);
extern bool upmu_get_pchr_chrdet(void);
#endif
#ifdef CONFIG_GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client *);
extern void uninit_wr_node(void);
#endif
extern u8 gup_check_fs_mounted(char *path_name);
extern u8 gup_clk_calibration(void);
extern s32 gup_fw_download_proc(void *dir, u8 dwn_mode);
void gtp_get_chip_type(struct i2c_client *client);
extern u8 cfg_len;
extern u8 gtp_resetting;

/*DTS2016122100332,yuquan/ywx422261 added DSM for touchpanel,begin @20161214 */
#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#include <linux/gpio.h>
#define GT9293_DSM_TYPE_FW_LEN_ERR 0
#define GT9293_DSM_TYPE_CHECK_SUM_ERR 1
#define GT9293_DSM_TYPE_PARSE_FW_ERR 2
#define GT9293_DSM_TYPE_UPDATE_FW_ERR 3
#define GT9293_DSM_TYPE_I2C_RW_ERR 4

#define CLIENT_NAME_TOUCH_GT9293    "dsm_tp"

#define GT9293_GPIO_HW_INFO  "i2c sda num:%d,status:%d;i2c scl num:%d,status:%d;irq gpio num:%d,status:%d;reset gpio num:%d,status:%d,"
#define GT9293_MSG_FW_LEN_ERR "firmware length error\n"
#define GT9293_MSG_FW_CHKSUM_ERR  "firmware checksum error\n"
#define GT9293_MSG_FW_PARSE_ERR  "firmware parse error\n"
#define GT9293_MSG_I2C_RW_ERR  "i2c write/read error\n"
#define GT9293_MSG_UPDATE_FW_ERR  "fw update process error\n"
#define GT9293_MSG_UNKNOW_ERR  "unknow error\n"
#define GT9293_GPIO_OFFSET 881
#define GT9293_GPIO_I2C_SDA 57
#define GT9293_GPIO_I2C_SCL 58
#define GT9293_GPIO_IRQ 29
#define GT9293_GPIO_RST 45
#define GT9293_GET_GPIO_VALUE(x)  gpio_get_value(x)
#define GT9293_MSG_MAX_LEN 256
#define GT9293_SUB_MSG_MAX_LEN(x) (GT9293_MSG_MAX_LEN-sizeof(x))
#define GT9293_SHOW_GPIO_INFO  \
        GT9293_GPIO_I2C_SDA,GT9293_GET_GPIO_VALUE((GT9293_GPIO_I2C_SDA+GT9293_GPIO_OFFSET)),\
        GT9293_GPIO_I2C_SCL,GT9293_GET_GPIO_VALUE((GT9293_GPIO_I2C_SCL+GT9293_GPIO_OFFSET)),\
        GT9293_GPIO_IRQ,GT9293_GET_GPIO_VALUE(GT9293_GPIO_IRQ+GT9293_GPIO_OFFSET),\
        GT9293_GPIO_RST,GT9293_GET_GPIO_VALUE(GT9293_GPIO_RST+GT9293_GPIO_OFFSET)
int gt9293_dsm_report_err(int errno,int type);
#endif
/*DTS2016122100332,yuquan/ywx422261 added DSM for touchpanel,end @20161214 */

#define GPIO_OUTPUT_HIGH_LEVEL 1
#define GPIO_OUTPUT_LOW_LEVEL 0
#define I2C_BYTES_RETRY_TIMES_MAX 5
#define GTP_IS_IN_RESETTING_STATE 1
#define GTP_ISNOT_IN_RESETTING_STATE 0
#define CHARGER_DETECT_ON 1
#define CHARGER_DETECT_OFF 0
#define OFILM_SENSOR_ID  2
#define KD_SENSOR_ID 0
#define NAME_MANUAL_UPGRADE "gt9xx_upgrade"
#define NAME_TOUCHSCREEN_PROC_DIR "touchscreen"
#define GET_SET_IC_MAX_TIMES 5
#define PROC_UPGRADE_NUMBUF     13
#define FORCE_LOADING_STATUS_OPENING  1
#define FORCE_LOADING_STATUS_END  0

//GOODID_TOOL_WRITE_CMD
#define GOODID_TOOL_WRITE 1
#define GOODID_TOOL_WRITE_IC_TYPE 3
#define GOODID_TOOL_GET_IC_TYPE 5
#define GOODID_TOOL_DISABLE_IRQ 7
#define GOODID_TOOL_ENABLE_IRQ 9
#define GOODID_TOOL_RAWDIFF  17
#define GOODID_TOOL_ENTER_UPDATE_MODE  11
#define GOODID_TOOL_LEAVE_UPDATE_MODE  13
#define GOODID_TOOL_UPDATE_FIRMWARE  15
#define GOODID_TOOL_LOAD_SUBSYSTEM  19
#define GOODID_TOOL_POLL  21
#define GOODID_TOOL_POLL_WAKEUP 23

//GOODID_TOOL_READ_CMD
#define GOODID_TOOL_READ 2
#define GOODID_TOOL_PROCESS_BUF 4
#define GOODID_TOOL_READ_ERR_CODE 6
#define GOODID_TOOL_READ_DRIVER_VERSION 8
#define GOODID_TOOL_FLAG_NEED_INTERRUPT 2
#define GOODID_TOOL_FLAG_CONFIRM 1

#define GOODIX_TOOL_DATA_LOAD_HOTKNOT 0
#define GOODIX_TOOL_DATA_LOAD_AHTHORIZATION 1
#define GOODIX_TOOL_DATA_RECOVERY 2
#define GOODIX_TOOL_DATA_LOAD_TOUCHFW 3

#define TPD_FLAG_SLEEP 0
#define TPD_FLAG_WAKE 1

#define ESD_RUNNINT_OFF 0
#define ESD_RUNNINT_ON 1

#if defined(CONFIG_GTP_COMPATIBLE_MODE) || defined(CONFIG_GTP_HOTKNOT)
extern u8 gtp_fw_startup(struct i2c_client *client);
extern unsigned char gtp_touch_fw[];
extern enum chip_type_t gtp_chip_type;
#endif

#ifdef CONFIG_GTP_COMPATIBLE_MODE
extern u8 rqst_processing;
s32 gup_load_touch_fw(char *filepath);
s32 gup_fw_download_proc(void *dir, u8 dwn_mode);
#endif

#ifdef CONFIG_GTP_GESTURE_WAKEUP
#define GESTURE_MAX_POINT_COUNT   64
enum doze_t {
DOZE_DISABLED = 0,
DOZE_ENABLED = 1,
DOZE_WAKEUP = 2,
};

#pragma pack(1)
struct gesture_data{
	int enabled;
	enum doze_t doze_status;
	u8 ic_msg[6];        /*from the first byte */
	u8 gestures[4];
	u8 data[3 + GESTURE_MAX_POINT_COUNT * 4 + 80];   /*80 bytes for extra data */
};
#pragma pack()

extern struct gesture_data gesture_data;
extern s32 gesture_event_handler(struct input_dev * dev);
extern s8 gtp_enter_doze(void);
extern s32 gtp_extents_init(void);
extern void gtp_extents_exit(void);
#endif
/*****************************End of Part III********************************/

#endif /* TPD_CUSTOM_GT9XX_H__ */
