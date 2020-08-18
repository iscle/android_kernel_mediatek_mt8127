/*
 * psx_data program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */
#ifndef SX9323_H
#define SX9323_H

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
#include <linux/wakelock.h>
/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/


#define SX9323_NAME "sx9323"
/*
 *  I2C Registers
 */
 //-Interrupt and status
#define SX9323_IRQSTAT_REG    	0x00
#define SX9323_STAT0_REG    	0x01
#define SX9323_STAT1_REG    	0x02
#define SX9323_STAT2_REG    	0x03
#define SX9323_STAT3_REG    	0x04
#define SX9323_IRQ_ENABLE_REG	0x05
#define SX9323_IRQCFG0_REG    	0x06
#define SX9323_IRQCFG1_REG    	0x07
#define SX9323_IRQCFG2_REG		0x08
//-General control
#define SX9323_CTRL0_REG    	0x10
#define SX9323_CTRL1_REG    	0x11
#define SX9323_I2CADDR_REG    	0x14
#define SX9323_CLKSPRD			0x15
//-AFE Control
#define SX9323_AFE_CTRL0_REG    0x20
#define SX9323_AFE_CTRL1_REG    0x21
#define SX9323_AFE_CTRL2_REG    0x22
#define SX9323_AFE_CTRL3_REG    0x23
#define SX9323_AFE_CTRL4_REG	0x24
#define SX9323_AFE_CTRL5_REG	0x25
#define SX9323_AFE_CTRL6_REG	0x26
#define SX9323_AFE_CTRL7_REG	0x27
#define SX9323_AFE_PH0_REG		0x28
#define SX9323_AFE_PH1_REG		0x29
#define SX9323_AFE_PH2_REG		0x2A
#define SX9323_AFE_PH3_REG		0x2B
#define SX9323_AFE_CTRL8		0x2C
#define SX9323_AFE_CTRL9		0x2D
//-Main Digital Processing (Prox) control
#define SX9323_PROX_CTRL0_REG	0x30
#define SX9323_PROX_CTRL1_REG	0x31
#define SX9323_PROX_CTRL2_REG	0x32
#define SX9323_PROX_CTRL3_REG	0x33
#define SX9323_PROX_CTRL4_REG	0x34
#define SX9323_PROX_CTRL5_REG	0x35
#define SX9323_PROX_CTRL6_REG	0x36
#define SX9323_PROX_CTRL7_REG	0x37
//-Advanced Digital Processing control
#define SX9323_ADV_CTRL0_REG	0x40
#define SX9323_ADV_CTRL1_REG	0x41
#define SX9323_ADV_CTRL2_REG	0x42
#define SX9323_ADV_CTRL3_REG	0x43
#define SX9323_ADV_CTRL4_REG	0x44
#define SX9323_ADV_CTRL5_REG	0x45
#define SX9323_ADV_CTRL6_REG	0x46
#define SX9323_ADV_CTRL7_REG	0x47
#define SX9323_ADV_CTRL8_REG	0x48
#define SX9323_ADV_CTRL9_REG	0x49
#define SX9323_ADV_CTRL10_REG	0x4A
#define SX9323_ADV_CTRL11_REG	0x4B
#define SX9323_ADV_CTRL12_REG	0x4C
#define SX9323_ADV_CTRL13_REG	0x4D
#define SX9323_ADV_CTRL14_REG	0x4E
#define SX9323_ADV_CTRL15_REG	0x4F
#define SX9323_ADV_CTRL16_REG	0x50
#define SX9323_ADV_CTRL17_REG	0x51
#define SX9323_ADV_CTRL18_REG	0x52
#define SX9323_ADV_CTRL19_REG	0x53
#define SX9323_ADV_CTRL20_REG	0x54
//-Sensor Readback
#define SX9323_CPSRD          	0x60
#define SX9323_USEMSB         	0x61
#define SX9323_USELSB         	0x62
#define SX9323_AVGMSB         	0x63
#define SX9323_AVGLSB         	0x64
#define SX9323_DIFFMSB        	0x65
#define SX9323_DIFFLSB        	0x66
#define SX9323_OFFSETMSB		0x67
#define SX9323_OFFSETLSB		0x68
#define SX9323_SARMSB			0x69
#define SX9323_SARLSB			0x6A

#define SX9323_SOFTRESET_REG  	0x9F
#define SX9323_WHOAMI_REG		0xFA
#define SX9323_REV_REG			0xFB

//-IrqStat 0:Inactive 1:Active
#define SX9323_IRQSTAT_RESET_FLAG      	0x80
#define SX9323_IRQSTAT_TOUCH_FLAG      	0x40
#define SX9323_IRQSTAT_RELEASE_FLAG    	0x20
#define SX9323_IRQSTAT_COMPDONE_FLAG   	0x10
#define SX9323_IRQSTAT_CONV_FLAG       	0x08
#define SX9323_IRQSTAT_PROG2_FLAG		0x04
#define SX9323_IRQSTAT_PROG1_FLAG     	0x02
#define SX9323_IRQSTAT_PROG0_FLAG   	0x01


//-RegStat0
#define SX9323_PROXSTAT_PH3_FLAG    0x08
#define SX9323_PROXSTAT_PH2_FLAG   	0x04
#define SX9323_PROXSTAT_PH1_FLAG   	0x02
#define SX9323_PROXSTAT_PH0_FLAG   	0x01

//-SoftReset
#define SX9323_SOFTRESET_VALUE  0xDE
#define SX9323_WHOAMI_VALUE		0x23
#define SX9323_REV_VALUE		0x11


/*< DTS2017032503837 xingbin/xwx427571 20170325 begin*/
#define CHIP_ID_CHECK_TIMES 3
/* DTS2017032503837 xingbin/xwx427571 20170325 end >*/
/**************************************
* define platform data
*
**************************************/
struct smtc_reg_data {
	unsigned char reg;
	unsigned char val;
};
typedef struct smtc_reg_data smtc_reg_data_t;
typedef struct smtc_reg_data *psmtc_reg_data_t;


struct _buttonInfo {
	/*! The Key to send to the input */
	int keycode;
	/*! Mask to look for on Touch Status */
	int mask;
	/*! Current state of button. */
	int state;
};

struct _totalButtonInformation {
	struct _buttonInfo *buttons;
	int buttonSize;
	struct input_dev *input;
};

typedef struct _totalButtonInformation buttonInformation_t;
typedef struct _totalButtonInformation *pbuttonInformation_t;

/* Define Registers that need to be initialized to values different than
 * default
 */
static struct smtc_reg_data sx9323_i2c_reg_setup[] = {
	//--------Interrupt and config
	{
		.reg = SX9323_IRQ_ENABLE_REG,
		.val = 0x60,
	},
	{
		.reg = SX9323_IRQCFG0_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_IRQCFG1_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_IRQCFG2_REG,
		.val = 0x00,
	},
	//--------General control
	{
		.reg = SX9323_CTRL0_REG,
		.val = 0x16,
	},
	{
		.reg = SX9323_I2CADDR_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_CLKSPRD,
		.val = 0x00,
	},
	//--------AFE Control
	{
		.reg = SX9323_AFE_CTRL0_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_AFE_CTRL1_REG,
		.val = 0x10,//reserved
	},
	{
		.reg = SX9323_AFE_CTRL2_REG,
		.val = 0x00,//reserved
	},
	{
		.reg = SX9323_AFE_CTRL3_REG,
		.val = 0x01,
	},
	{
		.reg = SX9323_AFE_CTRL4_REG,
		.val = 0x44,
	},
	{
		.reg = SX9323_AFE_CTRL5_REG,
		.val = 0x00,//reserved
	},
	{
		.reg = SX9323_AFE_CTRL6_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_AFE_CTRL7_REG,
		.val = 0x44,
	},
	{
		.reg = SX9323_AFE_PH0_REG,
		.val = 0x29,
	},
	{
		.reg = SX9323_AFE_PH1_REG,
		.val = 0x26,
	},
	{
		.reg = SX9323_AFE_PH2_REG,
		.val = 0x1A,
	},
	{
		.reg = SX9323_AFE_PH3_REG,
		.val = 0x16,
	},
	{
		.reg = SX9323_AFE_CTRL8,
		.val = 0x12,
	},
	{
		.reg = SX9323_AFE_CTRL9,
		.val = 0x08,
	},
	//--------PROX control
	{
		.reg = SX9323_PROX_CTRL0_REG,
		.val = 0x09,
	},
	{
		.reg = SX9323_PROX_CTRL1_REG,
		.val = 0x09,
	},
	{
		.reg = SX9323_PROX_CTRL2_REG,
		.val = 0x20,
	},
	{
		.reg = SX9323_PROX_CTRL3_REG,
		.val = 0x60,
	},
	{
		.reg = SX9323_PROX_CTRL4_REG,
		.val = 0x0C,
	},
	{
		.reg = SX9323_PROX_CTRL5_REG,
		.val = 0x15,
	},
	{
		.reg = SX9323_PROX_CTRL6_REG,
		.val = 0x10,
	},
	{
		.reg = SX9323_PROX_CTRL7_REG,
		.val = 0x1B,
	},
	//--------Advanced control
	{
		.reg = SX9323_ADV_CTRL0_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_ADV_CTRL1_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_ADV_CTRL2_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_ADV_CTRL3_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_ADV_CTRL4_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_ADV_CTRL5_REG,
		.val = 0x05,
	},
	{
		.reg = SX9323_ADV_CTRL6_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_ADV_CTRL7_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_ADV_CTRL8_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_ADV_CTRL9_REG,
		.val = 0x80,
	},
	{
		.reg = SX9323_ADV_CTRL10_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_ADV_CTRL11_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_ADV_CTRL12_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_ADV_CTRL13_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_ADV_CTRL14_REG,
		.val = 0x80,
	},
	{
		.reg = SX9323_ADV_CTRL15_REG,
		.val = 0x0C,
	},
	{
		.reg = SX9323_ADV_CTRL16_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_ADV_CTRL17_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_ADV_CTRL18_REG,
		.val = 0x00,
	},
	{
		.reg = SX9323_ADV_CTRL19_REG,
		.val = 0xF0,
	},
	{
		.reg = SX9323_ADV_CTRL20_REG,
		.val = 0xF0,
	},
	//--------Sensor enable
	{
		.reg = SX9323_CTRL1_REG,
		.val = 0x22,
	},
};

static struct _buttonInfo psmtcButtons[] = {
	{
		.keycode = KEY_0,
		.mask = SX9323_PROXSTAT_PH0_FLAG,
	},
	{
		.keycode = KEY_1,
		.mask = SX9323_PROXSTAT_PH1_FLAG,
	},
	{
		.keycode = KEY_2,
		.mask = SX9323_PROXSTAT_PH2_FLAG,
	},
	{
		.keycode = KEY_3,
		.mask = SX9323_PROXSTAT_PH3_FLAG,
	},
};

struct sx9323_platform_data {
	int i2c_reg_num;
	struct smtc_reg_data *pi2c_reg;
	pbuttonInformation_t pbuttonInformation;
	int (*get_is_nirq_low)(void);
	int     (*init_platform_hw)(void);
	void    (*exit_platform_hw)(void);
};
typedef struct sx9323_platform_data sx9323_platform_data_t;
typedef struct sx9323_platform_data *psx9323_platform_data_t;

/***************************************
*		define data struct/interrupt
*
***************************************/
//#define USE_THREADED_IRQ

#define MAX_NUM_STATUS_BITS (8)

typedef struct sx93XX sx93XX_t, *psx93XX_t;
struct sx93XX {
	void * bus; /* either i2c_client or spi_client */
	struct device *pdev; /* common device struction for linux */
	void *pDevice; /* device specific struct pointer */
	/* Function Pointers */
	int (*init)(psx93XX_t psx_data); /* (re)initialize device */

	/* since we are trying to avoid knowing registers, create a pointer to a
	* common read register which would be to read what the interrupt source
	* is from
	*/
	int (*refreshStatus)(psx93XX_t psx_data); /* read register status */

	int (*get_nirq_low)(void); /* get whether nirq is low (platform data) */
	/* array of functions to call for corresponding status bit */
	void (*statusFunc[MAX_NUM_STATUS_BITS])(psx93XX_t psx_data);

#if defined(USE_THREADED_IRQ)
	struct mutex mutex;
#else
	spinlock_t	      lock; /* Spin Lock used for nirq worker function */
#endif
	int irq; /* irq number used */

	/* whether irq should be ignored.. cases if enable/disable irq is not used
	* or does not work properly */
	/*< DTS2017022203108 yanfei ywx429352  20170122 begin */
	/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
	bool irq_is_disabled;
	/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
	int sar_status;
	/*DTS2017022203108 yanfei ywx429352  20170122 end > */
	u8 useIrqTimer; /* older models need irq timer for pen up cases */
	int irqTimeout; /* msecs only set if useIrqTimer is true */
	/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
	spinlock_t irq_lock; /* Spin Lock used for irq enable or disable */
	struct wake_lock prox_wake_lock;
	/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
  /* struct workqueue_struct	*ts_workq;  */  /* if want to use non default */
	struct delayed_work dworker; /* work struct for worker function */
};

void sx93XX_suspend(psx93XX_t psx_data);
void sx93XX_resume(psx93XX_t psx_data);
int sx93XX_init(psx93XX_t psx_data);
int sx93XX_remove(psx93XX_t psx_data);
#endif
