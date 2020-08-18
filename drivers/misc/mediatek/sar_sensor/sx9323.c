/*! \file sx9323.c
 * \brief  SX9323 Driver
 *
 * Driver for the SX9323
 * Copyright (c) 2011 Semtech Corp
 *
 *  psx_data program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#define DEBUG
#define DRIVER_NAME "sx9323"

#define MAX_WRITE_ARRAY_SIZE 32
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include "sx9323.h" /* main struct, interrupt,init,pointers */
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of.h>
/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
#include <linux/wakelock.h>
/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
#ifdef CONFIG_GET_HARDWARE_INFO
#include <asm/hardware_info.h>
#endif
#define IDLE 0
#define ACTIVE 1
/*<DTS2017022203108 yanfei ywx429352  20170122 begin*/
#define IRQENABLE 1
#define IRQDISABLE 0
#define SARENABLE 1
#define SARDISABLE 0
#define PHEN_CTRL1_OFF 0x20
#define SAR_INIT_STATE 0
/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
#define SARIRQDELAY 50
static void sx93XX_disable_irq(psx93XX_t psx_data, bool sync);
/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
static void sx93XX_enable_irq(psx93XX_t psx_data);
/*DTS2017022203108 yanfei ywx429352  20170122 end>*/

static unsigned int sar_irq_gpio;
static struct of_device_id sx9323_match_table[];
#ifdef CONFIG_GET_HARDWARE_INFO
#define SX9323_HARDWARE_INFO "semtech,sx9323"
#endif

/*! \struct sx9323
 * \Specialized struct containing input event data, platform data, and
 * \last cap state read if needed.
 */
typedef struct sx9323 {
	pbuttonInformation_t pbuttonInformation;
	psx9323_platform_data_t hw; /* specific platform data settings */
} sx9323_t, *psx9323_t;
/*< DTS2017032503837 xingbin/xwx427571 20170325 begin*/
static int sx9323_check_chip_id(struct i2c_client *client)
{
	int reg_val = 0;
	u8 retry = CHIP_ID_CHECK_TIMES;

	if(client == NULL){
		printk(KERN_ERR "ERROR:SX9323 I2C client ptr is NULL!\n");
		return -EINVAL;
	}

	while(retry){
		reg_val = i2c_smbus_read_byte_data(client,SX9323_WHOAMI_REG);
		if(reg_val != SX9323_WHOAMI_VALUE){
			printk(KERN_ERR "I2C error! SX9323 cannot be read!\n");
			retry--;
		}
		else
		{
			printk(KERN_INFO "SX9323 check chip ID success!\n");
			return SX9323_WHOAMI_VALUE;
		}
	}

	return -ENODEV;
}
/* DTS2017032503837 xingbin/xwx427571 20170325 end >*/
/*! \fn static int write_register(psx93XX_t psx_data, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param psx_data Pointer to main parent struct
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */
static int write_register(psx93XX_t psx_data, u8 address, u8 value)
{
  struct i2c_client *i2c = 0;
  char buffer[2];
  int returnValue = 0;
  buffer[0] = address;
  buffer[1] = value;
  returnValue = -ENOMEM;
  if (psx_data && psx_data->bus) {
    i2c = psx_data->bus;

    returnValue = i2c_master_send(i2c,buffer,2);
  }
  return returnValue;
}

/*! \fn static int read_register(psx93XX_t psx_data, u8 address, u8 *value)
* \brief Reads a register's value from the device
* \param psx_data Pointer to main parent struct
* \param address 8-Bit address to read from
* \param value Pointer to 8-bit value to save register value to
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int read_register(psx93XX_t psx_data, u8 address, u8 *value)
{
	struct i2c_client *i2c = 0;
	s32 returnValue = 0;
	if (psx_data && value && psx_data->bus) {
		i2c = psx_data->bus;
		returnValue = i2c_smbus_read_byte_data(i2c,address);
		/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
		dev_dbg(&i2c->dev, "read_register Address: 0x%x Return: 0x%x\n",address,returnValue);
		/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
		if (returnValue >= 0) {
			*value = returnValue;
			return 0;
		} else {
			return returnValue;
		}
	}
	return -ENOMEM;
}
static struct _totalButtonInformation smtcButtonInformation = {
	.buttons = psmtcButtons,
	.buttonSize = ARRAY_SIZE(psmtcButtons),
};

static sx9323_platform_data_t sx9323_config = {
	.init_platform_hw = NULL,
	.exit_platform_hw = NULL,
	.pi2c_reg = sx9323_i2c_reg_setup,
	.i2c_reg_num = ARRAY_SIZE(sx9323_i2c_reg_setup),
	.pbuttonInformation = &smtcButtonInformation,
};

/*! \brief Sends a write register range to the device
 * \param psx_data Pointer to main parent struct
 * \param reg 8-bit register address (base address)
 * \param data pointer to 8-bit register values
 * \param size size of the data pointer
 * \return Value from i2c_master_send
 */


/*********************************************************************/
/*! \brief Perform a manual offset calibration
* \param psx_data Pointer to main parent struct
* \return Value return value from the write register
 */
static int manual_offset_calibration(psx93XX_t psx_data)
{
	s32 returnValue = 0;
	returnValue = write_register(psx_data,SX9323_STAT2_REG,0x02);
	return returnValue;
}
/*! \brief sysfs show function for manual calibration which currently just
 * returns register value.
 */
static ssize_t manual_offset_calibration_show(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	u8 msb = 0;
	u8 lsb = 0;
	u16 prox_offset = 0;
	psx93XX_t psx_data = dev_get_drvdata(dev);
	manual_offset_calibration(psx_data);
	msleep(100);
	write_register(psx_data,SX9323_CPSRD, 1);//here to check the CS1, also can read other channel
	read_register(psx_data,SX9323_OFFSETMSB, &msb);
	read_register(psx_data,SX9323_OFFSETLSB, &lsb);
	prox_offset =(msb << 8) | lsb;
	dev_info(psx_data->pdev, "sx9323  prox_offset = %d\n", prox_offset);
	return sprintf(buf, "%d\n", prox_offset);
}

static ssize_t manual_offset_calibration_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	unsigned long val;
	psx93XX_t psx_data = dev_get_drvdata(dev);
	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val) {
		dev_info(psx_data->pdev, "Performing manual_offset_calibration()\n");
		manual_offset_calibration(psx_data);
	}
	return count;
}

static ssize_t rawdata_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	u8 msb = 0;
	u8 lsb = 0;
	u16 use_value = 0;
	u16 avg_value = 0;
	u16 diff_value = 0;
	psx93XX_t psx_data = dev_get_drvdata(dev);
	write_register(psx_data, SX9323_CPSRD, 1);//here to check the CS1, also can read other channel
	read_register(psx_data, SX9323_USEMSB, &msb);
	read_register(psx_data, SX9323_USELSB, &lsb);
	use_value = (msb << 8) | lsb;
	read_register(psx_data, SX9323_AVGMSB, &msb);
	read_register(psx_data, SX9323_AVGLSB, &lsb);
	avg_value = (msb << 8) | lsb;
	read_register(psx_data, SX9323_DIFFMSB, &msb);
	read_register(psx_data, SX9323_DIFFLSB, &lsb);
	diff_value = (msb << 8) | lsb;
	return sprintf(buf, "use_value=%d,avg_value=%d,diff_value=%d\n", use_value, avg_value, diff_value);
}
/*<DTS2017022203108 yanfei ywx429352  20170122 begin*/
static ssize_t sar_enable_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
    unsigned int data ;
    psx93XX_t psx_data = dev_get_drvdata(dev);
    sscanf(buf, "%d", &data);
    printk("%s:data = %d\n",__func__,data);
    if(data==SARDISABLE){
        if(psx_data->sar_status==SARENABLE){
            write_register(psx_data,SX9323_CTRL1_REG,PHEN_CTRL1_OFF);
			/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
            sx93XX_disable_irq(psx_data, true);
			/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
            psx_data->sar_status =SARDISABLE;
        }else{
            printk("faie to disable  sx9323 mode\n");
        }
    }else if(data==SARENABLE){
        if(psx_data->sar_status==SARDISABLE){
            sx93XX_enable_irq(psx_data);
            psx_data->sar_status =SARENABLE;
            if (psx_data->init)
                psx_data->init(psx_data);
       }else{
            printk("faie to enable  sx9323 mode\n");
       }
    }else{
        printk("invalid format\n");
    }
    return count;
}

static ssize_t sar_enable_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    int len = SAR_INIT_STATE;
    psx93XX_t psx_data = dev_get_drvdata(dev);
	/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
    len += sprintf(buf + len, "sar enabled is %d\n", psx_data->sar_status);
	/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
    return  len;
}
/*DTS2017022203108 yanfei ywx429352  20170122 end>*/

static ssize_t chipinfo_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0;
	psx93XX_t psx_data = dev_get_drvdata(dev);

	read_register(psx_data, SX9323_WHOAMI_REG, &reg_value);
	if(SX9323_WHOAMI_VALUE == reg_value){
		return sprintf(buf, "%s\n", "SX9323");
	}else{
		return sprintf(buf, "%s\n", "Read Error!");
	}

}
/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
static ssize_t irq_enable_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
    unsigned int data ;
    psx93XX_t psx_data = dev_get_drvdata(dev);
    sscanf(buf, "%d", &data);
    printk(KERN_ERR "%s:irq data = %d\n",__func__,data);

    if(data == IRQENABLE){
        sx93XX_enable_irq(psx_data);
    }else if(data == IRQDISABLE){
        sx93XX_disable_irq(psx_data, true);
    }else{
        printk("invalid format\n");
    }
    return count;
}

static ssize_t irq_enable_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    int len = 0;
    psx93XX_t psx_data = dev_get_drvdata(dev);
    len += sprintf(buf + len, "irq_enabled is %d\n", (!psx_data->irq_is_disabled));
    return  len;
}

static ssize_t dump_interrupts_status_registers_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    int len = 0;
    int reg_addr = 0;
    u8 reg_value = 0;
    psx93XX_t psx_data = dev_get_drvdata(dev);

    for(reg_addr = SX9323_IRQSTAT_REG;reg_addr <= SX9323_IRQCFG2_REG; reg_addr++){
        read_register(psx_data,reg_addr,&reg_value);
	 printk(KERN_ERR "%s: reg_addr = 0x%02x, reg_value = 0x%02x\n",__func__,reg_addr,reg_value);
	 len += sprintf(buf + len, "reg_addr 0x%02x: 0x%02x\n", reg_addr,reg_value);
    }

    return  len;
}

static ssize_t dump_general_control_registers_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    int len = 0;
    int reg_addr = 0;
    u8 reg_value = 0;
    psx93XX_t psx_data = dev_get_drvdata(dev);

    for(reg_addr = SX9323_CTRL0_REG;reg_addr <= SX9323_CLKSPRD; reg_addr++){
        read_register(psx_data,reg_addr,&reg_value);
	 printk(KERN_ERR "%s: reg_addr = 0x%02x, reg_value = 0x%02x\n",__func__,reg_addr,reg_value);
	 len += sprintf(buf + len, "reg_addr 0x%02x: 0x%02x\n", reg_addr,reg_value);
    }

    return  len;
}

static ssize_t dump_afe_control_registers_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    int len = 0;
    int reg_addr = 0;
    u8 reg_value = 0;
    psx93XX_t psx_data = dev_get_drvdata(dev);

    for(reg_addr = SX9323_AFE_CTRL0_REG;reg_addr <= SX9323_AFE_CTRL9; reg_addr++){
        read_register(psx_data,reg_addr,&reg_value);
	 printk(KERN_ERR "%s: reg_addr = 0x%02x, reg_value = 0x%02x\n",__func__,reg_addr,reg_value);
	 len += sprintf(buf + len, "reg_addr 0x%02x: 0x%02x\n", reg_addr,reg_value);
    }

    return  len;
}

static ssize_t dump_mdp_control_registers_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    int len = 0;
    int reg_addr = 0;
    u8 reg_value = 0;
    psx93XX_t psx_data = dev_get_drvdata(dev);

    for(reg_addr = SX9323_PROX_CTRL0_REG;reg_addr <= SX9323_PROX_CTRL7_REG; reg_addr++){
        read_register(psx_data,reg_addr,&reg_value);
	 printk(KERN_ERR "%s: reg_addr = 0x%02x, reg_value = 0x%02x\n",__func__,reg_addr,reg_value);
	 len += sprintf(buf + len, "reg_addr 0x%02x: 0x%02x\n", reg_addr,reg_value);
    }

    return  len;
}

static ssize_t dump_adp_control_registers_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    int len = 0;
    int reg_addr = 0;
    u8 reg_value = 0;
    psx93XX_t psx_data = dev_get_drvdata(dev);

    for(reg_addr = SX9323_ADV_CTRL0_REG;reg_addr <= SX9323_ADV_CTRL20_REG; reg_addr++){
        read_register(psx_data,reg_addr,&reg_value);
	 printk(KERN_ERR "%s: reg_addr = 0x%02x, reg_value = 0x%02x\n",__func__,reg_addr,reg_value);
	 len += sprintf(buf + len, "reg_addr 0x%02x: 0x%02x\n", reg_addr,reg_value);
    }

    return  len;
}

static ssize_t ic_irq_status_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    int len = 0;
    int reg_addr = SX9323_IRQSTAT_REG;
    u8 reg_value = 0;
    psx93XX_t psx_data = dev_get_drvdata(dev);

    read_register(psx_data,reg_addr,&reg_value);
    printk(KERN_ERR "%s: reg_addr = 0x%02x, reg_value = 0x%02x\n",__func__,reg_addr,reg_value);

    len += sprintf(buf + len, "reg_irq_status 0x%02x: 0x%02x\n", reg_addr,reg_value);

    return  len;
}
/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
static DEVICE_ATTR(calibrate, 0644, manual_offset_calibration_show,
					manual_offset_calibration_store);
static DEVICE_ATTR(chipinfo, 0644, chipinfo_show, NULL);
static DEVICE_ATTR(rawdata, 0644, rawdata_show, NULL);
static DEVICE_ATTR(sar_enable, 0644, sar_enable_show,
                                sar_enable_store);
/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
static DEVICE_ATTR(irq_enable, 0644, irq_enable_show,
                                irq_enable_store);
static DEVICE_ATTR(dump_interrupts_status_registers, 0644, dump_interrupts_status_registers_show,
                                NULL);
static DEVICE_ATTR(dump_general_control_registers, 0644, dump_general_control_registers_show,
                                NULL);
static DEVICE_ATTR(dump_afe_control_registers, 0644, dump_afe_control_registers_show,
                                NULL);
static DEVICE_ATTR(dump_mdp_control_registers, 0644, dump_mdp_control_registers_show,
                                NULL);
static DEVICE_ATTR(dump_adp_control_registers, 0644, dump_adp_control_registers_show,
                                NULL);
static DEVICE_ATTR(ic_irq_status, 0644, ic_irq_status_show,
                                NULL);
/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
static struct attribute *sx9323_attributes[] = {
	&dev_attr_calibrate.attr,
	&dev_attr_chipinfo.attr,
	&dev_attr_rawdata.attr,
	&dev_attr_sar_enable.attr,
	/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
	&dev_attr_irq_enable.attr,
	&dev_attr_dump_interrupts_status_registers.attr,
	&dev_attr_dump_general_control_registers.attr,
	&dev_attr_dump_afe_control_registers.attr,
	&dev_attr_dump_mdp_control_registers.attr,
	&dev_attr_dump_adp_control_registers.attr,
	&dev_attr_ic_irq_status.attr,
	/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
	NULL,
};
/*DTS2017022203108 yanfei ywx429352  20170122 end >*/

static struct attribute_group sx9323_attr_group = {
	.attrs = sx9323_attributes,
};

/*! \fn static int read_regStat(psx93XX_t psx_data)
 * \brief Shortcut to read what caused interrupt.
 * \details psx_data is to keep the drivers a unified
 * function that will read whatever register(s)
 * provide information on why the interrupt was caused.
 * \param psx_data Pointer to main parent struct
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(psx93XX_t psx_data)
{
	u8 data = 0;
	if (psx_data) {
		if (read_register(psx_data,SX9323_IRQSTAT_REG,&data) == 0)
			return (data & 0x00FF);
	}
	return 0;
}

static void read_rawData(psx93XX_t psx_data)
{
	u8 msb=0, lsb=0;
	if(psx_data){
		write_register(psx_data, SX9323_CPSRD, 1);//here to check the CS1, also can read other channel
		msleep(100);
		read_register(psx_data, SX9323_USEMSB, &msb);
		read_register(psx_data, SX9323_USELSB, &lsb);
		read_register(psx_data, SX9323_AVGMSB, &msb);
		read_register(psx_data, SX9323_AVGLSB, &lsb);
		read_register(psx_data, SX9323_DIFFMSB, &msb);
		read_register(psx_data, SX9323_DIFFLSB, &lsb);
		read_register(psx_data, SX9323_OFFSETMSB, &msb);
		read_register(psx_data, SX9323_OFFSETLSB, &lsb);
	}
}

/*! \brief  Initialize I2C config from platform data
 * \param psx_data Pointer to main parent struct
 */
static void hw_init(psx93XX_t psx_data)
{
	psx9323_t pDevice = 0;
	psx9323_platform_data_t pdata = 0;
	int i = 0;
	/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
	printk(KERN_ERR "Enter %s, line %d\n",__func__,__LINE__);
	/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
	/* configure device */
	if (psx_data && (pDevice = psx_data->pDevice) && (pdata = pDevice->hw))
	{
		while ( i < pdata->i2c_reg_num) {
			/* Write all registers/values contained in i2c_reg */
			/*dev_dbg(psx_data->pdev, "Going to Write Reg: 0x%x Value: 0x%x\n",
			pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);*/
			write_register(psx_data, pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
			i++;
		}
	} else {
		dev_err(psx_data->pdev, "ERROR! platform data 0x%p\n", pDevice->hw);
	}
}

/*! \fn static int initialize(psx93XX_t psx_data)
 * \brief Performs all initialization needed to configure the device
 * \param psx_data Pointer to main parent struct
 * \return Last used command's return value (negative if error)
 */
/*< DTS2017022203108 yanfei ywx429352  20170122 begin*/
static int initialize(psx93XX_t psx_data)
{
	if (psx_data) {
		/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
		printk(KERN_ERR "Enter %s, line %d\n",__func__,__LINE__);
		/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
		/* prepare reset by disabling any irq handling */
		/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
		sx93XX_disable_irq(psx_data, true);
		/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
		/* perform a reset */
		msleep(100);
		hw_init(psx_data);
		msleep(100); /* make sure everything is running */
		manual_offset_calibration(psx_data);

		/* re-enable interrupt handling */
		sx93XX_enable_irq(psx_data);
		/* make sure no interrupts are pending since enabling irq will only
		* work on next falling edge */
		read_regStat(psx_data);
		//dev_dbg(psx_data->pdev, "Exiting initialize(). NIRQ = %d\n",psx_data->get_nirq_low());
		return 0;
	}
	return -ENOMEM;
}
/*DTS2017022203108 yanfei ywx429352  20170122 end >*/

/*!
 * \brief Handle what to do when a touch occurs
 * \param psx_data Pointer to main parent struct
 */
static void touchProcess(psx93XX_t psx_data)
{
	u8 i = 0;
	int counter = 0;
	int numberOfButtons = 0;
	psx9323_t pDevice = NULL;
	struct _buttonInfo *buttons = NULL;
	struct input_dev *input = NULL;
	struct _buttonInfo *pCurrentButton = NULL;

	if (psx_data && (pDevice = psx_data->pDevice)) {
		/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
		printk(KERN_ERR "Enter %s, line %d\n",__func__,__LINE__);
		/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
		read_register(psx_data, SX9323_STAT0_REG, &i);
		buttons = pDevice->pbuttonInformation->buttons;
		input = pDevice->pbuttonInformation->input;
		numberOfButtons = pDevice->pbuttonInformation->buttonSize;

		if (unlikely((buttons == NULL) || (input == NULL))) {
			dev_err(psx_data->pdev, "ERROR! buttons or input NULL!\n");
			return;
		}

		for (counter = 0; counter < numberOfButtons; counter++) {
			pCurrentButton = &buttons[counter];
			if (pCurrentButton==NULL) {
				dev_err(psx_data->pdev,"ERROR! button at index: %d NULL!\n", counter);
				return; // ERRORR!!!!
			}
			switch (pCurrentButton->state) {
				case IDLE: /* Button is not being touched! */
					if (((i & pCurrentButton->mask) == pCurrentButton->mask)) {
						/* User pressed button */
						dev_info(psx_data->pdev, "cap button %d touched\n", counter);
						//input_report_key(input, pCurrentButton->keycode, 1);
						input_report_switch(input, SW_FRONT_PROXIMITY, 1);
						pCurrentButton->state = ACTIVE;
					}
					break;
				case ACTIVE: /* Button is being touched! */
					if (((i & pCurrentButton->mask) != pCurrentButton->mask)) {
						/* User released button */
						dev_info(psx_data->pdev, "cap button %d released\n",counter);
						//input_report_key(input, pCurrentButton->keycode, 0);
						input_report_switch(input, SW_FRONT_PROXIMITY, 0);
						pCurrentButton->state = IDLE;
					}
					break;
				default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
					break;
			};
		}
		input_sync(input);
	}
}

/*! \fn static int sx9323_probe(struct i2c_client *client, const struct i2c_device_id *id)
 * \brief Probe function
 * \param client pointer to i2c_client
 * \param id pointer to i2c_device_id
 * \return Whether probe was successful
 */
static int sx9323_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i = 0;
	int ret = 0;
	int err = 0;
	psx93XX_t psx_data = 0;
	psx9323_t pDevice = 0;
	psx9323_platform_data_t pplatData = 0;
	struct input_dev *input = NULL;

	dev_info(&client->dev, "sx9323_probe\n");
	pplatData = &sx9323_config;
	if (!pplatData) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

/*< DTS2017032503837 xingbin/xwx427571 20170325 begin*/
	ret = sx9323_check_chip_id(client);
	if(ret < 0){
		printk(KERN_ERR "ERROR! SX9323 cannot be read!ret = %d\n", ret);
		return ret;
	}
/* DTS2017032503837 xingbin/xwx427571 20170325 end >*/
	psx_data = kzalloc(sizeof(sx93XX_t), GFP_KERNEL);
	if (psx_data == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
	}
	if (client->dev.of_node) {
		const struct of_device_id *match;
		match = of_match_device(of_match_ptr(sx9323_match_table), &(client->dev));
		if (!match) {
			dev_err(&client->dev,"No device match found\n");
			return -ENODEV;
		}
	}
	sar_irq_gpio = of_get_named_gpio(client->dev.of_node, "sar-gpio", 0);
	err = gpio_request(sar_irq_gpio,  "sar-gpio");
	if (err < 0) {
		printk(KERN_ERR "%s: sx9323 gpio %d request, err=%d", __func__, sar_irq_gpio,err);
		gpio_free(sar_irq_gpio);
	}
	/* create memory for main struct */
	if (psx_data) {
		/* In case we need to reinitialize data
		 * (e.q. if suspend reset device) */
		psx_data->init = initialize;
		/* shortcut to read status of interrupt */
		psx_data->refreshStatus = read_regStat;
		/* pointer to function from platform data to get pendown
		 * (1->NIRQ=0, 0->NIRQ=1) */
		psx_data->get_nirq_low = pplatData->get_is_nirq_low;
		/* save irq in case we need to reference it */
		psx_data->irq = client->irq;
		/* do we need to create an irq timer after interrupt ? */
		psx_data->useIrqTimer = 0;

		/* Setup function to call on corresponding reg irq source bit */
		if (MAX_NUM_STATUS_BITS>= 8) {
			psx_data->statusFunc[0] = 0; /* TXEN_STAT */
			psx_data->statusFunc[1] = 0; /* UNUSED */
			psx_data->statusFunc[2] = 0; /* UNUSED */
			psx_data->statusFunc[3] = read_rawData; /* CONV_STAT */
			psx_data->statusFunc[4] = 0; /* COMP_STAT */
			psx_data->statusFunc[5] = touchProcess; /* RELEASE_STAT */
			psx_data->statusFunc[6] = touchProcess; /* TOUCH_STAT  */
			psx_data->statusFunc[7] = 0; /* RESET_STAT */
		}

		/* setup i2c communication */
		psx_data->bus = client;
		i2c_set_clientdata(client, psx_data);
		/* record device struct */
		psx_data->pdev = &client->dev;
		/* create memory for device specific struct */
		psx_data->pDevice = pDevice = kzalloc(sizeof(sx9323_t), GFP_KERNEL);
		if (pDevice) {
			/* for accessing items in user data (e.g. calibrate) */
			ret = sysfs_create_group(&client->dev.kobj, &sx9323_attr_group);
			if (ret < 0) {
				dev_err(&client->dev, "failed to register sysfs\n");
				return -EINVAL;
			}

			/* Check if we hava a platform initialization function to call*/
			if (pplatData->init_platform_hw)
				pplatData->init_platform_hw();

			/* Add Pointer to main platform data struct */
			pDevice->hw = pplatData;

			/* Initialize the button information initialized with keycodes */
			pDevice->pbuttonInformation = pplatData->pbuttonInformation;

			/* Create the input device */
			input = input_allocate_device();
			if (!input) {
				return -ENOMEM;
			}

			/* Set all the keycodes */
			__set_bit(EV_SW, input->evbit);
			__set_bit(SW_FRONT_PROXIMITY, input->swbit);
			for (i = 0; i < pDevice->pbuttonInformation->buttonSize; i++) {
				pDevice->pbuttonInformation->buttons[i].state = IDLE;
			}
			/* save the input pointer and finish initialization */
			pDevice->pbuttonInformation->input = input;
			input->name = "sx9323 sar";
			input->id.bustype = BUS_I2C;

			if(input_register_device(input))
				return -ENOMEM;
		}

#ifdef CONFIG_GET_HARDWARE_INFO
		register_hardware_info(SAR,SX9323_HARDWARE_INFO);
#endif
		sx93XX_init(psx_data);
		/*< DTS2017022203108 yanfei ywx429352  20170122 begin */
		write_register(psx_data,SX9323_CTRL1_REG,PHEN_CTRL1_OFF);
		/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
		sx93XX_disable_irq(psx_data, true);
		/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
		/*DTS2017022203108 yanfei ywx429352  20170122 end >*/

		return  0;
	}
	return -1;
}

/*! \fn static int sx9323_remove(struct i2c_client *client)
 * \brief Called when device is to be removed
 * \param client Pointer to i2c_client struct
 * \return Value from sx93XX_remove()
 */
static int sx9323_remove(struct i2c_client *client)
{
	psx9323_platform_data_t pplatData = 0;
	psx9323_t pDevice = 0;
	psx93XX_t psx_data = i2c_get_clientdata(client);
	if (psx_data && (pDevice = psx_data->pDevice)) {
		input_unregister_device(pDevice->pbuttonInformation->input);
		sysfs_remove_group(&client->dev.kobj, &sx9323_attr_group);
		pplatData = client->dev.platform_data;

		if (pplatData && pplatData->exit_platform_hw)
			pplatData->exit_platform_hw();

		kfree(psx_data->pDevice);
	}
	return sx93XX_remove(psx_data);
}

/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
#ifdef SX9323_ACTIVE_PM
/***** Kernel Suspend *****/
static int sx9323_suspend(struct i2c_client *client, pm_message_t mesg)
{
	psx93XX_t psx_data = i2c_get_clientdata(client);
	sx93XX_suspend(psx_data);
	return 0;
}

/***** Kernel Resume *****/
static int sx9323_resume(struct i2c_client *client)
{
	psx93XX_t psx_data = i2c_get_clientdata(client);
	sx93XX_resume(psx_data);
	return 0;
}
#endif
/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
static struct of_device_id sx9323_match_table[] =
{
	{.compatible = "mediatek,sx9323"},
	{},
};

static struct i2c_device_id sx9323_idtable[] = {
	{DRIVER_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, sx9323_idtable);
static struct i2c_driver sx9323_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.of_match_table = sx9323_match_table,
		.name   = SX9323_NAME,
	},
	.id_table = sx9323_idtable,
	.probe    = sx9323_probe,
	.remove   = sx9323_remove,
/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
#ifdef SX9323_ACTIVE_PM
	.suspend  = sx9323_suspend,
	.resume   = sx9323_resume,
#else
	.suspend  = NULL,
	.resume   = NULL,
#endif
/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
};
static int __init sx9323_init(void)
{
	return i2c_add_driver(&sx9323_driver);
}
static void __exit sx9323_exit(void)
{
	i2c_del_driver(&sx9323_driver);
}

#ifdef USE_THREADED_IRQ
static void sx93XX_process_interrupt(psx93XX_t psx_data,u8 nirqlow)
{
	int status = 0;
	int counter = 0;
	if (!psx_data) {
		printk(KERN_ERR "sx93XX_worker_func, NULL sx93XX_t\n");
		return;
	}
	/* since we are not in an interrupt don't need to disable irq. */
	status = psx_data->refreshStatus(psx_data);
	counter = -1;

	while((++counter) < MAX_NUM_STATUS_BITS) { /* counter start from MSB */
		if (((status>>counter) & 0x01) && (psx_data->statusFunc[counter])) {
			psx_data->statusFunc[counter](psx_data);
		}
	}
	if (unlikely(psx_data->useIrqTimer && nirqlow)) {
	/* In case we need to send a timer for example on a touchscreen
	 * checking penup, perform psx_data here
	 */
	cancel_delayed_work(&psx_data->dworker);
	schedule_delayed_work(&psx_data->dworker,msecs_to_jiffies(psx_data->irqTimeout));
	dev_info(psx_data->pdev,"Schedule Irq timer");
	}
}


static void sx93XX_worker_func(struct work_struct *work)
{
	psx93XX_t psx_data = 0;
	if (work) {
		psx_data = container_of(work,sx93XX_t,dworker.work);
		if (!psx_data) {
			printk(KERN_ERR "sx93XX_worker_func, NULL sx93XX_t\n");
			return;
		}
		if ((!psx_data->get_nirq_low) || (!psx_data->get_nirq_low())) {
			/* only run if nirq is high */
			sx93XX_process_interrupt(psx_data,0);
		}
	} else {
		printk(KERN_ERR "sx93XX_worker_func, NULL work_struct\n");
	}
}
static irqreturn_t sx93XX_interrupt_thread(int irq, void *data)
{
	psx93XX_t psx_data = 0;
	psx_data = data;
	mutex_lock(&psx_data->mutex);
	//dev_dbg(psx_data->pdev, "sx93XX_irq\n");
	if ((!psx_data->get_nirq_low) || psx_data->get_nirq_low()) {
		sx93XX_process_interrupt(psx_data, 1);
	} else
		dev_err(psx_data->pdev, "sx93XX_irq - nirq read high\n");

	mutex_unlock(&psx_data->mutex);
	return IRQ_HANDLED;
}
#else
static void sx93XX_schedule_work(psx93XX_t psx_data, unsigned long delay)
{
	unsigned long flags;
	if (psx_data) {
		spin_lock_irqsave(&psx_data->lock,flags);
		/* Stop any pending penup queues */
		cancel_delayed_work(&psx_data->dworker);
		//after waiting for a delay, psx_data put the job in the kernel-global workqueue. so no need to create new thread in work queue.
		schedule_delayed_work(&psx_data->dworker,delay);
		spin_unlock_irqrestore(&psx_data->lock,flags);
	} else
		printk(KERN_ERR "sx93XX_schedule_work, NULL psx93XX_t\n");
}

static irqreturn_t sx93XX_irq(int irq, void *pvoid)
{
	/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
	psx93XX_t psx_data = NULL;
	/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
	if (pvoid) {
		psx_data = (psx93XX_t)pvoid;
		/*< DTS2017022203108 yanfei ywx429352  20170122 begin */
		/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
		sx93XX_disable_irq(psx_data, false);
		wake_lock_timeout(&psx_data->prox_wake_lock, 5*HZ);
		/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
		/*DTS2017022203108 yanfei ywx429352  20170122 end > */
		if ((!psx_data->get_nirq_low) || psx_data->get_nirq_low()) {
			sx93XX_schedule_work(psx_data,0);
		} else
		dev_err(psx_data->pdev, "sx93XX_irq - nirq read high\n");
	} else
		printk(KERN_ERR "sx93XX_irq, NULL pvoid\n");

	return IRQ_HANDLED;
}
/*< DTS2017022203108 yanfei ywx429352  20170122 begin */
/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
static void sx93XX_disable_irq(psx93XX_t psx_data, bool sync)
{
  unsigned long irqflags;

  spin_lock_irqsave(&psx_data->irq_lock, irqflags);

  if (psx_data->irq_is_disabled == false){
    if(sync){
      disable_irq(psx_data->irq);
    }else{
      disable_irq_nosync(psx_data->irq);
    }
      psx_data->irq_is_disabled = true;
  }else{
      printk(KERN_ERR "%s:irq is already disabled!\n",__func__);
  }

	spin_unlock_irqrestore(&psx_data->irq_lock, irqflags);
}
static void sx93XX_enable_irq(psx93XX_t psx_data)
{
  unsigned long irqflags;

  spin_lock_irqsave(&psx_data->irq_lock, irqflags);

  if (psx_data->irq_is_disabled == true){
    enable_irq(psx_data->irq);
    psx_data->irq_is_disabled = false;
  }else{
    printk(KERN_ERR "%s:irq is already enabled!\n",__func__);
  }
	spin_unlock_irqrestore(&psx_data->irq_lock, irqflags);
}
/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
/*DTS2017022203108 yanfei ywx429352  20170122 end > */
static void sx93XX_worker_func(struct work_struct *work)
{
	psx93XX_t psx_data = 0;
	int status = 0;
	int counter = 0;
	u8 nirqLow = 0;
	/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
	printk(KERN_ERR "Enter %s, line %d\n",__func__,__LINE__);
	/*DTS2017022203108 yanfei ywx429352  20170122 end > */
	if (work) {
		psx_data = container_of(work,sx93XX_t,dworker.work);

		if (!psx_data) {
			printk(KERN_ERR "sx93XX_worker_func, NULL sx93XX_t\n");
			return;
		}
		/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
		msleep(SARIRQDELAY);
		/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
		if (unlikely(psx_data->useIrqTimer)) {
			if ((!psx_data->get_nirq_low) || psx_data->get_nirq_low()) {
				nirqLow = 1;
			}
		}
		/* since we are not in an interrupt don't need to disable irq. */
		status = psx_data->refreshStatus(psx_data);
		counter = -1;
		while((++counter) < MAX_NUM_STATUS_BITS) { /* counter start from MSB */
			if (((status>>counter) & 0x01) && (psx_data->statusFunc[counter])) {
				psx_data->statusFunc[counter](psx_data);
			}
		}
		/* Early models and if RATE=0 for newer models require a penup timer */
		if (unlikely(psx_data->useIrqTimer && nirqLow)) {
			/* Queue up the function again for checking on penup */
			sx93XX_schedule_work(psx_data,msecs_to_jiffies(psx_data->irqTimeout));
		}
	} else {
		printk(KERN_ERR "sx93XX_worker_func, NULL work_struct\n");
	}
	/*< DTS2017022203108 yanfei ywx429352  20170122 begin */
	sx93XX_enable_irq(psx_data);
	/*DTS2017022203108 yanfei ywx429352  20170122 end > */
}
#endif
/*< DTS2017022203108 yanfei ywx429352  20170122 begin */
void sx93XX_suspend(psx93XX_t psx_data)
{
	if ((psx_data)&&(psx_data->sar_status==SARENABLE)){
		/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
		sx93XX_disable_irq(psx_data, true);
		/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
		write_register(psx_data,SX9323_CTRL1_REG,PHEN_CTRL1_OFF);//make sx9323 in Sleep mode
	}
}

void sx93XX_resume(psx93XX_t psx_data)
{
	if ((psx_data)&&(psx_data->sar_status==SARENABLE)) {
#ifdef USE_THREADED_IRQ
		mutex_lock(&psx_data->mutex);
		/* Just in case need to reset any uncaught interrupts */
		sx93XX_process_interrupt(psx_data,0);
		mutex_unlock(&psx_data->mutex);
#else
		sx93XX_schedule_work(psx_data,0);
#endif
		if (psx_data->init)
			psx_data->init(psx_data);
		sx93XX_enable_irq(psx_data);
	}
}
/*DTS2017022203108 yanfei ywx429352  20170122 end > */
int sx93XX_init(psx93XX_t psx_data)
{
	struct device_node *node;
	int err = 0;
	if (psx_data && psx_data->pDevice) {
	/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
	printk(KERN_ERR "Enter %s, line %d\n",__func__,__LINE__);
	/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
#ifdef USE_THREADED_IRQ
		/* initialize worker function */
		INIT_DELAYED_WORK(&psx_data->dworker, sx93XX_worker_func);


		/* initialize mutex */
		mutex_init(&psx_data->mutex);
		/* initailize interrupt reporting */
		psx_data->irq_disabled = 0;
		err = request_threaded_irq(psx_data->irq, NULL, sx93XX_interrupt_thread,
					IRQF_TRIGGER_FALLING, psx_data->pdev->driver->name, psx_data);
#else
		/* initialize spin lock */
		spin_lock_init(&psx_data->lock);
		/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
		spin_lock_init(&psx_data->irq_lock);
		/* initialize wake lock */
		wake_lock_init(&psx_data->prox_wake_lock, WAKE_LOCK_SUSPEND, "sx9323_wake_lock");
		/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
		/* initialize worker function */
		INIT_DELAYED_WORK(&psx_data->dworker, sx93XX_worker_func);

		/* initailize interrupt reporting */

		node = of_find_matching_node(NULL, sx9323_match_table);
		if (node) {
			psx_data->irq = irq_of_parse_and_map(node, 0);
			err = request_irq(psx_data->irq, sx93XX_irq, IRQF_TRIGGER_FALLING,
									psx_data->pdev->driver->name, psx_data);
		}
		//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
/*< DTS2017022203108 yanfei ywx429352  20170122 begin */
		if (err) {
			dev_err(psx_data->pdev, "irq %d busy?\n", psx_data->irq);
		    wake_lock_destroy(&psx_data->prox_wake_lock);
			return err;
		}
        else{
             enable_irq_wake(psx_data->irq);
	         sx93XX_disable_irq(psx_data, true);
        }
/*DTS2017022203108 yanfei ywx429352  20170122 end > */
#ifdef USE_THREADED_IRQ
		dev_info(psx_data->pdev, "registered with threaded irq (%d)\n", psx_data->irq);
#else
		dev_info(psx_data->pdev, "registered with irq (%d)\n", psx_data->irq);
#endif
		/* call init function pointer (psx_data should initialize all registers */
		if (psx_data->init)
			return psx_data->init(psx_data);

		dev_err(psx_data->pdev,"No init function!!!!\n");
	}
	return -ENOMEM;
}

int sx93XX_remove(psx93XX_t psx_data)
{
	if (psx_data) {
		cancel_delayed_work_sync(&psx_data->dworker); /* Cancel the Worker Func */
		//destroy_workqueue(psx_data->workq);
		/*<DTS2017031305217 xingbin/xwx427571 20170313 begin */
		wake_lock_destroy(&psx_data->prox_wake_lock);
		/* DTS2017031305217 xingbin/xwx427571 20170313 end >*/
		free_irq(psx_data->irq, psx_data);
		kfree(psx_data);
		return 0;
	}
	return -ENOMEM;
}

module_init(sx9323_init);
module_exit(sx9323_exit);

MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com)");
MODULE_DESCRIPTION("SX9323 Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
