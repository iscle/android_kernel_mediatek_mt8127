#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"
#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
//#include <mach/mt_pm_ldo.h>
//#include <mach/mt_gpio.h>
#endif
//#include <cust_gpio_usage.h>

#include <linux/of_gpio.h>
#include <linux/gpio.h>


#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0

static unsigned int GPIO_LCD_PWR_EN;
static unsigned int GPIO_LCD_RST;
static unsigned int GPIO_LCD_BL_EN;
static unsigned int GPIO_LCD_RST2;

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))
#define MDELAY(n)                                           (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                       lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)


static const unsigned char LCD_MODULE_ID = 0x01;
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE 0
#define FRAME_WIDTH                                         (600)
#define FRAME_HEIGHT                                        (1024)

#define REGFLAG_DELAY                                           0xFFFC
#define REGFLAG_END_OF_TABLE                                0xFFFB   // END OF REGISTERS MARKER

#define DELAY_FOR_DISPLAY_ONOFF			120
#define	DELAY_FOR_POWER_ON				5
#define	DELAY_FOR_RST_READY				40
#define	DELAY_FOR_RST_H					20
#define	DELAY_FOR_RST_L					20

#define	LCM_SETTING_TABLE_NUM			64
#define	LCM_STATE_ERR                   -1
#define	LCM_STATE_ZERO                  0
#define	LCM_TABLE_COUNT                 10
#define	LCM_PACKET_SIZE                 256
#define	WORD_COUNT_WIDTH                3
#define	FORCE_UPDATE_FLAG1				1
#define	FORCE_UPDATE_FLAG0				0
#define GPIO_INDEX 0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
/*< DTS2017022304156 xingbin/xwx427571 20170307 begin*/
#define BOE_VER_SYNC_ACTIVE		1
#define BOE_VER_BACKPROCH		25
#define BOE_VER_FRONTPROCH		184

#define BOE_HOR_SYNC_ACTIVE		1
#define BOE_HOR_BACKPROCH		88
#define BOE_HOR_FRONTPROCH		184
#define BOE_DSI_MIPI_PLL_CLOCK		184
/*< DTS2017022304156 xingbin/xwx427571 20170307 end*/

#define BOE_DSI_MIPI_CONT_CLOCK		1
#define BOE_DSI_MIPI_HS_TRAIL		1
#define BOE_DSI_MIPI_SSC_DISABLE	1
/*< DTS2017012601939 xingbin/xwx427571 20170217 begin */
#define DEALY_FOR_RST_INIT			20
/*< DTS2017012601939 xingbin/xwx427571 20170217 end */

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[LCM_SETTING_TABLE_NUM];
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28,0,{}},
	{0x10,0,{}},
	{REGFLAG_DELAY, DELAY_FOR_DISPLAY_ONOFF, {}}
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x00}},
	{0xB1, 2, {0x68,0x41}},
	{0xB5, 1, {0x88}},
	{0xB6, 1, {0x0F}},
	{0xB8, 4, {0x01,0x01,0x12,0x01}},
	{0xBB, 2, {0x11,0x11}},
	{0xBC, 2, {0x05,0x05}},
	{0xC7, 1, {0x03}},
	{0xBD, 5, {0x03,0x02,0x19,0x17,0x00}},
	{0xC8, 1, {0x80}},

	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x01}},
	{0xB2, 2, {0x01,0x01}},
	{0xB3, 2, {0x28,0x28}},
	{0xB4, 2, {0x14,0x14}},
	{0xB8, 2, {0x05,0x05}},
	{0xB9, 2, {0x45,0x45}},
	{0xBA, 2, {0x25,0x25}},
	{0xBC, 2, {0x88,0x00}},
	{0xBD, 2, {0x88,0x00}},
	{0xBE, 1, {0x4B}},

	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x02}},
	{0xEE, 1, {0x03}},
	{0xB0, 16, {0x00,0x41,0x00,0x56,0x00,0x78,0x00,0x8F,0x00,0xA6,0x00,0xC9,0x00,0xE5,0x01,0x13}},
	{0xB1, 16, {0x01,0x37,0x01,0x73,0x01,0xA0,0x01,0xE5,0x02,0x1D,0x02,0x1E,0x02,0x51,0x02,0x88}},
	{0xB2, 16, {0x02,0xAD,0x02,0xDE,0x02,0xFE,0x03,0x29,0x03,0x46,0x03,0x6E,0x03,0x8E,0x03,0x95}},
	{0xB3, 4, {0x03,0xC5,0x03,0xD4}},
	{0xB4, 16, {0x00,0x41,0x00,0x56,0x00,0x78,0x00,0x8F,0x00,0xA6,0x00,0xC9,0x00,0xE5,0x01,0x13}},
	{0xB5, 16, {0x01,0x37,0x01,0x73,0x01,0xA0,0x01,0xE5,0x02,0x1D,0x02,0x1E,0x02,0x51,0x02,0x88}},
	{0xB6, 16, {0x02,0xAD,0x02,0xDE,0x02,0xFE,0x03,0x29,0x03,0x46,0x03,0x6E,0x03,0x8E,0x03,0x95}},
	{0xB7, 4, {0x03,0xC5,0x03,0xD4}},
	{0xB8, 16, {0x01,0x18,0x01,0x1D,0x01,0x26,0x01,0x2F,0x01,0x38,0x01,0x48,0x01,0x56,0x01,0x6D}},
	{0xB9, 16, {0x01,0x84,0x01,0xA7,0x01,0xCC,0x02,0x04,0x02,0x33,0x02,0x34,0x02,0x62,0x02,0x9A}},
	{0xBA, 16, {0x02,0xBB,0x02,0xE5,0x03,0x07,0x03,0x35,0x03,0x4C,0x03,0x6E,0x03,0x8E,0x03,0x95}},
	{0xBB, 4, {0x03,0xC5,0x03,0xD4}},

	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x04}},
	{0xB1, 5, {0x03,0x02,0x02,0x02,0x00}},

	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x06}},
	{0xB0, 2, {0x11,0x11}},
	{0xB1, 2, {0x13,0x13}},
	{0xB2, 2, {0x03,0x03}},
	{0xB3, 2, {0x34,0x34}},
	{0xB4, 2, {0x34,0x34}},
	{0xB5, 2, {0x34,0x34}},
	{0xB6, 2, {0x34,0x34}},
	{0xB7, 2, {0x34,0x34}},
	{0xB8, 2, {0x34,0x34}},
	{0xB9, 2, {0x34,0x34}},
	{0xBA, 2, {0x34,0x34}},
	{0xBB, 2, {0x34,0x34}},
	{0xBC, 2, {0x34,0x34}},
	{0xBD, 2, {0x34,0x34}},
	{0xBE, 2, {0x34,0x34}},
	{0xBF, 2, {0x34,0x34}},
	{0xC0, 2, {0x34,0x34}},
	{0xC1, 2, {0x02,0x02}},
	{0xC2, 2, {0x12,0x12}},
	{0xC3, 2, {0x10,0x10}},
	{0xE5, 2, {0x34,0x34}},
	{0xD8, 5, {0x00,0x00,0x00,0x00,0x00}},
	{0xD9, 5, {0x00,0x00,0x00,0x00,0x00}},

	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x05}},
	{0xC0, 1, {0x03}},
	{0xC1, 1, {0x02}},
	{0xC8, 2, {0x01,0x20}},
	{0xE5, 1, {0x03}},
	{0xE6, 1, {0x03}},
	{0xE7, 1, {0x03}},
	{0xE8, 1, {0x03}},
	{0xE9, 1, {0x03}},
	{0xD1, 4, {0x03,0x00,0x3D,0x00}},

	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x03}},
	{0xB0, 2, {0x11,0x00}},
	{0xB1, 2, {0x11,0x00}},
	{0xB2, 5, {0x03,0x00,0x00,0x00,0x00}},
	{0xB3, 5, {0x03,0x00,0x00,0x00,0x00}},
	{0xBA, 5, {0x31,0x00,0x00,0x00,0x00}},
	//CABC
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x00}},
	{0xE6, 4, {0xFF,0xF8,0xF2,0xE5}},
	{0xD5, 13, {0x11,0x12,0x1C,0x16,0x17,0x0B,0x09,0x08,0x02,0x00,0x00,0x04,0x0E}},
	//CE
	{0xCC, 6, {0x12,0x36,0x87,0x00,0x00,0x00}},
	{0xD1, 16, {0x00,0x04,0x08,0x0C,0x10,0x14,0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C}},
	{0xD7, 12, {0xC0,0x40,0x40,0x40,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x04}},
	{0xD8, 12, {0x08,0x0C,0x10,0x14,0x14,0x10,0x0C,0x08,0x04,0x00,0x00,0x40}},
	{0xD2, 16, {0x00,0x04,0x08,0x0D,0x10,0x12,0x14,0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38}},

	{0x35, 1, {0x00}},
	{0x53, 1, {0x2C}},
	{0x55, 1, {0x81}},
	{0x51, 1, {0x00}},
	{0x62, 1, {0x01}},
	{0x11, 0, {}},
	{REGFLAG_DELAY, DELAY_FOR_DISPLAY_ONOFF, {}},
	{0x29, 0, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static void lcm_gpio_init(void)
{
	static struct device_node *node;
	int err = LCM_STATE_ERR;

	node = of_find_compatible_node(NULL, NULL, "mediatek,lcm");

	GPIO_LCD_PWR_EN = of_get_named_gpio(node, "lcm_power_gpio", GPIO_INDEX);
	GPIO_LCD_RST = of_get_named_gpio(node, "lcm_reset_gpio", GPIO_INDEX);
	GPIO_LCD_BL_EN = of_get_named_gpio(node, "lcm_bl_en_gpio", GPIO_INDEX);
	GPIO_LCD_RST2 = of_get_named_gpio(node, "lcm_reset2_gpio", GPIO_INDEX);
	err = gpio_request(GPIO_LCD_PWR_EN,"GPIO_LCD_PWR_EN");
	if(err < LCM_STATE_ZERO)
	{
		printk(KERN_ERR "%s: gpio %d request, err=%d", __func__, GPIO_LCD_PWR_EN,err);
		return;
	}

	err = gpio_request(GPIO_LCD_RST,"GPIO_LCD_RST");
	if(err < LCM_STATE_ZERO)
	{
		printk(KERN_ERR "%s: gpio %d request, err=%d", __func__, GPIO_LCD_RST,err);
		return;
	}

	err = gpio_request(GPIO_LCD_BL_EN,"GPIO_LCD_BL_EN");
	if(err < LCM_STATE_ZERO)
	{
		printk(KERN_ERR "%s: gpio %d request, err=%d", __func__, GPIO_LCD_BL_EN,err);
		return;
	}

	err = gpio_request(GPIO_LCD_RST2,"GPIO_LCD_RST2");
	if(err < LCM_STATE_ZERO)
	{
		printk(KERN_ERR "%s: gpio %d request, err=%d", __func__, GPIO_LCD_RST2,err);
		return;
	}
}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = LCM_STATE_ZERO; i < count; i++) {
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY :
				if (table[i].count <= LCM_TABLE_COUNT)
					MDELAY(table[i].count);
				else
					MDELAY(table[i].count);
				break;

			case REGFLAG_END_OF_TABLE :
				break;

			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, LCM_STATE_ZERO, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_EVENT_VDO_MODE;
#endif

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM                = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order     = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq       = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding         = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format              = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	params->dsi.packet_size=LCM_PACKET_SIZE;
	//video mode timing
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.word_count=FRAME_WIDTH*WORD_COUNT_WIDTH;

	/*< DTS2017012204145 xingbin/xwx427571 20170122 begin */
	params->dsi.vertical_sync_active                = BOE_VER_SYNC_ACTIVE;
	params->dsi.vertical_backporch                  = BOE_VER_BACKPROCH;
	params->dsi.vertical_frontporch                 = BOE_VER_FRONTPROCH;//for 600*1024 :300;for 800*1280:8
	params->dsi.vertical_active_line                    = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active              = BOE_HOR_SYNC_ACTIVE;
	params->dsi.horizontal_backporch                = BOE_HOR_BACKPROCH;
	params->dsi.horizontal_frontporch               = BOE_HOR_FRONTPROCH;
	params->dsi.horizontal_active_pixel             = FRAME_WIDTH;
	params->dsi.ssc_disable                         = BOE_DSI_MIPI_SSC_DISABLE;
	params->dsi.cont_clock  = BOE_DSI_MIPI_CONT_CLOCK;
	params->dsi.PLL_CLOCK = BOE_DSI_MIPI_PLL_CLOCK;
	params->dsi.HS_TRAIL = BOE_DSI_MIPI_HS_TRAIL;

}

static void lcm_set_gpio_output(unsigned int gpio, unsigned int output)
{
	gpio_direction_output(gpio, output);
	gpio_set_value(gpio, output);
}

static void lcm_init_power(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_init_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ZERO);
	MDELAY(DELAY_FOR_POWER_ON);
	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ZERO);
#else
	printk("[Kernel/LCM] lcm_init_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ZERO);
	MDELAY(DELAY_FOR_POWER_ON);
	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ZERO);
#endif

}

static void lcm_suspend_power(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_suspend_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ZERO);
	MDELAY(DELAY_FOR_POWER_ON);
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ZERO);
#else
	printk("[Kernel/LCM] lcm_suspend_power() enter\n");
	/*< DTS2017030304401 xingbin/xwx427571 20170217 begin */
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	lcm_set_gpio_output(GPIO_LCD_RST2, GPIO_OUT_ONE);
	MDELAY(DELAY_FOR_RST_READY);
	/* DTS2017030304401 xingbin/xwx427571 20170217 end > */
	lcm_set_gpio_output(GPIO_LCD_BL_EN,GPIO_OUT_ZERO);
	MDELAY(DELAY_FOR_POWER_ON);
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ZERO);
#endif
}

static void lcm_resume_power(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_resume_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ONE);
	MDELAY(DELAY_FOR_POWER_ON);
	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
#else
	printk("[Kernel/LCM] lcm_resume_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ONE);
	MDELAY(DELAY_FOR_POWER_ON);
	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
#endif

}

static void lcm_init(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_init() enter\n");

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(DELAY_FOR_RST_H);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(DELAY_FOR_RST_L);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(DELAY_FOR_RST_H);

	// when phone initial , config output high, enable backlight drv chip
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), FORCE_UPDATE_FLAG1);
	printf("[LK/LCM] lcm_init() end\n");
#else

	printk("[Kernel/LCM] lcm_init() enter\n");

#endif
}
/*< DTS2017012601939 xingbin/xwx427571 20170217 begin */
static void lcm_early_suspend(void)
{

	printk("[Kernel/LCM] KING lcm_early_suspend() enter\n");
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(DEALY_FOR_RST_INIT);
}
/*< DTS2017012601939 xingbin/xwx427571 20170217 end */

static void lcm_suspend(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_suspend() enter\n");
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), FORCE_UPDATE_FLAG1);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(DELAY_FOR_RST_L);
#else
	printk("[Kernel/LCM] lcm_suspend() enter\n");
	/*< DTS2017012601939 xingbin/xwx427571 20170217 begin */
	//push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), FORCE_UPDATE_FLAG1);
	//MDELAY(DELAY_FOR_RST_H);
	/*< DTS2017012601939 xingbin/xwx427571 20170217 end */
	/*< DTS2017030304401 xingbin/xwx427571 20170217 begin */
	//lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	//lcm_set_gpio_output(GPIO_LCD_RST2, GPIO_OUT_ONE);
	//MDELAY(DELAY_FOR_RST_READY);
	/* DTS2017030304401 xingbin/xwx427571 20170217 end > */
	printk("[Kernel/LCM] lcm_suspend() end\n");

#endif
}

static void lcm_resume(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_resume() enter\n");

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(DELAY_FOR_RST_H);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(DELAY_FOR_RST_L);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(DELAY_FOR_RST_H);

	// when phone initial , config output high, enable backlight drv chip
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), FORCE_UPDATE_FLAG1);
#else
	printk("[Kernel/LCM] lcm_resume() enter\n");
	MDELAY(DELAY_FOR_RST_READY);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	lcm_set_gpio_output(GPIO_LCD_RST2, GPIO_OUT_ZERO);
	MDELAY(DELAY_FOR_RST_H);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	lcm_set_gpio_output(GPIO_LCD_RST2, GPIO_OUT_ONE);
	MDELAY(DELAY_FOR_RST_L);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	lcm_set_gpio_output(GPIO_LCD_RST2, GPIO_OUT_ZERO);
	MDELAY(DELAY_FOR_RST_H);

	// when phone initial , config output high, enable backlight drv chip
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), FORCE_UPDATE_FLAG1);
	printk("[Kernel/LCM] lcm_resume() end\n");
#endif
}
#define LCM_XY_WH 1
#define LCM_XY_SHIFT_8  8
#define LCM_XY_SHIFT_F  0xFF
#define LCM_XY_SHIFT_16 16
#define LCM_XY_SHIFT_24 24

#define DATA_ARRAY_NUM	16
#define DATA_ARRAY_0	0
#define DATA_ARRAY_1	1
#define DATA_ARRAY_2	2

#define DATA_ARRAY0_ADDRESS 0x00053902
#define DATA_ARRAY0_ADDRESS0 0x002c3909

#define QUEUE_SIZE_NUM 3

#define MIPI_CMD_PARA_2A 0x2a
#define MIPI_CMD_PARA_2B 0x2b

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - LCM_XY_WH;
	unsigned int y1 = y0 + height - LCM_XY_WH;

	unsigned char x0_MSB = ((x0>>LCM_XY_SHIFT_8)&LCM_XY_SHIFT_F);
	unsigned char x0_LSB = (x0&LCM_XY_SHIFT_F);
	unsigned char x1_MSB = ((x1>>LCM_XY_SHIFT_8)&LCM_XY_SHIFT_F);
	unsigned char x1_LSB = (x1&LCM_XY_SHIFT_F);
	unsigned char y0_MSB = ((y0>>LCM_XY_SHIFT_8)&LCM_XY_SHIFT_F);
	unsigned char y0_LSB = (y0&LCM_XY_SHIFT_F);
	unsigned char y1_MSB = ((y1>>LCM_XY_SHIFT_8)&LCM_XY_SHIFT_F);
	unsigned char y1_LSB = (y1&LCM_XY_SHIFT_F);

	unsigned int data_array[DATA_ARRAY_NUM];

	data_array[DATA_ARRAY_0]= DATA_ARRAY0_ADDRESS;
	data_array[DATA_ARRAY_1]= (x1_MSB<<LCM_XY_SHIFT_24)|(x0_LSB<<LCM_XY_SHIFT_16)|(x0_MSB<<LCM_XY_SHIFT_8)|MIPI_CMD_PARA_2A;
	data_array[DATA_ARRAY_2]= (x1_LSB);
	dsi_set_cmdq(data_array, QUEUE_SIZE_NUM, FORCE_UPDATE_FLAG1);

	data_array[DATA_ARRAY_0]= DATA_ARRAY0_ADDRESS;
	data_array[DATA_ARRAY_1]= (y1_MSB<<LCM_XY_SHIFT_24)|(y0_LSB<<LCM_XY_SHIFT_16)|(y0_MSB<<LCM_XY_SHIFT_8)|MIPI_CMD_PARA_2B;
	data_array[DATA_ARRAY_2]= (y1_LSB);
	dsi_set_cmdq(data_array, QUEUE_SIZE_NUM, FORCE_UPDATE_FLAG1);

	data_array[DATA_ARRAY_0]= DATA_ARRAY0_ADDRESS0;
	dsi_set_cmdq(data_array, QUEUE_SIZE_NUM, FORCE_UPDATE_FLAG0);
}
#endif
/*< DTS2017021306702 xingbin/xwx427571 20170213 begin */
static void lcm_setbacklight(unsigned int level)
{
	static int level_last = LCD_BRIGHTNESS_DACK;
	pr_err("BOE %s, kernel nt51021 backlight: level = %d\n", __func__, level);

	if((level<=RT9293_MIN_BRIGHTNESS_VALUE)&&(level!=LCD_BRIGHTNESS_DACK))
		level = RT9293_MIN_BRIGHTNESS_VALUE;

	/*Refresh value of backlight level.*/
	lcm_backlight_level_setting[DATA_ARRAY_0].para_list[DATA_ARRAY_0] = level;
	push_table(lcm_backlight_level_setting,
	sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), FORCE_UPDATE_FLAG1);
	level_last = level;
}
/*< DTS2017021306702 xingbin/xwx427571 20170213 end */
LCM_DRIVER nt35521s_wsvga_dsi_vdo_boe_lcm_drv= {
	.name               = "nt35521s_wsvga_dsi_vdo_boe",
	.set_util_funcs     = lcm_set_util_funcs,
	.get_params         = lcm_get_params,
	.init                   = lcm_init,
	.suspend            = lcm_suspend,
	.resume             = lcm_resume,
	.init_power        = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.gpio_init = lcm_gpio_init,
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
	.set_backlight = lcm_setbacklight,
	/*< DTS2017012601939 xingbin/xwx427571 20170217 begin */
	.early_suspend =	lcm_early_suspend,
	/*< DTS2017012601939 xingbin/xwx427571 20170217 end */
};
