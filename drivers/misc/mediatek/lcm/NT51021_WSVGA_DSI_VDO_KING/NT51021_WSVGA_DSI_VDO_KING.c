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
/* A1911-87 litao2 2016-11-23 begin */
static unsigned int GPIO_LCD_RST2;
/* A1911-87 litao2 2016-11-23 end */

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


//static unsigned char lcd_id_pins_value = 0xFF;
static const unsigned char LCD_MODULE_ID = 0x01; //  haobing modified 2013.07.11
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE 0
#define FRAME_WIDTH                                         (600)
#define FRAME_HEIGHT                                        (1024)

/* A1911-67 litao2 20161121 begin */
/*< DTS2016123002286  xingbin/xwx427571 20170106 begin */
#define REGFLAG_DELAY                                           0xFFFC
#define REGFLAG_END_OF_TABLE                                0xFFFB   // END OF REGISTERS MARKER
/*< DTS2016123002286  xingbin/xwx427571 20170106 begin */
/* A1911-67 litao2 20161121 end */

/*< DTS2017012302027 xingbin/xwx427571 20170207 begin */
/*< DTS2016122102033 xingbin/xwx427571 20170103 begin */
/*< DTS2016122915506 xingbin/xwx427571 20170117 begin */
#define DEALY_FOR_VDD_VBAT			5
#define DEALY_FOR_VBAT_RST			35
#define DEALY_FOR_RST_INIT			20
/*< DTS2017021706071  xingbin/xwx427571 20170217 begin */
#define DEALY_FOR_VDD_OFF			500
/* DTS2017021706071  xingbin/xwx427571 20170217 end > */

#define MDELAY_FOR_ENTER_SLEEP	120
/*< DTS2016122915506 xingbin/xwx427571 20170117 end */
/*< DTS2016122102033 xingbin/xwx427571 20170103 end */
/*< DTS2017012302027 xingbin/xwx427571 20170207 end */
/*< DTS2017012204145 xingbin/xwx427571 20170122 begin */
/*< DTS2017022209018 xingbin/xwx427571 20170222 begin */
/*< DTS2017022304156 xingbin/xwx427571 20170307 begin*/
#define KING_VER_SYNC_ACTIVE		1
#define KING_VER_BACKPROCH		25
#define KING_VER_FRONTPROCH		184

#define KING_HOR_SYNC_ACTIVE		1
#define KING_HOR_BACKPROCH		88
#define KING_HOR_FRONTPROCH		184

#define KING_DSI_MIPI_PLL_CLOCK	184
/* DTS2017022304156 xingbin/xwx427571 20170307 end >*/
/*< DTS2017022209018 xingbin/xwx427571 20170222 end */
/*< DTS2017012204145 xingbin/xwx427571 20170122 end */
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

/*< DTS2016123002286  xingbin/xwx427571 20170106 begin */
struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};
/*< DTS2017011300079 xingbin/xwx427571 20170111 begin */
static struct LCM_setting_table lcm_mipi_reg_setting[] = {
	{0x83,1,{0x00}},
	{REGFLAG_DELAY, 5, {}},
};
/*< DTS2017011300079 xingbin/xwx427571 20170111 end */
/*< DTS2016123002286  xingbin/xwx427571 20170106 end */
/* A1911-67 litao2 20161121 begin */
static struct LCM_setting_table lcm_backlight_level_setting[] = {
//	{0x83, 1, {0xBB} },
//	{0x84, 1, {0x22} },
//	{0x95, 1, {0xA0} },
	{0x83, 1, {0x00} },
	{0x84, 1, {0x00} },
	{0x9F, 1, {0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
/* A1911-67 litao2 20161121 end */

//update initial param for IC nt51021 0.01
static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x10,0,{}},
	{REGFLAG_DELAY, MDELAY_FOR_ENTER_SLEEP, {}}
};

//update initial param for IC nt51021 0.01

/*< DTS2016123002286 xingbin/xwx427571 20170108 begin */
/*< DTS2017010800689 xingbin/xwx427571 20170111 begin */
/*< DTS2017012005870 xingbin/xwx427571 20170120 begin */
/*< DTS2017012302027 xingbin/xwx427571 20170207 begin */
static struct LCM_setting_table lcm_initialization_setting[] = {
	{0x83,1,{0x00}},
	{0x84,1,{0x00}},
	{0x9f,1,{0x00}},
	{0x83,1,{0xBB}},
	{0x84,1,{0x22}},
	{0x91,1,{0xA0}},
	/*< DTS2016122915506 xingbin/xwx427571 20170117 begin */
	{0x95,1,{0xB0}},
	/*< DTS2016122915506 xingbin/xwx427571 20170117 begin */

	/*< DTS2017021603061 xingbin/xwx427571 20170216 begin */
	{0x83,1,{0x00}},
	{0x84,1,{0x00}},
	{0x84,1,{0x00}},
	{0x85,1,{0x04}},
	{0x86,1,{0x08}},
	{0x8C,1,{0x8A}},
	{0xC5,1,{0x2B}},
	{0xC7,1,{0x2B}},
	{0x83,1,{0xAA}},
	{0x84,1,{0x11}},
	{0xA9,1,{0x4B}},
	{0x83,1,{0x00}},
	{0x84,1,{0x00}},
	{0xFD,1,{0x5B}},
	{0xFA,1,{0x14}},
	/*< DTS2017021603061 xingbin/xwx427571 20170216 end */
	/*gamma otp*/
	{0x83,1,{0x00}},
	{0x84,1,{0x00}},
	{0x8C,1,{0x8E}},
	{0xC5,1,{0x2B}},
	{0xC7,1,{0x2B}},
	{0xFD,1,{0x5B}},
	{0xFA,1,{0x14}},

	{0x83,1,{0xAA}},
	{0x84,1,{0x11}},
	{0xC0,1,{0x19}},
	{0xC1,1,{0x1A}},
	{0xC2,1,{0x23}},
	{0xC3,1,{0x31}},
	{0xC4,1,{0x3E}},
	{0xC5,1,{0x49}},
	{0xC6,1,{0x52}},
	{0xC7,1,{0x5B}},
	{0xC8,1,{0x63}},
	{0xC9,1,{0xDB}},
	{0xCA,1,{0xE0}},
	{0xCB,1,{0xFB}},
	{0xCC,1,{0x06}},
	{0xCD,1,{0x0A}},
	{0xCE,1,{0x08}},
	{0xCF,1,{0x09}},
	{0xD0,1,{0x09}},
	{0xD1,1,{0x17}},
	{0xD2,1,{0x28}},
	{0xD3,1,{0x45}},
	{0xD4,1,{0x4B}},
	{0xD5,1,{0xAE}},
	{0xD6,1,{0xB4}},
	{0xD7,1,{0xBB}},
	{0xD8,1,{0xC4}},
	{0xD9,1,{0xCF}},
	{0xDA,1,{0xDA}},
	{0xDB,1,{0xE6}},
	{0xDC,1,{0xF5}},
	{0xDD,1,{0xFF}},
	{0xDE,1,{0xF8}},
	{0xDF,1,{0x0F}},
	{0xE0,1,{0x19}},
	{0xE1,1,{0x1A}},
	{0xE2,1,{0x23}},
	{0xE3,1,{0x31}},
	{0xE4,1,{0x3E}},
	{0xE5,1,{0x49}},
	{0xE6,1,{0x52}},
	{0xE7,1,{0x5B}},
	{0xE8,1,{0x63}},
	{0xE9,1,{0xDB}},
	{0xEA,1,{0xE0}},
	{0xEB,1,{0xFB}},
	{0xEC,1,{0x06}},
	{0xED,1,{0x0A}},
	{0xEE,1,{0x08}},
	{0xEF,1,{0x09}},
	{0xF0,1,{0x09}},
	{0xF1,1,{0x17}},
	{0xF2,1,{0x28}},
	{0xF3,1,{0x45}},
	{0xF4,1,{0x4B}},
	{0xF5,1,{0xAE}},
	{0xF6,1,{0xB4}},
	{0xF7,1,{0xBB}},
	{0xF8,1,{0xC4}},
	{0xF9,1,{0xCF}},
	{0xFA,1,{0xDA}},
	{0xFB,1,{0xE6}},
	{0xFC,1,{0xF5}},
	{0xFD,1,{0xFF}},
	{0xFE,1,{0xF8}},
	{0xFF,1,{0x0F}},

	{0x83,1,{0xBB}},
	{0x84,1,{0x22}},
	{0xC0,1,{0x19}},
	{0xC1,1,{0x1A}},
	{0xC2,1,{0x24}},
	{0xC3,1,{0x31}},
	{0xC4,1,{0x3D}},
	{0xC5,1,{0x47}},
	{0xC6,1,{0x50}},
	{0xC7,1,{0x58}},
	{0xC8,1,{0x60}},
	{0xC9,1,{0xD6}},
	{0xCA,1,{0xDA}},
	{0xCB,1,{0xF2}},
	{0xCC,1,{0xFE}},
	{0xCD,1,{0xFE}},
	{0xCE,1,{0xF8}},
	{0xCF,1,{0xF9}},
	{0xD0,1,{0xF2}},
	{0xD1,1,{0xF5}},
	{0xD2,1,{0xFC}},
	{0xD3,1,{0x0A}},
	{0xD4,1,{0x05}},
	{0xD5,1,{0x81}},
	{0xD6,1,{0x86}},
	{0xD7,1,{0x8A}},
	{0xD8,1,{0x8F}},
	{0xD9,1,{0x93}},
	{0xDA,1,{0x98}},
	{0xDB,1,{0x9E}},
	{0xDC,1,{0xA4}},
	{0xDD,1,{0xA7}},
	{0xDE,1,{0x00}},
	{0xDF,1,{0x0C}},
	{0xE0,1,{0x19}},
	{0xE1,1,{0x1A}},
	{0xE2,1,{0x24}},
	{0xE3,1,{0x31}},
	{0xE4,1,{0x3D}},
	{0xE5,1,{0x47}},
	{0xE6,1,{0x50}},
	{0xE7,1,{0x58}},
	{0xE8,1,{0x60}},
	{0xE9,1,{0xD6}},
	{0xEA,1,{0xDA}},
	{0xEB,1,{0xF2}},
	{0xEC,1,{0xFE}},
	{0xED,1,{0xFE}},
	{0xEE,1,{0xF8}},
	{0xEF,1,{0xF9}},
	{0xF0,1,{0xF2}},
	{0xF1,1,{0xF5}},
	{0xF2,1,{0xFC}},
	{0xF3,1,{0x0A}},
	{0xF4,1,{0x05}},
	{0xF5,1,{0x81}},
	{0xF6,1,{0x86}},
	{0xF7,1,{0x8A}},
	{0xF8,1,{0x8F}},
	{0xF9,1,{0x93}},
	{0xFA,1,{0x98}},
	{0xFB,1,{0x9E}},
	{0xFC,1,{0xA4}},
	{0xFD,1,{0xA7}},
	{0xFE,1,{0x00}},
	{0xFF,1,{0x0C}},

	{0x83,1,{0xCC}},
	{0x84,1,{0x33}},
	{0xC0,1,{0x19}},
	{0xC1,1,{0x1A}},
	{0xC2,1,{0x24}},
	{0xC3,1,{0x32}},
	{0xC4,1,{0x3E}},
	{0xC5,1,{0x49}},
	{0xC6,1,{0x52}},
	{0xC7,1,{0x5B}},
	{0xC8,1,{0x62}},
	{0xC9,1,{0xD7}},
	{0xCA,1,{0xDD}},
	{0xCB,1,{0xF8}},
	{0xCC,1,{0x04}},
	{0xCD,1,{0x08}},
	{0xCE,1,{0x06}},
	{0xCF,1,{0x08}},
	{0xD0,1,{0x08}},
	{0xD1,1,{0x16}},
	{0xD2,1,{0x26}},
	{0xD3,1,{0x41}},
	{0xD4,1,{0x45}},
	{0xD5,1,{0xA8}},
	{0xD6,1,{0xAF}},
	{0xD7,1,{0xB6}},
	{0xD8,1,{0xBD}},
	{0xD9,1,{0xC5}},
	{0xDA,1,{0xCD}},
	{0xDB,1,{0xD7}},
	{0xDC,1,{0xE2}},
	{0xDD,1,{0xE9}},
	{0xDE,1,{0xF8}},
	{0xDF,1,{0x0F}},
	{0xE0,1,{0x19}},
	{0xE1,1,{0x1A}},
	{0xE2,1,{0x24}},
	{0xE3,1,{0x32}},
	{0xE4,1,{0x3E}},
	{0xE5,1,{0x49}},
	{0xE6,1,{0x52}},
	{0xE7,1,{0x5B}},
	{0xE8,1,{0x62}},
	{0xE9,1,{0xD7}},
	{0xEA,1,{0xDD}},
	{0xEB,1,{0xF8}},
	{0xEC,1,{0x04}},
	{0xED,1,{0x08}},
	{0xEE,1,{0x06}},
	{0xEF,1,{0x08}},
	{0xF0,1,{0x08}},
	{0xF1,1,{0x16}},
	{0xF2,1,{0x26}},
	{0xF3,1,{0x41}},
	{0xF4,1,{0x45}},
	{0xF5,1,{0xA8}},
	{0xF6,1,{0xAF}},
	{0xF7,1,{0xB6}},
	{0xF8,1,{0xBD}},
	{0xF9,1,{0xC5}},
	{0xFA,1,{0xCD}},
	{0xFB,1,{0xD7}},
	{0xFC,1,{0xE2}},
	{0xFD,1,{0xE9}},
	{0xFE,1,{0xF8}},
	{0xFF,1,{0x0F}},
/*< DTS2017010800689 xingbin/xwx427571 20170113 begin */
/*< DTS2017021603061 xingbin/xwx427571 20170216 begin */
	/*ce setting begin*/
	{0x83,1,{0xCC}},
	{0x84,1,{0x33}},
	{0x90,1,{0x17}},
	{0x92,1,{0x3f}},
	{0x93,1,{0x1A}},
	{0x94,1,{0x25}},
	{0x95,1,{0x1E}},
	{0x96,1,{0x25}},
	{0x97,1,{0x19}},
	{0x98,1,{0x94}},
	/*ce setting end*/
/*< DTS2017021603061 xingbin/xwx427571 20170216 end */
/*< DTS2017010800689 xingbin/xwx427571 20170113 end */

	/*< DTS2017021603061 xingbin/xwx427571 20170216 begin */
	/*cabc setting begin*/
	{0x83,1,{0xBB}},
	{0x84,1,{0x22}},
	{0x94,1,{0x78}},//PWM freq set 18~23k
	{0x9A,1,{0x10}},

	{0x90,1,{0x00}}, //NOTE: should set to 0x40 to stillmode ,0xc0 to off mode, 0x80 to uimode
	{0xA1,1,{0xFF}},
	{0xA2,1,{0xFE}},
	{0xA3,1,{0xFD}},
	{0xA4,1,{0xF9}},
	{0xA5,1,{0xF5}},
	{0xA6,1,{0xF1}},
	{0xA7,1,{0xED}},
	{0xA8,1,{0xEB}},
	{0xA9,1,{0xE9}},
	{0xAA,1,{0xE6}},
	{0xB4,1,{0x0D}},
	{0xB5,1,{0x1A}},
	{0xB6,1,{0x16}},
	{0xAD,1,{0x06}},
	{0x9B,1,{0x00}},
	{0x96,1,{0x00}},
	/*cabc setting end*/
	/*< DTS2017010800689 xingbin/xwx427571 20170113 end */
};
/*< DTS2017012302027 xingbin/xwx427571 20170207 end */
/*< DTS2017012005870 xingbin/xwx427571 20170120 end */
/*< DTS2017010800689 xingbin/xwx427571 20170111 end */
/*< DTS2016123002286 xingbin/xwx427571 20170108 end */
/* A1911-87 fanpei 2016-12-29 begin */
static void lcm_gpio_init(void)
{
	static struct device_node *node;
	int err = -1;

	node = of_find_compatible_node(NULL, NULL, "mediatek,lcm");


	GPIO_LCD_PWR_EN = of_get_named_gpio(node, "lcm_power_gpio", 0);
	GPIO_LCD_RST = of_get_named_gpio(node, "lcm_reset_gpio", 0);
	GPIO_LCD_BL_EN = of_get_named_gpio(node, "lcm_bl_en_gpio", 0);
/* A1911-87 litao2 2016-11-23 begin */
	GPIO_LCD_RST2 = of_get_named_gpio(node, "lcm_reset2_gpio", 0);
/* A1911-87 litao2 2016-11-23 end */
	err = gpio_request(GPIO_LCD_PWR_EN,"GPIO_LCD_PWR_EN");
	if(err < 0)
	{
		printk(KERN_ERR "%s: gpio %d request, err=%d", __func__, GPIO_LCD_PWR_EN,err);
		return;
	}

	err = gpio_request(GPIO_LCD_RST,"GPIO_LCD_RST");
	if(err < 0)
	{
		printk(KERN_ERR "%s: gpio %d request, err=%d", __func__, GPIO_LCD_RST,err);
		return;
	}

	err = gpio_request(GPIO_LCD_BL_EN,"GPIO_LCD_BL_EN");
	if(err < 0)
	{
		printk(KERN_ERR "%s: gpio %d request, err=%d", __func__, GPIO_LCD_BL_EN,err);
		return;
	}

/* A1911-87 litao2 2016-11-23 begin */
	err = gpio_request(GPIO_LCD_RST2,"GPIO_LCD_RST2");
	if(err < 0)
	{
		printk(KERN_ERR "%s: gpio %d request, err=%d", __func__, GPIO_LCD_RST2,err);
		return;
	}
/* A1911-87 litao2 2016-11-23 end */
}
/* A1911-87 fanpei 2016-12-29 end */


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY :
				if (table[i].count <= 10)
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
	memset(params, 0, sizeof(LCM_PARAMS));

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
	params->dsi.packet_size=256;
	//video mode timing
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.word_count=FRAME_WIDTH*3;

	/*< DTS2017012204145 xingbin/xwx427571 20170122 begin */
	params->dsi.vertical_sync_active                = KING_VER_SYNC_ACTIVE;
	params->dsi.vertical_backporch                  = KING_VER_BACKPROCH;
	params->dsi.vertical_frontporch                 = KING_VER_FRONTPROCH;//for 600*1024 :300;for 800*1280:8
	params->dsi.vertical_active_line                    = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active              = KING_HOR_SYNC_ACTIVE;
	params->dsi.horizontal_backporch                = KING_HOR_BACKPROCH;
	params->dsi.horizontal_frontporch               = KING_HOR_FRONTPROCH;
	params->dsi.horizontal_active_pixel             = FRAME_WIDTH;
	params->dsi.ssc_disable                         = 1;
	params->dsi.cont_clock  = 1;
	params->dsi.PLL_CLOCK = KING_DSI_MIPI_PLL_CLOCK;
	params->dsi.HS_TRAIL = 1;
	/*< DTS2017012204145 xingbin/xwx427571 20170122 end */

}

static void lcm_set_gpio_output(unsigned int gpio, unsigned int output)
{
	gpio_direction_output(gpio, output);
	gpio_set_value(gpio, output);
}

/*< DTS2017012302027 xingbin/xwx427571 20170207 begin */
static void lcm_init_power(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_init_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ONE);
	MDELAY(DEALY_FOR_VDD_VBAT);
	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
#else
	printk("[Kernel/LCM] lcm_init_power() enter\n");

#endif

}
/*< DTS2017012302027 xingbin/xwx427571 20170207 end */
/*< DTS2017012302027 xingbin/xwx427571 20170207 begin */
static void lcm_suspend_power(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_suspend_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ZERO);
	MDELAY(DEALY_FOR_VDD_VBAT);
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ZERO);
#else
	printk("[Kernel/LCM] lcm_suspend_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_BL_EN,GPIO_OUT_ZERO);
	MDELAY(DEALY_FOR_VDD_VBAT);
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ZERO);
	/*< DTS2017021706071  xingbin/xwx427571 20170217 begin */
	MDELAY(DEALY_FOR_VDD_OFF);
	/* DTS2017021706071  xingbin/xwx427571 20170217 end > */

#endif

}
/*< DTS2017012302027 xingbin/xwx427571 20170207 end */
/*< DTS2017012302027 xingbin/xwx427571 20170207 begin */
static void lcm_resume_power(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_resume_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ONE);
	MDELAY(DEALY_FOR_VDD_VBAT);
	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
#else
	printk("[Kernel/LCM] lcm_resume_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ONE);
	/*< DTS2016122915506 xingbin/xwx427571 20170117 begin */
	MDELAY(DEALY_FOR_VDD_VBAT);
	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
	/*< DTS2016122915506 xingbin/xwx427571 20170117 end */
#endif

}
/*< DTS2017012302027 xingbin/xwx427571 20170207 end */
/*< DTS2017011300079 xingbin/xwx427571 20170111 begin */
static void lcm_write_mipi_reg(u32 mipi_reg,u32 reg_val)
{
	printk("%s, kernel lcm_write_mipi_reg: mipi_reg = 0x%x,reg_val = 0x%x\n", __func__, mipi_reg,reg_val);
	if(mipi_reg > 0xFF||mipi_reg < 0x00){
		pr_err("%s invild mipi reg\n",__func__);
		return;
	}
	if(reg_val > 0xFF||reg_val < 0x00){
		pr_err("%s invild mipi reg val\n",__func__);
		return;
	}
	lcm_mipi_reg_setting[0].cmd		= mipi_reg;
	lcm_mipi_reg_setting[0].para_list[0]	= reg_val;
	push_table(lcm_mipi_reg_setting,
	sizeof(lcm_mipi_reg_setting) / sizeof(struct LCM_setting_table), 1);
}
/*< DTS2017011300079 xingbin/xwx427571 20170111 end */
/*< DTS2017012302027 xingbin/xwx427571 20170207 begin */
extern void DSI_clk_HS_mode(bool enter);

static void lcm_init(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_init() enter\n");
	MDELAY(DEALY_FOR_VBAT_RST);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	lcm_set_gpio_output(GPIO_LCD_RST2, GPIO_OUT_ZERO);
	DSI_clk_HS_mode(1);
	MDELAY(DEALY_FOR_RST_INIT);
	// when phone initial , config output high, enable backlight drv chip
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	printf("[LK/LCM] lcm_init() end\n");
#else
	printk("[Kernel/LCM] lcm_init() enter\n");
#endif
}
/*< DTS2017012302027 xingbin/xwx427571 20170207 end */
/*< DTS2017012601939 xingbin/xwx427571 20170217 begin */
static void lcm_early_suspend(void)
{

	printk("[Kernel/LCM] KING lcm_early_suspend() enter\n");
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(DEALY_FOR_RST_INIT);
}
/*< DTS2017012601939 xingbin/xwx427571 20170217 end */
/*< DTS2017012302027 xingbin/xwx427571 20170207 begin */
static void lcm_suspend(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_suspend() enter\n");
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(DEALY_FOR_RST_INIT);
	lcm_set_gpio_output(GPIO_LCD_RST2, GPIO_OUT_ZERO);
	MDELAY(DEALY_FOR_VBAT_RST);
#else
	printk("[Kernel/LCM] lcm_suspend() enter\n");
	/*< DTS2017012601939 xingbin/xwx427571 20170217 begin */
	//push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	//MDELAY(DEALY_FOR_RST_INIT);
	/*< DTS2017012601939 xingbin/xwx427571 20170217 end */
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
/* A1911-87 litao2 2016-11-23 begin */
	lcm_set_gpio_output(GPIO_LCD_RST2, GPIO_OUT_ONE);
/* A1911-87 litao2 2016-11-23 end */
	MDELAY(DEALY_FOR_VBAT_RST);
	printk("[Kernel/LCM] lcm_suspend() end\n");
#endif
}
/*< DTS2017012302027 xingbin/xwx427571 20170207 end */
/*< DTS2017012302027 xingbin/xwx427571 20170207 begin */
static void lcm_resume(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_resume() enter\n");

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(10);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(10);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(10);

	// when phone initial , config output high, enable backlight drv chip
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

	MDELAY(10);

#else
	printk("[Kernel/LCM] lcm_resume() enter\n");

	MDELAY(DEALY_FOR_VBAT_RST);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
/* A1911-87 litao2 2016-11-23 begin */
	lcm_set_gpio_output(GPIO_LCD_RST2, GPIO_OUT_ZERO);
/* A1911-87 litao2 2016-11-23 end */
	DSI_clk_HS_mode(1);
	MDELAY(DEALY_FOR_RST_INIT);

	// when phone initial , config output high, enable backlight drv chip
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

	printk("[Kernel/LCM] lcm_resume() end\n");
#endif
}
/*< DTS2017012302027 xingbin/xwx427571 20170207 end */
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{

}
#endif

#if 0
static unsigned int lcm_compare_id(void)
{

}
#endif
/* A1911-87 fanpei 2016-12-29 begin */
/* A1911-67 litao2 20161121 begin */
/*< DTS2017012302027 xingbin/xwx427571 20170207 begin */
/*< DTS2017021306702 xingbin/xwx427571 20170213 begin */
static void lcm_setbacklight(unsigned int level)
{
	static int level_last = LCD_BRIGHTNESS_DACK;
#ifdef BUILD_LK
	dprintf(0, "KING %s,lk nt51021 backlight: level = %d\n", __func__, level);
#else
	pr_err("KING %s, kernel nt51021 backlight: level = %d\n", __func__, level);
#endif

	if((level<=RT9293_MIN_BRIGHTNESS_VALUE)&&(level!=LCD_BRIGHTNESS_DACK))
		level = RT9293_MIN_BRIGHTNESS_VALUE;

	/*Refresh value of backlight level.*/
	lcm_backlight_level_setting[2].para_list[0] = level;

	push_table(lcm_backlight_level_setting,
	sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
	level_last = level;
}
/*< DTS2017021306702 xingbin/xwx427571 20170213 end */
/*< DTS2017012302027 xingbin/xwx427571 20170207 end */
/* A1911-67 litao2 20161121 end */
/* A1911-87 fanpei 2016-12-29 end */
LCM_DRIVER nt51021_wsvga_dsi_vdo_king_lcm_drv= {
	.name               = "nt51021_wsvga_dsi_vdo_king",
	.set_util_funcs     = lcm_set_util_funcs,
	.get_params         = lcm_get_params,
	.init                   = lcm_init,
	.suspend            = lcm_suspend,
	.resume             = lcm_resume,
	//.compare_id      = lcm_compare_id,
	.init_power        = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.gpio_init = lcm_gpio_init,
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
/* A1911-67 litao2 20161121 begin */
	.set_backlight = lcm_setbacklight,
/* A1911-67 litao2 20161121 end */
/*< DTS2017012601939 xingbin/xwx427571 20170217 begin */
	.early_suspend =	lcm_early_suspend,
/*< DTS2017012601939 xingbin/xwx427571 20170217 end */
/*< DTS2017011300079 xingbin/xwx427571 20170111 begin */
	.set_lcd_reg		= lcm_write_mipi_reg,
/*< DTS2017011300079 xingbin/xwx427571 20170111 end */
};
