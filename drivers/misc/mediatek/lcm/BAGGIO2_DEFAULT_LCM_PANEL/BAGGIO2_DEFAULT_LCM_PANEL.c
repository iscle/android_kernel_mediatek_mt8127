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


//static unsigned char lcd_id_pins_value = 0xFF;
static const unsigned char LCD_MODULE_ID = 0x01; //  haobing modified 2013.07.11
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE 0
#define FRAME_WIDTH                                         (600)
#define FRAME_HEIGHT                                        (1024)

#define REGFLAG_DELAY                                           0xFFFC
#define REGFLAG_END_OF_TABLE                                0xFFFB   // END OF REGISTERS MARKER

#define	LCM_STATE_ERR                   -1
#define	LCM_STATE_ZERO                  0
#define	LCM_PACKET_SIZE                 256
#define	WORD_COUNT_WIDTH                3
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

#define DEFAULT_VER_SYNC_ACTIVE		1
#define DEFAULT_VER_BACKPROCH		98
#define DEFAULT_VER_FRONTPROCH		198

#define DEFAULT_HOR_SYNC_ACTIVE		1
#define DEFAULT_HOR_BACKPROCH		25
#define DEFAULT_HOR_FRONTPROCH		154
#define DEFAULT_DSI_MIPI_PLL_CLOCK		185
#define DEFAULT_DSI_MIPI_CONT_CLOCK		1
#define DEFAULT_DSI_MIPI_HS_TRAIL		1
#define DEFAULT_DSI_MIPI_SSC_DISABLE	1
static void lcm_gpio_init(void)
{
	static struct device_node *node;
	int err = LCM_STATE_ERR;

	node = of_find_compatible_node(NULL, NULL, "mediatek,lcm");

	GPIO_LCD_PWR_EN = of_get_named_gpio(node, "lcm_power_gpio", 0);
	GPIO_LCD_RST = of_get_named_gpio(node, "lcm_reset_gpio", 0);
	GPIO_LCD_BL_EN = of_get_named_gpio(node, "lcm_bl_en_gpio", 0);
	GPIO_LCD_RST2 = of_get_named_gpio(node, "lcm_reset2_gpio", 0);
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


	params->dsi.vertical_sync_active                = DEFAULT_VER_SYNC_ACTIVE;
	params->dsi.vertical_backporch                  = DEFAULT_VER_BACKPROCH;
	params->dsi.vertical_frontporch                 = DEFAULT_VER_FRONTPROCH;//for 600*1024 :300;for 800*1280:8
	params->dsi.vertical_active_line                    = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active              = DEFAULT_HOR_SYNC_ACTIVE;
	params->dsi.horizontal_backporch                = DEFAULT_HOR_BACKPROCH;
	params->dsi.horizontal_frontporch               = DEFAULT_HOR_FRONTPROCH;
	params->dsi.horizontal_active_pixel             = FRAME_WIDTH;
	params->dsi.ssc_disable                         = DEFAULT_DSI_MIPI_SSC_DISABLE;
	params->dsi.cont_clock  = DEFAULT_DSI_MIPI_CONT_CLOCK;
	params->dsi.PLL_CLOCK = DEFAULT_DSI_MIPI_PLL_CLOCK;
	params->dsi.HS_TRAIL = DEFAULT_DSI_MIPI_HS_TRAIL;

}

LCM_DRIVER baggio2_default_panel_lcm_drv= {
	.name               = "baggio2_default_lcm_panel",
	.set_util_funcs     = lcm_set_util_funcs,
	.get_params         = lcm_get_params,
	.gpio_init = lcm_gpio_init,
};
