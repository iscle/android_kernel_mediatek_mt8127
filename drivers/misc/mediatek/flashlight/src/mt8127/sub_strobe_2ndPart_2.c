#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */
#define TAG_NAME "leds_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug("%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_debug("%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_debug("%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_debug("%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug("<%s>\n", __func__);
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_debug("%s: " fmt, __func__ , ##arg)


#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#define PK_VER(a, ...)
#define PK_ERR(a, ...)
#endif


static int strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	PK_DBG("sub dummy ioctl");
	return 0;
}


static int strobe_open(void *pArg)
{
	PK_DBG("sub dummy open");
	return 0;
}

static int strobe_release(void *pArg)
{
	PK_DBG("sub dummy release");
	return 0;
}

static FLASHLIGHT_FUNCTION_STRUCT strobeFunc = {
	strobe_open,
	strobe_release,
	strobe_ioctl
};

MUINT32 subStrobeInit_2ndPart_2(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL) {
		*pfFunc = &strobeFunc;
	}
	return 0;
}
