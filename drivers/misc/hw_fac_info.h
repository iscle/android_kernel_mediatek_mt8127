/*=============================================================================

FILE: hw_fac_info.h

DESCRIPTION:
         This file defines log print interface by driver group's and Android-Log-v2 regulation
         of HUAWEI
DateTime: 2014/07/16
Author:  wanglili
        Copyright 2014 HUAWEI.
        All Rights Reserved.
=============================================================================*/
#ifndef __HW_FAC_INFO_H__
#define __HW_FAC_INFO_H__

#include <linux/printk.h>
#include <linux/of.h>
#include <linux/string.h>

#define FAC_DBG  1
#define FAC_INFO 2
#define FAC_ERR  3
#define HW_FAC_INTO_DEBUG_MASK FAC_DBG
#define HW_FAC_TAG  "[HW_FAC_INFO] "

#define FAC_HW_INFO_ERROR   -1
#define FAC_HW_INFO_SUCESS 0

/*ERROR*/
#ifndef fac_log_err
#define fac_log_err(x...) \
do{ \
	if( HW_FAC_INTO_DEBUG_MASK >= FAC_ERR ) \
	{ \
		printk(KERN_ERR HW_FAC_TAG x); \
} \
 \
}while(0)
#endif

/*INFO*/
#ifndef fac_log_info
#define fac_log_info(x...) \
do{ \
	if(HW_FAC_INTO_DEBUG_MASK >= FAC_INFO) \
	{ \
		printk(KERN_ERR HW_FAC_TAG x); \
	} \
 \
}while(0)
#endif

/*DEBUG*/
#ifndef fac_log_debug
#define fac_log_debug(x...) \
do{ \
	if( HW_FAC_INTO_DEBUG_MASK >= FAC_DBG ) \
	{ \
		printk(KERN_ERR HW_FAC_TAG x); \
	} \
 \
}while(0)
#endif

#endif/*__HW_FAC_INFO_H__*/
