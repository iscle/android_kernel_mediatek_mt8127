/*
 * Copyright (C) huawei company
 *
 * This    program    is free    software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public    License    version    2 as
 * published by    the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/of.h>
#include <hw_fac_info.h>
#include <asm/hardware_info.h>

static const void* dt_get_property_for_fac (char* key, int* pvalue_len)
{
	struct device_node *dp = NULL;
	if((NULL == key)||(NULL == pvalue_len))
	{
		fac_log_err(KERN_ERR "param is NULL!\n");
		return NULL;
	}
	dp = of_find_node_by_path("/huawei_fac_info");
	if(!of_device_is_available(dp))
	{
		fac_log_err(KERN_ERR "device is not available!\n");
		return NULL;
	}
	return of_get_property(dp, key, pvalue_len);
}
static int set_hardware_ver(void)
{
	int hardware_ver_len = 0;
	char *hardware_ver = NULL;

	hardware_ver = (char *)dt_get_property_for_fac("fac,hardware_ver",&hardware_ver_len);
	if(NULL == hardware_ver)
	{
		fac_log_err(KERN_ERR " get hardware version fail!\n");
		return FAC_HW_INFO_ERROR;
	}
	register_hardware_info(HARDWARE_VER,hardware_ver);
	return FAC_HW_INFO_SUCESS;
}
static int set_board_id(void)
{
	int board_id_len = 0;
	char* board_id = NULL;

	board_id = (char *)dt_get_property_for_fac("fac,board_id" ,&board_id_len);
	if(NULL == board_id)
	{
		fac_log_err(KERN_ERR " get board id fail!\n");
		return FAC_HW_INFO_ERROR;
	}
	register_hardware_info(BOARD_ID, board_id);
	return FAC_HW_INFO_SUCESS;
}
static int  fac_info_probe(struct platform_device *pdev)
{
	int ret = 0;
	ret = set_hardware_ver();
	if(FAC_HW_INFO_SUCESS != ret)
	{
		fac_log_err("%s:set hardware version fail!\n", __func__);
	}
	ret = set_board_id();
	if(FAC_HW_INFO_SUCESS != ret)
	{
		fac_log_err("%s:set borad id fail!\n", __func__);
	}
	return ret;
}
static struct of_device_id fac_info_match_table[] = {
	{ .compatible = "huawei,hw_fac_info",},
	{ },
};

static struct platform_driver fac_info_driver = {
	.driver = {
		.name  = "hw_fac_info",
		.owner  = THIS_MODULE,
		.of_match_table = fac_info_match_table,
	},
	.probe = fac_info_probe,
	.remove = NULL,
};

static int __init  fac_info_init(void)
{
	return platform_driver_register(&fac_info_driver);
}
static void __exit fac_info_exit(void)
{
	platform_driver_unregister(&fac_info_driver);
}

module_init(fac_info_init);
module_exit(fac_info_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("for getting Factory Info");
MODULE_AUTHOR("zhaorenjie <zhaorenjie@huawei.com>");
