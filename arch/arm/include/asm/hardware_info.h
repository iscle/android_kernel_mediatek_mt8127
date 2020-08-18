/* Copyright (c) 2012-2012, OEM Telecom. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#ifndef HARDWARE_INFO_H
#define HARDWARE_INFO_H
/* DTS2016122706947 ywx422261 20161227 begin */
#define HW_INFO_MAX_LEN 125
/* DTS2016122706947 ywx422261 20161227 end */
typedef enum {
	LCM = 0,
	CTP,
	MAIN_CAM,
	SUB_CAM,
	FLASH,
	G_SENSOR,/*ACCELEROMETER*/
	M_SENSOR,/*MAGNETIC_FIELD*/
	ALS_PS,/*LIGHT and PROXIMITY*/
	GYRO,/*GYROSCOPE*/
	SAR,
	HALL,
	EMCP,
	ROM, /* ROM INFO */
	RAM, /* RAM INFO */
	NFC, /* NFC INFO */
	WIFI,/*WIFI INFO*/
	BLUETOOTH,/*BLUETOOTH INFO*/
    /*DTS2016122104616 liuguangchao/lwx422269  20161229 begin >*/
	#ifdef CONFIG_MTK_MULTI_BAT_PROFILE_SUPPORT
	BATTERY_TYPE,
	#endif
    /*DTS2016122104616 liuguangchao/lwx422269  20161229 end >*/
	BOARD_ID,
/* DTS2016122706947 ywx422261 20161227 begin */
	GPS,
/* DTS2016122706947 ywx422261 20161227 end */
/* DTS2016122500642 ywx422261 20161227 begin */
	HARDWARE_VER,
/* DTS2016122500642 ywx422261 20161227 end */
	HARDWARE_ID_MAX
} HARDWARE_ID;
struct hardware_info_pdata{
	char lcm[125];
	char ctp[125];
	char main_cam[125];
	char sub_cam[125];
	char flash[125];
	char gsensor[125];
	char msensor[125];
	char als_ps[125];
	char gyro[125];
	char sar[125];
	char hall[125];
	char emcp[125];
	char rom[125];
	char ram[125];
	char nfc[125];
	char wifi[125];
	char bluetooth[125];
    /*DTS2016122104616 liuguangchao/lwx422269  20161229 begin >*/
	#ifdef CONFIG_MTK_MULTI_BAT_PROFILE_SUPPORT
	char battery_type[125];
	#endif
    /*DTS2016122104616 liuguangchao/lwx422269  20161229 end >*/
	char board_id[125];
/* DTS2016121309782 lwx422271 20161213 begin */
	char gps[HW_INFO_MAX_LEN];
/* DTS2016121309782 lwx422271 20161213 end */
/*DTS2016122500642 ywx422261 20161227 begin */
	char hardware_ver[HW_INFO_MAX_LEN];
/*DTS2016122500642 ywx422261 20161227 end */
};
int register_hardware_info(HARDWARE_ID id, char* name);

#endif /* HARDWARE_INFO_H */

