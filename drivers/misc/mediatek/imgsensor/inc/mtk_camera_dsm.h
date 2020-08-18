/*< DTS2016052708980 zhangyumin zwx354019 20160527 begin */
/* <DTS2015102008509 z00285045 20151021 begin */
/* < DTS2014081209165 Zhangbo 00166742 20140812 begin */
/* < DTS2014111105636 Houzhipeng hwx231787 20141111 begin */
#ifndef __MTK_CAMERA_DSM_H__
#define __MTK_CAMERA_DSM_H__

#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#define MTK_CAMERA_DSM_BUFFER_SIZE 1024
extern int camera_is_closing;
/* < DTS2014121000600 Houzhipeng hwx231787 20141210 begin */
extern int camera_is_in_probe;
/* DTS2014121000600 Houzhipeng hwx231787 20141210 end > */
extern char camera_dsm_log_buff[MTK_CAMERA_DSM_BUFFER_SIZE];
extern struct dsm_client *camera_dsm_client;

extern struct dsm_client* camera_dsm_get_client(void);
extern int camera_report_dsm_err( int type, int err_num , const char* str);
#endif

#endif
/* DTS2014111105636 Houzhipeng hwx231787 20141111 end > */
/* DTS2014081209165 Zhangbo 00166742 20140812 end > */
/* DTS2015102008509 z00285045 20151021 end> */
/* DTS2016052708980 zhangyumin zwx354019 20160527 end >*/
