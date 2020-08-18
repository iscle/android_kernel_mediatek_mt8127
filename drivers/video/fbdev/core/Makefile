obj-y                             += fb_notify.o
obj-$(CONFIG_FB_CMDLINE)          += fb_cmdline.o
obj-$(CONFIG_FB)                  += fb.o
fb-y                              := fbmem.o fbmon.o fbcmap.o fbsysfs.o \
                                     modedb.o fbcvt.o
fb-objs                           := $(fb-y)

obj-$(CONFIG_FB_CFB_FILLRECT)  += cfbfillrect.o
obj-$(CONFIG_FB_CFB_COPYAREA)  += cfbcopyarea.o
obj-$(CONFIG_FB_CFB_IMAGEBLIT) += cfbimgblt.o
obj-$(CONFIG_FB_SYS_FILLRECT)  += sysfillrect.o
obj-$(CONFIG_FB_SYS_COPYAREA)  += syscopyarea.o
obj-$(CONFIG_FB_SYS_IMAGEBLIT) += sysimgblt.o
obj-$(CONFIG_FB_SYS_FOPS)      += fb_sys_fops.o
obj-$(CONFIG_FB_SVGALIB)       += svgalib.o
obj-$(CONFIG_FB_DDC)           += fb_ddc.o
obj-$(CONFIG_FB_DEFERRED_IO)   += fb_defio.o
#DTS2017012601939 xingbin/xwx427571 20170217 begin
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
#DTS2017012601939 xingbin/xwx427571 20170217 end