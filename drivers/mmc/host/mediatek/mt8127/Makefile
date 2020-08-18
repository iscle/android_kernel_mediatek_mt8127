#add kernel source code path as head file search path
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)
#workaround for include mmc/card/queue.h
ccflags-y += -I$(srctree)/drivers/mmc/card

obj-$(CONFIG_MMC_MTK) += mt8127-sd.o
ifneq ($(CONFIG_MTK_GPT_SCHEME_SUPPORT),y)
obj-$(CONFIG_MMC_MTK) += mt_sd_misc.o
obj-$(CONFIG_MTK_EMMC_SUPPORT) += dumchar.o
obj-$(CONFIG_MTK_EMMC_SUPPORT) += pmt.o
endif
obj-$(CONFIG_MTK_EMMC_SUPPORT) += sysenv.o
