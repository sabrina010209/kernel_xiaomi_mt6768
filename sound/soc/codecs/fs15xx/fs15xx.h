/**
 * Copyright (C) Fourier Semiconductor Inc. 2016-2020. All rights reserved.
 * 2020-04-29 File created.
 */

#ifndef __FS15XX_H__
#define __FS15XX_H__

#include <sound/core.h>
#include <sound/soc.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/version.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/power_supply.h>

#define FS15XX_PINCTRL_SUPPORT
#define FSM_CHECK_PLUSE_TIME
// #define FS15XX_USE_HRTIMER
#define FS15XX_ID_DETECT_SUPPORT
#define FS15XX_ADP_BOOST_SUPPORT

#define FS15XX_GPIO_LOW     0
#define FS15XX_GPIO_HIGH    1
#define FS15XX_OFF_MODE     0
#define FS15XX_MIN_MODE     0
#define FS15XX_MAX_MODE     6
#define SPC1910_DEFAULT_MODE (4)
#define FS15XX_DEFAULT_MODE  (4)
#define FS15XX_DRV_VERSION  "V1.1.3-a"
#define FS15XX_DRV_NAME     "fs15xx"
#define AW87XX_DEFAULT_MODE  (7)
#define AW87XX_DEV_NAME     "aw87359CSR"

#define SPC1910_HDR_PULSES    (15)
#define FS15XX_PULSE_DELAY_US (10)
#define FS15XX_RETRY          (10)
#define FS15XX_MONITOR_PEROID (500) // ms

#define FS15XX_IOC_MAGIC      (0x7D)
#define FS15XX_IOC_GET_VBAT       _IOR(FS15XX_IOC_MAGIC, 1, int)

enum fs15xx_dev_type {
	FS15XX_DEV_1910 = 0,
	FS15XX_DEV_1515,
	FS15XX_DEV_MAX
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
#define snd_soc_codec              snd_soc_component
#define snd_soc_add_codec_controls snd_soc_add_component_controls
#endif

struct aw87xx_dev_t {
	int aw_id_gpio;
	int aw_cmd_gpio;
	int aw87xx_mode;
	spinlock_t aw87xx_lock;
};

struct fs15xx_dev {
#ifdef FS15XX_USE_HRTIMER
	struct hrtimer timer;
	struct work_struct monitor_work;
	atomic_t running;
#endif
	spinlock_t fs15xx_lock;
	ktime_t last_time;
	ktime_t cur_time;
	bool fs15xx_timeout;
	int dev_type;
	int fs15xx_mode;
#ifdef FS15XX_PINCTRL_SUPPORT
	struct pinctrl *pinctrl;
	struct pinctrl_state *fs15xx_id_default;
	struct pinctrl_state *fs15xx_id_active;
	struct pinctrl_state *fs15xx_cmd_default;
	struct pinctrl_state *fs15xx_mod_default;
#endif
	u32 gpio_id;
	u32 gpio_cmd;
	u32 gpio_mod;
};
typedef struct fs15xx_dev fs15xx_dev_t;

int fs15xx_set_timer(bool enable);
int fs15xx_ext_amp_set(int enable);
int ext_amp_poweron(int enable);
int fs15xx_set_mode_simple(int mode);
int spc19xx_set_mode_simple(int mode);
int fs15xx_set_ext_amp(struct snd_soc_codec *codec, int enable);
void fs15xx_add_card_kcontrol(struct snd_soc_card *card);
void fs15xx_add_codec_kcontrol(struct snd_soc_codec *codec);

#endif
