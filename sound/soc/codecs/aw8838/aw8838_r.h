#ifndef _AW8838_R_H_
#define _AW8838_R_H_


/*
 * i2c transaction on Linux limited to 64k
 * (See Linux kernel documentation: Documentation/i2c/writing-clients)
*/
#define MAX_I2C_BUFFER_SIZE 65536
#define AW_KERNEL_VER_OVER_4_19_1
#define aw8838_r_FLAG_START_ON_MUTE   (1 << 0)
#define aw8838_r_FLAG_SKIP_INTERRUPTS     (1 << 1)
#define aw8838_r_FLAG_SAAM_AVAILABLE      (1 << 2)
#define aw8838_r_FLAG_STEREO_DEVICE       (1 << 3)
#define aw8838_r_FLAG_MULTI_MIC_INPUTS    (1 << 4)

#define aw8838_r_NUM_RATES                9

#define aw8838_r_MAX_REGISTER             0xff


enum aw8838_r_chipid{
    aw8838_r_ID,
};

enum aw8838_r_mode_spk_rcv{
    aw8838_r_SPEAKER_MODE = 0,
    aw8838_r_RECEIVER_MODE = 1,
};

#ifdef AW_KERNEL_VER_OVER_4_19_1
typedef struct snd_soc_component aw_snd_soc_codec_t;
typedef struct snd_soc_component_driver aw_snd_soc_codec_driver_t;
#else
typedef struct snd_soc_codec aw_snd_soc_codec_t;
typedef struct snd_soc_codec_driver aw_snd_soc_codec_driver_t;
#endif

struct aw8838_r {
    struct regmap *regmap;
    struct i2c_client *i2c;
    aw_snd_soc_codec_t *codec;
    struct device *dev;
    struct mutex cfg_lock;
#ifdef aw8838_r_VBAT_MONITOR
    struct hrtimer vbat_monitor_timer;
    struct work_struct vbat_monitor_work;
#endif
    int sysclk;
    int rate;
    int pstream;
    int cstream;

    int reset_gpio;
    int irq_gpio;

#ifdef CONFIG_DEBUG_FS
    struct dentry *dbg_dir;
#endif
    u8 reg;

    unsigned int flags;
    unsigned int chipid;
    unsigned int init;
    unsigned int spk_rcv_mode;

};



struct aw_componet_codec_ops {
	aw_snd_soc_codec_t *(*kcontrol_codec)(struct snd_kcontrol *kcontrol);
	void *(*codec_get_drvdata)(aw_snd_soc_codec_t *codec);
	int (*add_codec_controls)(aw_snd_soc_codec_t *codec,
		const struct snd_kcontrol_new *controls, unsigned int num_controls);
	void (*unregister_codec)(struct device *dev);
	int (*register_codec)(struct device *dev,
			const aw_snd_soc_codec_driver_t *codec_drv,
			struct snd_soc_dai_driver *dai_drv,
			int num_dai);
};

struct aw8838_r_container{
    int len;
    unsigned char data[];
};


#endif
