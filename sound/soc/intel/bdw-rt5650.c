/*
 * ASoC machine driver for Intel Broadwell platforms with RT5650 codec
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/jack.h>

#include "sst-dsp.h"
#include "sst-haswell-ipc.h"

#include "../codecs/rt5645.h"

struct bdw_rt5650_priv {
	struct gpio_desc *gpio_hp_en;
	struct snd_soc_codec *codec;
};

static const struct snd_soc_dapm_widget bdw_rt5650_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("DMIC Pair1", NULL),
	SND_SOC_DAPM_MIC("DMIC Pair2", NULL),
};

static const struct snd_soc_dapm_route bdw_rt5650_map[] = {
	/* Speakers */
	{"Speaker", NULL, "SPOL"},
	{"Speaker", NULL, "SPOR"},

	/* Headset jack connectors */
	{"Headphone", NULL, "HPOL"},
	{"Headphone", NULL, "HPOR"},
	{"IN1P", NULL, "Headset Mic"},
	{"IN1N", NULL, "Headset Mic"},

	/* Digital MICs
	 * DMIC Pair1 are the two DMICs connected on the DMICN1 connector.
	 * DMIC Pair2 are the two DMICs connected on the DMICN2 connector.
	 * Facing the camera, DMIC Pair1 are on the left side, DMIC Pair2
	 * are on the right side.
	 */
	{"DMIC L1", NULL, "DMIC Pair1"},
	{"DMIC R1", NULL, "DMIC Pair1"},
	{"DMIC L2", NULL, "DMIC Pair2"},
	{"DMIC R2", NULL, "DMIC Pair2"},

	/* CODEC BE connections */
	{"SSP0 CODEC IN", NULL, "AIF1 Capture"},
	{"AIF1 Playback", NULL, "SSP0 CODEC OUT"},
};

static const struct snd_kcontrol_new bdw_rt5650_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("DMIC Pair1"),
	SOC_DAPM_PIN_SWITCH("DMIC Pair2"),
};


static struct snd_soc_jack headphone_jack;
static struct snd_soc_jack mic_jack;
static struct snd_soc_jack headset_btn;

static int broadwell_ssp0_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	/* The ADSP will covert the FE rate to 48k, max 4-channels */
	rate->min = rate->max = 48000;
	channels->min = 2;
	channels->max = 4;

	/* set SSP0 to 24 bit */
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
				    SNDRV_PCM_HW_PARAM_FIRST_MASK],
				    SNDRV_PCM_FORMAT_S24_LE);
	return 0;
}

static int bdw_rt5650_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	/* The actual MCLK freq is 24MHz. The codec is told that MCLK is
	 * 24.576MHz to satisfy the requirement of rl6231_get_clk_info.
	 * ASRC is enabled on AD and DA filters to ensure good audio quality.
	 */
	ret = snd_soc_dai_set_sysclk(codec_dai, RT5645_SCLK_S_MCLK, 24576000,
		SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec sysclk configuration\n");
		return ret;
	}

	return ret;
}

static struct snd_soc_ops bdw_rt5650_ops = {
	.hw_params = bdw_rt5650_hw_params,
};

static int bdw_rt5650_rtd_init(struct snd_soc_pcm_runtime *rtd)
{
	struct sst_pdata *pdata = dev_get_platdata(rtd->platform->dev);
	struct sst_hsw *broadwell = pdata->dsp;
	int ret;

	/* Set ADSP SSP port settings
	 * clock_divider = 4 means BCLK = MCLK/5 = 24MHz/5 = 4.8MHz
	 */
	ret = sst_hsw_device_set_config(broadwell, SST_HSW_DEVICE_SSP_0,
		SST_HSW_DEVICE_MCLK_FREQ_24_MHZ,
		SST_HSW_DEVICE_TDM_CLOCK_MASTER, 4);
	if (ret < 0) {
		dev_err(rtd->dev, "error: failed to set device config\n");
		return ret;
	}

	return 0;
}

static int bdw_rt5650_init(struct snd_soc_pcm_runtime *rtd)
{
	struct bdw_rt5650_priv *bdw_rt5650 =
			snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	/* Enable codec ASRC function for Stereo DAC/Stereo1 ADC/DMIC/I2S1.
	 * The ASRC clock source is clk_i2s1_asrc.
	 */
	rt5645_sel_asrc_clk_src(codec,
				RT5645_DA_STEREO_FILTER |
				RT5645_DA_MONO_L_FILTER |
				RT5645_DA_MONO_R_FILTER |
				RT5645_AD_STEREO_FILTER |
				RT5645_AD_MONO_L_FILTER |
				RT5645_AD_MONO_R_FILTER,
				RT5645_CLK_SEL_I2S1_ASRC);

	/* TDM 4 slots 24 bit, set Rx & Tx bitmask to 4 active slots */
	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0xF, 0xF, 4, 24);

	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec TDM slot %d\n", ret);
		return ret;
	}

	/* Create and initialize headphone jack */
	if (snd_soc_jack_new(codec, "Headphone Jack", SND_JACK_HEADPHONE,
			&headphone_jack)) {
		dev_err(codec->dev, "Can't create headphone jack\n");
	}

	/* Create and initialize mic jack */
	if (snd_soc_jack_new(codec, "Mic Jack", SND_JACK_MICROPHONE,
			&mic_jack)) {
		dev_err(codec->dev, "Can't create mic jack\n");
	}

	/* Create and initialize headset button */
	if (snd_soc_jack_new(codec, "Headset Button", SND_JACK_BTN_0 |
		SND_JACK_BTN_1 | SND_JACK_BTN_2 | SND_JACK_BTN_3,
		&headset_btn)) {
		dev_err(codec->dev, "Can't create Headset Button\n");
	}

	rt5645_set_jack_detect(codec, &headphone_jack, &mic_jack,
		&headset_btn);

	snd_jack_set_key(headset_btn.jack, SND_JACK_BTN_0, KEY_MEDIA);
	snd_jack_set_key(headset_btn.jack, SND_JACK_BTN_1, KEY_VOICECOMMAND);
	snd_jack_set_key(headset_btn.jack, SND_JACK_BTN_2, KEY_VOLUMEUP);
	snd_jack_set_key(headset_btn.jack, SND_JACK_BTN_3, KEY_VOLUMEDOWN);

	bdw_rt5650->codec = codec;

	return 0;
}

/* broadwell digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link bdw_rt5650_dais[] = {
	/* Front End DAI links */
	{
		.name = "System PCM",
		.stream_name = "System Playback",
		.cpu_dai_name = "System Pin",
		.platform_name = "haswell-pcm-audio",
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.init = bdw_rt5650_rtd_init,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dpcm_playback = 1,
	},
	{
		.name = "Capture PCM",
		.stream_name = "Capture",
		.cpu_dai_name = "Capture Pin",
		.platform_name = "haswell-pcm-audio",
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dpcm_capture = 1,
	},

	/* Back End DAI links */
	{
		/* SSP0 - Codec */
		.name = "Codec",
		.be_id = 0,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.no_pcm = 1,
		.codec_name = "i2c-10EC5650:00",
		.codec_dai_name = "rt5645-aif1",
		.dai_fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = broadwell_ssp0_fixup,
		.ops = &bdw_rt5650_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.init = bdw_rt5650_init,
	},
};

/* ASoC machine driver for Broadwell DSP + RT5650 */
static struct snd_soc_card bdw_rt5650_card = {
	.name = "bdw-rt5650",
	.owner = THIS_MODULE,
	.dai_link = bdw_rt5650_dais,
	.num_links = ARRAY_SIZE(bdw_rt5650_dais),
	.dapm_widgets = bdw_rt5650_widgets,
	.num_dapm_widgets = ARRAY_SIZE(bdw_rt5650_widgets),
	.dapm_routes = bdw_rt5650_map,
	.num_dapm_routes = ARRAY_SIZE(bdw_rt5650_map),
	.controls = bdw_rt5650_controls,
	.num_controls = ARRAY_SIZE(bdw_rt5650_controls),
	.fully_routed = true,
};

static int bdw_rt5650_probe(struct platform_device *pdev)
{
	struct bdw_rt5650_priv *bdw_rt5650;

	bdw_rt5650_card.dev = &pdev->dev;

	/* Allocate driver private struct */
	bdw_rt5650 = devm_kzalloc(&pdev->dev, sizeof(struct bdw_rt5650_priv),
		GFP_KERNEL);
	if (!bdw_rt5650) {
		dev_err(&pdev->dev, "Can't allocate bdw_rt5650\n");
		return -ENOMEM;
	}
	snd_soc_card_set_drvdata(&bdw_rt5650_card, bdw_rt5650);

	return snd_soc_register_card(&bdw_rt5650_card);
}

static int bdw_rt5650_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&bdw_rt5650_card);
	return 0;
}

static struct platform_driver bdw_rt5650_audio = {
	.probe = bdw_rt5650_probe,
	.remove = bdw_rt5650_remove,
	.driver = {
		.name = "bdw-rt5650",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(bdw_rt5650_audio)

/* Module information */
MODULE_AUTHOR("Ben Zhang <benzh@chromium.org>");
MODULE_DESCRIPTION("Intel Broadwell RT5650 machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bdw-rt5650");
