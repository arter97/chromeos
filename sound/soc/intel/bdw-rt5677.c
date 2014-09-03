/*
 * ASoC machine driver for Intel Broadwell platforms with RT5677 codec
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/jack.h>

#include "sst-dsp.h"
#include "sst-haswell-ipc.h"

#include "../codecs/rt5677.h"

struct bdw_rt5677_priv {
	struct snd_soc_jack hs_jack;
};

static const struct snd_soc_dapm_widget bdw_rt5677_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_MIC("Local DMIC L", NULL),
	SND_SOC_DAPM_MIC("Local DMIC R", NULL),
	SND_SOC_DAPM_MIC("Remote DMIC L", NULL),
	SND_SOC_DAPM_MIC("Remote DMIC R", NULL),
};

static const struct snd_soc_dapm_route bdw_rt5677_map[] = {

	/* speaker */
	{"Speaker", NULL, "PDM1L"},
	{"Speaker", NULL, "PDM1R"},

	/* HP jack connectors */
	{"Headphones", NULL, "LOUT1"},
	{"Headphones", NULL, "LOUT2"},

	/* other jacks */
	{"IN1P", NULL, "Mic Jack"},
	{"IN1N", NULL, "Mic Jack"},

	/* digital mics */
	{"DMIC L1", NULL, "Remote DMIC L"},
	{"DMIC R1", NULL, "Remote DMIC R"},
	{"DMIC L2", NULL, "Local DMIC L"},
	{"DMIC R2", NULL, "Local DMIC R"},

	/* CODEC BE connections */
	{"SSP0 CODEC IN", NULL, "AIF1 Capture"},
	{"AIF1 Playback", NULL, "SSP0 CODEC OUT"},
};

static struct snd_soc_jack_pin hs_jack_pins[] = {
	{
		.pin	= "Headphones",
		.mask	= SND_JACK_HEADPHONE,
	},
	{
		.pin	= "Mic Jack",
		.mask	= SND_JACK_MICROPHONE,
	},
};

static int broadwell_ssp0_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	/* The ADSP will covert the FE rate to 48k, stereo */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	/* set SSP0 to 16 bit */
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
				    SNDRV_PCM_HW_PARAM_FIRST_MASK],
				    SNDRV_PCM_FORMAT_S16_LE);
	return 0;
}

static int bdw_rt5677_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

#if 0
	//Option 1: Use PLL
	ret = snd_soc_dai_set_sysclk(codec_dai, RT5677_SCLK_S_PLL1, 24576000,
		SND_SOC_CLOCK_IN);
	snd_soc_dai_set_pll(codec_dai,0,RT5677_PLL1_S_MCLK,24000000,24576000);
#else
	//Option 2: No PLL
	ret = snd_soc_dai_set_sysclk(codec_dai, RT5677_SCLK_S_MCLK, 24576000,
		SND_SOC_CLOCK_IN);
#endif

	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec sysclk configuration\n");
		return ret;
	}

	return ret;
}

static struct snd_soc_ops bdw_rt5677_ops = {
	.hw_params = bdw_rt5677_hw_params,
};

static int bdw_rt5677_rtd_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct sst_pdata *pdata = dev_get_platdata(rtd->platform->dev);
	struct sst_hsw *broadwell = pdata->dsp;
	int ret;

	/* Set ADSP SSP port settings */

	ret = sst_hsw_device_set_config(broadwell, SST_HSW_DEVICE_SSP_0,
		SST_HSW_DEVICE_MCLK_FREQ_24_MHZ,
		SST_HSW_DEVICE_CLOCK_MASTER, 9);
	if (ret < 0) {
		dev_err(rtd->dev, "error: failed to set device config\n");
		return ret;
	}

	/* always connected - check HP for jack detect */
	snd_soc_dapm_enable_pin(dapm, "Headphones");
	snd_soc_dapm_enable_pin(dapm, "Speaker");
	snd_soc_dapm_enable_pin(dapm, "Mic Jack");
	snd_soc_dapm_enable_pin(dapm, "Remote DMIC L");
	snd_soc_dapm_enable_pin(dapm, "Remote DMIC R");
	snd_soc_dapm_enable_pin(dapm, "Local DMIC L");
	snd_soc_dapm_enable_pin(dapm, "Local DMIC R");

	return 0;
}

static int bdw_rt5677_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct bdw_rt5677_priv *priv = snd_soc_card_get_drvdata(codec->card);
	int ret;

	ret = snd_soc_jack_new(codec, "Headphone Jack", SND_JACK_HEADSET,
				&priv->hs_jack);
	if (ret)
		return ret;

	ret = snd_soc_jack_add_pins(&priv->hs_jack, ARRAY_SIZE(hs_jack_pins), hs_jack_pins);
	if (ret)
		return ret;

	// HACK: always report that the headphone and mic exist.
	// Need to add jack detection code the the codec driver later.
	snd_soc_jack_report(&priv->hs_jack, SND_JACK_HEADPHONE | SND_JACK_MICROPHONE,
		SND_JACK_MICROPHONE | SND_JACK_HEADPHONE);

	return 0;
}

/* broadwell digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link bdw_rt5677_dais[] = {
	/* Front End DAI links */
	{
		.name = "System PCM",
		.stream_name = "System Playback",
		.cpu_dai_name = "System Pin",
		.platform_name = "haswell-pcm-audio",
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.init = bdw_rt5677_rtd_init,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
	},
	{
		.name = "Offload0",
		.stream_name = "Offload0 Playback",
		.cpu_dai_name = "Offload0 Pin",
		.platform_name = "haswell-pcm-audio",
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
	},
	{
		.name = "Offload1",
		.stream_name = "Offload1 Playback",
		.cpu_dai_name = "Offload1 Pin",
		.platform_name = "haswell-pcm-audio",
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
	},
	{
		.name = "Loopback PCM",
		.stream_name = "Loopback",
		.cpu_dai_name = "Loopback Pin",
		.platform_name = "haswell-pcm-audio",
		.dynamic = 0,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_capture = 1,
	},
	{
		.name = "Capture PCM",
		.stream_name = "Capture",
		.cpu_dai_name = "Capture Pin",
		.platform_name = "haswell-pcm-audio",
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
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
		.codec_name = "i2c-RT5677CE:00",
		.codec_dai_name = "rt5677-aif1",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = broadwell_ssp0_fixup,
		.ops = &bdw_rt5677_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.init = bdw_rt5677_init,
	},
};

/* ASoC machine driver for Broadwell DSP + RT5677 */
static struct snd_soc_card bdw_rt5677 = {
	.name = "bdw-rt5677",
	.owner = THIS_MODULE,
	.dai_link = bdw_rt5677_dais,
	.num_links = ARRAY_SIZE(bdw_rt5677_dais),
	.dapm_widgets = bdw_rt5677_widgets,
	.num_dapm_widgets = ARRAY_SIZE(bdw_rt5677_widgets),
	.dapm_routes = bdw_rt5677_map,
	.num_dapm_routes = ARRAY_SIZE(bdw_rt5677_map),
	.fully_routed = true,
};

static int bdw_rt5677_probe(struct platform_device *pdev)
{
	struct bdw_rt5677_priv *priv;

	bdw_rt5677.dev = &pdev->dev;

	priv = devm_kzalloc(&pdev->dev, sizeof(struct bdw_rt5677_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	snd_soc_card_set_drvdata(&bdw_rt5677, priv);

	return snd_soc_register_card(&bdw_rt5677);
}

static int bdw_rt5677_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&bdw_rt5677);
	return 0;
}

static struct platform_driver bdw_rt5677_audio = {
	.probe = bdw_rt5677_probe,
	.remove = bdw_rt5677_remove,
	.driver = {
		.name = "bdw-rt5677",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(bdw_rt5677_audio)

/* Module information */
MODULE_AUTHOR("");
MODULE_DESCRIPTION("Intel Broadwell RT5677 machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bdw-rt5677");
