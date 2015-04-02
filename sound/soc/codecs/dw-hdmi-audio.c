/*
 * dw-hdmi-codec.c
 *
 * DesignerWare ALSA SoC DAI driver for DW HDMI audio.
 * Copyright (c) 2014,  CORPORATION. All rights reserved.
 * Authors: Yakir Yang <ykk@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.*
 *
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/moduleparam.h>

#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>

#include <drm/bridge/dw_hdmi.h>
#include "dw-hdmi-audio.h"

struct snd_dw_hdmi {
	struct device *dev;
	struct dw_hdmi_audio_data data;

	u8 jack_status;
	bool is_jack_ready;
	struct snd_soc_jack jack;

	bool is_playback_status;
	struct hdmi_audio_fmt fmt;
};

int snd_dw_hdmi_jack_detect(struct snd_dw_hdmi *hdmi)
{
	u8 jack_status;

	if (!hdmi->is_jack_ready)
		return -EINVAL;

	/* TODO(ykk): Find a more reliable way to trigger hdmi audio jack
	 * status from dw_hdmi HPD interrupt. */
	usleep_range(50, 100);

	jack_status = !!(hdmi->data.read(hdmi->data.dw, HDMI_PHY_STAT0)&
		      HDMI_PHY_HPD) ? SND_JACK_LINEOUT : 0;

	if (jack_status != hdmi->jack_status) {
		snd_soc_jack_report(&hdmi->jack, jack_status,
				    SND_JACK_LINEOUT);
		hdmi->jack_status = jack_status;

		dev_info(hdmi->dev, "jack report [%d]\n", hdmi->jack_status);
	}

	return 0;
}

/* we don't want this irq mark with IRQF_ONESHOT flags,
 * so we build an irq_default_primary_handler here */
static irqreturn_t snd_dw_hdmi_hardirq(int irq, void *dev_id)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t snd_dw_hdmi_irq(int irq, void *dev_id)
{
	struct snd_dw_hdmi *hdmi = dev_id;

	snd_dw_hdmi_jack_detect(hdmi);

	return IRQ_HANDLED;
}

static void dw_hdmi_audio_set_fmt(struct snd_dw_hdmi *hdmi,
				  const struct hdmi_audio_fmt *fmt)
{
	hdmi->data.mod(hdmi->data.dw, fmt->input_type,
		       AUDIO_CONF0_INTERFACE_MSK, HDMI_AUD_CONF0);

	hdmi->data.mod(hdmi->data.dw, fmt->chan_num,
		       AUDIO_CONF0_I2SINEN_MSK, HDMI_AUD_CONF0);

	hdmi->data.mod(hdmi->data.dw, fmt->word_length,
		       AUDIO_CONF1_DATWIDTH_MSK, HDMI_AUD_CONF1);

	hdmi->data.mod(hdmi->data.dw, fmt->dai_fmt,
		       AUDIO_CONF1_DATAMODE_MSK, HDMI_AUD_CONF1);

	hdmi->data.write(hdmi->data.dw, 0, HDMI_AUD_INPUTCLKFS);

	hdmi->data.set_sample_rate(hdmi->data.dw, fmt->sample_rate);
}

static void hdmi_audio_set_fmt(struct snd_dw_hdmi *hdmi,
			       const struct hdmi_audio_fmt *fmt)
{
	if (fmt)
		hdmi->fmt = *fmt;
	dw_hdmi_audio_set_fmt(hdmi, &hdmi->fmt);
}

static int snd_dw_hdmi_dai_startup(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *codec_dai)
{
	struct snd_dw_hdmi *hdmi = snd_soc_dai_get_drvdata(codec_dai);

	dev_info(codec_dai->dev, "startup.\n");

	hdmi->is_playback_status = true;
	hdmi->data.enable(hdmi->data.dw);

	return 0;
}

static int snd_dw_hdmi_dai_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params,
				     struct snd_soc_dai *codec_dai)
{
	struct snd_dw_hdmi *hdmi = snd_soc_dai_get_drvdata(codec_dai);
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct hdmi_audio_fmt hdmi_fmt;
	unsigned int fmt, rate, chan, width;

	fmt = rtd->dai_link->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK;
	switch (fmt) {
	case SND_SOC_DAIFMT_I2S:
		hdmi_fmt.dai_fmt = AUDIO_DAIFMT_IIS;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		hdmi_fmt.dai_fmt = AUDIO_DAIFMT_LEFT_J;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		hdmi_fmt.dai_fmt = AUDIO_DAIFMT_RIGHT_J;
		break;
	default:
		dev_err(codec_dai->dev, "DAI format unsupported");
		return -EINVAL;
	}
	dev_dbg(codec_dai->dev, "[codec_dai]: dai_fmt = %d.\n", fmt);

	width = params_width(params);
	switch (width) {
	case 16:
	case 24:
		hdmi_fmt.word_length = width;
		break;
	default:
		dev_err(codec_dai->dev, "width[%d] not support!\n", width);
		return -EINVAL;
	}
	dev_dbg(codec_dai->dev, "[codec_dai]: word_length = %d.\n", width);

	chan = params_channels(params);
	switch (chan) {
	case 2:
		hdmi_fmt.chan_num = AUDIO_CHANNELNUM_2;
		break;
	case 4:
		hdmi_fmt.chan_num = AUDIO_CHANNELNUM_4;
		break;
	case 6:
		hdmi_fmt.chan_num = AUDIO_CHANNELNUM_6;
		break;
	case 8:
		hdmi_fmt.chan_num = AUDIO_CHANNELNUM_8;
		break;
	default:
		dev_err(codec_dai->dev, "channel[%d] not support!\n", chan);
		return -EINVAL;
	}
	dev_dbg(codec_dai->dev, "[codec_dai]: chan_num = %d.\n", chan);

	rate = params_rate(params);
	switch (rate) {
	case 32000:
	case 44100:
	case 48000:
	case 88200:
	case 96000:
	case 176400:
	case 192000:
		hdmi_fmt.sample_rate = rate;
		break;
	default:
		dev_err(codec_dai->dev, "rate[%d] not support!\n", rate);
		return -EINVAL;
	}
	dev_dbg(codec_dai->dev, "[codec_dai]: sample_rate = %d.\n", rate);

	hdmi_fmt.input_type = AUDIO_INPUTTYPE_IIS;

	hdmi_audio_set_fmt(hdmi, &hdmi_fmt);

	return 0;
}

static int snd_dw_hdmi_dai_trigger(struct snd_pcm_substream *substream,
				   int cmd, struct snd_soc_dai *codec_dai)
{
	struct snd_dw_hdmi *hdmi = snd_soc_dai_get_drvdata(codec_dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		hdmi->data.enable(hdmi->data.dw);
		dev_info(codec_dai->dev, "[codec_dai]: trigger enable.\n");
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		hdmi->data.disable(hdmi->data.dw);
		dev_info(codec_dai->dev, "[codec_dai]: trigger disable.\n");
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void snd_dw_hdmi_dai_shutdown(struct snd_pcm_substream *substream,
				     struct snd_soc_dai *codec_dai)
{
	struct snd_dw_hdmi *hdmi = snd_soc_dai_get_drvdata(codec_dai);

	dev_info(codec_dai->dev, "shutdown.\n");

	hdmi->is_playback_status = false;
	hdmi->data.disable(hdmi->data.dw);
}

static int snd_dw_hdmi_audio_probe(struct snd_soc_codec *codec)
{
	struct snd_dw_hdmi *hdmi = snd_soc_codec_get_drvdata(codec);
	int ret;

	ret = snd_soc_jack_new(codec, "HDMI Jack", SND_JACK_LINEOUT,
			       &hdmi->jack);
	if (ret) {
		dev_err(hdmi->dev, "jack new failed (%d)\n", ret);
		hdmi->is_jack_ready = false;
		return ret;
	}

	hdmi->is_jack_ready = true;

	return snd_dw_hdmi_jack_detect(hdmi);
}

static const struct snd_soc_dapm_widget snd_dw_hdmi_audio_widgets[] = {
	SND_SOC_DAPM_OUTPUT("TX"),
};

static const struct snd_soc_dapm_route snd_dw_hdmi_audio_routes[] = {
	{ "TX", NULL, "Playback" },
};

static const struct snd_soc_dai_ops dw_hdmi_dai_ops = {
	.startup = snd_dw_hdmi_dai_startup,
	.hw_params = snd_dw_hdmi_dai_hw_params,
	.trigger = snd_dw_hdmi_dai_trigger,
	.shutdown = snd_dw_hdmi_dai_shutdown,
};

static struct snd_soc_dai_driver dw_hdmi_audio_dai = {
	.name = "dw-hdmi-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_32000 |
			 SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
			 SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
			 SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &dw_hdmi_dai_ops,
};

static const struct snd_soc_codec_driver dw_hdmi_audio = {
	.probe = snd_dw_hdmi_audio_probe,
	.dapm_widgets = snd_dw_hdmi_audio_widgets,
	.num_dapm_widgets = ARRAY_SIZE(snd_dw_hdmi_audio_widgets),
	.dapm_routes = snd_dw_hdmi_audio_routes,
	.num_dapm_routes = ARRAY_SIZE(snd_dw_hdmi_audio_routes),
};

static int dw_hdmi_audio_probe(struct platform_device *pdev)
{
	struct dw_hdmi_audio_data *data = pdev->dev.platform_data;
	struct snd_dw_hdmi *hdmi;
	int ret;

	hdmi = devm_kzalloc(&pdev->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	hdmi->data = *data;
	hdmi->dev = &pdev->dev;
	hdmi->is_jack_ready = false;
	platform_set_drvdata(pdev, hdmi);

	ret = devm_request_threaded_irq(&pdev->dev, hdmi->data.irq,
					snd_dw_hdmi_hardirq, snd_dw_hdmi_irq,
					IRQF_SHARED, "dw-hdmi-audio", hdmi);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed (%d)\n", ret);
		goto free_hdmi_data;
	}

	ret = snd_soc_register_codec(&pdev->dev, &dw_hdmi_audio,
				     &dw_hdmi_audio_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "register codec failed (%d)\n", ret);
		goto free_irq;
	}

	dev_info(&pdev->dev, "hdmi audio init success.\n");

	return 0;

free_irq:
	devm_free_irq(&pdev->dev, hdmi->data.irq, hdmi);
free_hdmi_data:
	devm_kfree(&pdev->dev, hdmi);

	return ret;
}

static int dw_hdmi_audio_remove(struct platform_device *pdev)
{
	struct snd_dw_hdmi *hdmi = platform_get_drvdata(pdev);

	snd_soc_unregister_codec(&pdev->dev);
	devm_free_irq(&pdev->dev, hdmi->data.irq, hdmi);
	devm_kfree(&pdev->dev, hdmi);

	return 0;
}

#ifdef CONFIG_PM
static int dw_hdmi_audio_resume(struct device *dev)
{
	struct snd_dw_hdmi *hdmi = dev_get_drvdata(dev);

	snd_dw_hdmi_jack_detect(hdmi);

	if (hdmi->is_playback_status) {
		dw_hdmi_audio_set_fmt(hdmi, &hdmi->fmt);
	}

	return 0;
}

static int dw_hdmi_audio_suspend(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops dw_hdmi_audio_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(dw_hdmi_audio_suspend, dw_hdmi_audio_resume)
};

static const struct of_device_id dw_hdmi_audio_ids[] = {
	{ .compatible = "dw-hdmi-audio", },
	{ }
};

static struct platform_driver dw_hdmi_audio_driver = {
	.driver = {
		.name = "dw-hdmi-audio",
		.owner = THIS_MODULE,
		.pm = &dw_hdmi_audio_pm,
		.of_match_table = of_match_ptr(dw_hdmi_audio_ids),
	},
	.probe = dw_hdmi_audio_probe,
	.remove = dw_hdmi_audio_remove,
};
module_platform_driver(dw_hdmi_audio_driver);

MODULE_AUTHOR("Yakir Yang <ykk@rock-chips.com>");
MODULE_DESCRIPTION("DW HDMI Audio ASoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" "dw-hdmi-audio");
MODULE_DEVICE_TABLE(of, dw_hdmi_audio_ids);
