/*
 * tegra_t210ref_mobile.c - Tegra T210 Machine driver for mobile
 *
 * Copyright (c) 2013-2015 NVIDIA CORPORATION.  All rights reserved.
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/sysedp.h>
#include <linux/input.h>
#include <linux/tegra-pmc.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif
#include <linux/pm_runtime.h>
#include <mach/tegra_asoc_pdata.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "tegra_asoc_utils_alt.h"
#include "tegra_asoc_machine_alt.h"

#include "../codecs/audience/es755.h"
#include "../codecs/tas2552.h"

#define DRV_NAME "tegra-snd-t210ref-mobile"

#define GPIO_SPKR_EN    BIT(0)
#define GPIO_HP_MUTE    BIT(1)
#define GPIO_INT_MIC_EN BIT(2)
#define GPIO_EXT_MIC_EN BIT(3)
#define GPIO_HP_DET     BIT(4)

#define MAX_AMX_SLOT_SIZE 64

#define TDM_SLOT_MAP(stream_id, nth_channel, nth_byte)	\
	((stream_id << 16) | (nth_channel << 8) | nth_byte)

struct tegra_t210ref {
	struct snd_soc_card *pcard;
	struct tegra_asoc_platform_data *pdata;
	struct tegra_asoc_audio_clock_info audio_clock;
	struct sysedp_consumer *sysedpc;
	struct regulator *codec_reg;
	struct regulator *digital_reg;
	struct regulator *analog_reg;
	struct regulator *spk_reg;
	struct regulator *dmic_reg;
	unsigned int num_codec_links;
	int gpio_requested;
#ifdef CONFIG_SWITCH
	int jack_status;
#endif
	int clock_enabled;
	enum snd_soc_bias_level bias_level;
	const char *edp_name;
};

static struct snd_soc_jack tegra_t210ref_hp_jack;

static struct snd_soc_jack_gpio tegra_t210ref_hp_jack_gpio = {
	.name = "headphone detect",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,
	.invert = 1,
};

#ifdef CONFIG_SWITCH
static struct switch_dev tegra_t210ref_headset_switch = {
	.name = "h2w",
};

static int tegra_t210ref_jack_notifier(struct notifier_block *self,
			      unsigned long action, void *dev)
{
	struct snd_soc_jack *jack = dev;
	struct snd_soc_codec *codec = jack->codec;
	struct snd_soc_card *card = codec->component.card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	enum headset_state state = BIT_NO_HEADSET;
	static bool button_pressed;

	if (button_pressed) {
		button_pressed = false;
		return NOTIFY_OK;
	}

	if (action & (SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2)) {
		button_pressed = true;
		return NOTIFY_OK;
	}

	if (jack == &tegra_t210ref_hp_jack) {
		machine->jack_status &= ~SND_JACK_HEADPHONE;
		machine->jack_status &= ~SND_JACK_MICROPHONE;
		machine->jack_status |= (action & SND_JACK_HEADSET);
	}

	switch (machine->jack_status) {
	case SND_JACK_HEADPHONE:
		state = BIT_HEADSET_NO_MIC;
		break;
	case SND_JACK_HEADSET:
		state = BIT_HEADSET;
		break;
	case SND_JACK_MICROPHONE:
		/* mic: would not report */
	default:
		state = BIT_NO_HEADSET;
	}

	switch_set_state(&tegra_t210ref_headset_switch, state);

	return NOTIFY_OK;
}

static struct notifier_block tegra_t210ref_jack_detect_nb = {
	.notifier_call = tegra_t210ref_jack_notifier,
};
#else
static struct snd_soc_jack_pin tegra_t210ref_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};
#endif

static int tegra_t210ref_dai_init(struct snd_soc_pcm_runtime *rtd,
					int rate,
					int channels,
					u64 formats)
{
	struct snd_soc_card *card = rtd->card;
	struct device_node *np = card->dev->of_node;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_stream *dai_params;
	unsigned int idx, mclk, clk_out_rate;
	int err;

	switch (rate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
	case 176000:
		clk_out_rate = 19200000; /* Codec rate */
		mclk = 11289600 * 2; /* PLL_A rate */
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
	case 192000:
	default:
		clk_out_rate = 19200000;
		mclk = 12288000 * 2;
		break;
	}

	pr_info("Setting pll_a = %d Hz clk_out = %d Hz\n", mclk, clk_out_rate);
	err = tegra_alt_asoc_utils_set_rate(&machine->audio_clock,
				rate, mclk, clk_out_rate);
	if (err < 0) {
		dev_err(card->dev, "Can't configure clocks\n");
		return err;
	}

	if (!of_device_is_compatible(np,
	  "nvidia,tegra-audio-t210ref-mobile-foster")) {
		idx = tegra_machine_get_codec_dai_link_idx("earSmart-playback");
		/* check if idx has valid number */
		if (idx != -EINVAL) {
			dai_params =
			  (struct snd_soc_pcm_stream *)
			  card->rtd[idx].dai_link->params;

			/* update link_param to update hw_param for DAPM */
			dai_params->rate_min = rate;
			dai_params->channels_min = channels;
			dai_params->formats = formats;

			err = snd_soc_dai_set_bclk_ratio(card->rtd[idx].cpu_dai,
				tegra_machine_get_bclk_ratio(&card->rtd[idx]));
			if (err < 0) {
				dev_err(card->dev, "Can't set cpu dai bclk ratio\n");
				return err;
			}
		}
	}

	idx = tegra_machine_get_codec_dai_link_idx("spdif-dit-1");
	if (idx != -EINVAL) {
		dai_params =
		  (struct snd_soc_pcm_stream *)card->rtd[idx].dai_link->params;

		/* update link_param to update hw_param for DAPM */
		dai_params->rate_min = rate;
		dai_params->channels_min = channels;
		dai_params->formats = formats;

		err = snd_soc_dai_set_bclk_ratio(card->rtd[idx].cpu_dai,
			tegra_machine_get_bclk_ratio(&card->rtd[idx]));
		if (err < 0) {
			dev_err(card->dev, "Can't set cpu dai bclk ratio\n");
			return err;
		}

		err = snd_soc_dai_set_tdm_slot(card->rtd[idx].cpu_dai,
			(1 << channels) - 1, (1 << channels) - 1, 0, 0);
		if (err < 0) {
			dev_err(card->dev, "Can't set cpu dai slot ctrl\n");
			return err;
		}
	}

	/* update dai link hw_params for non pcm links */
	for (idx = 0; idx < card->num_rtd; idx++) {
		if (card->rtd[idx].dai_link->params) {
			dai_params =
			  (struct snd_soc_pcm_stream *)
			  card->rtd[idx].dai_link->params;
			dai_params->rate_min = rate;
			dai_params->channels_min = channels;
			dai_params->formats = formats;
		}
	}

	return 0;
}

static int tegra_t210ref_amx1_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *amx_dai = rtd->cpu_dai;
	unsigned int tx_slot[MAX_AMX_SLOT_SIZE];
	unsigned int slot_size;

	/* HACK: TODO: Add mixer controls to configute byte ram map */
	slot_size = 8;
	tx_slot[0] = TDM_SLOT_MAP(0, 1, 0);
	tx_slot[1] = TDM_SLOT_MAP(0, 1, 1);
	tx_slot[2] = 0;
	tx_slot[3] = 0;
	tx_slot[4] = TDM_SLOT_MAP(1, 1, 0);
	tx_slot[5] = TDM_SLOT_MAP(1, 1, 1);
	tx_slot[6] = 0;
	tx_slot[7] = 0;

	if (amx_dai->driver->ops->set_channel_map)
		amx_dai->driver->ops->set_channel_map(amx_dai,
			slot_size, tx_slot, 0, NULL);

	return 0;
}

static int tegra_t210ref_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	int err;

	err = tegra_t210ref_dai_init(rtd, params_rate(params),
			params_channels(params),
			(1ULL << (params_format(params))));
	if (err < 0) {
		dev_err(card->dev, "Failed dai init\n");
		return err;
	}

	return 0;
}

static int tegra_t210ref_compr_set_params(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_platform *platform = rtd->platform;
	struct snd_codec codec_params;
	int srate;
	int err;

	if (platform->driver->compr_ops &&
		platform->driver->compr_ops->get_params) {
		err = platform->driver->compr_ops->get_params(cstream,
			&codec_params);
		if (err < 0) {
			dev_err(card->dev, "Failed to get compr params\n");
			return err;
		}
	} else {
		dev_err(card->dev, "compr ops not set\n");
		return -EINVAL;
	}

	/* TODO: Add SRC in offload path */
	srate = 48000;

	err = tegra_t210ref_dai_init(rtd, srate, codec_params.ch_out,
			SNDRV_PCM_FMTBIT_S16_LE);
	if (err < 0) {
		dev_err(card->dev, "Failed dai init\n");
		return err;
	}

	return 0;
}

static int tegra_t210ref_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = rtd->card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	struct snd_soc_pcm_stream *dai_params =
		(struct snd_soc_pcm_stream *)rtd->dai_link->params;
	unsigned int srate;
	int err;

	srate = dai_params->rate_min;

	err = tegra_alt_asoc_utils_set_extern_parent(&machine->audio_clock,
							"clk_m");
	if (err < 0) {
		dev_err(card->dev, "Failed to set extern clk parent\n");
		return err;
	}

	if (gpio_is_valid(pdata->gpio_hp_det)) {
		tegra_t210ref_hp_jack_gpio.gpio = pdata->gpio_hp_det;
		tegra_t210ref_hp_jack_gpio.invert =
			!pdata->gpio_hp_det_active_high;
		err = snd_soc_jack_new(codec, "Headphone Jack",
			       (SND_JACK_HEADSET | SND_JACK_BTN_0 |
			       SND_JACK_BTN_1 | SND_JACK_BTN_2),
			       &tegra_t210ref_hp_jack);
		if (err) {
			pr_err("failed to create a jack for es755\n");
			return err;
		}

		err = snd_jack_set_key(tegra_t210ref_hp_jack.jack,
				SND_JACK_BTN_0, KEY_MEDIA);
		if (err < 0)
			pr_err("Failed to set KEY_MEDIA: %d\n", err);

		err = snd_jack_set_key(tegra_t210ref_hp_jack.jack,
				SND_JACK_BTN_1, KEY_VOLUMEUP);
		if (err < 0)
			pr_err("Failed to set KEY_VOLUMEUP: %d\n", err);

		err = snd_jack_set_key(tegra_t210ref_hp_jack.jack,
				SND_JACK_BTN_2, KEY_VOLUMEDOWN);
		if (err < 0)
			pr_err("Failed to set KEY_VOLUMEDOWN: %d\n", err);
#ifdef CONFIG_SWITCH
		snd_soc_jack_notifier_register(&tegra_t210ref_hp_jack,
						&tegra_t210ref_jack_detect_nb);
#else /* gpio based headset detection */
		snd_soc_jack_add_pins(&tegra_es755_hp_jack,
					ARRAY_SIZE(tegra_es755_hs_jack_pins),
					tegra_es755_hs_jack_pins);
#endif
		snd_soc_jack_add_gpios(&tegra_t210ref_hp_jack,
					1, &tegra_t210ref_hp_jack_gpio);
		machine->gpio_requested |= GPIO_HP_DET;

		es755_detect(codec, &tegra_t210ref_hp_jack);
	}

	return 0;
}

static int tegra_t210ref_sfc_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int in_srate, out_srate;
	int err;

	in_srate = 48000;
	out_srate = 8000;

	err = snd_soc_dai_set_sysclk(codec_dai, 0, out_srate,
					SND_SOC_CLOCK_OUT);
	err = snd_soc_dai_set_sysclk(codec_dai, 0, in_srate,
					SND_SOC_CLOCK_IN);

	return 0;
}

static int tegra_es755_startup(struct snd_pcm_substream *substream)
{
	escore_pm_get_sync();
	return 0;
}

static void tegra_es755_shutdown(struct snd_pcm_substream *substream)
{
	escore_pm_put_autosuspend();
}

static int tegra_es755_startup_compr(struct snd_compr_stream *cstream)
{
	escore_pm_get_sync();
	return 0;
}

static void tegra_es755_shutdown_compr(struct snd_compr_stream *cstream)
{
	escore_pm_put_autosuspend();
}

static struct snd_soc_ops tegra_t210ref_ops = {
	.hw_params = tegra_t210ref_hw_params,
	.startup = tegra_es755_startup,
	.shutdown = tegra_es755_shutdown,
};

static struct snd_soc_compr_ops tegra_t210ref_compr_ops = {
	.set_params = tegra_t210ref_compr_set_params,
	.startup = tegra_es755_startup_compr,
	.shutdown = tegra_es755_shutdown_compr,
};

static int tegra_t210ref_event_int_spk(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	int err;

	if (machine->spk_reg) {
		if (SND_SOC_DAPM_EVENT_ON(event)) {
			if (machine->sysedpc)
				sysedp_set_state(machine->sysedpc, 1);
			err = regulator_enable(machine->spk_reg);
			if (err < 0)
				dev_err(card->dev, "Failed to enable spk regulator\n");
		} else {
			regulator_disable(machine->spk_reg);
			if (machine->sysedpc)
				sysedp_set_state(machine->sysedpc, 0);
		}
	}

	if (!(machine->gpio_requested & GPIO_SPKR_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_spkr_en,
				!!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}


static const struct snd_soc_dapm_widget tegra_t210ref_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", tegra_t210ref_event_int_spk),
	SND_SOC_DAPM_HP("y Headphone", NULL),
	SND_SOC_DAPM_MIC("y Mic", NULL),
	SND_SOC_DAPM_HP("k Headphone", NULL),
	SND_SOC_DAPM_MIC("k Mic", NULL),
};

static const struct snd_kcontrol_new tegra_t210ref_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
};

static int tegra_t210ref_suspend_pre(struct snd_soc_card *card)
{
	struct snd_soc_jack_gpio *gpio = &tegra_t210ref_hp_jack_gpio;
	unsigned int idx;

	/* DAPM dai link stream work for non pcm links */
	for (idx = 0; idx < card->num_rtd; idx++) {
		if (card->rtd[idx].dai_link->params)
			INIT_DELAYED_WORK(&card->rtd[idx].delayed_work, NULL);
	}

	if (gpio_is_valid(gpio->gpio))
		disable_irq(gpio_to_irq(gpio->gpio));

	return 0;
}

static int tegra_t210ref_suspend_post(struct snd_soc_card *card)
{
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);

	if (machine->clock_enabled) {
		machine->clock_enabled = 0;
		tegra_alt_asoc_utils_clk_disable(&machine->audio_clock);
	}

	if (machine->digital_reg)
		regulator_disable(machine->digital_reg);

	return 0;
}

static int tegra_t210ref_resume_pre(struct snd_soc_card *card)
{
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct snd_soc_jack_gpio *gpio = &tegra_t210ref_hp_jack_gpio;
	int ret, val;

	if (machine->digital_reg)
		ret = regulator_enable(machine->digital_reg);

	if (gpio_is_valid(gpio->gpio)) {
		val = gpio_get_value(gpio->gpio);
		val = gpio->invert ? !val : val;
		if (gpio->jack)
			snd_soc_jack_report(gpio->jack, val, gpio->report);
		enable_irq(gpio_to_irq(gpio->gpio));
	}

	if (!machine->clock_enabled) {
		machine->clock_enabled = 1;
		tegra_alt_asoc_utils_clk_enable(&machine->audio_clock);
	}

	return 0;
}

static const struct snd_soc_dapm_route tegra_t210ref_audio_map[] = {
};

static int tegra_t210ref_remove(struct snd_soc_card *card)
{
	return 0;
}

static struct snd_soc_card snd_soc_tegra_t210ref = {
	.name = "tegra-t210ref",
	.owner = THIS_MODULE,
	.remove = tegra_t210ref_remove,
	.suspend_post = tegra_t210ref_suspend_post,
	.suspend_pre = tegra_t210ref_suspend_pre,
	.resume_pre = tegra_t210ref_resume_pre,
	.controls = tegra_t210ref_controls,
	.num_controls = ARRAY_SIZE(tegra_t210ref_controls),
	.dapm_widgets = tegra_t210ref_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra_t210ref_dapm_widgets),
	.fully_routed = true,
};

static int tegra_t210ref_driver_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &snd_soc_tegra_t210ref;
	struct tegra_t210ref *machine;
	struct snd_soc_dai_link *tegra_machine_dai_links = NULL;
	struct snd_soc_dai_link *tegra_t210ref_codec_links = NULL;
	struct snd_soc_codec_conf *tegra_machine_codec_conf = NULL;
	struct snd_soc_codec_conf *tegra_t210ref_codec_conf = NULL;
	struct tegra_asoc_platform_data *pdata = NULL;
	int ret = 0, i;

	if (!np) {
		dev_err(&pdev->dev, "No device tree node for t210ref driver");
		return -ENODEV;
	}

	if (!of_device_is_compatible(np,
	  "nvidia,tegra-audio-t210ref-mobile-foster")) {
		if (escore_is_probe_error()) {
			dev_err(&pdev->dev, "No ES755 CODEC found");
			return -ENODEV;
		}
	}

	machine = devm_kzalloc(&pdev->dev, sizeof(struct tegra_t210ref),
			       GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_t210ref struct\n");
		ret = -ENOMEM;
		goto err;
	}

	pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct tegra_asoc_platform_data),
				GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev,
			"Can't allocate tegra_asoc_platform_data struct\n");
		goto err;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

	ret = snd_soc_of_parse_card_name(card, "nvidia,model");
	if (ret)
		goto err;

	ret = snd_soc_of_parse_audio_routing(card,
				"nvidia,audio-routing");
	if (ret)
		goto err;

	/* set new codec links and conf */
	tegra_t210ref_codec_links = tegra_machine_new_codec_links(pdev,
		tegra_t210ref_codec_links,
		&machine->num_codec_links);
	if (!tegra_t210ref_codec_links)
		goto err_alloc_dai_link;

	tegra_t210ref_codec_conf = tegra_machine_new_codec_conf(pdev,
		tegra_t210ref_codec_conf,
		&machine->num_codec_links);
	if (!tegra_t210ref_codec_conf)
		goto err_alloc_dai_link;


	/* get the xbar dai link/codec conf structure */
	tegra_machine_dai_links = tegra_machine_get_dai_link();
	if (!tegra_machine_dai_links)
		goto err_alloc_dai_link;
	tegra_machine_codec_conf = tegra_machine_get_codec_conf();
	if (!tegra_machine_codec_conf)
		goto err_alloc_dai_link;

	/* set ADMAIF dai_ops */
	for (i = TEGRA210_DAI_LINK_ADMAIF1;
		i <= TEGRA210_DAI_LINK_ADMAIF10; i++)
		tegra_machine_set_dai_ops(i, &tegra_t210ref_ops);

	/* set sfc dai_init */
	tegra_machine_set_dai_init(TEGRA210_DAI_LINK_SFC1_RX,
		&tegra_t210ref_sfc_init);

	/* set AMX/ADX dai_init */
	tegra_machine_set_dai_init(TEGRA210_DAI_LINK_AMX1,
		&tegra_t210ref_amx1_dai_init);

	/* set ADSP PCM */
	tegra_machine_set_dai_ops(TEGRA210_DAI_LINK_ADSP_PCM,
			&tegra_t210ref_ops);

	/* set ADSP COMPR */
	for (i = TEGRA210_DAI_LINK_ADSP_COMPR1;
		i <= TEGRA210_DAI_LINK_ADSP_COMPR2; i++) {
		tegra_machine_set_dai_compr_ops(i,
			&tegra_t210ref_compr_ops);
	}

	pdata->gpio_hp_det = of_get_named_gpio(np,
					"nvidia,hp-det-gpios", 0);
	if (pdata->gpio_hp_det < 0)
		dev_warn(&pdev->dev, "Failed to get HP Det GPIO\n");

	pdata->gpio_codec1 = pdata->gpio_codec2 = pdata->gpio_codec3 =
	pdata->gpio_spkr_en = pdata->gpio_hp_mute =
	pdata->gpio_int_mic_en = pdata->gpio_ext_mic_en =
	pdata->gpio_ldo1_en =  -1;

	/* set codec init */
	for (i = 0; i < machine->num_codec_links; i++) {
		if (tegra_t210ref_codec_links[i].name) {
			if (strstr(tegra_t210ref_codec_links[i]
				.name, "earSmart-playback"))
				tegra_t210ref_codec_links[i].init =
						tegra_t210ref_init;
		}
	}

	/* append t210ref specific dai_links */
	card->num_links =
		tegra_machine_append_dai_link(tegra_t210ref_codec_links,
			2 * machine->num_codec_links);
	tegra_machine_dai_links = tegra_machine_get_dai_link();
	card->dai_link = tegra_machine_dai_links;

	/* append t210ref specific codec_conf */
	card->num_configs =
		tegra_machine_append_codec_conf(tegra_t210ref_codec_conf,
			machine->num_codec_links);
	tegra_machine_codec_conf = tegra_machine_get_codec_conf();
	card->codec_conf = tegra_machine_codec_conf;

#ifdef CONFIG_SWITCH
	/* Addd h2w swith class support */
	ret = tegra_alt_asoc_switch_register(&tegra_t210ref_headset_switch);
	if (ret < 0)
		goto err_alloc_dai_link;
#endif

	/*
	*digital_reg - provided the digital power for the codec and must be
	*ON always
	*/
	machine->digital_reg = regulator_get(&pdev->dev, "dbvdd");
	if (IS_ERR(machine->digital_reg))
		machine->digital_reg = NULL;
	else
		ret = regulator_enable(machine->digital_reg);

	/*
	*spk_reg - provided the speaker power and can be turned ON
	*on need basis, when required
	*/
	machine->spk_reg = regulator_get(&pdev->dev, "spkvdd");
	if (IS_ERR(machine->spk_reg))
		machine->spk_reg = NULL;

	/*
	*dmic_reg - provided the DMIC power and can be turned ON
	*on need basis, when required
	*/
	machine->dmic_reg = regulator_get(&pdev->dev, "dmicvdd");
	if (IS_ERR(machine->dmic_reg))
		machine->dmic_reg = NULL;

	machine->pdata = pdata;
	machine->pcard = card;
	machine->clock_enabled = 1;

	ret = tegra_alt_asoc_utils_init(&machine->audio_clock,
					&pdev->dev,
					card);
	if (ret)
		goto err_switch_unregister;

	card->dapm.idle_bias_off = 1;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_fini_utils;
	}

	if (!of_device_is_compatible(np,
	  "nvidia,tegra-audio-t210ref-mobile-foster")) {
		if (of_property_read_string(np, "edp-consumer-name",
			&machine->edp_name)) {
			machine->sysedpc = NULL;
			dev_err(&pdev->dev, "property 'edp-consumer-name' missing or invalid\n");
		} else {
			machine->sysedpc =
				sysedp_create_consumer("codec+speaker",
					machine->edp_name);
		}
	}

	return 0;

err_fini_utils:
	tegra_alt_asoc_utils_fini(&machine->audio_clock);
err_switch_unregister:
#ifdef CONFIG_SWITCH
	tegra_alt_asoc_switch_unregister(&tegra_t210ref_headset_switch);
#endif
	if (machine->gpio_requested & GPIO_HP_DET)
		snd_soc_jack_free_gpios(&tegra_t210ref_hp_jack,
					1,
					&tegra_t210ref_hp_jack_gpio);

	if (machine->digital_reg) {
		regulator_disable(machine->digital_reg);
		regulator_put(machine->digital_reg);
	}
	if (machine->analog_reg) {
		regulator_disable(machine->analog_reg);
		regulator_put(machine->analog_reg);
	}
	if (machine->spk_reg)
		regulator_put(machine->spk_reg);

	if (machine->dmic_reg)
		regulator_put(machine->dmic_reg);
	if (machine->codec_reg) {
		regulator_disable(machine->codec_reg);
		regulator_put(machine->codec_reg);
	}
err_alloc_dai_link:
	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();
err:
	return ret;
}

static int tegra_t210ref_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);

	if (machine->gpio_requested & GPIO_HP_DET)
		snd_soc_jack_free_gpios(&tegra_t210ref_hp_jack,
					1,
					&tegra_t210ref_hp_jack_gpio);

	if (machine->digital_reg) {
		regulator_disable(machine->digital_reg);
		regulator_put(machine->digital_reg);
	}
	if (machine->analog_reg) {
		regulator_disable(machine->analog_reg);
		regulator_put(machine->analog_reg);
	}
	if (machine->spk_reg)
		regulator_put(machine->spk_reg);

	if (machine->dmic_reg)
		regulator_put(machine->dmic_reg);
	if (machine->codec_reg) {
		regulator_disable(machine->codec_reg);
		regulator_put(machine->codec_reg);
	}

	snd_soc_unregister_card(card);

	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();
	tegra_alt_asoc_utils_fini(&machine->audio_clock);

#ifdef CONFIG_SWITCH
	tegra_alt_asoc_switch_unregister(&tegra_t210ref_headset_switch);
#endif
	if (machine->sysedpc)
		sysedp_free_consumer(machine->sysedpc);

	return 0;
}

static const struct of_device_id tegra_t210ref_of_match[] = {
	{ .compatible = "nvidia,tegra-audio-t210ref-mobile-es755", },
	{ .compatible = "nvidia,tegra-audio-t210ref-mobile-foster", },
	{},
};

static struct platform_driver tegra_t210ref_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = tegra_t210ref_of_match,
	},
	.probe = tegra_t210ref_driver_probe,
	.remove = tegra_t210ref_driver_remove,
};
module_platform_driver(tegra_t210ref_driver);

MODULE_AUTHOR("Dara Ramesh <dramesh@nvidia.com>");
MODULE_DESCRIPTION("Tegra+t210ref machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra_t210ref_of_match);
