/*
 * NAU8825 audio codec driver
 *
 * Copyright 2015 Nuvoton Technology Corp.
 * Copyright 2015 Google Chromium project.
 *  Author: Anatol Pomozov <anatol@chromium.org>
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/input.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>

#include "nau8825.h"

struct nau8825 {
	struct regmap *regmap;
	struct snd_soc_dapm_context *dapm;
	int irq;
	struct snd_soc_jack *jack;
	int button_pressed;
	bool long_press;
};

static const struct reg_default nau8825_reg_defaults[] = {
	{ NAU8825_REG_ENA_CTRL, 0x00ff },
	{ NAU8825_REG_CLK_DIVIDER, 0x0050 },
	{ NAU8825_REG_FLL1, 0x0 },
	{ NAU8825_REG_FLL2, 0x3126 },
	{ NAU8825_REG_FLL3, 0x0008 },
	{ NAU8825_REG_FLL4, 0x0010 },
	{ NAU8825_REG_FLL5, 0x0 },
	{ NAU8825_REG_FLL6, 0x6000 },
	{ NAU8825_REG_FLL_VCO_RSV, 0xf13c },
	{ NAU8825_REG_HSD_CTRL, 0x000c },
	{ NAU8825_REG_JACK_DET_CTRL, 0x0 },
	{ NAU8825_REG_INTERRUPT_MASK, 0x0 },
	{ NAU8825_REG_INTERRUPT_DIS_CTRL, 0xffff },
	{ NAU8825_REG_SAR_CTRL, 0x0015 },
	{ NAU8825_REG_KEYDET_CTRL, 0x0110 },
	{ NAU8825_REG_VDET_THRESHOLD_1, 0x0 },
	{ NAU8825_REG_VDET_THRESHOLD_2, 0x0 },
	{ NAU8825_REG_VDET_THRESHOLD_3, 0x0 },
	{ NAU8825_REG_VDET_THRESHOLD_4, 0x0 },
	{ NAU8825_REG_GPIO34_CTRL, 0x0 },
	{ NAU8825_REG_GPIO12_CTRL, 0x0 },
	{ NAU8825_REG_TDM_CTRL, 0x0 },
	{ NAU8825_REG_I2S_PCM_CTRL1, 0x000b },
	{ NAU8825_REG_I2S_PCM_CTRL2, 0x8010 },
	{ NAU8825_REG_LEFT_TIME_SLOT, 0x0 },
	{ NAU8825_REG_RIGHT_TIME_SLOT, 0x0 },
	{ NAU8825_REG_BIQ_CTRL, 0x0 },
	{ NAU8825_REG_BIQ_COF1, 0x0 },
	{ NAU8825_REG_BIQ_COF2, 0x0 },
	{ NAU8825_REG_BIQ_COF3, 0x0 },
	{ NAU8825_REG_BIQ_COF4, 0x0 },
	{ NAU8825_REG_BIQ_COF5, 0x0 },
	{ NAU8825_REG_BIQ_COF6, 0x0 },
	{ NAU8825_REG_BIQ_COF7, 0x0 },
	{ NAU8825_REG_BIQ_COF8, 0x0 },
	{ NAU8825_REG_BIQ_COF9, 0x0 },
	{ NAU8825_REG_BIQ_COF10, 0x0 },
	{ NAU8825_REG_ADC_RATE, 0x0010 },
	{ NAU8825_REG_DAC_CTRL1, 0x0001 },
	{ NAU8825_REG_DAC_CTRL2, 0x0 },
	{ NAU8825_REG_DAC_DGAIN_CTRL, 0x0 },
	{ NAU8825_REG_ADC_DGAIN_CTRL, 0x00cf },
	{ NAU8825_REG_MUTE_CTRL, 0x0 },
	{ NAU8825_REG_HSVOL_CTRL, 0x0 },
	{ NAU8825_REG_DACL_CTRL, 0x02cf },
	{ NAU8825_REG_DACR_CTRL, 0x00cf },
	{ NAU8825_REG_ADC_DRC_KNEE_IP12, 0x1486 },
	{ NAU8825_REG_ADC_DRC_KNEE_IP34, 0x0f12 },
	{ NAU8825_REG_ADC_DRC_SLOPES, 0x25ff },
	{ NAU8825_REG_ADC_DRC_ATKDCY, 0x3457 },
	{ NAU8825_REG_DAC_DRC_KNEE_IP12, 0x1486 },
	{ NAU8825_REG_DAC_DRC_KNEE_IP34, 0x0f12 },
	{ NAU8825_REG_DAC_DRC_SLOPES, 0x25f9 },
	{ NAU8825_REG_DAC_DRC_ATKDCY, 0x3457 },
	{ NAU8825_REG_IMM_MODE_CTRL, 0x0 },
	{ NAU8825_REG_CLASSG_CTRL, 0x0 },
	{ NAU8825_REG_OPT_EFUSE_CTRL, 0x0 },
	{ NAU8825_REG_MISC_CTRL, 0x0 },
	{ NAU8825_REG_BIAS_ADJ, 0x0 },
	{ NAU8825_REG_TRIM_SETTINGS, 0x0 },
	{ NAU8825_REG_ANALOG_CONTROL_1, 0x0 },
	{ NAU8825_REG_ANALOG_CONTROL_2, 0x0 },
	{ NAU8825_REG_ANALOG_ADC_1, 0x0011 },
	{ NAU8825_REG_ANALOG_ADC_2, 0x0020 },
	{ NAU8825_REG_RDAC, 0x0008 },
	{ NAU8825_REG_MIC_BIAS, 0x0006 },
	{ NAU8825_REG_BOOST, 0x0 },
	{ NAU8825_REG_FEPGA, 0x0 },
	{ NAU8825_REG_POWER_UP_CONTROL, 0x0 },
	{ NAU8825_REG_CHARGE_PUMP_AND_POWER_DOWN_CONTROL, 0x0 },
};

static bool nau8825_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8825_REG_ENA_CTRL:
	case NAU8825_REG_CLK_DIVIDER ... NAU8825_REG_FLL_VCO_RSV:
	case NAU8825_REG_HSD_CTRL ... NAU8825_REG_JACK_DET_CTRL:
	case NAU8825_REG_INTERRUPT_MASK ... NAU8825_REG_KEYDET_CTRL:
	case NAU8825_REG_VDET_THRESHOLD_1 ... NAU8825_REG_DACR_CTRL:
	case NAU8825_REG_ADC_DRC_KNEE_IP12 ... NAU8825_REG_ADC_DRC_ATKDCY:
	case NAU8825_REG_DAC_DRC_KNEE_IP12 ... NAU8825_REG_DAC_DRC_ATKDCY:
	case NAU8825_REG_IMM_MODE_CTRL ... NAU8825_REG_IMM_RMS_R:
	case NAU8825_REG_CLASSG_CTRL ... NAU8825_REG_OPT_EFUSE_CTRL:
	case NAU8825_REG_MISC_CTRL:
	case NAU8825_REG_I2C_DEVICE_ID ... NAU8825_REG_SARDOUT_RAM_STATUS:
	case NAU8825_REG_BIAS_ADJ:
	case NAU8825_REG_TRIM_SETTINGS ... NAU8825_REG_ANALOG_CONTROL_2:
	case NAU8825_REG_ANALOG_ADC_1 ... NAU8825_REG_MIC_BIAS:
	case NAU8825_REG_BOOST ... NAU8825_REG_FEPGA:
	case NAU8825_REG_POWER_UP_CONTROL ... NAU8825_REG_GENERAL_STATUS:
		return true;
	default:
		return false;
	}

}

static bool nau8825_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8825_REG_RESET ... NAU8825_REG_ENA_CTRL:
	case NAU8825_REG_CLK_DIVIDER ... NAU8825_REG_FLL_VCO_RSV:
	case NAU8825_REG_HSD_CTRL ... NAU8825_REG_JACK_DET_CTRL:
	case NAU8825_REG_INTERRUPT_MASK:
	case NAU8825_REG_INT_CLR_KEY_STATUS ... NAU8825_REG_KEYDET_CTRL:
	case NAU8825_REG_VDET_THRESHOLD_1 ... NAU8825_REG_DACR_CTRL:
	case NAU8825_REG_ADC_DRC_KNEE_IP12 ... NAU8825_REG_ADC_DRC_ATKDCY:
	case NAU8825_REG_DAC_DRC_KNEE_IP12 ... NAU8825_REG_DAC_DRC_ATKDCY:
	case NAU8825_REG_IMM_MODE_CTRL:
	case NAU8825_REG_CLASSG_CTRL ... NAU8825_REG_OPT_EFUSE_CTRL:
	case NAU8825_REG_MISC_CTRL:
	case NAU8825_REG_BIAS_ADJ:
	case NAU8825_REG_TRIM_SETTINGS ... NAU8825_REG_ANALOG_CONTROL_2:
	case NAU8825_REG_ANALOG_ADC_1 ... NAU8825_REG_MIC_BIAS:
	case NAU8825_REG_BOOST ... NAU8825_REG_FEPGA:
	case NAU8825_REG_POWER_UP_CONTROL ... NAU8825_REG_CHARGE_PUMP_AND_POWER_DOWN_CONTROL:
		return true;
	default:
		return false;
	}
}

static bool nau8825_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case NAU8825_REG_RESET:
	case NAU8825_REG_IRQ_STATUS:
	case NAU8825_REG_INT_CLR_KEY_STATUS:
	case NAU8825_REG_IMM_RMS_L:
	case NAU8825_REG_IMM_RMS_R:
	case NAU8825_REG_I2C_DEVICE_ID:
	case NAU8825_REG_SARDOUT_RAM_STATUS:
	case NAU8825_REG_CHARGE_PUMP_INPUT_READ:
	case NAU8825_REG_GENERAL_STATUS:
		return true;
	default:
		return false;
	}
}

static const DECLARE_TLV_DB_MINMAX_MUTE(adc_vol_tlv, -10300, 2400);
static const DECLARE_TLV_DB_MINMAX_MUTE(sidetone_vol_tlv, -4200, 0);
static const DECLARE_TLV_DB_MINMAX(dac_vol_tlv, -5400, 0);
//static const DECLARE_TLV_DB_MINMAX_MUTE(dac_crosstalk_vol_tlv, -9650, 2400);
static const DECLARE_TLV_DB_MINMAX(fepga_gain_tlv, -100, 3600);

static const struct snd_kcontrol_new nau8825_snd_controls[] = {
	SOC_SINGLE_TLV("MIC Volume", NAU8825_REG_ADC_DGAIN_CTRL,
		0, 0xff, 0, adc_vol_tlv),
	SOC_DOUBLE_TLV("HP Sidetone Volume", NAU8825_REG_ADC_DGAIN_CTRL,
		12, 8, 0x0f, 0, sidetone_vol_tlv),
	SOC_DOUBLE_TLV("HP Volume", NAU8825_REG_HSVOL_CTRL,
		6, 0, 0x3f, 1, dac_vol_tlv),
	SOC_SINGLE_TLV("Frontend PGA Gain", NAU8825_REG_POWER_UP_CONTROL,
		8, 37, 0, fepga_gain_tlv), // does it work? Ask Nuvoton
};

static int charge_pump_event(struct snd_soc_dapm_widget *w,
        struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct nau8825 *nau8825 = snd_soc_codec_get_drvdata(codec);
	struct regmap *regmap = nau8825->regmap;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		// 0x0080[10]=0x1
		regmap_update_bits(regmap, NAU8825_REG_CHARGE_PUMP_AND_POWER_DOWN_CONTROL, NAU8825_JAMNODCLOW, NAU8825_JAMNODCLOW);
	} else {
		regmap_update_bits(regmap, NAU8825_REG_CHARGE_PUMP_AND_POWER_DOWN_CONTROL, NAU8825_JAMNODCLOW, 0);
	}

	return 0;
}

static int output_driver_event(struct snd_soc_dapm_widget *w,
        struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct nau8825 *nau8825 = snd_soc_codec_get_drvdata(codec);
	struct regmap *regmap = nau8825->regmap;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		// Power Up L and R Output Drivers
		regmap_update_bits(regmap, NAU8825_REG_POWER_UP_CONTROL, 0x3c, 0x3c);
		// Power up Main Output Drivers (The main driver must be turned on after the predriver to avoid pops)
		regmap_update_bits(regmap, NAU8825_REG_POWER_UP_CONTROL, 0x3, 0x3);
	} else {
		// Power Down L and R Output Drivers
		regmap_update_bits(regmap, NAU8825_REG_POWER_UP_CONTROL, 0x3f, 0x0);
	}

	return 0;
}

static const struct snd_soc_dapm_widget nau8825_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("MIC"),
	SND_SOC_DAPM_MICBIAS("MICBIAS", NAU8825_REG_MIC_BIAS, 8, 0),

	SND_SOC_DAPM_PGA("Frontend PGA", NAU8825_REG_POWER_UP_CONTROL, 14, 0, NULL, 0), // if there is a record pop then try "mute 0x0066[4]=0x1	before capture and unmute after".

	SND_SOC_DAPM_ADC("ADC", NULL, NAU8825_REG_ENA_CTRL, 8, 0),
	SND_SOC_DAPM_SUPPLY("ADC Clock", NAU8825_REG_ENA_CTRL, 7, 0, NULL, 0),

	// ADC for button press detection
	SND_SOC_DAPM_ADC("SAR", NULL, NAU8825_REG_SAR_CTRL, NAU8825_SAR_ADC_EN_SFT, 0),

	SND_SOC_DAPM_DAC("ADACL", NULL, NAU8825_REG_RDAC, 12, 0),
	SND_SOC_DAPM_DAC("ADACR", NULL, NAU8825_REG_RDAC, 13, 0),
	SND_SOC_DAPM_SUPPLY("ADACL Clock", NAU8825_REG_RDAC, 8, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADACR Clock", NAU8825_REG_RDAC, 9, 0, NULL, 0),

	SND_SOC_DAPM_DAC("DDACR", NULL, NAU8825_REG_ENA_CTRL, NAU8825_ENABLE_DACR_SFT, 0),
	SND_SOC_DAPM_DAC("DDACL", NULL, NAU8825_REG_ENA_CTRL, NAU8825_ENABLE_DACL_SFT, 0),
	SND_SOC_DAPM_SUPPLY("DDAC Clock", NAU8825_REG_ENA_CTRL, 6, 0, NULL, 0),

	SND_SOC_DAPM_PGA("HP amp L", NAU8825_REG_CLASSG_CTRL, 1, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HP amp R", NAU8825_REG_CLASSG_CTRL, 2, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("HP amp power", NAU8825_REG_CLASSG_CTRL, 0, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("Change Pump", NAU8825_REG_CHARGE_PUMP_AND_POWER_DOWN_CONTROL, 5, 0, charge_pump_event, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	// Datasheet says 'power down disable'. What exactly block is powers down? Ask Nuvoton
	SND_SOC_DAPM_SUPPLY("DACL Power", NAU8825_REG_CHARGE_PUMP_AND_POWER_DOWN_CONTROL, 8, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DACR Power", NAU8825_REG_CHARGE_PUMP_AND_POWER_DOWN_CONTROL, 9, 1, NULL, 0),

	// it is possible to turn on/off driver for each channel separately. But why would want to do this? Ask upstream?
	SND_SOC_DAPM_OUT_DRV_E("Output Driver", SND_SOC_NOPM, 0, 0, NULL, 0, output_driver_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_OUTPUT("HPOL"),
	SND_SOC_DAPM_OUTPUT("HPOR"),

	// mixers and switches see 0x33 0x34
};

static const struct snd_soc_dapm_route nau8825_dapm_routes[] = {
	{"Frontend PGA", NULL, "MIC"},
	{"ADC", NULL, "Frontend PGA"},
	{"ADC", NULL, "ADC Clock"},
	{"Capture", NULL, "ADC"},

	{"DDACL", NULL, "Playback"},
	{"DDACR", NULL, "Playback"},
	{"DDACL", NULL, "DDAC Clock"},
	{"DDACR", NULL, "DDAC Clock"},
	{"DDACL", NULL, "DACL Power"},
	{"DDACR", NULL, "DACR Power"},
	{"HP amp L", NULL, "Change Pump"},
	{"HP amp R", NULL, "Change Pump"},
	{"HP amp L", NULL, "DDACL"},
	{"HP amp R", NULL, "DDACR"},
	{"HP amp L", NULL, "HP amp power"},
	{"HP amp R", NULL, "HP amp power"},
	{"ADACL", NULL, "HP amp L"},
	{"ADACR", NULL, "HP amp R"},
	{"ADACL", NULL, "ADACL Clock"},
	{"ADACR", NULL, "ADACR Clock"},
	{"Output Driver", NULL, "ADACL"},
	{"Output Driver", NULL, "ADACR"},
	{"HPOL", NULL, "Output Driver"},
	{"HPOR", NULL, "Output Driver"},
};

static int nau8825_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct nau8825 *nau8825 = snd_soc_codec_get_drvdata(codec);
	unsigned int val_len = 0;

	switch (params_width(params)) {
	case 16:
		val_len |= NAU8825_I2S_DL_16;
		break;
	case 20:
		val_len |= NAU8825_I2S_DL_20;
		break;
	case 24:
		val_len |= NAU8825_I2S_DL_24;
		break;
	case 32:
		val_len |= NAU8825_I2S_DL_32;
		break;
	default:
		return -EINVAL;
	}
	regmap_update_bits(nau8825->regmap, NAU8825_REG_I2S_PCM_CTRL1,
		NAU8825_I2S_DL_MASK, val_len);

	/* set ADC_RATE:SAMPL_RATE according to params? */
	return 0;
}

static int nau8825_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct nau8825 *nau8825 = snd_soc_codec_get_drvdata(codec);
	int reg_val;

	// what is better to use for digital mute this, or disabling output driver?
	reg_val = mute ? NAU8825_HP_MUTE : 0;
	regmap_update_bits(nau8825->regmap, NAU8825_REG_HSVOL_CTRL,
		NAU8825_HP_MUTE, reg_val);

	return 0;
}

static int nau8825_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct nau8825 *nau8825 = snd_soc_codec_get_drvdata(codec);
	unsigned int ctrl1_val = 0, ctrl2_val = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		ctrl2_val |= NAU8825_I2S_MS_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		ctrl1_val |= NAU8825_I2S_BP_INV;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		ctrl1_val |= NAU8825_I2S_DF_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		ctrl1_val |= NAU8825_I2S_DF_LEFT;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		ctrl1_val |= NAU8825_I2S_DF_RIGTH;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		ctrl1_val |= NAU8825_I2S_DF_PCM_AB;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		ctrl1_val |= NAU8825_I2S_DF_PCM_AB;
		ctrl1_val |= NAU8825_I2S_PCMB_EN;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(nau8825->regmap, NAU8825_REG_I2S_PCM_CTRL1,
		NAU8825_I2S_DL_MASK | NAU8825_I2S_DF_MASK |
		NAU8825_I2S_BP_MASK | NAU8825_I2S_PCMB_MASK,
		ctrl1_val);
	regmap_update_bits(nau8825->regmap, NAU8825_REG_I2S_PCM_CTRL2,
		NAU8825_I2S_MS_MASK, ctrl2_val);

	return 0;
}

static int nau8825_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	//struct nau8825 *nau8825 = snd_soc_codec_get_drvdata(codec);

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (snd_soc_codec_get_bias_level(codec) != SND_SOC_BIAS_OFF)
			break;
/*		regmap_update_bits(nau8825->regmap, NAU8825_REG_BIAS_ADJ,
			NAU8825_BIAS_VMID, NAU8825_BIAS_VMID);
		msleep(2);
		regmap_update_bits(nau8825->regmap, NAU8825_REG_BOOST,
			NAU8825_BOOST_BIAS_EN, NAU8825_BOOST_BIAS_EN);
		regmap_update_bits(nau8825->regmap, NAU8825_REG_BOOST,
			NAU8825_PRECHARGE_DIS, NAU8825_PRECHARGE_DIS);
*/		break;
	case SND_SOC_BIAS_OFF:
/*		regmap_update_bits(nau8825->regmap, NAU8825_REG_BOOST,
			NAU8825_PRECHARGE_DIS, 0);
		regmap_update_bits(nau8825->regmap, NAU8825_REG_BOOST,
			NAU8825_BOOST_BIAS_EN, 0);
		regmap_update_bits(nau8825->regmap, NAU8825_REG_BIAS_ADJ,
			NAU8825_BIAS_VMID, 0);
*/		break;
	}

	/* following line removed upstream */
	codec->dapm.bias_level = level;
	return 0;
}

static const struct snd_soc_dai_ops nau8825_dai_ops = {
	.hw_params	= nau8825_hw_params,
	.digital_mute	= nau8825_mute,
	.set_fmt	= nau8825_set_dai_fmt,
};

#define NAU8825_RATES	SNDRV_PCM_RATE_8000_192000
#define NAU8825_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE \
			 | SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver nau8825_dai = {
	.name = "nau8825-hifi",
	.playback = {
		.stream_name	 = "Playback",
		.channels_min	 = 1,
		.channels_max	 = 2,
		.rates		 = NAU8825_RATES,
		.formats	 = NAU8825_FORMATS,
	},
	.capture = {
		.stream_name	 = "Capture",
		.channels_min	 = 1,
		.channels_max	 = 2,
		.rates		 = NAU8825_RATES,
		.formats	 = NAU8825_FORMATS,
	},
	.ops = &nau8825_dai_ops,
};

/**
 * nau8825_enable_jack_detect - Specify a jack for event reporting
 *
 * @component:  component to register the jack with
 * @jack: jack to use to report headset and button events on
 *
 * After this function has been called the headset insert/remove and button
 * events will be routed to the given jack.  Jack can be null to stop
 * reporting.
 */
int nau8825_enable_jack_detect(struct snd_soc_codec *codec,
				struct snd_soc_jack *jack)
{
	struct nau8825 *nau8825 = snd_soc_codec_get_drvdata(codec);

	snd_jack_set_key(jack->jack, SND_JACK_BTN_0, KEY_MEDIA);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_1, KEY_VOICECOMMAND);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_2, KEY_VOLUMEUP);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_3, KEY_VOLUMEDOWN);

	nau8825->jack = jack;

	return 0;
}
EXPORT_SYMBOL_GPL(nau8825_enable_jack_detect);


static bool nau8825_is_jack_inserted(struct regmap *regmap)
{
	int status;
	regmap_read(regmap, NAU8825_REG_I2C_DEVICE_ID, &status);
	return !(status & NAU8825_GPIO2JD1);
}

static void nau8825_restart_jack_detection(struct regmap *regmap)
{
	/* Restart jack detection debounce (this will restart the entire jack detection process including MIC/GND switching and create interrupts) */
	regmap_update_bits(regmap, NAU8825_REG_JACK_DET_CTRL, NAU8825_JACK_DET_RESTART, NAU8825_JACK_DET_RESTART);
	/* You have to go from 0 to 1 and back to 0 to restart */
	regmap_update_bits(regmap, NAU8825_REG_JACK_DET_CTRL, NAU8825_JACK_DET_RESTART, 0);
}

static void nau8825_eject_jack(struct nau8825 *nau8825)
{
	struct snd_soc_dapm_context *dapm = nau8825->dapm;
	struct regmap *regmap = nau8825->regmap;
/*
0x0013[12]=0x0	//Disable SAR ADC
0x0074[8]=0x0	//Disable MICBIAS
0x0074[14:12]=0x0	//Disconnect MICBIAS from jack pins
0x000C[1:0]=0x3	//Ground HPL/R
0x000C[3:2]=0x3	//Ground MICGND1/2
0x0080[9:8]=0x3	//Power Down DACs
0x0080[5]=0x0	//Disable Chargepump
0x0080[10]=0x0	//Disable Improve Chargepump
0x0050[2:0]=0x0	//Disable Class G and Comparators
0x0073[9:8]=0x0	//Disable DAC Clocks
0x0073[13:12]=0x0	//Disable DACs
0x007F[14]=0x0	//Disable PGA
0x007F[5:0]=0x0	//Disable all output drivers
0x0001[15:0]=0x416	//Reset Clocks
0x0030[7:0]=0xD2	//Reset ADC DGAIN
0x000F[15:0]=0x801	//Unmask all interrupts but Insert (We are looking for headset detect complete)
*/

	snd_soc_dapm_disable_pin(dapm, "SAR");
	snd_soc_dapm_disable_pin(dapm, "MICBIAS");
	// Detach 2kOhm Resistors from MICBIAS to MICGND1/2
	regmap_update_bits(regmap, NAU8825_REG_MIC_BIAS, 1 << 14 | 1 << 12, 0);
	// ground HPL/HPR, MICGRND1/2
	regmap_update_bits(regmap, NAU8825_REG_HSD_CTRL, 0xf, 0xf);

	snd_soc_dapm_sync(dapm);
}

static int nau8825_button_decode(int value) {
	/* NAU8825 supports up to 8 buttons configuration.
	   Last 8 bits of 'value' represent buttons */
	int buttons;
	if (value & BIT(0))
		buttons |= SND_JACK_BTN_0;
	if (value & BIT(1))
		buttons |= SND_JACK_BTN_1;
	if (value & BIT(2))
		buttons |= SND_JACK_BTN_2;
	if (value & BIT(3))
		buttons |= SND_JACK_BTN_3;

	return buttons;
}

static irqreturn_t nau8825_interrupt(int irq, void *data)
{
	struct nau8825 *nau8825 = (struct nau8825 *)data;
	struct regmap *regmap = nau8825->regmap;
	struct snd_soc_dapm_context *dapm = nau8825->dapm;
	int irq_status, event = 0, event_mask = 0;

	regmap_read(regmap, NAU8825_REG_IRQ_STATUS, &irq_status);
	/* clears the rightmost interruption */
	regmap_write(regmap, NAU8825_REG_INT_CLR_KEY_STATUS, irq_status);

	if (irq_status & NAU8825_JACK_INSERTION_IRQ_MASK) {
		// ignore NAU8825_JACK_INSERTION_DETECTED as it will handle in headset completion
	} else if (irq_status & NAU8825_JACK_EJECTION_IRQ_MASK) {
		// Ask Nuvoton: will chip receive several interruptions in case of eject-then-insert event?
		if ((irq_status & NAU8825_JACK_EJECTION_IRQ_MASK) == NAU8825_JACK_EJECTION_DETECTED) {
			nau8825_eject_jack(nau8825);
			event_mask |= SND_JACK_HEADSET;
		}
	} else if (irq_status & NAU8825_KEY_SHORT_PRESS_IRQ) {
		int key_status;
		regmap_read(regmap, NAU8825_REG_INT_CLR_KEY_STATUS, &key_status);

		/* upper 8 bits of the register are for short pressed keys,
		   low 8 bits - for long pressed buttons */
		nau8825->button_pressed = nau8825_button_decode(key_status >> 8);
		nau8825->long_press = false;

		event |= nau8825->button_pressed;
		event_mask |= SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2 | SND_JACK_BTN_3;
	} else if (irq_status & NAU8825_KEY_LONG_PRESS_IRQ) {
		int key_status;
		regmap_read(regmap, NAU8825_REG_INT_CLR_KEY_STATUS, &key_status);

		nau8825->button_pressed = nau8825_button_decode(key_status);
		nau8825->long_press = true;
	} else if (irq_status & NAU8825_KEY_RELEASE_IRQ) {
		event_mask |= SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2 | SND_JACK_BTN_3;
	} else if (irq_status & NAU8825_IMPEDANCE_MEAS_IRQ) {
		// ignore
	} else if (irq_status & NAU8825_SHORT_CIRCUIT_IRQ) {
		// ignore
	} else if (irq_status & NAU8825_HEADSET_COMPLETION_IRQ) {
		event_mask |= SND_JACK_HEADSET;

		if (nau8825_is_jack_inserted(regmap)) {
			int jack_status_reg, mic_detected;
			regmap_read(regmap, NAU8825_REG_GENERAL_STATUS, &jack_status_reg);
			mic_detected = (jack_status_reg >> 10) & 3;

			switch(mic_detected) {
				case 0:
					// no mic
					event |= SND_JACK_HEADPHONE;
					break;
				case 1:
					pr_info("nau8825: OMTP (micgnd1) mic connected\n");
					event |= SND_JACK_HEADSET;

					// Unground MICGND1
					regmap_update_bits(regmap, NAU8825_REG_HSD_CTRL, 3 << 2, 1 << 2);
					// Unground HPL/R
					regmap_update_bits(regmap, NAU8825_REG_HSD_CTRL, 0x3, 0);
					// Attach 2kOhm Resistor from MICBIAS to MICGND1
					regmap_update_bits(regmap, NAU8825_REG_MIC_BIAS, 1 << 14 | 1 << 12, 1 << 12);
					// Attach SARADC to MICGND1
					regmap_update_bits(regmap, NAU8825_REG_SAR_CTRL, 1 << 11, 0);

					snd_soc_dapm_force_enable_pin(dapm, "MICBIAS");
					snd_soc_dapm_force_enable_pin(dapm, "SAR");
					snd_soc_dapm_sync(dapm);
					break;
				case 2:
					pr_info("nau8825: CTIA (micgnd2) mic connected\n");
					event |= SND_JACK_HEADSET;

					// Unground MICGND2
					regmap_update_bits(regmap, NAU8825_REG_HSD_CTRL, 3 << 2, 2 << 2);
					// Unground HPL/R
					regmap_update_bits(regmap, NAU8825_REG_HSD_CTRL, 0x3, 0);
					// Attach 2kOhm Resistor from MICBIAS to MICGND2
					regmap_update_bits(regmap, NAU8825_REG_MIC_BIAS, 1 << 14 | 1 << 12, 1 << 14);
					// Attach SARADC to MICGND2
					regmap_update_bits(regmap, NAU8825_REG_SAR_CTRL, 1 << 11, 1 << 11);

					snd_soc_dapm_force_enable_pin(dapm, "MICBIAS");
					snd_soc_dapm_force_enable_pin(dapm, "SAR");
					snd_soc_dapm_sync(dapm);
					break;
				case 3:
					// error
					pr_warn("nau8825: Both MICGND1 and MICGND2 are connected to MICBIAS\n");
					nau8825_restart_jack_detection(regmap);
					// Ask Nuvoton: Bose Quiet Comfort 15 hits this case. We want better way to recognize high-impedance input
					break;
			}
		} else {
			pr_warn("nau8825: Headset completion IRQ fired but no headset connected\n");
			nau8825_eject_jack(nau8825);
		}
	}

	pr_err("IRQ value = 0x%x event = 0x%x, mask = 0x%x\n", irq_status, event, event_mask);
	if (event_mask)
		snd_soc_jack_report(nau8825->jack, event, event_mask);

	return IRQ_HANDLED;
}

static void nau8825_setup_buttons(struct regmap *regmap)
{
	/* Setup Button Detect (Debounce, number of buttons, and Hysteresis) */
	regmap_write(regmap, NAU8825_REG_KEYDET_CTRL, 0x7311);

	/* Setup 4 buttons impedane according to Android specification
	 * https://source.android.com/accessories/headset-spec.html
	 * Button 0 - 0-70 Ohm
	 * Button 1 - 110-180 Ohm
	 * Button 2 - 210-290 Ohm
	 * Button 3 - 360-680 Ohm
	 */
	regmap_write(regmap, NAU8825_REG_VDET_THRESHOLD_1, 0x0f1f);
	regmap_write(regmap, NAU8825_REG_VDET_THRESHOLD_2, 0x325f);
}

static void nau8825_init_regs(struct regmap *regmap)
{
/*
0080   0300  //Power down DACs for power savings
0031   1000  //DAC Zero Crossing Enable
0066   0060  //VMID Enable and Tieoff
0076   3140  //Analog Bias Enable, Disable Boost Driver, Automatic Short circuit protection enable
000C   004F  //Ground HP Outputs[1:0], Enable Automatic Mic/Gnd switching reading on insert interrupt[6]
000D   01E0  //This can be removed as reg0xD is set latter
000F   0801  //IRQ Output Enable, Mask Inert Interrupt (we want to look for the Headset Detect Complete Interrupt as this will signal when we can read regx82[11:10] to see the MIC/GND status)
001A   0800  //Jack Detect pull up
0013   0280  //Setup SAR ADC
0014   7310  //Setup Button Detect (Debounce, number of buttons, and Hysteresis)
0015   0513  //Setup Button Detect (Button Values)
0016   2534  //Setup Button Detect (Button Values)
001C   000E  //Setup I2S
002B   0002  //Setup ADC x128 OSR
002C   0082  //Setup DAC x128 OSR
0030   00D2  //Setup ADC dgain adjustment
0033   00CF  //Setup LDAC path and dgain adjustment (this may be able to be dropped entirely)
0034   02CF  //Setup RDAC path and dgain adjustment
006A   1003  //DAC bias settings
0072   0260  //ADC Bias Settings
000D   00E0  //Maximize Jack Insert Debounce
0001   0416  what this step does? Does it try to enable some needed clocks or disable unneeded? //It does both, default is 0xFF and [10] is needed the rest are not
001D   2019  //Our chip needs one FSCLK cycle in order to generate interrupts, as we cannot guarantee one will be provided by the system turning master mode on then off enables us to generate that FSCLK cycle with a minimum of contention on the clock bus. Enables master mode and correctly divides BCLK and FSCLK from MCLK
001D   0010  //Disable Master mode and dividers
0012   0010  //Unmask all interrupts but MIC Detect (default =xFFFF), we will read x82[11:10] later on to figure out if a MIC is attached so we do not need the additional interrupt
*/

	/* Power down DACs for power savings */
	regmap_write(regmap, NAU8825_REG_CHARGE_PUMP_AND_POWER_DOWN_CONTROL, 0x0300);
	/* DAC Zero Crossing Enable */
	regmap_write(regmap, NAU8825_REG_MUTE_CTRL, 0x1000);
	/* VMID Enable and Tieoff */
	regmap_write(regmap, NAU8825_REG_BIAS_ADJ, 0x0060);
	/* Analog Bias Enable, Disable Boost Driver, Automatic Short circuit protection enable */
	regmap_write(regmap, NAU8825_REG_BOOST, 0x3140); // coudld we move part of it to set_bias?
	/* Ground HP Outputs[1:0], Enable Automatic Mic/Gnd switching reading on insert interrupt[6] */
	regmap_write(regmap, NAU8825_REG_HSD_CTRL, 0x004f);

	regmap_write(regmap, NAU8825_REG_JACK_DET_CTRL, 0x01e0);
	/* IRQ Output Enable, Mask Insert Interrupt (we want to look for the Headset Detect Complete Interrupt as this will signal when we can read regx82[11:10] to see the MIC/GND status) */
	regmap_write(regmap, NAU8825_REG_INTERRUPT_MASK, 0x0801);
	/* Jack Detect pull up (High=eject, Low=insert) */
	regmap_write(regmap, NAU8825_REG_GPIO12_CTRL, 0x0800);
	/* Setup SAR ADC */
	regmap_write(regmap, NAU8825_REG_SAR_CTRL, 0x0280);
	nau8825_setup_buttons(regmap);

	/* Setup ADC x128 OSR */
	regmap_write(regmap, NAU8825_REG_ADC_RATE, 0x0002);
	/* Setup DAC x128 OSR */
	regmap_write(regmap, NAU8825_REG_DAC_CTRL1, 0x0082);
	/* DAC bias settings */
	regmap_write(regmap, NAU8825_REG_ANALOG_CONTROL_2, 0x1003);
	/* ADC Bias Settings */
	regmap_write(regmap, NAU8825_REG_ANALOG_ADC_2, 0x0260);
	/* Maximize Jack Insert Debounce */
	regmap_write(regmap, NAU8825_REG_JACK_DET_CTRL, 0x00e0);

	/* Setup clocks */
	regmap_write(regmap, NAU8825_REG_ENA_CTRL, 0x0416);

	/* chip needs one FSCLK cycle in order to generate interrupts,
	   as we cannot guarantee one will be provided by the system turning
	   master mode on then off enables us to generate that FSCLK cycle
	   with a minimum of contention on the clock bus.
	   Enables master mode and correctly divides BCLK and FSCLK from MCLK.

	   Disable tri-state I2S.
	*/
	regmap_write(regmap, NAU8825_REG_I2S_PCM_CTRL2, 0x2019);
	regmap_write(regmap, NAU8825_REG_I2S_PCM_CTRL2, 0x0010);

	/* Unmask all interrupts but MIC Detect (default =xFFFF), we will read x82[11:10] later on to figure out if a MIC is attached so we do not need the additional interrupt */
	regmap_write(regmap, NAU8825_REG_INTERRUPT_DIS_CTRL, 0x0011);
}

static const struct regmap_config nau8825_regmap_config = {
	.val_bits = 16,
	.reg_bits = 16,

	.max_register = NAU8825_REG_MAX,
	.readable_reg = nau8825_readable_reg,
	.writeable_reg = nau8825_writeable_reg,
	.volatile_reg = nau8825_volatile_reg,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = nau8825_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(nau8825_reg_defaults),
};

static int nau8825_codec_probe(struct snd_soc_codec *codec)
{
	struct nau8825 *nau8825 = snd_soc_codec_get_drvdata(codec);
	struct device *dev = codec->dev;

	nau8825->dapm = &codec->dapm;

	nau8825_init_regs(nau8825->regmap);
	snd_soc_dapm_force_enable_pin(&codec->dapm, "DDACR");
	snd_soc_dapm_sync(&codec->dapm);

	/* IRQ initialization requires MCLK signal enabled, thus we can do it only in codec probe */
	if (nau8825->irq) {
		int ret = devm_request_threaded_irq(dev, nau8825->irq, NULL, nau8825_interrupt,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			"nau8825", nau8825);

		if (ret) {
			dev_err(dev, "Cannot request irq %d (%d)\n", nau8825->irq, ret);
			return ret;
		}
	}
	nau8825_restart_jack_detection(nau8825->regmap);

	return 0;
}

static struct snd_soc_codec_driver nau8825_codec_driver = {
	.probe = nau8825_codec_probe,
	.set_bias_level = nau8825_set_bias_level,
	.idle_bias_off = true,

	.controls = nau8825_snd_controls,
	.num_controls = ARRAY_SIZE(nau8825_snd_controls),
	.dapm_widgets = nau8825_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(nau8825_dapm_widgets),
	.dapm_routes = nau8825_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(nau8825_dapm_routes),
};

static void nau8825_reset_chip(struct regmap *regmap)
{
	regmap_write(regmap, NAU8825_REG_RESET, 0x00);
	regmap_write(regmap, NAU8825_REG_RESET, 0x00);
}

static int nau8825_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	struct device *dev = &i2c->dev;
	struct nau8825 *nau8825;
	int ret, value;

	nau8825 = devm_kzalloc(dev, sizeof(*nau8825), GFP_KERNEL);
	if (!nau8825)
		return -ENOMEM;

	i2c_set_clientdata(i2c, nau8825);

	nau8825->regmap = devm_regmap_init_i2c(i2c, &nau8825_regmap_config);
	if (IS_ERR(nau8825->regmap))
		return PTR_ERR(nau8825->regmap);
	nau8825->irq = i2c->irq;

	nau8825_reset_chip(nau8825->regmap);
	ret = regmap_read(nau8825->regmap, NAU8825_REG_I2C_DEVICE_ID, &value);
	if (ret < 0) {
		dev_err(dev, "Failed to read device id from the NAU8825: %d\n",
			ret);
		return ret;
	}
	if ((value & NAU8825_SOFTWARE_ID_MASK) !=
			NAU8825_SOFTWARE_ID_NAU8825) {
		dev_err(dev, "Not a NAU8825 chip\n");
		return -ENODEV;
	}

	return snd_soc_register_codec(&i2c->dev, &nau8825_codec_driver,
			&nau8825_dai, 1);
}

static int nau8825_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct i2c_device_id nau8825_i2c_ids[] = {
	{ "nau8825", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nau8825_i2c_ids);

static struct i2c_driver nau8825_driver = {
	.driver = {
		.name = "nau8825",
		.owner = THIS_MODULE,
	},
	.probe = nau8825_i2c_probe,
	.remove = nau8825_i2c_remove,
	.id_table = nau8825_i2c_ids,
};
module_i2c_driver(nau8825_driver);

MODULE_DESCRIPTION("ASoC nau8825 driver");
MODULE_AUTHOR("Anatol Pomozov <anatol@chromium.org>");
MODULE_LICENSE("GPL");
