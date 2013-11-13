/*
 * ad73311.c  --  ALSA Soc AD73311 codec support
 *
 * Copyright:	Analog Device Inc.
 * Author:	Cliff Cai <cliff.cai@analog.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include "ad73311.h"

static const struct snd_soc_dapm_widget ad73311_dapm_widgets[] = {
SND_SOC_DAPM_INPUT("VINP"),
SND_SOC_DAPM_INPUT("VINN"),
SND_SOC_DAPM_OUTPUT("VOUTN"),
SND_SOC_DAPM_OUTPUT("VOUTP"),
};

static const struct snd_soc_dapm_route ad73311_dapm_routes[] = {
	{ "Capture", NULL, "VINP" },
	{ "Capture", NULL, "VINN" },

	{ "VOUTN", NULL, "Playback" },
	{ "VOUTP", NULL, "Playback" },
};

static struct snd_soc_dai_driver ad73311_dai = {
	.name = "ad73311-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE, },
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE, },
};

static struct ad73311_snd_ctrls ad73311_ctrls = {
	.dirate = 0,
	.igs = 2,
	.ogs = 2,
	.se_en = 1,
};

static int ad73311_ogs_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	if (ad73311_ctrls.ogs != ucontrol->value.integer.value[0]) {
		ad73311_ctrls.ogs = ucontrol->value.integer.value[0];
		return codec->hw_write(codec->control_data, (char *)&ad73311_ctrls,
				sizeof(ad73311_ctrls));
	}
	return 0;
}

static int ad73311_ogs_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = ad73311_ctrls.ogs;
	return 0;
}

static int ad73311_igs_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	if (ad73311_ctrls.igs != ucontrol->value.integer.value[0]) {
		ad73311_ctrls.igs = ucontrol->value.integer.value[0];
		return codec->hw_write(codec->control_data, (char *)&ad73311_ctrls,
				sizeof(ad73311_ctrls));
	}
	return 0;
}

static int ad73311_igs_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = ad73311_ctrls.igs;
	return 0;
}

static int ad73311_dirate_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int rate = SNDRV_PCM_RATE_8000;

	if (ad73311_ctrls.dirate != ucontrol->value.integer.value[0]) {
		ad73311_ctrls.dirate = ucontrol->value.integer.value[0];
		switch (ad73311_ctrls.dirate) {
		case 0:
			rate = SNDRV_PCM_RATE_8000;
			break;
		case 1:
			rate = SNDRV_PCM_RATE_16000;
			break;
		case 2:
			rate = SNDRV_PCM_RATE_32000;
			break;
		case 3:
			rate = SNDRV_PCM_RATE_64000;
			break;
		}
		ad73311_dai.playback.rates = rate;
		ad73311_dai.capture.rates = rate;
		return codec->hw_write(codec->control_data, (char *)&ad73311_ctrls,
				sizeof(ad73311_ctrls));
	}
	return 0;
}

static int ad73311_dirate_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = ad73311_ctrls.dirate;
	return 0;
}

static int ad73311_seen_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	if (ad73311_ctrls.se_en != ucontrol->value.integer.value[0]) {
		ad73311_ctrls.se_en = ucontrol->value.integer.value[0];
		return codec->hw_write(codec->control_data, (char *)&ad73311_ctrls,
				sizeof(ad73311_ctrls));
	}
	return 0;
}

static int ad73311_seen_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = ad73311_ctrls.se_en;
	return 0;
}


static const struct snd_kcontrol_new ad73311_snd_controls[] = {
	SOC_SINGLE_EXT("ADC Capture Volume", CTRL_REG_D, 0, 7, 0,
			ad73311_igs_get, ad73311_igs_put),
	SOC_SINGLE_EXT("DAC Playback Volume", CTRL_REG_D, 4, 7, 0,
			ad73311_ogs_get, ad73311_ogs_put),
	SOC_SINGLE_EXT("Sample Rate", CTRL_REG_B, 0, 3, 0,
			ad73311_dirate_get, ad73311_dirate_put),
	SOC_SINGLE_EXT("Single-Ended Switch", CTRL_REG_F, 5, 1, 0,
			ad73311_seen_get, ad73311_seen_put),
};

static int ad73311_soc_probe(struct snd_soc_codec *codec)
{
	snd_soc_add_codec_controls(codec, ad73311_snd_controls,
			ARRAY_SIZE(ad73311_snd_controls));
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_ad73311 = {
	.probe = ad73311_soc_probe,
	.dapm_widgets = ad73311_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ad73311_dapm_widgets),
	.dapm_routes = ad73311_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(ad73311_dapm_routes),
};

static int ad73311_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev,
			&soc_codec_dev_ad73311, &ad73311_dai, 1);
}

static int ad73311_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver ad73311_codec_driver = {
	.driver = {
			.name = "ad73311",
			.owner = THIS_MODULE,
	},

	.probe = ad73311_probe,
	.remove = ad73311_remove,
};

module_platform_driver(ad73311_codec_driver);

MODULE_DESCRIPTION("ASoC ad73311 driver");
MODULE_AUTHOR("Cliff Cai ");
MODULE_LICENSE("GPL");
