/*
 * bf6xx-tdm.c - Analog Devices BF6XX tdm interface driver
 *
 * Copyright (c) 2012 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

#include "bf6xx-sport.h"

struct sport_params param;

static int bfin_tdm_set_dai_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct sport_device *sport = snd_soc_dai_get_drvdata(cpu_dai);
	struct device *dev = &sport->pdev->dev;
	int ret = 0;

	param.spctl &= ~(SPORT_CTL_OPMODE | SPORT_CTL_CKRE | SPORT_CTL_FSR
			| SPORT_CTL_LFS | SPORT_CTL_LAFS);
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		param.spctl |= SPORT_CTL_FSR;
		break;
	default:
		dev_err(dev, "%s: DAI format type is not supported\n",
				__func__);
		ret = -EINVAL;
		break;
	}

	param.spctl &= ~(SPORT_CTL_ICLK | SPORT_CTL_IFS);
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
	case SND_SOC_DAIFMT_CBM_CFS:
	case SND_SOC_DAIFMT_CBS_CFM:
		ret = -EINVAL;
		break;
	default:
		dev_err(dev, "%s: Unknown DAI master type\n", __func__);
		ret = -EINVAL;
		break;
	}

	param.spmctl = SPORT_MCTL_MCE | SPORT_MCTL_MCPDE
			| (0x10 & SPORT_MCTL_MFD)
			| (0x700 & SPORT_MCTL_WSIZE);
	param.spcs0 = 0xff;

	return ret;
}

static int bfin_tdm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct sport_device *sport = snd_soc_dai_get_drvdata(dai);
	struct device *dev = &sport->pdev->dev;
	int ret = 0;

	param.spctl &= ~SPORT_CTL_SLEN;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S32_LE:
		param.spctl |= 0x1f0;
		sport->wdsize = 4;
		break;
	default:
		dev_err(dev, "%s: PCM format is not supported\n", __func__);
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ret = sport_set_tx_params(sport, &param);
		if (ret) {
			dev_err(dev, "SPORT tx is busy!\n");
			return ret;
		}
	} else {
		ret = sport_set_rx_params(sport, &param);
		if (ret) {
			dev_err(dev, "SPORT rx is busy!\n");
			return ret;
		}
	}
	return 0;
}

static int bfin_tdm_set_channel_map(struct snd_soc_dai *dai,
		unsigned int tx_num, unsigned int *tx_slot,
		unsigned int rx_num, unsigned int *rx_slot)
{
	struct sport_device *sport = snd_soc_dai_get_drvdata(dai);
	int i;

	if ((tx_num > TDM_MAX_SLOTS) || (rx_num > TDM_MAX_SLOTS))
		return -EINVAL;

	for (i = 0; i < tx_num; i++)
		sport->tx_map[i] = tx_slot[i];
	for (i = 0; i < rx_num; i++)
		sport->rx_map[i] = rx_slot[i];

	return 0;
}

#ifdef CONFIG_PM
static int bfin_tdm_suspend(struct snd_soc_dai *dai)
{
	return 0;
}

static int bfin_tdm_resume(struct snd_soc_dai *dai)
{
	struct sport_device *sport = snd_soc_dai_get_drvdata(dai);
	struct device *dev = &sport->pdev->dev;
	int ret;

	ret = sport_set_tx_params(sport, &param);
	if (ret) {
		dev_err(dev, "SPORT tx is busy!\n");
		return ret;
	}
	ret = sport_set_rx_params(sport, &param);
	if (ret) {
		dev_err(dev, "SPORT rx is busy!\n");
		return ret;
	}

	return 0;
}
#else
#define bfin_tdm_suspend      NULL
#define bfin_tdm_resume       NULL
#endif

static const struct snd_soc_dai_ops bfin_tdm_dai_ops = {
	.hw_params       = bfin_tdm_hw_params,
	.set_fmt         = bfin_tdm_set_dai_fmt,
	.set_channel_map = bfin_tdm_set_channel_map,
};

static struct snd_soc_dai_driver bfin_tdm_dai = {
	.suspend = bfin_tdm_suspend,
	.resume = bfin_tdm_resume,
	.playback = {
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &bfin_tdm_dai_ops,
};

static const struct snd_soc_component_driver bfin_tdm_component = {
	.name	= "bfin-tdm",
};

static int bfin_tdm_probe(struct platform_device *pdev)
{
	struct sport_device *sport;
	struct device *dev = &pdev->dev;
	int ret;

	sport = sport_create(pdev);
	if (!sport)
		return -ENODEV;

	/* register with the ASoC layers */
	ret = snd_soc_register_component(dev, &bfin_tdm_component,
					 &bfin_tdm_dai, 1);
	if (ret) {
		dev_err(dev, "Failed to register DAI: %d\n", ret);
		sport_delete(sport);
		return ret;
	}
	platform_set_drvdata(pdev, sport);

	return 0;
}

static int bfin_tdm_remove(struct platform_device *pdev)
{
	struct sport_device *sport = platform_get_drvdata(pdev);

	snd_soc_unregister_component(&pdev->dev);
	sport_delete(sport);

	return 0;
}

static struct platform_driver bfin_tdm_driver = {
	.probe  = bfin_tdm_probe,
	.remove = bfin_tdm_remove,
	.driver = {
		.name   = "bfin-tdm",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver(bfin_tdm_driver);

MODULE_DESCRIPTION("Analog Devices BF6XX tdm interface driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
