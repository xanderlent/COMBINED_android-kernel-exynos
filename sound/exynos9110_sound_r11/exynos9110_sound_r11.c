/*
 *  exynos9110_sound.c
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>

#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>

#include <sound/samsung/abox.h>

#define SOUND_BASECLK_48K	49152000
#define SOUND_BASECLK_44K1	45158400

#define SOUND_AMP_RATE	48000
#define SOUND_AMP_BCLK	(SOUND_AMP_RATE * 16 * 4)

#define EXYNOS_PMU_PMU_DEBUG_OFFSET	0x0A00
#define SOUND_DAI_ID			0xCAFE
#define SOUND_CODEC_MAX			10
#define SOUND_AUX_MAX			2
#define NUM_SIFS			3

enum FLL_ID { FLL1, FLL2, FLL3, FLLAO };
enum CLK_ID { SYSCLK, ASYNCCLK, DSPCLK, OPCLK, OUTCLK };

struct clk_conf {
	int id;
	const char *name;
	int source;
	int rate;
	int fout;

	bool valid;
};

#define SOUND_MAX_CLOCKS 10

struct sound_drvdata {
	struct device *dev;

	struct clk_conf fll1_refclk;
	struct clk_conf fll2_refclk;
	struct clk_conf fllao_refclk;
	struct clk_conf sysclk;
	struct clk_conf asyncclk;
	struct clk_conf dspclk;
	struct clk_conf opclk;
	struct clk_conf outclk;

	struct notifier_block nb;

	int left_amp_dai;
	int right_amp_dai;
	struct clk *clk[SOUND_MAX_CLOCKS];
};

static struct sound_drvdata exynos9110_drvdata;

static struct snd_soc_pcm_runtime *sound_get_rtd(struct snd_soc_card *card,
		int id)
{
	struct snd_soc_dai_link *dai_link;
	struct snd_soc_pcm_runtime *rtd = NULL;

	for (dai_link = card->dai_link;
			dai_link - card->dai_link < card->num_links;
			dai_link++) {
		if (id == dai_link->id) {
			rtd = snd_soc_get_pcm_runtime(card, dai_link->name);
			break;
		}
	}

	return rtd;
}

static const struct snd_soc_ops rdma_ops = {
};

static const struct snd_soc_ops wdma_ops = {
};

static const struct snd_soc_ops uaif0_ops = {
};

static const struct snd_soc_ops uaif_ops = {
};

static int dsif_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *hw_params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int tx_slot[] = {0, 1};

	/* bclk ratio 64 for DSD64, 128 for DSD128 */
	snd_soc_dai_set_bclk_ratio(cpu_dai, 64);

	/* channel map 0 1 if left is first, 1 0 if right is first */
	snd_soc_dai_set_channel_map(cpu_dai, 2, tx_slot, 0, NULL);
	return 0;
}

static const struct snd_soc_ops dsif_ops = {
	.hw_params = dsif_hw_params,
};

static void sound_init_debugfs(struct snd_soc_card *card)
{
}

static int exynos9110_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_dai *aif_dai;
	struct snd_soc_component *cpu;
	int ret = 0;

	aif_dai = sound_get_rtd(card, 0)->cpu_dai;
	cpu = aif_dai->component;

	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX RDMA0 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX RDMA1 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX RDMA2 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX RDMA3 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX RDMA4 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX RDMA5 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX RDMA6 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX RDMA7 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX WDMA0 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX WDMA1 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX WDMA2 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX WDMA3 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX WDMA4 Capture");
	snd_soc_dapm_sync(snd_soc_component_get_dapm(cpu));

	sound_init_debugfs(card);

	return ret;
}

static struct snd_soc_pcm_stream sound_amp_params[] = {
	{
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = SOUND_AMP_RATE,
		.rate_max = SOUND_AMP_RATE,
		.channels_min = 1,
		.channels_max = 1,
	},
};
static struct snd_soc_dai_link exynos9110_dai[] = {
	{
		.name = "RDMA0",
		.stream_name = "RDMA0",
		.platform_name = "14a51000.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_PRE},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA1",
		.stream_name = "RDMA1",
		.platform_name = "14a51100.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_PRE},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA2",
		.stream_name = "RDMA2",
		.platform_name = "14a51200.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_PRE},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA3",
		.stream_name = "RDMA3",
		.platform_name = "14a51300.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_PRE},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA4",
		.stream_name = "RDMA4",
		.platform_name = "14a51400.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_PRE},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA5",
		.stream_name = "RDMA5",
		.platform_name = "14a51500.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_PRE},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA6",
		.stream_name = "RDMA6",
		.platform_name = "14a51600.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_PRE},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA7",
		.stream_name = "RDMA7",
		.platform_name = "14a51700.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_PRE},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "WDMA0",
		.stream_name = "WDMA0",
		.platform_name = "14a52000.abox_wdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_PRE},
		.ops = &wdma_ops,
		.dpcm_capture = 1,
	},
	{
		.name = "WDMA1",
		.stream_name = "WDMA1",
		.platform_name = "14a52100.abox_wdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_PRE},
		.ops = &wdma_ops,
		.dpcm_capture = 1,
	},
	{
		.name = "WDMA2",
		.stream_name = "WDMA2",
		.platform_name = "14a52200.abox_wdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_PRE},
		.ops = &wdma_ops,
		.dpcm_capture = 1,
	},
	{
		.name = "WDMA3",
		.stream_name = "WDMA3",
		.platform_name = "14a52300.abox_wdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_PRE},
		.ops = &wdma_ops,
		.dpcm_capture = 1,
	},
	{
		.name = "WDMA4",
		.stream_name = "WDMA4",
		.platform_name = "14a52400.abox_wdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_PRE},
		.ops = &wdma_ops,
		.dpcm_capture = 1,
	},
	{
		.name = "UAIF0",
		.stream_name = "UAIF0",
		.platform_name = "snd-soc-dummy",
		//.codec_dai_name = "snd-soc-dummy-dai",
		//.codec_dai_name = "cod9005x-aif",
		//.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.ops = &uaif_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "UAIF1",
		.stream_name = "UAIF1",
		.platform_name = "snd-soc-dummy",
		//.codec_name = "snd-soc-dummy",
		//.codec_dai_name = "snd-soc-dummy-dai",
		//.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.ops = &uaif_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "UAIF4",
		.stream_name = "UAIF4",
		.platform_name = "snd-soc-dummy",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.ops = &uaif_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "VTS-Trigger",
		.stream_name = "VTS-Trigger",
		.cpu_dai_name = "vts-tri",
		.platform_name = "11110000.vts:vts_dma@0",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.capture_only = true,
	},
	{
		.name = "VTS-Record",
		.stream_name = "VTS-Record",
		.cpu_dai_name = "vts-rec",
		.platform_name = "11110000.vts:vts_dma@1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.capture_only = true,
	},
	{
		.name = "SIFS0",
		.stream_name = "SIFS0",
		.cpu_dai_name = "SIFS0",
		.platform_name = "snd-soc-dummy",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "SIFS1",
		.stream_name = "SIFS1",
		.cpu_dai_name = "SIFS1",
		.platform_name = "snd-soc-dummy",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "SIFS2",
		.stream_name = "SIFS2",
		.cpu_dai_name = "SIFS2",
		.platform_name = "snd-soc-dummy",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
};

static const char * const vts_output_texts[] = {
        "None",
        "DMIC1",
};

static const struct soc_enum vts_output_enum =
        SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(vts_output_texts),
                        vts_output_texts);


static const struct snd_kcontrol_new vts_output_mux[] = {
        SOC_DAPM_ENUM("VTS Virtual Output Mux", vts_output_enum),
};

static const struct snd_kcontrol_new exynos9110_controls[] = {
};

static struct snd_soc_dapm_widget exynos9110_widgets[] = {
	SND_SOC_DAPM_OUTPUT("VOUTPUT"),
	SND_SOC_DAPM_INPUT("VINPUT1"),
	SND_SOC_DAPM_INPUT("VINPUT2"),
	SND_SOC_DAPM_OUTPUT("VOUTPUTCALL"),
	SND_SOC_DAPM_INPUT("VINPUTCALL"),
	SND_SOC_DAPM_OUTPUT("VTS Virtual Output"),
	SND_SOC_DAPM_MUX("VTS Virtual Output Mux", SND_SOC_NOPM, 0, 0,
                      &vts_output_mux[0]),
};

//static struct snd_soc_codec_conf codec_conf[] = {
//	{.name_prefix = "ABOX", },
//	{.name_prefix = "ABOX", },
//	{.name_prefix = "ABOX", },
/*	{.name_prefix = "VTS", },*/
//};

static struct snd_soc_codec_conf codec_conf[SOUND_CODEC_MAX];

static struct snd_soc_aux_dev aux_dev[SOUND_AUX_MAX];

static struct snd_soc_card exynos9110_sound = {
	.name = "Exynos9110-Sound",
	.owner = THIS_MODULE,
	.dai_link = exynos9110_dai,
	.num_links = ARRAY_SIZE(exynos9110_dai),

	.late_probe = exynos9110_late_probe,

	.controls = exynos9110_controls,
	.num_controls = ARRAY_SIZE(exynos9110_controls),
	.dapm_widgets = exynos9110_widgets,
	.num_dapm_widgets = ARRAY_SIZE(exynos9110_widgets),

	.drvdata = (void *)&exynos9110_drvdata,

	.codec_conf = codec_conf,
	.num_configs = ARRAY_SIZE(codec_conf),

	.aux_dev = aux_dev,
	.num_aux_devs = ARRAY_SIZE(aux_dev),
};
/*
static int read_dai(struct device_node *np, const char * const prop,
			      struct device_node **dai, const char **name)
{
	int ret = 0;

	np = of_get_child_by_name(np, prop);
	if (!np)
		return -ENOENT;

	*dai = of_parse_phandle(np, "sound-dai", 0);
	if (!*dai) {
		ret = -ENODEV;
		goto out;
	}

	if (*name == NULL) {
		// Ignoring the return as we don't register DAIs to the platform
		ret = snd_soc_of_get_dai_name(np, name);
		if (ret && !*name)
			return ret;
	}
out:
	of_node_put(np);

	return ret;
}
*/
static struct clk *xclkout;

static void control_xclkout(bool on)
{
	if (on) {
		clk_prepare_enable(xclkout);
	} else {
		clk_disable_unprepare(xclkout);
	}
}

static int read_platform(struct device_node *np, const char * const prop,
		struct device_node **dai)
{
	int ret = 0;

	np = of_get_child_by_name(np, prop);
	if (!np)
		return -ENOENT;

	*dai = of_parse_phandle(np, "sound-dai", 0);
	if (!*dai) {
		ret = -ENODEV;
		goto out;
	}
out:
	of_node_put(np);

	return ret;
}

static int read_cpu(struct device_node *np, struct device *dev,
		struct snd_soc_dai_link *dai_link)
{
	int ret = 0;

	np = of_get_child_by_name(np, "cpu");
	if (!np)
		return -ENOENT;

	dai_link->cpu_of_node = of_parse_phandle(np, "sound-dai", 0);
	if (!dai_link->cpu_of_node) {
		ret = -ENODEV;
		goto out;
	}

	if (dai_link->cpu_dai_name == NULL) {
		// Ignoring the return as we don't register DAIs to the platform
		ret = snd_soc_of_get_dai_name(np, &dai_link->cpu_dai_name);
		if (ret)
			goto out;
	}
out:
	of_node_put(np);

	return ret;
}

static int read_codec(struct device_node *np, struct device *dev,
		struct snd_soc_dai_link *dai_link)
{
	np = of_get_child_by_name(np, "codec");
	if (!np)
		return -ENOENT;

	return snd_soc_of_get_dai_link_codecs(dev, np, dai_link);
}

static void exynos9110_register_card_work_func(struct work_struct *work)
{
	struct snd_soc_card *card = &exynos9110_sound;
	int ret;

	dev_info(card->dev, "%s\n", __func__);

	ret = devm_snd_soc_register_card(card->dev, card);
	if (ret)
		dev_err(card->dev, "sound card register failed: %d\n", ret);
}
DECLARE_WORK(exynos9110_register_card_work, exynos9110_register_card_work_func);

static int exynos9110_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &exynos9110_sound;
	struct sound_drvdata *drvdata = card->drvdata;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *dai;
	struct snd_soc_dai_link *link;
	int nlink = card->num_links - 1;
	int rc, ret;
	unsigned int i;
	const char *cur = NULL;
	struct property *p;

//	nlink = card->num_links - NUM_SIFS - 1;
	card->dev = &pdev->dev;
	drvdata->dev = card->dev;

	dev_info(&pdev->dev, "%s\n", __func__);

	snd_soc_card_set_drvdata(card, drvdata);

	xclkout = devm_clk_get(&pdev->dev, "xclkout");
	if (IS_ERR(xclkout)) {
		dev_err(&pdev->dev, "xclkout get failed\n");
		xclkout = NULL;
	}
	control_xclkout(true);
	dev_info(&pdev->dev, "xclkout is enabled\n");

	i = 0;
	p = of_find_property(np, "clock-names", NULL);
	if (p) {
		while ((cur = of_prop_next_string(p, cur)) != NULL) {
			drvdata->clk[i] = devm_clk_get(drvdata->dev, cur);
			if (IS_ERR(drvdata->clk[i])) {
				dev_info(drvdata->dev, "Failed to get %s: %ld\n",
					 cur, PTR_ERR(drvdata->clk[i]));
				drvdata->clk[i] = NULL;
				break;
			}

			clk_prepare_enable(drvdata->clk[i]);

			if (++i == SOUND_MAX_CLOCKS)
				break;
		}
	}

	for_each_child_of_node(np, dai) {

		link = &exynos9110_dai[nlink];
		dev_info(drvdata->dev, "%s - (%s)(%s)\n", __func__,
				link->name, dai->name);

		if (!link->name)
			link->name = dai->name;
		if (!link->stream_name)
			link->stream_name = dai->name;

		if (!link->cpu_name) {
			ret = read_cpu(dai, card->dev, link);
			if (ret) {
				dev_err(card->dev, "Failed to parse cpu DAI for %s: %d\n",
						dai->name, ret);
				return ret;
			}
		}

		if (!link->platform_name) {
			ret = read_platform(dai, "platform",
					&link->platform_of_node);
			if (ret) {
				link->platform_of_node = link->cpu_of_node;
				dev_info(card->dev, "Cpu node is used as platform for %s: %d\n",
						dai->name, ret);
			}
		}

		if (!link->codec_name) {
			ret = read_codec(dai, card->dev, link);
			if (ret) {
				dev_warn(card->dev, "Failed to parse codec DAI for %s: %d\n",
						dai->name, ret);
				link->codec_name = "snd-soc-dummy";
				link->codec_dai_name = "snd-soc-dummy-dai";
				ret = 0;
			}
			dev_info(card->dev, "# codecs: %d\n", link->num_codecs);
			for (i = 0; i < link->num_codecs; i++) {
				dev_info(card->dev, "num: %d, dai_name: %s\n",
					i, link->codecs[i].dai_name);
			}

//			if (link->codecs && strstr(link->codecs[0].dai_name,
//						"cs35l41"))
//				link->ops = &cs35l41_ops;
		}

		if (strstr(dai->name, "left-amp")) {
			link->params = sound_amp_params;
			drvdata->left_amp_dai = nlink;
		} else if (strstr(dai->name, "right-amp")) {
			link->params = sound_amp_params;
			drvdata->right_amp_dai = nlink;
		}

		link->dai_fmt = snd_soc_of_parse_daifmt(dai, NULL, NULL,
				NULL);

		if (nlink-- == 0)
			break;
	}

/*
		// search dai_link
		for (nlink = 0; nlink < card->num_links; nlink++) {
			const char *dai_link_name;
			ret = of_property_read_string(dai, "dai_link",
					&dai_link_name);
			dev_info(drvdata->dev, "(%d) (%s)\n", nlink,
					dai_link_name);
			if (ret < 0)
				continue;

			if (dai_link_name && !strncmp(exynos9110_dai[nlink].name,
					dai_link_name,
					sizeof(exynos9110_dai[nlink].name) - 1)) {
				dev_info(drvdata->dev, "(%s[%d])(%d) (%s)(%s)\n",
					__func__, __LINE__,
					nlink,
					dai_link_name,
					exynos9110_dai[nlink].name);

				break;
			}
		}

		if (!exynos9110_dai[nlink].name)
			exynos9110_dai[nlink].name = dai->name;
		if (!exynos9110_dai[nlink].stream_name)
			exynos9110_dai[nlink].stream_name = dai->name;

		if (!exynos9110_dai[nlink].cpu_name) {
			ret = read_dai(dai, "cpu",
					&exynos9110_dai[nlink].cpu_of_node,
					&exynos9110_dai[nlink].cpu_dai_name);
			if (ret) {
				dev_err(card->dev,
					"Failed to parse cpu DAI for %s: %d\n",
					dai->name, ret);
				return ret;
			}
		}

		dev_info(drvdata->dev, "(%s[%d])(%d) (%s)(%s)(%s)->(%s)\n",
				__func__, __LINE__,
				nlink,
				exynos9110_dai[nlink].cpu_dai_name,
				exynos9110_dai[nlink].name,
				dai->name,
				dai->sibling ? dai->sibling->name : NULL);

		if (!exynos9110_dai[nlink].platform_name) {
			ret = read_dai(dai, "platform",
				&exynos9110_dai[nlink].platform_of_node,
				&exynos9110_dai[nlink].platform_name);
			if (ret) {
				exynos9110_dai[nlink].platform_of_node =
					exynos9110_dai[nlink].cpu_of_node;
				dev_info(card->dev,
					"Cpu node is used as platform for %s: %d\n",
					dai->name, ret);
			}
		}

		if (!exynos9110_dai[nlink].codec_name) {
			ret = read_dai(dai, "codec",
					&exynos9110_dai[nlink].codec_of_node,
					&exynos9110_dai[nlink].codec_dai_name);
			if (ret) {
				dev_err(card->dev,
					"Failed to parse codec DAI for %s: %d\n",
					dai->name, ret);
				return ret;
			}
		}

		if (strstr(dai->name, "left-amp")) {
			exynos9110_dai[nlink].params = sound_amp_params;
			drvdata->left_amp_dai = nlink;
		} else if (strstr(dai->name, "right-amp")) {
			exynos9110_dai[nlink].params = sound_amp_params;
			drvdata->right_amp_dai = nlink;
		}

		exynos9110_dai[nlink].dai_fmt =
				snd_soc_of_parse_daifmt(dai, NULL, NULL, NULL);

		if (--nlink < 0) // card->num_links)
			break;
	}
*/
	/* if (!nlink) { */
	if (nlink < -1) {
		dev_err(card->dev, "No DAIs specified\n");
		return -EINVAL;
	}

	if (of_property_read_bool(np, "samsung,routing")) {
		ret = snd_soc_of_parse_audio_routing(card, "samsung,routing");
		if (ret)
			return ret;
	}

	for (i = 0; i < ARRAY_SIZE(codec_conf); i++) {
		codec_conf[i].of_node = of_parse_phandle(np, "samsung,codec", i);
		if (IS_ERR_OR_NULL(codec_conf[i].of_node)) {
			exynos9110_sound.num_configs = i;
			break;
		}

		rc = of_property_read_string_index(np, "samsung,prefix", i,
				&codec_conf[i].name_prefix);
		if (rc < 0)
			codec_conf[i].name_prefix = "";
	}

	for (i = 0; i < ARRAY_SIZE(aux_dev); i++) {
		aux_dev[i].codec_of_node = of_parse_phandle(np, "samsung,aux", i);
		if (IS_ERR_OR_NULL(aux_dev[i].codec_of_node)) {
			exynos9110_sound.num_aux_devs = i;
			break;
		}
	}

	schedule_work(&exynos9110_register_card_work);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id exynos9110_of_match[] = {
	{ .compatible = "samsung,exynos9110-sound-r11", },
	{},
};
MODULE_DEVICE_TABLE(of, exynos9110_of_match);
#endif /* CONFIG_OF */

static struct platform_driver exynos9110_audio_driver = {
	.driver		= {
		.name	= "exynos9110-sound-r11",
		.owner	= THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(exynos9110_of_match),
	},

	.probe		= exynos9110_audio_probe,
};

module_platform_driver(exynos9110_audio_driver);

MODULE_DESCRIPTION("ALSA SoC R11 sound driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:exynos9110-sound-r11");
