/**
 * The module represents a ASOC codec driver responsible for turning Microphone
 * on the MCU on or off.
 *
 * Currently it's only a stub: it does not actually communicate with the MCU
 * yet, that will be added later when the communication lib is available.
 *
 * Copyright 2020 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/completion.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define NANOHUB_AUDIO_CHANNEL_ID 16

// Values here should match MCU FW values defined in audio_services.cc
#define DMIC_MCU_MESSAGE_VERSION 1
#define DMIC_MCU_MESSAGE_MIC_ON 0x01
#define DMIC_MCU_MESSAGE_SAMPLE_RATE_KHZ 0x02
#define DMIC_MCU_MESSAGE_CHANNEL 0x03
#define DMIC_MCU_MESSAGE_GAIN 0x04

enum dmic_mcu_channel {
	DMIC_MCU_CHANNEL_LEFT,
	DMIC_MCU_CHANNEL_RIGHT,
};

static const char *const mcu_dmic_channel_enum_texts[] = {
	"Left",
	"Right",
};
static const unsigned int mcu_dmic_channel_enum_values[] = {
	DMIC_MCU_CHANNEL_LEFT,
	DMIC_MCU_CHANNEL_RIGHT,
};
SOC_VALUE_ENUM_SINGLE_DECL(mcu_dmic_channel_enum, SND_SOC_NOPM, 0, 0,
			   mcu_dmic_channel_enum_texts,
			   mcu_dmic_channel_enum_values);

extern ssize_t nanohub_send_message(int channel_id, const char *buffer,
				    size_t length);

struct mcu_mic_codec_data {
	int mic_on;
	int sample_rate_hz;
	enum dmic_mcu_channel channel;
	int gain;
};

static int dmic_mcu_send_message(struct snd_soc_component *component,
				 char mcu_message_id, char data)
{
	char buffer[3];
	ssize_t bytes;

	buffer[0] = DMIC_MCU_MESSAGE_VERSION; // version
	buffer[1] = mcu_message_id;	      // message identifier
	buffer[2] = data;

	bytes = nanohub_send_message(NANOHUB_AUDIO_CHANNEL_ID, buffer,
				     sizeof(buffer));
	if (bytes != sizeof(buffer)) {
		dev_err(component->dev, "Bytes sent expected = %zd, actual = %zd\n",
			sizeof(buffer), bytes);
		return -EIO;
	}
	return 0;
}

// The callback that is called when ASOC needs to fetch the value of the
// property 'DMIC_MCU Input Gain'.
static int dmic_mcu_input_gain_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);

	dev_info(component->dev, "Called dmic_mcu_input_gain_get\n");

	ucontrol->value.integer.value[0] = codec_data->gain;

	dev_info(component->dev, "mcu_input_gain: %d\n", codec_data->gain);

	return 0;
}

// The callback that is called when ASOC needs to set the value of the
// property 'DMIC_MCU Input Gain'.
static int dmic_mcu_input_gain_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];
	int ret;

	dev_info(component->dev, "Called dmic_mcu_input_gain_put\n");

	if (value < 0 || value > 63) {
		dev_info(component->dev,
			 "Invalid value: %d, it must be between 0 and 63\n",
			 value);
		return -EINVAL;
	}

	ret = dmic_mcu_send_message(component, DMIC_MCU_MESSAGE_GAIN, value);
	if (ret != 0) {
		return ret;
	}

	codec_data->gain = value;
	dev_info(component->dev, "new mcu_input_gain: %d\n", codec_data->gain);

	return 0;
}

// The callback that is called when ASOC needs to fetch the value of the
// property 'DMIC_MCU Channel'.
static int dmic_mcu_channel_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);

	dev_info(component->dev, "Called dmic_mcu_channel_get\n");

	ucontrol->value.integer.value[0] = codec_data->channel;

	dev_info(component->dev, "mcu_mic_channel: %d (%s)\n", codec_data->channel,
		 mcu_dmic_channel_enum_texts[codec_data->channel]);

	return 0;
}

// The callback that is called when ASOC needs to set the value of the
// property 'DMIC_MCU Channel'.
static int dmic_mcu_channel_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];
	int ret;

	dev_info(component->dev, "Called dmic_mcu_channel_put\n");

	if (value != DMIC_MCU_CHANNEL_LEFT && value != DMIC_MCU_CHANNEL_RIGHT) {
		dev_info(component->dev,
			 "Invalid value: %d, it must be %d or %d.\n", value,
			 DMIC_MCU_CHANNEL_LEFT, DMIC_MCU_CHANNEL_RIGHT);
		return -EINVAL;
	}

	ret = dmic_mcu_send_message(component, DMIC_MCU_MESSAGE_CHANNEL, value);
	if (ret != 0) {
		return ret;
	}

	codec_data->channel = value;
	dev_info(component->dev, "new mcu_mic_channel: %d (%s)\n",
		 codec_data->channel,
		 mcu_dmic_channel_enum_texts[codec_data->channel]);

	return 0;
}

// The callback that is called when ASOC needs to fetch the value of the
// property 'DMIC_MCU Sample Rate'.
static int dmic_mcu_sample_rate_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);
	dev_info(component->dev, "Called dmic_mcu_sample_rate_get\n");
	ucontrol->value.integer.value[0] = codec_data->sample_rate_hz;
	dev_info(component->dev, "mcu_mic_sample_rate: %d\n",
		 codec_data->sample_rate_hz);
	return 0;
}

// The callback that is called when ASOC needs to set the value of the
// property 'DMIC_MCU Sample Rate'.
static int dmic_mcu_sample_rate_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];
	int ret;

	dev_info(component->dev, "Called dmic_mcu_sample_rate_put\n");

	if (value != 8000 && value != 16000 && value != 48000) {
		dev_info(component->dev,
			 "Invalid Sample Rate: %d. (Valid rates: 8000, 16000, 48000 Hertz)\n",
			 value);
		return -EINVAL;
	}

	ret = dmic_mcu_send_message(component, DMIC_MCU_MESSAGE_SAMPLE_RATE_KHZ,
				    value / 1000);
	if (ret != 0) {
		return ret;
	}

	codec_data->sample_rate_hz = value;
	dev_info(component->dev, "new mcu_mic_sample_rate: %d\n",
		 codec_data->sample_rate_hz);

	return 0;
}

// The callback that is called when ASOC needs to fetch the value of the
// property 'DMIC_MCU On'.
static int dmic_mcu_on_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);

	dev_info(component->dev, "Called dmic_mcu_on_get\n");

	ucontrol->value.integer.value[0] = codec_data->mic_on;

	dev_info(component->dev, "mcu_mic_on: %d\n", codec_data->mic_on);

	return 0;
}

// The callback that is called when ASOC needs to set the value of the
// property 'DMIC_MCU On'.
static int dmic_mcu_on_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct mcu_mic_codec_data *codec_data =
		snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];
	int ret;

	dev_info(component->dev, "Called static int dmic_mcu_on_put\n");

	if (value != 0 && value != 1) {
		dev_info(component->dev,
			 "Invalid value: %d, it must be 1 or 0.\n", value);
		return -EINVAL;
	}

	ret = dmic_mcu_send_message(component, DMIC_MCU_MESSAGE_MIC_ON,
				    value ? 1 : 0);
	if (ret != 0) {
		return ret;
	}

	codec_data->mic_on = value;
	dev_info(component->dev, "new mcu_mic_on: %d\n", codec_data->mic_on);

	return 0;
}

static const struct snd_kcontrol_new snd_controls[] = {
	// Defines a property DMIC_MCU On.
	// It can be switched in console for example using tinyalsa:
	// tinymix 'DMIC_MCU On' 1
	SOC_SINGLE_EXT("DMIC_MCU On", SND_SOC_NOPM, 0, 1, 0, dmic_mcu_on_get,
		       dmic_mcu_on_put),
	SOC_SINGLE_EXT("DMIC_MCU Sample Rate", SND_SOC_NOPM, 0, 48000, 0,
		       dmic_mcu_sample_rate_get, dmic_mcu_sample_rate_put),
	SOC_VALUE_ENUM_EXT("DMIC_MCU Channel", mcu_dmic_channel_enum,
			   dmic_mcu_channel_get, dmic_mcu_channel_put),
	// Input Gain is mapped from [-32, 31] to [0, 63]. This is because
	// tinymix does not accept negative numbers. It processes the
	// negative sign as an argument option prefix.
	SOC_SINGLE_EXT("DMIC_MCU Input Gain", SND_SOC_NOPM, 0, 63, 0,
		       dmic_mcu_input_gain_get, dmic_mcu_input_gain_put),
};

static int mcu_codec_probe(struct snd_soc_component *component)
{
	dev_info(component->dev, "Called mcu_codec_probe.\n");

	return 0;
}

static void mcu_codec_remove(struct snd_soc_component *component)
{
	dev_info(component->dev, "Called mcu_codec_remove.\n");
}

static struct snd_soc_component_driver mcu_mic_soc_component_driver = {
	.probe = mcu_codec_probe,
	.remove = mcu_codec_remove,
	.controls = snd_controls,
	.num_controls = ARRAY_SIZE(snd_controls),
};

/*
 * The codec DAI driver. It's properties are not used at the moment, as the main
 * goal of the codec is to turn the Mic on/off.
 */
static struct snd_soc_dai_driver mcu_mic_soc_dai_driver = {
	.name = "mcu-mic-codec-dai",
	.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 1,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_U16_LE),
		},
};

static int mcu_mic_probe(struct platform_device *pdev)
{
	int ret;
	// Structure used to store mcu dmic state:
	// [on/off] 1: it is on, 0: it is off.
	// [sample rate]: Units of hertz
	struct mcu_mic_codec_data *codec_data;

	dev_info(&pdev->dev, "Called mcu_mic_probe\n");

	codec_data = devm_kzalloc(&pdev->dev, sizeof(struct mcu_mic_codec_data),
				  GFP_KERNEL);
	if (!codec_data) {
		dev_err(&pdev->dev, "No memory for codec_data\n");
		return -ENOMEM;
	}
	codec_data->mic_on = 0;
	codec_data->sample_rate_hz = 16000;
	codec_data->channel = DMIC_MCU_CHANNEL_RIGHT;
	// gain is offset by 32. Therefore 32 means 0 gain
	codec_data->gain = 37;
	platform_set_drvdata(pdev, codec_data);

	ret = snd_soc_register_component(&pdev->dev,
					 &mcu_mic_soc_component_driver,
					 &mcu_mic_soc_dai_driver, 1);

	dev_info(&pdev->dev, "mcu_mic_probe ret: %d\n", ret);

	return ret;
}

static int mcu_mic_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "Called mcu_mic_remove\n");

	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

const struct of_device_id mcu_mic_of_match[] = {
	{
		.compatible = "google,mcu_mic_codec",
	},
	{},
};
MODULE_DEVICE_TABLE(of, mcu_mic_of_match);

static struct platform_driver mcu_mic_driver = {
	.driver = {
			.name = "mcu_mic_codec",
			.owner = THIS_MODULE,
			.of_match_table = of_match_ptr(mcu_mic_of_match),
		},
	.probe = mcu_mic_probe,
	.remove = mcu_mic_remove,
};

/* Register the platform driver */
module_platform_driver(mcu_mic_driver);

MODULE_DESCRIPTION("ASoC mcu mic codec driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mcu_mic-codec");
