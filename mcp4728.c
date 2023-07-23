// SPDX-License-Identifier: GPL-2.0-only
/*
 * Support for Microchip MCP4728
 *
 * Copyright (C) 2023 Andrea Collamati <andrea.collamati@gmail.com>
 *
 * Based on mcp4725 by Peter Meerwald <pmeerw@pmeerw.net>
 *
 * Driver for the Microchip I2C 12-bit digital-to-analog quad channels
 * converter (DAC).
 *
 * (7-bit I2C slave address 0x60, the three LSBs can be configured in
 * hardware)
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>


#define MCP4728_RESOLUTION 12
#define MCP4728_N_CHANNELS 4

#define MCP4728_CMD_FIELD	GENMASK(7, 3)
#define MCP4728_CHSEL_FIELD	GENMASK(2, 1)
#define MCP4728_UDAC_FIELD	BIT(0)

#define MCP4728_VREF_FIELD	BIT(7)
#define MCP4728_PDMODE_FIELD	GENMASK(6,5)
#define MCP4728_GAIN_FIELD	BIT(4)

#define MCP4728_DAC_H_FIELD	GENMASK(3, 0)
#define MCP4728_DAC_L_FIELD	GENMASK(7, 0)

#define MCP4728_RDY_FIELD	BIT(7)

#define MCP4728_MW_CMD		0x08 /* Multiwrite Command */
#define MCP4728_SW_CMD		0x0A /* Sequential Write Command with EEPROM */

#define MCP4728_READ_RESPONSE_LEN (MCP4728_N_CHANNELS * 3 * 2)
#define MCP4728_WRITE_EEPROM_LEN (1 + MCP4728_N_CHANNELS * 2)

enum vref_mode {
	MCP4728_VREF_EXTERNAL_VDD = 0,
	MCP4728_VRED_INTERNAL_2048mV = 1,
};

enum gain_mode {
	MCP4728_GAIN_X1 = 0,
	MCP4728_GAIN_X2 = 1,
};

enum iio_powerdown_mode {
	MCP4728_IIO_1K,
	MCP4728_IIO_100K,
	MCP4728_IIO_500K,
};

struct mcp4728_channel_data {
	enum vref_mode ref_mode;
	enum iio_powerdown_mode pd_mode;
	enum gain_mode g_mode;
	u16 dac_value;
};

struct mcp4728_data {
	struct i2c_client *client;
	struct regulator *vdd_reg;
	bool powerdown;
	struct mcp4728_channel_data channel_data[MCP4728_N_CHANNELS];
};

#define MCP4728_CHAN(chan) {					\
	.type = IIO_VOLTAGE,					\
	.output = 1,						\
	.indexed = 1,						\
	.channel = chan,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.ext_info = mcp4728_ext_info,				\
}

static int mcp4728_suspend(struct device *dev);
static int mcp4728_resume(struct device *dev);

static ssize_t mcp4728_store_eeprom(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct mcp4728_data *data = iio_priv(indio_dev);
	u8 outbuf[MCP4728_WRITE_EEPROM_LEN];
	int tries = 20;
	u8 inbuf[3];
	bool state;
	int ret;
	unsigned int i;

	ret = kstrtobool(buf, &state);
	if (ret < 0)
		return ret;

	if (!state)
		return 0;

	outbuf[0] = FIELD_PREP(MCP4728_CMD_FIELD, MCP4728_SW_CMD);

	for (i = 0; i < MCP4728_N_CHANNELS; i++) {
		struct mcp4728_channel_data *ch = &data->channel_data[i];
		int offset = 1 + i * 2;

		outbuf[offset] = FIELD_PREP(MCP4728_VREF_FIELD, ch->ref_mode);

		if (data->powerdown) {
			u8 mcp4728_pd_mode = ch->pd_mode + 1;

			outbuf[offset] |= FIELD_PREP(MCP4728_PDMODE_FIELD, mcp4728_pd_mode);
		}

		outbuf[offset] |= FIELD_PREP(MCP4728_GAIN_FIELD, ch->g_mode);
		outbuf[offset] |= FIELD_PREP(MCP4728_DAC_H_FIELD, ch->dac_value >> 8);
		outbuf[offset + 1] = FIELD_PREP(MCP4728_DAC_L_FIELD, ch->dac_value);
	}

	ret = i2c_master_send(data->client, outbuf, MCP4728_WRITE_EEPROM_LEN);
	if (ret < 0)
		return ret;
	else if (ret != MCP4728_WRITE_EEPROM_LEN)
		return -EIO;

	/* wait RDY signal for write complete, takes up to 50ms */
	while (tries--) {
		msleep(20);
		ret = i2c_master_recv(data->client, inbuf, 3);
		if (ret < 0)
			return ret;
		else if (ret != 3)
			return -EIO;

		if (FIELD_GET(MCP4728_RDY_FIELD, inbuf[0]))
			break;
	}

	if (tries < 0) {
		dev_err(&data->client->dev, "%s failed, incomplete\n",
			__func__);
		return -EIO;
	}
	return len;
}

static IIO_DEVICE_ATTR(store_eeprom, 0200, NULL, mcp4728_store_eeprom, 0);

static struct attribute *mcp4728_attributes[] = {
	&iio_dev_attr_store_eeprom.dev_attr.attr,
	NULL,
};

static const struct attribute_group mcp4728_attribute_group = {
	.attrs = mcp4728_attributes,
};

static int mcp4728_program_channel_cfg(int channel, struct iio_dev *indio_dev)
{
	struct mcp4728_data *data = iio_priv(indio_dev);
	struct mcp4728_channel_data *ch = &data->channel_data[channel];
	u8 outbuf[3];
	int ret;

	outbuf[0] = FIELD_PREP(MCP4728_CMD_FIELD, MCP4728_MW_CMD);
	outbuf[0] |= FIELD_PREP(MCP4728_CHSEL_FIELD, channel);
	outbuf[0] |= FIELD_PREP(MCP4728_UDAC_FIELD, 0);

	outbuf[1] = FIELD_PREP(MCP4728_VREF_FIELD, ch->ref_mode);

	if (data->powerdown) {
		u8 mcp4728_pd_mode = ch->pd_mode + 1;

		outbuf[1] |= FIELD_PREP(MCP4728_PDMODE_FIELD, mcp4728_pd_mode);
	}

	outbuf[1] |= FIELD_PREP(MCP4728_GAIN_FIELD, ch->g_mode);
	outbuf[1] |= FIELD_PREP(MCP4728_DAC_H_FIELD, ch->dac_value >> 8);
	outbuf[2] = FIELD_PREP(MCP4728_DAC_L_FIELD, ch->dac_value);

	ret = i2c_master_send(data->client, outbuf, 3);
	if (ret < 0)
		return ret;
	else if (ret != 3)
		return -EIO;

	return 0;
}

static const char *const mcp4728_powerdown_modes[] = { "1kohm_to_gnd",
						       "100kohm_to_gnd",
						       "500kohm_to_gnd" };

static int mcp4728_get_powerdown_mode(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan)
{
	struct mcp4728_data *data = iio_priv(indio_dev);

	return data->channel_data[chan->channel].pd_mode;
}

static int mcp4728_set_powerdown_mode(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      unsigned int mode)
{
	struct mcp4728_data *data = iio_priv(indio_dev);

	data->channel_data[chan->channel].pd_mode = mode;

	return 0;
}

static ssize_t mcp4728_read_powerdown(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      char *buf)
{
	struct mcp4728_data *data = iio_priv(indio_dev);

	return sysfs_emit(buf, "%d\n", data->powerdown);
}

static ssize_t mcp4728_write_powerdown(struct iio_dev *indio_dev,
				       uintptr_t private,
				       const struct iio_chan_spec *chan,
				       const char *buf, size_t len)
{
	struct mcp4728_data *data = iio_priv(indio_dev);
	bool state;
	int ret;

	ret = kstrtobool(buf, &state);
	if (ret)
		return ret;

	if (state)
		ret = mcp4728_suspend(&data->client->dev);
	else
		ret = mcp4728_resume(&data->client->dev);

	if (ret < 0)
		return ret;

	return len;
}

static const struct iio_enum mcp4728_powerdown_mode_enum = {
	.items = mcp4728_powerdown_modes,
	.num_items = ARRAY_SIZE(mcp4728_powerdown_modes),
	.get = mcp4728_get_powerdown_mode,
	.set = mcp4728_set_powerdown_mode,
};

static const char *const mcp4728_vref_modes[] = {
	"vdd_ext",
	"internal",
};

static int mcp4728_get_vref_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan)
{
	struct mcp4728_data *data = iio_priv(indio_dev);

	return data->channel_data[chan->channel].ref_mode;
}

static int mcp4728_set_vref_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 unsigned int mode)
{
	struct mcp4728_data *data = iio_priv(indio_dev);

	data->channel_data[chan->channel].ref_mode = mode;

	if (mode == MCP4728_VREF_EXTERNAL_VDD &&
	    data->channel_data[chan->channel].g_mode == MCP4728_GAIN_X2) {
		dev_warn(&data->client->dev,
			 "CH%d: Gain x2 not effective when vref is vdd, force to x1",
			 chan->channel);
		data->channel_data[chan->channel].g_mode = MCP4728_GAIN_X1;
	}

	return mcp4728_program_channel_cfg(chan->channel, indio_dev);
}

static const struct iio_enum mcp4728_vref_mode_enum = {
	.items = mcp4728_vref_modes,
	.num_items = ARRAY_SIZE(mcp4728_vref_modes),
	.get = mcp4728_get_vref_mode,
	.set = mcp4728_set_vref_mode,
};

static const char *const mcp4728_gain_modes[] = {
	"x1",
	"x2",
};

static int mcp4728_get_gain_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan)
{
	struct mcp4728_data *data = iio_priv(indio_dev);

	return data->channel_data[chan->channel].g_mode;
}

static int mcp4728_set_gain_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 unsigned int mode)
{
	struct mcp4728_data *data = iio_priv(indio_dev);
	int ret;

	if (mode == MCP4728_GAIN_X2 &&
	    data->channel_data[chan->channel].ref_mode ==
		    MCP4728_VREF_EXTERNAL_VDD) {
		dev_err(&data->client->dev,
			"CH%d: Gain x2 not effective when vref is vdd",
			chan->channel);
		return -EINVAL;
	}

	data->channel_data[chan->channel].g_mode = mode;

	ret = mcp4728_program_channel_cfg(chan->channel, indio_dev);

	return ret;
}

static const struct iio_enum mcp4728_gain_mode_enum = {
	.items = mcp4728_gain_modes,
	.num_items = ARRAY_SIZE(mcp4728_gain_modes),
	.get = mcp4728_get_gain_mode,
	.set = mcp4728_set_gain_mode,
};

static const struct iio_chan_spec_ext_info mcp4728_ext_info[] = {
	{
		.name = "powerdown",
		.read = mcp4728_read_powerdown,
		.write = mcp4728_write_powerdown,
		.shared = IIO_SEPARATE,
	},
	IIO_ENUM("powerdown_mode", IIO_SEPARATE, &mcp4728_powerdown_mode_enum),
	IIO_ENUM_AVAILABLE("powerdown_mode", IIO_SHARED_BY_TYPE,
			   &mcp4728_powerdown_mode_enum),
	IIO_ENUM("vref_mode", IIO_SEPARATE, &mcp4728_vref_mode_enum),
	IIO_ENUM_AVAILABLE("vref_mode", IIO_SHARED_BY_TYPE,
			   &mcp4728_vref_mode_enum),
	IIO_ENUM("gain_mode", IIO_SEPARATE, &mcp4728_gain_mode_enum),
	IIO_ENUM_AVAILABLE("gain_mode", IIO_SHARED_BY_TYPE,
			   &mcp4728_gain_mode_enum),
	{},
};

static const struct iio_chan_spec mcp4728_channels[MCP4728_N_CHANNELS] = {
	MCP4728_CHAN(0),
	MCP4728_CHAN(1),
	MCP4728_CHAN(2),
	MCP4728_CHAN(3),
};

static int mcp4728_full_scale_mV(u32 *full_scale_mV, int channel,
				 struct mcp4728_data *data)
{
	int ret;

	if (data->channel_data[channel].ref_mode == MCP4728_VREF_EXTERNAL_VDD)
		ret = regulator_get_voltage(data->vdd_reg);
	else
		ret = 2048000;

	if (ret < 0)
		return ret;

	if (ret == 0)
		return -EINVAL;

	*full_scale_mV = ret / 1000;
	return 0;
}

static u32 mcp4728_raw_to_mV(u32 raw, int channel, struct mcp4728_data *data)
{
	int ret;
	u32 full_scale_mV;

	ret = mcp4728_full_scale_mV(&full_scale_mV, channel, data);
	if (ret)
		return ret;

	return (((raw + 1) * full_scale_mV) >> MCP4728_RESOLUTION);
}

static int mcp4728_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int *val,
			    int *val2, long mask)
{
	struct mcp4728_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = data->channel_data[chan->channel].dac_value;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (data->channel_data[chan->channel].ref_mode ==
		    MCP4728_VREF_EXTERNAL_VDD)
			ret = regulator_get_voltage(data->vdd_reg);
		else
			ret = 2048000;

		if (ret < 0)
			return ret;

		*val = ret / 1000;
		*val2 = MCP4728_RESOLUTION;
		return IIO_VAL_FRACTIONAL_LOG2;
	}
	return -EINVAL;
}

static int mcp4728_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int val,
			     int val2, long mask)
{
	struct mcp4728_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val < 0 || val > GENMASK(MCP4728_RESOLUTION - 1, 0))
			return -EINVAL;
		data->channel_data[chan->channel].dac_value = val;
		ret = mcp4728_program_channel_cfg(chan->channel, indio_dev);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct iio_info mcp4728_info = {
	.read_raw = mcp4728_read_raw,
	.write_raw = mcp4728_write_raw,
	.attrs = &mcp4728_attribute_group,
};

static int mcp4728_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct mcp4728_data *data = iio_priv(indio_dev);
	unsigned int i;

	data->powerdown = true;

	for (i = 0; i < MCP4728_N_CHANNELS; i++) {
		int err = mcp4728_program_channel_cfg(i, indio_dev);

		if (err)
			return err;
	}
	return 0;
}

static int mcp4728_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct mcp4728_data *data = iio_priv(indio_dev);
	int err = 0;
	unsigned int i;

	data->powerdown = false;

	for (i = 0; i < MCP4728_N_CHANNELS; i++) {
		int ret = mcp4728_program_channel_cfg(i, indio_dev);

		if (ret)
			err = ret;
	}
	return err;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mcp4728_pm_ops, mcp4728_suspend,
				mcp4728_resume);

static int mcp4728_init_channels_data(struct mcp4728_data *data)
{
	u8 inbuf[MCP4728_READ_RESPONSE_LEN];
	int ret;
	unsigned int i;

	ret = i2c_master_recv(data->client, inbuf, MCP4728_READ_RESPONSE_LEN);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"failed to read mcp4728 conf. Err=%d\n", ret);
		return ret;
	} else if (ret != MCP4728_READ_RESPONSE_LEN) {
		dev_err(&data->client->dev,
			"failed to read mcp4728 conf. Wrong Response Len ret=%d\n",
			ret);
		return -EIO;
	}

	for (i = 0; i < MCP4728_N_CHANNELS; i++) {
		struct mcp4728_channel_data *ch = &data->channel_data[i];
		u8 r2 = inbuf[i * 6 + 1];
		u8 r3 = inbuf[i * 6 + 2];
		u32 dac_mv;

		ch->dac_value = FIELD_GET(MCP4728_DAC_H_FIELD, r2) << 8 | FIELD_GET(MCP4728_DAC_L_FIELD, r3);
		ch->ref_mode  = FIELD_GET(MCP4728_VREF_FIELD, r2);
		ch->pd_mode   = FIELD_GET(MCP4728_PDMODE_FIELD, r2);
		ch->g_mode    = FIELD_GET(MCP4728_GAIN_FIELD, r2);

		if (ch->g_mode == MCP4728_GAIN_X2 && ch->ref_mode == MCP4728_VREF_EXTERNAL_VDD)
			dev_warn(&data->client->dev,
				 "CH%d: Gain x2 not effective when vref is vdd",
				 i);

		dac_mv = mcp4728_raw_to_mV(ch->dac_value, i, data);
		dev_dbg(&data->client->dev,
			"CH%d: Voltage=%dmV VRef=%d PowerDown=%d Gain=%d\n", i,
			dac_mv, ch->ref_mode, ch->pd_mode, ch->g_mode);
	}

	return 0;
}

static void mcp4728_reg_disable(void *reg)
{
	regulator_disable(reg);
}

static int mcp4728_probe(struct i2c_client *client)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(client);
	struct mcp4728_data *data;
	struct iio_dev *indio_dev;
	int err;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	data->vdd_reg = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(data->vdd_reg))
		return PTR_ERR(data->vdd_reg);

	err = regulator_enable(data->vdd_reg);
	if (err)
		return err;

	err = devm_add_action_or_reset(&client->dev, mcp4728_reg_disable, data->vdd_reg);
	if (err)
		return err;

	err = mcp4728_init_channels_data(data);
	if (err) {
		dev_err(&client->dev,
			"failed to read mcp4728 current configuration\n");
		return err;
	}

	indio_dev->name = id->name;
	indio_dev->info = &mcp4728_info;
	indio_dev->channels = mcp4728_channels;
	indio_dev->num_channels = MCP4728_N_CHANNELS;
	indio_dev->modes = INDIO_DIRECT_MODE;

	return iio_device_register(indio_dev);
}

static const struct i2c_device_id mcp4728_id[] = {
	{ "mcp4728", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, mcp4728_id);

static const struct of_device_id mcp4728_of_match[] = {
	{ .compatible = "microchip,mcp4728" },
	{}
};
MODULE_DEVICE_TABLE(of, mcp4728_of_match);

static struct i2c_driver mcp4728_driver = {
	.driver = {
		.name = "mcp4728",
		.of_match_table = mcp4728_of_match,
		.pm = pm_sleep_ptr(&mcp4728_pm_ops),
	},
	.probe = mcp4728_probe,
	.id_table = mcp4728_id,
};
module_i2c_driver(mcp4728_driver);

MODULE_AUTHOR("Andrea Collamati <andrea.collamati@gmail.com>");
MODULE_DESCRIPTION("MCP4728 12-bit DAC");
MODULE_LICENSE("GPL");
