/*
 * A sensor driver for AS726x Multi-Spectral Sensing Engines
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA	02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

#include <linux/gpio.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define AS726X_STATUS_REG         0x00
#define AS726X_WRITE_REG          0x01
#define AS726X_READ_REG           0x02

#define AS726X_TX_VALID           0x02
#define AS726X_RX_VALID           0x01

#define AS726X_VREG_DEVICE_TYPE   0x00
#define AS726X_VREG_HW_VERSION    0x01
#define AS726X_VREG_FW_VERSION_L  0x02
#define AS726X_VREG_FW_VERSION_H  0x03
#define AS726X_VREG_CONTROL_SETUP 0x04
#define AS726X_VREG_INT_T         0x05
#define AS726X_VREG_DEVICE_TEMP   0x06
#define AS726X_VREG_LED_CONTROL   0x07
#define AS726X_VREG_RAW_DATA      0x08
#define AS726X_VREG_CALIB_DATA    0x14

struct as726x_data {
	struct i2c_client *client;
	struct attribute_group attrs;
	struct mutex lock;
	struct delayed_work work;
	int channel_results_calib[6]; /* float */
	u16 channel_results_raw[6];
	int int_time; /* [ms] */
};

static int as726x_write_vreg(struct i2c_client *client,
			     u8 reg, u8 val)
{
	int ret;
	u8 stat = 0;

	while (1) {
		stat = i2c_smbus_read_byte_data(client, AS726X_STATUS_REG);
		if((stat & AS726X_TX_VALID) == 0)
			break;
	}
	ret = i2c_smbus_write_byte_data(client, AS726X_WRITE_REG, (reg | 0x80));
	if (ret < 0)
		return ret;

	while (1) {
		stat = i2c_smbus_read_byte_data(client, AS726X_STATUS_REG);
		if((stat & AS726X_TX_VALID) == 0)
			break;
	}
	ret = i2c_smbus_write_byte_data(client, AS726X_WRITE_REG, val);
	if (ret < 0)
		return ret;

	return 0;
}

static void as726x_reset(struct i2c_client *c)
{
	printk("reset\n");
	as726x_write_vreg(c, AS726X_VREG_CONTROL_SETUP, 0x80);
	msleep(800);
}

static int as726x_read_vreg(struct i2c_client *client,
			     u8 reg, u8 *val)
{
	int ret = 0;
	u8 stat = 0;

	while (1) {
		stat = i2c_smbus_read_byte_data(client, AS726X_STATUS_REG);
		if (stat < 0)
			goto err;
		if((stat & AS726X_TX_VALID) == 0)
			break;
		msleep(50);
	}
	ret = i2c_smbus_write_byte_data(client, AS726X_WRITE_REG, reg);
	if (ret < 0)
		goto err;

	while (1) {
		stat = i2c_smbus_read_byte_data(client, AS726X_STATUS_REG);
		if (stat < 0)
			goto err;
		if((stat & AS726X_TX_VALID) == 0)
			break;
		msleep(50);
	}
	ret = i2c_smbus_read_byte_data(client, AS726X_READ_REG);
	if (ret < 0)
		goto err;

	*val = (u8)ret;

	return 0;

err:
	printk("Failed reading vreg\n");
	return -1;
}

static void as726x_workfunc(struct work_struct *w)
{
	struct as726x_data *data = container_of((struct delayed_work *)w,
			struct as726x_data, work);
	int ch, i;
	u8 v;

	/* read temp */
	as726x_read_vreg(data->client, AS726X_VREG_DEVICE_TEMP, &v);
	printk("temp: %d\n", v);

	/* read calibrated data */
	for (ch=0; ch<6; ch++) {
		data->channel_results_calib[ch] = 0;
		for (i=0; i<4; i++) {
			as726x_read_vreg(data->client,
				AS726X_VREG_CALIB_DATA+(4*ch)+i, &v);
			//data->channel_results_calib[ch] |= (v << (24-(i*8)));
			data->channel_results_calib[ch] |= (v << (i*8));
		}
		printk("%d calib -> 0x%08x\n", ch, data->channel_results_calib[ch]);
	}
	printk("\n");

	/* read raw data */
	for (ch=0; ch<6; ch++) {
		data->channel_results_raw[ch] = 0;
		for (i=0; i<2; i++) {
			as726x_read_vreg(data->client,
				AS726X_VREG_RAW_DATA+(2*ch)+i, &v);
			data->channel_results_raw[ch] |= (v << (8-(i*8)));
		}
		printk("%d -> 0x%04x\n", ch, data->channel_results_raw[ch]);
	}
	printk("\n");

	schedule_delayed_work(&data->work, msecs_to_jiffies(data->int_time*2*10));
}

static int as726x_read_raw_data(struct i2c_client *client, int ch)
{
	u8 v;
	int data = 0;
	int ret;

	ret = as726x_read_vreg(client, AS726X_VREG_RAW_DATA+(2*ch)+0, &v);
	if (ret < 0)
		return ret;
	data = (v << 8);

	ret = as726x_read_vreg(client, AS726X_VREG_RAW_DATA+(2*ch)+1, &v);
	if (ret < 0)
		return ret;
	data |= v;

	return data;
}

#if 0
static int as726x_read_calib_data(struct i2c_client *client, int ch)
{
	u8 v, c;
	int i, r;
	int data = 0;

	int cnt = 0;

	r = as726x_read_vreg(client, AS726X_VREG_CONTROL_SETUP, &c);
	if (r != 0)
		goto err;
	c &= ~0x02; /* Clear DATA_RDY */
	r = as726x_write_vreg(client, AS726X_VREG_CONTROL_SETUP, c);
	if (r != 0)
		goto err;

	/* start conversion */
	r = as726x_read_vreg(client, AS726X_VREG_CONTROL_SETUP, &c);
	if (r != 0)
		goto err;
	c |= (0x03 << 2); /* Mode = 3, One Shot */
	r = as726x_write_vreg(client, AS726X_VREG_CONTROL_SETUP, c);
	if (r != 0)
		goto err;

	/* wait for DATA_RDY  */
	while (1) {
		r = as726x_read_vreg(client, AS726X_VREG_CONTROL_SETUP, &c);
		if (r != 0)
			goto err;
		if (c & 0x02)
			break;
		msleep(200);
		cnt++;
	}

	printk("reading data\n");

	for (i=0; i<4; i++) {
		r = as726x_read_vreg(client, AS726X_VREG_CALIB_DATA+(4*ch)+i, &v);
		if (r < 0)
			goto err;
		data |= (v << (24-(i*8)));
	}

	printk("cnt = %d\n", cnt);

	return data;

err:
	printk("Failed reading data\n");
	as726x_reset(client);
	return -1;
}
#endif

static int as726x_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2,
			   long mask)
{
	struct as726x_data *data = iio_priv(indio_dev);
	struct i2c_client *client = data->client;

	if (mask == 0)
		*val = data->channel_results_raw[chan->address];

	if (mask == 1)
		*val = data->channel_results_calib[chan->address];

	return IIO_VAL_INT;
}

#define AS726X_CHANNEL(index)                               \
	{                                                       \
		.type = IIO_INTENSITY,                              \
		.channel = index,                                   \
		.modified = 0,                                      \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),       \
		.address = index,                                   \
		.indexed = 1,                                       \
	},                                                      \
	{                                                       \
		.type = IIO_INTENSITY,                              \
		.channel = index,                                   \
		.modified = 0,                                      \
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED), \
		.address = index,                                   \
		.indexed = 1,                                       \
	}

static const struct iio_chan_spec as726x_channels[] = {
	AS726X_CHANNEL(0), AS726X_CHANNEL(1), AS726X_CHANNEL(2),
	AS726X_CHANNEL(3), AS726X_CHANNEL(4), AS726X_CHANNEL(5),
};

static const struct iio_info as726x_info = {
	.read_raw = &as726x_read_raw,
	.driver_module = THIS_MODULE,
};

static int as726x_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct as726x_data *data;
	struct iio_dev *indio_dev;
	int err;
	u16 hw_version = 0;
	u8 v = 0;

	/* Register with IIO */
	indio_dev = iio_device_alloc(sizeof(*data));
	if (indio_dev == NULL) {
		err = -ENOMEM;
	}
	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);

	data->client = client;
	data->int_time = 50; // [ms]
	mutex_init(&data->lock);
	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = as726x_channels;
	indio_dev->num_channels = ARRAY_SIZE(as726x_channels);
	indio_dev->info = &as726x_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	as726x_read_vreg(client, AS726X_VREG_HW_VERSION, &v);
	if (v < 0)
		goto exit_free_iio;
	hw_version = v;
	printk("Found AS726X sensor (hw_ver 0x%02x)\n", hw_version);

	/* reset */
	as726x_reset(client);

	/* led control */
	v = 0x00 | (0x3 << 4); /* current limit 100mA */
	as726x_write_vreg(client, AS726X_VREG_LED_CONTROL, v);
	as726x_read_vreg(client, AS726X_VREG_LED_CONTROL, &v);
	printk("led_cfg = 0x%x\n", v);

	/* integration time x*2.8ms */
	as726x_write_vreg(client, AS726X_VREG_INT_T, data->int_time);
	as726x_read_vreg(client, AS726X_VREG_INT_T, &v);
	printk("int_t = 0x%x\n", v);

	/* control setup */
	v = 0x08 | (0x11 << 4); /* GAIN 64x , Mode = 2 */
	as726x_write_vreg(client, AS726X_VREG_CONTROL_SETUP, v);
	as726x_read_vreg(client, AS726X_VREG_CONTROL_SETUP, &v);
	printk("control_setup = 0x%x\n", v);

	err = iio_device_register(indio_dev);
	if (err < 0)
		goto exit_free_iio;

	INIT_DELAYED_WORK(&data->work, as726x_workfunc);
	schedule_delayed_work(&data->work,
		msecs_to_jiffies(data->int_time*2));

	return 0;

exit_free_iio:
	iio_device_free(indio_dev);
	return -1;
}

static int as726x_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct as726x_data *data = iio_priv(indio_dev);

	cancel_delayed_work(&data->work);

	iio_device_unregister(indio_dev);

	printk("remove\n");

	iio_device_free(indio_dev);

	return 0;
}

static const struct i2c_device_id as726x_id[] = {
	{"as7262", 0},
	{"as7263", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, as726x_id);

static const struct of_device_id as726x_of_match[] = {
	{ .compatible = "ams,as7262", },
	{ .compatible = "ams,as7263", },
	{ }
};
MODULE_DEVICE_TABLE(of, as726x_of_match);

static struct i2c_driver as726x_driver = {
	.driver = {
		.name	= "as726x",
		.of_match_table = as726x_of_match,
	},
	.probe		= as726x_probe,
	.remove		= as726x_remove,
	.id_table	= as726x_id,
};
module_i2c_driver(as726x_driver);

MODULE_AUTHOR("Wojciech Bieganski <wojtekbe@gmail.com>");
MODULE_DESCRIPTION("as726x driver");
MODULE_LICENSE("GPL");
