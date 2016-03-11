/*
 * Driver for Semtech SX8651 I2C touchscreen controller.
 *
 * Copyright (c) 2015-2016 Aesys S.p.A.
 *      Moris Ravasio <moris.ravasio@aesys.com>
 *
 * Using code from:
 *  - sx8654.c
 *      S�bastien Szymanski <sebastien.szymanski@armadeus.com>
 *  - sx865x.c
 *	Copyright (c) 2013 U-MoBo Srl
 *	Pierluigi Passaro <p.passaro@u-mobo.com>
 *  - sx8650.c
 *      Copyright (c) 2009 Wayne Roberts
 *  - tsc2007.c
 *      Copyright (c) 2008 Kwangwoo Lee
 *  - ads7846.c
 *      Copyright (c) 2005 David Brownell
 *      Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *      Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *      Copyright (C) 2002 MontaVista Software
 *      Copyright (C) 2004 Texas Instruments
 *      Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/input.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>

/* register addresses */
#define I2C_REG_TOUCH0			0x00
#define I2C_REG_TOUCH1			0x01
#define I2C_REG_CHANMASK		0x04
#define I2C_REG_IRQMASK			0x22
#define I2C_REG_IRQSRC			0x23
#define I2C_REG_SOFTRESET		0x3f

/* commands */
#define CMD_READ_REGISTER		0x40
#define CMD_MANUAL			0xc0
#define CMD_PENTRG			0xe0

/* value for I2C_REG_SOFTRESET */
#define SOFTRESET_VALUE			0xde

/* bits for I2C_REG_IRQSRC */
#define IRQ_PENTOUCH_TOUCHCONVDONE	0x08
#define IRQ_PENRELEASE			0x04

/* bits for RegTouch1 */
#define CONDIRQ				0x20
#define FILT_7SA			0x03

/* bits for I2C_REG_CHANMASK */
#define CONV_X				0x80
#define CONV_Y				0x40

/* coordinates rate: higher nibble of CTRL0 register */
#define RATE_MANUAL			0x00
#define RATE_5000CPS			0xf0

/* power delay: lower nibble of CTRL0 register */
#define POWDLY_1_1MS			0x0b

#define MAX_12BIT			((1 << 12) - 1)

#define NULL_READ_BEFORE_TOUCH_RELEASE	3

struct sx8651 {
	struct input_dev *input;
	struct i2c_client *client;
};

static void sx8651_readloop(void *handle)
{
	struct sx8651 *sx8651 = handle;
	u8 data[4];
	unsigned int x, y;
	int retval;
	int nullcounter;

	nullcounter = 0;

	/* go on reading from touchscreen channels as long as the touchscreen is being pressed */
	while(1)
	{
		retval = i2c_master_recv(sx8651->client, data, sizeof(data));

		/* invalid response */
		if (unlikely(retval != sizeof(data)))
			nullcounter++;
		else
		{
			/* invalid data */
			if (unlikely(data[0] & 0x80 || data[2] & 0x80))
				nullcounter++;
			else
			{
				nullcounter = 0;

				x = ((data[0] & 0xf) << 8) | (data[1]);
				y = ((data[2] & 0xf) << 8) | (data[3]);

				input_report_abs(sx8651->input, ABS_X, x);
				input_report_abs(sx8651->input, ABS_Y, y);
				input_report_key(sx8651->input, BTN_TOUCH, 1);
				input_sync(sx8651->input);

				dev_dbg(&sx8651->client->dev, "point(%4d,%4d)\n", x, y);
			}
		}

		/* check if the touchscreen has been released */
		if(nullcounter >= NULL_READ_BEFORE_TOUCH_RELEASE)
		{
			input_report_key(sx8651->input, BTN_TOUCH, 0);
			input_sync(sx8651->input);

			break;
		}

		/* breath */
		msleep(20);
	}
}

static irqreturn_t sx8651_irq(int irq, void *handle)
{
	struct sx8651 *sx8651 = handle;

	dev_dbg(&sx8651->client->dev, "pen touch interrupt");

	/* perform read */
	sx8651_readloop(sx8651);

	return IRQ_HANDLED;
}

static int sx8651_open(struct input_dev *dev)
{
    /* Simply does nothing, since the chip has already been configured and enabled in probe */
	return 0;
}

static void sx8651_close(struct input_dev *dev)
{
    /* Simply does nothing: the chip is left configured and enabled */
}

static int sx8651_init(struct i2c_client *client)
{
    int error;
    
    error = i2c_smbus_write_byte_data(client, I2C_REG_SOFTRESET,
					  SOFTRESET_VALUE);
	if (error) {
		dev_err(&client->dev, "writing softreset value failed");
		return error;
	}

	error = i2c_smbus_write_byte_data(client, I2C_REG_CHANMASK,
					  CONV_X | CONV_Y);
	if (error) {
		dev_err(&client->dev, "writing to I2C_REG_CHANMASK failed");
		return error;
	}

	error = i2c_smbus_write_byte_data(client, I2C_REG_IRQMASK,
					  IRQ_PENTOUCH_TOUCHCONVDONE |
						IRQ_PENRELEASE);
	if (error) {
		dev_err(&client->dev, "writing to I2C_REG_IRQMASK failed");
		return error;
	}

	error = i2c_smbus_write_byte_data(client, I2C_REG_TOUCH1,
					  CONDIRQ | FILT_7SA);
	if (error) {
		dev_err(&client->dev, "writing to I2C_REG_TOUCH1 failed");
		return error;
	}

	/* set data rate */
    error = i2c_smbus_write_byte_data(client, I2C_REG_TOUCH0,
	                  POWDLY_1_1MS);
	if (error) {
		dev_err(&client->dev, "writing to I2C_REG_TOUCH0 failed");
		return error;
	}

    /* enable pen trigger mode */
	error = i2c_smbus_write_byte(client, CMD_PENTRG);
	if (error) {
		dev_err(&client->dev, "writing command CMD_PENTRG failed");
		return error;
	}
    
    return error;
}

static ssize_t sx8651_reinit(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
    struct sx8651 *priv = i2c_get_clientdata(client);

    /* Perform initialization again */
    sx8651_init(priv->client);
    
    /* perform read */
	sx8651_readloop(priv);
    
    return count;
}

static DEVICE_ATTR(reinit, S_IWUSR | S_IWGRP | S_IWOTH, NULL, sx8651_reinit);

static struct attribute *sx8651_attributes[] = {
	&dev_attr_reinit.attr,
	NULL,
};

static const struct attribute_group sx8651_attr_group = {
	.attrs = sx8651_attributes,
};

static const struct attribute_group *sx8651_attr_groups[] = {
    &sx8651_attr_group,
    NULL,
};

static int sx8651_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sx8651 *sx8651;
	struct input_dev *input;
	int error;

    dev_dbg(&client->dev, "probing...");

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -ENXIO;

	sx8651 = devm_kzalloc(&client->dev, sizeof(*sx8651), GFP_KERNEL);
	if (!sx8651)
		return -ENOMEM;

	input = devm_input_allocate_device(&client->dev);
	if (!input)
		return -ENOMEM;

	input->name = "SX8651 I2C Touchscreen";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
    input->dev.groups = sx8651_attr_groups;
	input->open = sx8651_open;
	input->close = sx8651_close;

	__set_bit(INPUT_PROP_DIRECT, input->propbit);

	input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	
	input_set_abs_params(input, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, MAX_12BIT, 0, 0);

	sx8651->client = client;
	sx8651->input = input;

	input_set_drvdata(sx8651->input, sx8651);

    /* Perform initialization */
	sx8651_init(client);
    
    /* request and enable irq */
	error = devm_request_threaded_irq(&client->dev, client->irq,
					  NULL, sx8651_irq,
					  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					  client->name, sx8651);
                      
	if (error) {
		dev_err(&client->dev,
			"Failed to enable IRQ %d, error: %d\n",
			client->irq, error);
		return error;
	}
    
    /* perform read */
	sx8651_readloop(sx8651);
    
    /* register input device */
	error = input_register_device(sx8651->input);
	if (error)
		return error;

	i2c_set_clientdata(client, sx8651);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sx8651_of_match[] = {
	{ .compatible = "semtech,sx8651", },
	{ },
};
MODULE_DEVICE_TABLE(of, sx8651_of_match);
#endif

static const struct i2c_device_id sx8651_id_table[] = {
	{ "semtech_sx8651", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sx8651_id_table);

static struct i2c_driver sx8651_driver = {
	.driver = {
		.name = "sx8651",
		.of_match_table = of_match_ptr(sx8651_of_match),
	},
	.id_table = sx8651_id_table,
	.probe = sx8651_probe,
};
module_i2c_driver(sx8651_driver);

MODULE_AUTHOR("Moris Ravasio <moris.ravasio@aesys.com>");
MODULE_DESCRIPTION("Semtech SX8651 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
