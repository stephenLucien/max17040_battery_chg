/*
 *  max17040_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/max17040_battery.h>
#include <linux/slab.h>

#define MAX17040_VCELL_MSB	0x02
#define MAX17040_VCELL_LSB	0x03
#define MAX17040_SOC_MSB	0x04
#define MAX17040_SOC_LSB	0x05
#define MAX17040_MODE_MSB	0x06
#define MAX17040_MODE_LSB	0x07
#define MAX17040_VER_MSB	0x08
#define MAX17040_VER_LSB	0x09
#define MAX17040_RCOMP_MSB	0x0C
#define MAX17040_RCOMP_LSB	0x0D
#define MAX17040_CMD_MSB	0xFE
#define MAX17040_CMD_LSB	0xFF

#define MAX17040_DELAY		1000
#define MAX17040_BATTERY_FULL	95

#define REIMP 1

#define MAX17049 0
#define HAVE_CHARGE_PIN 0

#if HAVE_CHARGE_PIN
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

struct max17040_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct power_supply		*battery;
	struct max17040_platform_data	*pdata;

	/* State Of Connect */
	int online;
	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* State Of Charge */
	int status;
};

#if HAVE_CHARGE_PIN
static int gpio_charge_pin=-1;
#endif

static int max17040_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17040_chip *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->soc;
		break;
#if MAX17049
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = 8400000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 6400000;
		break;	
#endif
	default:
		return -EINVAL;
	}
	return 0;
}

#if REIMP
#else
static int max17040_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;
	int cnt=0;
	int maxtry=5;

max17040_retry_write:
	++cnt;


	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0){
		//dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		
		if(cnt<=maxtry){
			printk("%s: err %d, retry %d\n", __func__, ret, cnt);
			goto max17040_retry_write;
		}
		else{
			printk("retry timeout, %s: err %d.\n", __func__, ret);
		}
	}

	return ret;
}

static int max17040_read_reg(struct i2c_client *client, int reg)
{
	int ret;
	int cnt=0;
	int maxtry=5;

max17040_retry_read:
	++cnt;
	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0){
		//dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		
		if(cnt<=maxtry){
			printk("%s: err %d, retry %d\n", __func__, ret, cnt);
			goto max17040_retry_read;
		}
		else{
			printk("retry timeout, %s: err %d.\n", __func__, ret);
		}
	}

	return ret;
}
#endif


static void max17040_reset(struct i2c_client *client)
{
#if REIMP
	int ret=0;
	u8 outbuf[3] ={MAX17040_CMD_MSB, 0x54, 0x00};

	ret = i2c_master_send(client, outbuf, 3);
	if (ret < 0) {
		//return ret;
		return;
	} else if (ret != sizeof(outbuf) ) {
		//return -EIO;
		return;
	}
#else
	max17040_write_reg(client, MAX17040_CMD_MSB, 0x54);
	max17040_write_reg(client, MAX17040_CMD_LSB, 0x00);
#endif

}

static void max17040_get_vcell(struct i2c_client *client)
{
	struct max17040_chip *chip;
	int msb;
	int lsb;
	
#if REIMP
	int ret=0;
	u8 outbuf[1],inbuf[2];
	outbuf[0]=MAX17040_VCELL_MSB;
	ret = i2c_master_send(client, outbuf, 1);
	if (ret < 0) {
		//return ret;
		return;
	} else if (ret != 1) {
		//return -EIO;
		//return;
	}

	inbuf[0]=0;inbuf[1]=0;
	ret = i2c_master_recv(client, inbuf, 2);
	if (ret < 0) {
		//return ret;
		return;
	} else if (ret != 2) {
		//return -EIO;
		//return;
	}
	msb=inbuf[0];
	lsb=inbuf[1];
#else
	
	msb = max17040_read_reg(client, MAX17040_VCELL_MSB);
	lsb = max17040_read_reg(client, MAX17040_VCELL_LSB);
#endif	
	
	chip = i2c_get_clientdata(client);
	if( msb<0 || lsb <0 || chip == NULL){
		//printk("error\n");
	}
	else {
#if MAX17049
		// kernel module cannot perform float point ops ?
    		//chip->vcell = 2 * ( (msb * 256 + lsb) * 78.125f / 1000.0f); //mV
		//
		// uV
		chip->vcell= ( msb * 64 + lsb / 4 ) * 625 ;
#else
		chip->vcell = ( (msb & 0x0f) << 4) + ( (lsb & 0xf0) >> 4);
#endif
	}
}

static void max17040_get_soc(struct i2c_client *client)
{
	struct max17040_chip *chip;
	int msb;
	int lsb;
#if REIMP
	int ret=0;
	u8 outbuf[1],inbuf[2];
	
	outbuf[0]=MAX17040_SOC_MSB;
	ret = i2c_master_send(client, outbuf, 1);
	if (ret < 0) {
		//return ret;
		return;
	} else if (ret != 1) {
		//return -EIO;
		return;
	}

	inbuf[0]=0;inbuf[1]=0;
	ret = i2c_master_recv(client, inbuf, 2);
	if (ret < 0) {
		//return ret;
		return;
	} else if (ret != 2) {
		//return -EIO;
		return;
	}
	msb=inbuf[0];
	lsb=inbuf[1];
#else
	msb = max17040_read_reg(client, MAX17040_SOC_MSB);
	lsb = max17040_read_reg(client, MAX17040_SOC_LSB);
#endif
	chip = i2c_get_clientdata(client);
	if( msb<0 
	|| lsb <0 
	|| chip == NULL
	){
		//printk("error\n");
	}
	else {
		chip->soc = msb>100?100:msb;
	}
	
}

static void max17040_get_version(struct i2c_client *client)
{
	int msb;
	int lsb;
#if REIMP
	int ret=0;
	u8 outbuf[1],inbuf[2];
	outbuf[0]=MAX17040_VER_MSB;
	ret = i2c_master_send(client, outbuf, 1);
	if (ret < 0) {
		//return ret;
		return;
	} else if (ret != 1) {
		//return -EIO;
		return;
	}

	inbuf[0]=0;inbuf[1]=0;
	ret = i2c_master_recv(client, inbuf, 2);
	if (ret < 0) {
		//return ret;
		return;
	} else if (ret != 2) {
		//return -EIO;
		return;
	}
	msb=inbuf[0];
	lsb=inbuf[1];
#else
	msb = max17040_read_reg(client, MAX17040_VER_MSB);
	lsb = max17040_read_reg(client, MAX17040_VER_LSB);
#endif
	if(msb<0 || lsb<0){
		//printk("[%s]: error.\n", __func__);
	}
	else {
		dev_info(&client->dev, "MAX17040 Fuel-Gauge Ver %.2X%.2X\n", msb, lsb);
	}
}

static void max17040_get_online(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	if (chip->pdata && chip->pdata->battery_online)
		chip->online = chip->pdata->battery_online();
	else
		chip->online = 1;
}

static void max17040_get_status(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
#if HAVE_CHARGE_PIN
	int value;
	if(gpio_charge_pin<0){
		chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
	}
	else{
		value=gpio_get_value(gpio_charge_pin);
		/* this charge GPIO pin activate low */
		if(value){
			chip->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		else{
			chip->status = POWER_SUPPLY_STATUS_CHARGING;
			return;
		}
	}
#else
	
	if(!chip->pdata){
		chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
		return;
	}

	if(!chip->pdata->charger_online){
		printk("%s: offline.\n",__FUNCTION__);
		chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
		return;
	}

	if (!chip->pdata->charger_enable) {
		printk("%s: disable.\n",__FUNCTION__);
		chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
		return;
	}

	if (chip->pdata->charger_online()) {
		if (chip->pdata->charger_enable())
			chip->status = POWER_SUPPLY_STATUS_CHARGING;
		else
			chip->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
#endif

	if (chip->soc > MAX17040_BATTERY_FULL)
		chip->status = POWER_SUPPLY_STATUS_FULL;
}

static void max17040_work(struct work_struct *work)
{
	struct max17040_chip *chip;

	chip = container_of(work, struct max17040_chip, work.work);

	max17040_get_vcell(chip->client);
	max17040_get_soc(chip->client);
	max17040_get_online(chip->client);
	max17040_get_status(chip->client);

	queue_delayed_work(system_power_efficient_wq, &chip->work,
			   MAX17040_DELAY);
}

static enum power_supply_property max17040_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
#if MAX17049
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,	
#endif
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,

};

static const struct power_supply_desc max17040_battery_desc = {
	.name		= "max17040_battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= max17040_get_property,
	.properties	= max17040_battery_props,
	.num_properties	= ARRAY_SIZE(max17040_battery_props),
};

static int max17040_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct power_supply_config psy_cfg = {};
	struct max17040_chip *chip;
	struct device_node *np;
#if HAVE_CHARGE_PIN
	int ret;
#endif

	printk("%s\n",__FUNCTION__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)){
		printk("error: max17040 i2c_check_functionality.\n");
		return -EIO;
	}
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip){
		printk("error: max17040 devm_kzalloc.\n");
		return -ENOMEM;
	}
	chip->client = client;
	chip->pdata = client->dev.platform_data;
	if(chip->pdata == NULL)
	{
		//printk("[ Line: %d, Func: %s ]: error, platform data is empty.\n",__LINE__,__FUNCTION__);
	}

	i2c_set_clientdata(client, chip);
	psy_cfg.drv_data = chip;

	chip->battery = power_supply_register(&client->dev,
				&max17040_battery_desc, &psy_cfg);
	if (IS_ERR(chip->battery)) {
		dev_err(&client->dev, "failed: power supply register\n");
		return PTR_ERR(chip->battery);
	}

	max17040_reset(client);
	max17040_get_version(client);

	INIT_DEFERRABLE_WORK(&chip->work, max17040_work);
	queue_delayed_work(system_power_efficient_wq, &chip->work,
			   MAX17040_DELAY);

#if HAVE_CHARGE_PIN
	np=client->dev.of_node;
	if(np){
		gpio_charge_pin = of_get_named_gpio(np, "charge_pin-gpio", 0);
		if(gpio_charge_pin<0){
			printk("invalid charger-pin %d\n",gpio_charge_pin);
		}
		else {
			printk("charger-pin %d\n",gpio_charge_pin);
			ret=gpio_direction_input(gpio_charge_pin);
			if(ret){
				printk("set charger-pin as input error: %X\n",ret);
				//gpio_free(gpio_charge_pin);
				gpio_charge_pin=-1;
				
			}
		}


	}
#endif

	return 0;
}

static int max17040_remove(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	power_supply_unregister(chip->battery);
	cancel_delayed_work(&chip->work);
#if HAVE_CHARGE_PIN
	if(gpio_charge_pin>=0)
	{
		//gpio_free(gpio_charge_pin);
	}
#endif
	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int max17040_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max17040_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work(&chip->work);
	return 0;
}

static int max17040_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max17040_chip *chip = i2c_get_clientdata(client);

	queue_delayed_work(system_power_efficient_wq, &chip->work,
			   MAX17040_DELAY);
	return 0;
}

static SIMPLE_DEV_PM_OPS(max17040_pm_ops, max17040_suspend, max17040_resume);
#define MAX17040_PM_OPS (&max17040_pm_ops)

#else

#define MAX17040_PM_OPS NULL

#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id max17040_id[] = {
	{ "max17040" },
	{ "max77836-battery" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17040_id);

static struct i2c_driver max17040_i2c_driver = {
	.driver	= {
		.name	= "max17040",
		.pm	= MAX17040_PM_OPS,
	},
	.probe		= max17040_probe,
	.remove		= max17040_remove,
	.id_table	= max17040_id,
};
module_i2c_driver(max17040_i2c_driver);

MODULE_AUTHOR("modify: stephenLucien; origin: Minkyu Kang <mk7.kang@samsung.com>");
MODULE_DESCRIPTION("MAX17040 Fuel Gauge");
MODULE_LICENSE("GPL");
