/*
 * LTC4291 IEEE 802.3at Quad Port Power-over-Ethernet PSE Controller
 *
 * Version 1.0 Release notes:
 * 1. Semiautomatic mode is untested.
 * 2. Legacy detect is untested.
 * 3. All condition values (periods, limits etc.) left as hardware default.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include "ltc4291.h"

static int ltc4291_read_reg(struct ltc4291_data *ddata, u8 reg)
{
	int res;
	res  = i2c_smbus_read_byte_data(ddata->client, reg);
	if (res < 0)
		dev_err(&ddata->client->dev, "read byte error %d\n", res);
	return res;
}

static int ltc4291_write_reg(struct ltc4291_data *ddata, u8 reg, u8 val)
{
	int res;
	res  = i2c_smbus_write_byte_data(ddata->client, reg, val);
	if (res < 0)
		dev_err(&ddata->client->dev, "write byte error %d\n", res);
	return res;
}

static int ltc4291_read(struct ltc4291_data *ddata,
			u8 reg, int len, void * values) {
	int res;
	u8 l;
	u8 *data = (char *)values;
	for (l = 0; l < len; l++) {
		res = i2c_smbus_read_byte_data(ddata->client, reg + l);
		if (res < 0) {
			dev_err(&ddata->client->dev, "read error %d\n", res);
			return res;
		}
		data[l] = (u8)res;
	}
	return l;
}

static int ltc4291_irq_enable(struct ltc4291_data *ddata, u8 interrupts) {
	return ltc4291_write_reg(ddata, INTMASK_REG, interrupts);
}

static int ltc4291_irq_clear(struct ltc4291_data *ddata, int irqn) {
	return ltc4291_write_reg(ddata, RSTPB_REG, irqn);
}

static irqreturn_t ltc4291_irq_handler(int irq, void *device_data)
{
	int i;
	u8 ports;
	u8 irq_flags;
	struct ltc4291_data *ddata = (struct ltc4291_data *)device_data;
	irq_flags = ltc4291_read_reg(ddata, INTSTAT_REG);
	/* Critical faults */
	if (TCUT_EVENT & irq_flags) {
		/* Icut */
		ports = ltc4291_read_reg(ddata, FLTEVN_REG);
		for (i = 0; i < LTC4291_PORTS_NUM; i++) {
			if (ports & (1 << i))
				dev_info(&ddata->client->dev,
				    "port%d: overload occured", i);
		}
		/* Ilim */
		ports = ltc4291_read_reg(ddata, TSEVN_REG);
		for (i = 0; i < LTC4291_PORTS_NUM; i++) {
			if (ports & (1 << (i + 4)))
				dev_info(&ddata->client->dev,
				    "port%d: exceeded current limit", i);
		}
	}
	if (SUPPLY_EVENT & irq_flags)
		dev_info(&ddata->client->dev, "power supply fault\n");
	/* Deferrable events: PEC, PGC, DISF, DETC, CLASC, STRTF */
	if (TSTRAT_EVENT & irq_flags) {
		ports = ltc4291_read_reg(ddata, TSEVN_REG);
		for (i = 0; i < LTC4291_PORTS_NUM; i++) {
			if (ports & (1 << i))
				dev_info(&ddata->client->dev,
				    "port%d: start fault", i);
		}
	}
	if (DIS_EVENT & irq_flags) {
		ports = ltc4291_read_reg(ddata, FLTEVN_REG);
		for (i = 0; i < LTC4291_PORTS_NUM; i++) {
			if (ports & (1 << (i + 4)))
				dev_info(&ddata->client->dev,
				    "port%d: powered device disconnected", i);
		}
	}
	if (CLASS_EVENT & irq_flags) {
		ports = ltc4291_read_reg(ddata, DETEVN_REG);
		for (i = 0; i < LTC4291_PORTS_NUM; i++) {
			if (ports & (1 << (i + 4)))
				dev_info(&ddata->client->dev,
				    "port%d: powered device connected", i);
		}
	}
	ltc4291_irq_clear(ddata, CLRAIN);
	return IRQ_HANDLED;
}

int port_is_valid(struct device *dev, int pt_number) {
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	if (ddata->pdata.pt_df[pt_number].enable == 1)
		return 1;
	else
		return 0;
}

static u8 append_port_mode_reg(u8 old_mode_reg, int pt_number, int mode) {
	u8 new_mode_reg;
	u8 pt_mask;
	pt_mask = ~(PT_MODE_MASK << (pt_number * PT_MODE_FIELD_WIDTH));
	new_mode_reg = mode << (pt_number * PT_MODE_FIELD_WIDTH);
	new_mode_reg = new_mode_reg | (pt_mask & old_mode_reg);
	return new_mode_reg;
}

static int ltc4291_get_operation_mode(struct device *dev, int porti) {
	u8 mode;
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	mode = ltc4291_read_reg(ddata, OPMD_REG);
	mode = (mode >> porti * PT_MODE_FIELD_WIDTH) & PT_MODE_MASK;
	return (int)mode;
}

static int ltc4291_set_operation_mode(struct device *dev, int porti, int mode) {
	u8 mcfg;
	struct ltc4291_data *ddata = dev_get_drvdata(dev);

	mcfg = ltc4291_read_reg(ddata, OPMD_REG);
	mcfg = append_port_mode_reg(mcfg, porti, mode);
	ltc4291_write_reg(ddata, OPMD_REG, mcfg);

	if (mode == PORT_MODE_MANUAL) {
		/* Disable connection monitoring */
		mcfg = ltc4291_read_reg(ddata, DETENA_REG);
		mcfg &= ~(DET_CLAS_EN << porti);
		ltc4291_write_reg(ddata, DETENA_REG, mcfg);
		mcfg = ltc4291_read_reg(ddata, DISENA_REG);
		mcfg &= ~(DC_EN << porti);
		ltc4291_write_reg(ddata, DISENA_REG, mcfg);
	} else if (mode == PORT_MODE_AUTO) {
		/* Enable connect detection */
		mcfg = ltc4291_read_reg(ddata, DETENA_REG);
		mcfg |= DET_CLAS_EN << porti;
		ltc4291_write_reg(ddata, DETENA_REG, mcfg);
		mcfg = ltc4291_read_reg(ddata, DISENA_REG);
		mcfg |= DC_EN << porti;
		ltc4291_write_reg(ddata, DISENA_REG, mcfg);
	}
	return 0;
}

static int ltc4291_switch_port_power(struct device *dev, int porti, int state) {
	int res;
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	if (porti >= 0 && porti < LTC4291_PORTS_NUM) {
		if (PORT_MODE_AUTO == ltc4291_get_operation_mode(dev, porti))
			return -EINVAL;
		if (state == 1)
			state = PORT_SELECT(porti) << PT_PWR_ON_SHIFT;
		else
			state = PORT_SELECT(porti) << PT_PWR_OFF_SHIFT;
	}
	else
		return (res = -EINVAL);
	return ltc4291_write_reg(ddata, PWRPB_REG, state);
}

static u8 mask_value (u8 value, u8 offset,  u8 length)
{
	return (u8)(((((u8)(1 << length ) - 1) << offset) & value) >> offset);
}	

static u8 xor_value (u8 value, u8 offset, u8 length, u8 config)
{
	value &= ~(((1 << length) - 1) << offset);
	
//	printk(KERN_INFO"%s: %x %x %x %x %x\n", __func__, value, offset, length, config, config << offset);
	return (value | (config << offset));
}

static int ltc4291_set_legen(struct device *dev, int port, int val) {
	
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	u8 value = ltc4291_read_reg(ddata, HPMD_REG(port));
	u8 write_value = xor_value (value, LEGEN_BIT, 1, val);

	return ltc4291_write_reg(ddata, HPMD_REG(port), (u8)write_value);
}	

static int ltc4291_set_pse_avail_pwr_ss(struct device *dev, int port, int val) {
	
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	u8 value = ltc4291_read_reg(ddata, HPMD_REG(port));
	u8 write_value = xor_value (value, PSE_AVAIL_PWR_SS_BIT, PSE_AVAIL_PWR_SS_LEN, val);
	
//	dev_info(dev, "%s: %x %x %x %x\n", __func__, value, write_value, HPMD_REG(port), val);
	return ltc4291_write_reg(ddata, HPMD_REG(port), (u8)write_value);
}	

static int ltc4291_set_pse_avail_pwr_ds(struct device *dev, int port, int val) {
	
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	u8 value = ltc4291_read_reg(ddata, HPMD_REG(port));
	u8 write_value = xor_value (value, PSE_AVAIL_PWR_DS_BIT, PSE_AVAIL_PWR_DS_LEN, val);

//	dev_info(dev, "%s: %x %x %x %x\n", __func__, value, write_value, HPMD_REG(port), val);
	return ltc4291_write_reg(ddata, HPMD_REG(port), (u8)write_value);
}	

static int ltc4291_set_dis(struct device *dev, int port, int val) {
	
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	u8 value = ltc4291_read_reg(ddata, CUT_REG(port));
	u8 write_value = xor_value (value, DIS_BIT, 1, val);
	return ltc4291_write_reg(ddata, CUT_REG(port), (u8)write_value);
}

static int ltc4291_set_cutrng(struct device *dev, int port, int val) {
	
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	u8 value = ltc4291_read_reg(ddata, CUT_REG(port));
	u8 write_value = xor_value (value, CUTRNG_BIT, 1, val);
	return ltc4291_write_reg(ddata, CUT_REG(port), (u8)write_value);
}

static int ltc4291_set_cut(struct device *dev, int port, int val) {
	
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	u8 value = ltc4291_read_reg(ddata, CUT_REG(port));
	u8 write_value = xor_value (value, CUT_BIT, CUT_LEN, val);
	return ltc4291_write_reg(ddata, CUT_REG(port), (u8)write_value);
}

static int ltc4291_set_lim(struct device *dev, int port, int val) {
	
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	return ltc4291_write_reg(ddata, LIM_REG(port), (u8)val);
}

static int ltc4291_get_pse_allocated_pwr(struct device *dev, int port, int channel) {
	
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	u8 value = ltc4291_read_reg(ddata, HPSTAT_REG(port));
	u8 offset;
	
	offset = (channel == 0)? PSE_ALLOCATED_PWR_A_BIT : PSE_ALLOCATED_PWR_B_BIT;
	return mask_value(value, offset, PSE_ALLOCATED_PWR_LEN);
}

static int ltc4291_get_fetbad(struct device *dev, int port) {
	
	int res;
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	u8 value = ltc4291_read_reg(ddata, HPSTAT_REG(port));
	return mask_value(value, FETBAD_BIT, 1);
}

static int ltc4291_get_pd_acs_req(struct device *dev, int port) {
	int res;
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	u8 value = ltc4291_read_reg(ddata, HPSTAT_REG(port));
	return mask_value(value, PD_ACS_REQ_BIT, 1);
}

static int ltc4291_apply_dt_defaults(struct device *dev) {
	int i;
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	for (i = 0; i < LTC4291_PORTS_NUM; i++) {
		if (!port_is_valid(dev, i))
			ltc4291_set_operation_mode(dev, i, PORT_MODE_OFF);
		else {
			ltc4291_set_operation_mode(dev, i, ddata->pdata.pt_df[i].mode);
			if (ddata->pdata.pt_df[i].mode == PORT_MODE_MANUAL)
				ltc4291_switch_port_power(dev, i,
				    ddata->pdata.pt_df[i].pwr);
			ltc4291_set_pse_avail_pwr_ss(dev, i,
                                    ddata->pdata.pt_df[i].max_class_s_signature - 3);
			ltc4291_set_pse_avail_pwr_ds(dev, i,
                                    ddata->pdata.pt_df[i].max_class_d_signature - 3);
		}
	}
	return 0;
}

static ssize_t ltc4291_show_temp(struct device *dev,
		struct device_attribute *attr, char *buf) {
	int val;
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	val = ltc4291_read_reg(ddata, TEMPERATURE_REG);
	val = val * TEMP_LSB - 20000;
	return snprintf(buf, 6, "%d.%01d\n", val/1000, (val%1000)/100);
}

static ssize_t ltc4291_show_vin(struct device *dev,
		struct device_attribute *attr, char *buf) {
	u16 val;
	long vin;
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	ltc4291_read(ddata, VEELSB_REG, INPUT_V_SZ, &val);
	vin = VOLT_LSB * __le16_to_cpu(val);
	return snprintf(buf, 6, "%ld.%01ld\n",
			vin/1000000, (vin%1000000)/100000);
}

static ssize_t ltc4291_store_port_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	u8 mode;
	int port_idx = ASCII_TO_DIGIT(buf[0]);
	if (!port_is_valid(dev, port_idx))
		return -EINVAL;
	if (!strncmp(buf + 1, "off", 3))
		mode = PORT_MODE_OFF;
	else if (!strncmp(buf + 1, "manual", 6))
		mode = PORT_MODE_MANUAL;
	else if (!strncmp(buf + 1, "semiauto", 8))
		mode = PORT_MODE_SEMIAUTO;
	else if (!strncmp(buf + 1, "auto", 4))
		mode = PORT_MODE_AUTO;
	else
		return -EINVAL;
	ltc4291_set_operation_mode(dev, port_idx, mode);
	return count;
}

static ssize_t ltc4291_store_port_power(struct device *dev,
		const char *buf, size_t count, int state) {
	int res;
	long port_idx;
	res = kstrtol(buf, 10, &port_idx);
	if (res != 0)
		return res;
	if (!port_is_valid(dev, port_idx))
		return -EINVAL;
	res = ltc4291_switch_port_power(dev, port_idx, state);
	if (res == 0)
		return count;
	else
		return res;
}

static ssize_t ltc4291_store_port_power_on(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	return ltc4291_store_port_power(dev, buf, count, P_ON);
}

static ssize_t ltc4291_store_port_power_off(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	return ltc4291_store_port_power(dev, buf, count, P_OFF);
}

static ssize_t ltc4291_show_port_info(struct device *dev,
		struct device_attribute *attr, char *buf) {
	int i;
	int len;
	unsigned long val = 0;
	struct ltc4291_data *ddata = dev_get_drvdata(dev);
	ltc4291_read(ddata, PT_DC_REGS, PT_DC_INFO_SZ, ddata->pt_dc);
	len = sprintf(buf, "# mode power current\n"); /* column names */
	for (i = 0; i < LTC4291_PORTS_NUM; i++) {
		/* Port number */
		len += sprintf(buf+len, "%d ", i);
		/* Port mode of operation */
		val = ltc4291_get_operation_mode(dev, i);
		if (val == PORT_MODE_OFF)
			len += sprintf(buf+len, "off ");
		else if (val == PORT_MODE_MANUAL)
			len += sprintf(buf+len, "manual ");
		else if (val == PORT_MODE_SEMIAUTO)
			len += sprintf(buf+len, "semiauto ");
		else if (val == PORT_MODE_AUTO)
			len += sprintf(buf+len, "auto ");
		else
			len += sprintf(buf+len, "unknown ");
		/* Port voltage */
		if (!port_is_valid(dev, i))
			len += sprintf(buf+len,"* ");
		else {
			val = WA_LSB * __le16_to_cpu(ddata->pt_dc[i].v);
			len += sprintf(buf+len,"%lu.%01lu ",
					val/1000000, (val%1000000)/100000);
		}
		/* Port current intensity */
		if (!port_is_valid(dev, i))
			len += sprintf(buf+len,"*\n");
		else {
			val = CURR_S250_LSB * __le16_to_cpu(ddata->pt_dc[i].i);
			len += sprintf(buf+len, "%lu.%03lu\n",
					val/1000000000, val/1000000);
		}
	}
	return len;
}

static DEVICE_ATTR(temp, S_IRUGO, ltc4291_show_temp, NULL);
static DEVICE_ATTR(vin, S_IRUGO, ltc4291_show_vin, NULL);
static DEVICE_ATTR(port_mode, S_IWUSR|S_IWGRP,
		NULL, ltc4291_store_port_mode);
static DEVICE_ATTR(port_power_on, S_IWUSR|S_IWUSR,
		NULL, ltc4291_store_port_power_on);
static DEVICE_ATTR(port_power_off, S_IWUSR|S_IWUSR,
		NULL, ltc4291_store_port_power_off);
static DEVICE_ATTR(port_info, S_IRUGO,
		ltc4291_show_port_info, NULL);

static struct attribute *ltc4291_attributes[] = {
	&dev_attr_temp.attr,
	&dev_attr_vin.attr,
	&dev_attr_port_mode.attr,
	&dev_attr_port_power_on.attr,
	&dev_attr_port_power_off.attr,
	&dev_attr_port_info.attr,
	NULL,
};

static const struct attribute_group ltc4291_attr_group = {
	.attrs = ltc4291_attributes,
};

static const struct of_device_id ltc4291_dt_id[] = {
	{ .compatible = "analog,ltc4291"},
	{ }
};

static int ltc4291_of_probe(struct device *dev,
		struct ltc4291_platform_data *pdata) {
	int i;
	const char *pmode;
	struct device_node *np = dev->of_node;
	struct device_node *child;

	if (!of_match_device(ltc4291_dt_id, dev))
		return -ENODEV;

	if ((!np) || !of_get_next_child(np, NULL))
		return -EINVAL;
	
	of_property_read_u32(np, "irq-gpio", &pdata->irq);
	i = 0;
	for_each_child_of_node(np, child) {
		of_property_read_u32(child, "enable", &pdata->pt_df[i].enable);

		if (0 == of_property_read_string(child, "mode", &pmode)) {
			if (!strncmp(pmode, "manual", 6))
				pdata->pt_df[i].mode = PORT_MODE_MANUAL;
			else if (!strncmp(pmode, "semiauto", 8))
				pdata->pt_df[i].mode = PORT_MODE_SEMIAUTO;
			else if (!strncmp(pmode, "auto", 4))
				pdata->pt_df[i].mode = PORT_MODE_AUTO;
			else
				pdata->pt_df[i].mode = PORT_MODE_OFF;
		}
		of_property_read_u32(child, "power", &pdata->pt_df[i].pwr);
		of_property_read_u32(child, "max-class-single-signature", &pdata->pt_df[i].max_class_s_signature);
		of_property_read_u32(child, "max-class-dual-signature", &pdata->pt_df[i].max_class_d_signature);
		i++;
	}
	return 0;
}

static int ltc4291_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{	
	int res;
	static struct ltc4291_data *ddata;
	struct ltc4291_platform_data *pdata;
	struct device *dev = &client->dev;

	ddata = devm_kzalloc(dev, sizeof(struct ltc4291_data), GFP_KERNEL);
	if (ddata == NULL)
		return -ENOMEM;
	res = sysfs_create_group(&dev->kobj, &ltc4291_attr_group);
	if (res) {
		dev_err(dev, "failed to create sysfs attributes\n");
		devm_kfree(&client->dev, ddata);
		return res;	
	}
	ddata->client = client;
	i2c_set_clientdata(client, ddata);
	dev_set_drvdata(dev, ddata);
	pdata = &ddata->pdata;
	res = ltc4291_of_probe(dev, pdata);
	ltc4291_apply_dt_defaults(dev);

	dev_info(dev, "LTC4291 ID 0x%x, silicon rev. %u, fw %u",
			ltc4291_read_reg(ddata, DEVID_REG) >> 5 & 0x7,
			ltc4291_read_reg(ddata, DEVID_REG) & 0x1F,
			ltc4291_read_reg(ddata, FIRMWARE_REG) & 0x7);

	ltc4291_irq_enable(ddata, SUPPLY_EN | TSTRAT_EN | TCUT_EN | CLASS_EN | DIS_EN);
	res = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					ltc4291_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"ltc4291", ddata);
	if (res) {
		dev_err(dev, "failed to register interrupt\n");
		sysfs_remove_group(&client->dev.kobj, &ltc4291_attr_group);
		devm_kfree(&client->dev, ddata);
		return res;
	}
	ltc4291_irq_clear(ddata, SUPPLY_EN);
	ltc4291_irq_handler(client->irq, ddata);
	return 0;
}

static int ltc4291_remove(struct i2c_client *client)
{
	struct ltc4291_data *ddata = i2c_get_clientdata(client);
	devm_free_irq(&client->dev, client->irq, ddata);
	sysfs_remove_group(&client->dev.kobj, &ltc4291_attr_group);
	devm_kfree(&client->dev, ddata);
	return 0;
}

static const struct i2c_device_id ltc4291_id[] = {
	{ "ltc4291", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltc4291_id);

static struct i2c_driver ltc4291_driver = {
	.driver = {
		.name   = "ltc4291",
		.owner  = THIS_MODULE,
		.of_match_table = ltc4291_dt_id,
	},
	.probe          = ltc4291_probe,
	.remove         = ltc4291_remove,
	.id_table	= ltc4291_id,
};

static int __init ltc4291_init( void ) {
	return i2c_add_driver(&ltc4291_driver);
}

static void __exit ltc4291_exit( void ) {
	i2c_del_driver(&ltc4291_driver);
}

module_init( ltc4291_init );
module_exit( ltc4291_exit );

MODULE_AUTHOR("Shih Yuan Huang <timmy@ubiqconn.com>");
MODULE_DESCRIPTION("LTC4291 Quad Port POE PSE Controller Driver");
MODULE_LICENSE("GPL v2");
