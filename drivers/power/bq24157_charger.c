/*
 * BQ2591x battery charging driver
 *
 * Copyright (C) 2017 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#define pr_fmt(fmt) "[bq2415x]:%s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/alarmtimer.h>
#include <linux/qpnp/power-on.h>

#include "bq24157_reg.h"

enum bq2415x_part_no {
	BQ24157 = 0x02,
};

enum {
	USER		= BIT(0),
	JEITA		= BIT(1),
	BATT_FC		= BIT(2),
	BATT_PRES	= BIT(3),
};


enum wakeup_src {
	WAKEUP_SRC_MONITOR = 0,
	WAKEUP_SRC_JEITA,
	WAKEUP_SRC_MAX,
};

#define WAKEUP_SRC_MASK (~(~0 << WAKEUP_SRC_MAX))
struct bq2415x_wakeup_source {
	struct wakeup_source source;
	unsigned long enabled_bitmap;
	spinlock_t ws_lock;
};

struct bq2415x_otg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};


struct bq2415x {
	struct device	*dev;
	struct i2c_client *client;
	
	enum bq2415x_part_no part_no;
	int revision;

	struct mutex data_lock;
	struct mutex i2c_rw_lock;
	struct mutex profile_change_lock;
	struct mutex charging_disable_lock;
	struct mutex irq_complete;

	struct bq2415x_wakeup_source bq2415x_ws;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	bool batt_present;
	bool usb_present;

	bool batt_full;

	bool otg_enabled;
	bool batfet_enabled;
	bool in_hiz;
	bool dis_safety;
	
	bool power_good;
	bool vbus_good;

	bool charge_enabled;

	int chg_mv;
	int chg_ma;
	int ivl_mv;
	int icl_ma;
	
	int safety_chg_mv;
	int safety_chg_ma;

	int iterm_ma;
	int batlow_mv;
	
	int sensor_mohm;
	
	bool enable_term;

	bool boost_mode;
	bool otg_pin_status;
	
	int fault_status;

	int prev_stat_flag;
	int prev_fault_flag;

	int reg_stat;
	int reg_fault;
	int reg_stat_flag;
	int reg_fault_flag;

	/* if use software jeita in case of NTC is connected to gauge */
	bool software_jeita_supported;
	bool jeita_active;

	bool batt_hot;
	bool batt_cold;
	bool batt_warm;
	bool batt_cool;

	int batt_hot_degc;
	int batt_warm_degc;
	int batt_cool_degc;
	int batt_cold_degc;
	int hot_temp_hysteresis;
	int cold_temp_hysteresis;

	int batt_cool_ma;
	int batt_warm_ma;
	int batt_cool_mv;
	int batt_warm_mv;


	int batt_temp;

	int jeita_ma;
	int jeita_mv;

	unsigned int thermal_levels;
	unsigned int therm_lvl_sel;
	unsigned int *thermal_mitigation;

	int usb_psy_ma;
	int charge_state;
	int charging_disabled_status;

	int gpio_cd; /* CD pin control */

	int skip_reads;
	int skip_writes;

	struct delayed_work discharge_jeita_work; /*normal no charge mode*/
	struct delayed_work charge_jeita_work; /*charge mode jeita work*/
	struct delayed_work vbus_changed_work;
	
	struct alarm jeita_alarm;

	struct dentry *debug_root;

	struct bq2415x_otg_regulator otg_vreg;

	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	struct power_supply batt_psy;
};

static struct bq2415x *g_bq; 
static int BatteryTestStatus_enable = 0;

static int __bq2415x_read_reg(struct bq2415x *bq, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}
	*data = (u8)ret;
	return 0;
}

static int __bq2415x_write_reg(struct bq2415x *bq, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(bq->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
				val, reg, ret);
		return ret;
	}
	return 0;
}

static int bq2415x_read_byte(struct bq2415x *bq, u8 reg, u8 *data)
{
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2415x_read_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}


static int bq2415x_write_byte(struct bq2415x *bq, u8 reg, u8 data)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2415x_write_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

	return ret;
}


static int bq2415x_update_bits(struct bq2415x *bq, u8 reg,
					u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2415x_read_reg(bq, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq2415x_write_reg(bq, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

static void bq2415x_stay_awake(struct bq2415x_wakeup_source *source,
	enum wakeup_src wk_src)
{
	unsigned long flags;

	spin_lock_irqsave(&source->ws_lock, flags);

	if (!__test_and_set_bit(wk_src, &source->enabled_bitmap)) {
		__pm_stay_awake(&source->source);
		pr_debug("enabled source %s, wakeup_src %d\n",
			source->source.name, wk_src);
	}
	spin_unlock_irqrestore(&source->ws_lock, flags);
}

static void bq2415x_relax(struct bq2415x_wakeup_source *source,
	enum wakeup_src wk_src)
{
	unsigned long flags;

	spin_lock_irqsave(&source->ws_lock, flags);
	if (__test_and_clear_bit(wk_src, &source->enabled_bitmap) &&
		!(source->enabled_bitmap & WAKEUP_SRC_MASK)) {
		__pm_relax(&source->source);
		pr_debug("disabled source %s\n", source->source.name);
	}
	spin_unlock_irqrestore(&source->ws_lock, flags);

	pr_debug("relax source %s, wakeup_src %d\n",
		source->source.name, wk_src);
}

static void bq2415x_wakeup_src_init(struct bq2415x *bq)
{
	spin_lock_init(&bq->bq2415x_ws.ws_lock);
	wakeup_source_init(&bq->bq2415x_ws.source, "bq2415x");
}



static int bq2415x_enable_charger(struct bq2415x *bq)
{
	int ret;
	u8 val = BQ2415X_CHARGE_ENABLE << BQ2415X_CHARGE_ENABLE_SHIFT;

	ret = bq2415x_update_bits(bq, BQ2415X_REG_01,
				BQ2415X_CHARGE_ENABLE_MASK, val);
	return ret;
}

static int bq2415x_disable_charger(struct bq2415x *bq)
{
	int ret;
	u8 val = BQ2415X_CHARGE_DISABLE << BQ2415X_CHARGE_ENABLE_SHIFT;

	ret = bq2415x_update_bits(bq, BQ2415X_REG_01,
				BQ2415X_CHARGE_ENABLE_MASK, val);

	return ret;
}

static int bq2415x_enable_term(struct bq2415x *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2415X_TERM_ENABLE;
	else
		val = BQ2415X_TERM_DISABLE;

	val <<= BQ2415X_TERM_ENABLE_SHIFT;

	ret = bq2415x_update_bits(bq, BQ2415X_REG_01,
				BQ2415X_TERM_ENABLE_MASK, val);

	return ret;
}

int bq2415x_reset_chip(struct bq2415x *bq)
{
	int ret;
	u8 val = BQ2415X_RESET << BQ2415X_RESET_SHIFT;

	ret = bq2415x_update_bits(bq, BQ2415X_REG_04,
				BQ2415X_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2415x_reset_chip);

static int bq2415x_set_vbatlow_volt(struct bq2415x *bq, int volt)
{
	int ret;
	u8 val;

	val = (volt - BQ2415X_WEAK_BATT_VOLT_BASE) / BQ2415X_WEAK_BATT_VOLT_LSB;

	val <<= BQ2415X_WEAK_BATT_VOLT_SHIFT;

	pr_debug("val:0x%02X\n", val);
	
	ret = bq2415x_update_bits(bq, BQ2415X_REG_01,
				BQ2415X_WEAK_BATT_VOLT_MASK, val);

	return ret;
}

static int bq2415x_set_safety_reg(struct bq2415x *bq, int volt, int curr)
{
	u8 ichg;
	u8 vchg;
	u8 val;
	
	ichg = (curr * bq->sensor_mohm / 100 -  BQ2415X_MAX_ICHG_BASE) / BQ2415X_MAX_ICHG_LSB;

	ichg <<= BQ2415X_MAX_ICHG_SHIFT;
	
	vchg = (volt - BQ2415X_MAX_VREG_BASE) / BQ2415X_MAX_VREG_LSB;
	vchg <<= BQ2415X_MAX_VREG_SHIFT;
	
	val = ichg | vchg;
	
	pr_debug("val:0x%02X\n", val);
	
	return bq2415x_update_bits(bq, BQ2415X_REG_06,
				BQ2415X_MAX_VREG_MASK | BQ2415X_MAX_ICHG_MASK, val);	
	
}

static ssize_t bq2415x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bq2415x *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[100];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq2415x Reg");
	for (addr = 0x0; addr <= 0x06; addr++) {
		ret = bq2415x_read_byte(bq, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
					"Reg[%02X] = 0x%02X\n",	addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bq2415x_store_registers(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2415x *bq = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x06)
		bq2415x_write_byte(bq, (unsigned char)reg, (unsigned char)val);

	return count;
}

static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, bq2415x_show_registers,
						bq2415x_store_registers);

static struct attribute *bq2415x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2415x_attr_group = {
	.attrs = bq2415x_attributes,
};


static int bq2415x_parse_dt(struct device *dev, 
					struct bq2415x *bq)
{
	int ret;
	struct device_node *np = dev->of_node;

	bq->charge_enabled = !(of_property_read_bool(np, "ti,charging-disabled"));

	bq->enable_term = of_property_read_bool(np, "ti,bq2415x,enable-term");

	bq->gpio_cd = of_get_named_gpio(np, "ti,bq2415x,chip-disable-gpio", 0);
	if (bq->gpio_cd < 0) {
		pr_err("failed to get node of ti,bq2415x,chip-disable-gpio");
		return -EIO;
	}

	ret = of_property_read_u32(np, "ti,bq2415x,current-sensor-mohm",
					&bq->sensor_mohm);
	if (ret)
		pr_err("Failed to read node ti,bq2415x,current-sensor-mohm");
	
	if (bq->sensor_mohm == 0) {
		pr_err("invalid sensor resistor value, use 68mohm by default\n");
		bq->sensor_mohm = 55;
	}
	pr_debug("sensor_mohm:0x%02X\n", bq->sensor_mohm);
	
	ret = of_property_read_u32(np, "ti,bq2415x,charge-voltage",
					&bq->chg_mv);
	if (ret)
		pr_err("Failed to read node ti,bq2415x,charge-voltage");

	ret = of_property_read_u32(np, "ti,bq2415x,charge-current",
					&bq->chg_ma);
	if (ret)
		pr_err("Failed to read node ti,bq2415x,charge-current");

	ret = of_property_read_u32(np, "ti,bq2415x,input-current-limit",
					&bq->icl_ma);
	if (ret)
		pr_err("Failed to read node ti,bq2415x,input-current-limit");

	ret = of_property_read_u32(np, "ti,bq2415x,input-voltage-limit",
					&bq->ivl_mv);
	if (ret)
		pr_err("Failed to read node ti,bq2415x,input-voltage-limit");
	
	ret = of_property_read_u32(np, "ti,bq2415x,safety-max-charge-voltage",
					&bq->safety_chg_mv);
	if (ret)
		pr_err("Failed to read node ti,bq2415x,safety-max-charge-voltage");
	
	ret = of_property_read_u32(np, "ti,bq2415x,safety-max-charge-current",
					&bq->safety_chg_ma);
	if (ret)
		pr_err("Failed to read node ti,bq2415x,safety-max-charge-current");
	
	ret = of_property_read_u32(np, "ti,bq2415x,vbatlow-volt",
					&bq->batlow_mv);

	ret = of_property_read_u32(np, "ti,bq2415x,term-current",
					&bq->iterm_ma);

	if (of_find_property(np, "ti,thermal-mitigation",
					&bq->thermal_levels)) {
		bq->thermal_mitigation = devm_kzalloc(bq->dev,
					bq->thermal_levels,
						GFP_KERNEL);

		if (bq->thermal_mitigation == NULL) {
			pr_err("thermal mitigation kzalloc() failed.\n");
		}

		bq->thermal_levels /= sizeof(int);
		ret = of_property_read_u32_array(np,
				"ti,thermal-mitigation",
				bq->thermal_mitigation, bq->thermal_levels);
		if (ret) {
			pr_err("Couldn't read thermal limits ret = %d\n", ret);
		}
	}


	return 0;
}

static int bq2415x_detect_device(struct bq2415x *bq)
{
	int ret;
	u8 data;

	ret = bq2415x_read_byte(bq, BQ2415X_REG_03, &data);
	if (ret == 0) {
		bq->part_no = (data & BQ2415X_PN_MASK) >> BQ2415X_PN_SHIFT;
		bq->revision = (data & BQ2415X_REVISION_MASK) >> BQ2415X_REVISION_SHIFT;
	}

	return ret;
}

static int bq2415x_set_term_current(struct bq2415x *bq, int curr_ma)
{
	u8 ichg;
	
	ichg = (curr_ma * bq->sensor_mohm / 100 -  BQ2415X_ITERM_BASE) / BQ2415X_ITERM_LSB;

	ichg <<= BQ2415X_ITERM_SHIFT;
	
	pr_debug("ichg:0x%02X\n", ichg);
	return bq2415x_update_bits(bq, BQ2415X_REG_04,
				BQ2415X_ITERM_MASK, ichg);
}

static int bq2415x_set_chargecurrent(struct bq2415x *bq, u32 curr)
{
	u8 ichg;
	
	ichg = (curr * bq->sensor_mohm / 100 -  BQ2415X_ICHG_BASE) / BQ2415X_ICHG_LSB;

	ichg <<= BQ2415X_ICHG_SHIFT;
	
	pr_debug("val:0x%02X\n", ichg);
	
	return bq2415x_update_bits(bq, BQ2415X_REG_04,
				BQ2415X_ICHG_MASK, ichg);

}

static int bq2415x_set_chargevoltage(struct bq2415x *bq, int volt)
{
	u8 val;

	val = (volt - BQ2415X_VREG_BASE)/BQ2415X_VREG_LSB;
	val <<= BQ2415X_VREG_SHIFT;
	
	pr_debug("val:0x%02X\n", val);
		
	return bq2415x_update_bits(bq, BQ2415X_REG_02,
				BQ2415X_VREG_MASK, val);
}

static int bq2415x_set_input_volt_limit(struct bq2415x *bq, int volt)
{
	u8 val;

	val = (volt - BQ2415X_VSREG_BASE) / BQ2415X_VSREG_LSB;
	val <<= BQ2415X_VSREG_SHIFT;

	pr_debug("val:0x%02X\n", val);
	
	return bq2415x_update_bits(bq, BQ2415X_REG_05,
				BQ2415X_VSREG_MASK, val);
}


static int bq2415x_set_input_current_limit(struct bq2415x *bq, int curr)
{
	u8 val;
	
	if (curr == 100)
		val = BQ2415X_IINLIM_100MA;
	else if (curr == 500)
		val = BQ2415X_IINLIM_500MA;
	else if (curr == 800)
		val = BQ2415X_IINLIM_800MA;
	else if (curr >= 1500)
		val = BQ2415X_IINLIM_NOLIM;
	else
		val = BQ2415X_IINLIM_100MA;

	val <<= BQ2415X_IINLIM_SHIFT;

	pr_debug("val:0x%02X\n", val);
		
	return bq2415x_update_bits(bq, BQ2415X_REG_01,
				BQ2415X_IINLIM_MASK, val);
}


static int bq2415x_enable_otg(struct bq2415x *bq, bool en)
{
	int ret;
	u8 val;
	
	if (en)
		val = BQ2415X_BOOST_MODE;
	else
		val = BQ2415X_CHARGER_MODE;
	
	val <<= BQ2415X_OPA_MODE_SHIFT;
	
	ret = bq2415x_update_bits(bq, BQ2415X_REG_01,
				BQ2415X_OPA_MODE_MASK, val);
	
	return ret;
}

static int bq2415x_enter_hiz_mode(struct bq2415x *bq)
{
	u8 val = BQ2415X_HZ_MODE_ENABLE << BQ2415X_HZ_MODE_SHIFT;

	return bq2415x_update_bits(bq, BQ2415X_REG_01, BQ2415X_HZ_MODE_MASK, val);

}

static int bq2415x_exit_hiz_mode(struct bq2415x *bq)
{

	u8 val = BQ2415X_HZ_MODE_DISABLE << BQ2415X_HZ_MODE_SHIFT;

	return bq2415x_update_bits(bq, BQ2415X_REG_01, BQ2415X_HZ_MODE_MASK, val);

}

int bq2415x_get_hiz_mode(struct bq2415x *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = bq2415x_read_byte(bq, BQ2415X_REG_01, &val);
	if (ret)
		return ret;
	*state = (val & BQ2415X_HZ_MODE_MASK) >> BQ2415X_HZ_MODE_SHIFT;

	return 0;
}


static int bq2415x_set_charge_profile(struct bq2415x *bq)
{
	int ret;

	pr_err("chg_mv:%d, chg_ma:%d, icl_ma:%d, ivl_mv:%d\n",
			bq->chg_mv, bq->chg_ma, bq->icl_ma, bq->ivl_mv);
			
	ret = bq2415x_set_chargevoltage(bq, bq->chg_mv);
	if (ret < 0) {
		pr_err("Failed to set charge voltage:%d\n", ret);
		return ret;
	}

	ret = bq2415x_set_chargecurrent(bq, bq->chg_ma);
	if (ret < 0) {
		pr_err("Failed to set charge current:%d\n", ret);
		return ret;
	}

	ret = bq2415x_set_input_current_limit(bq, bq->icl_ma);
	if (ret < 0) {
		pr_err("Failed to set input current limit:%d\n", ret);
		return ret;
	}

	ret = bq2415x_set_input_volt_limit(bq, bq->ivl_mv);
	if (ret < 0) {
		pr_err("Failed to set input voltage limit:%d\n", ret);
		return ret;
	}
	return 0;
}

static int bq2415x_init_device(struct bq2415x *bq)
{
	int ret;


	/*safety register can only be written before other writes occur,
	  if lk code exist, it should write this register firstly before
	  write any other registers
	*/
	ret = bq2415x_set_safety_reg(bq, bq->safety_chg_mv, bq->safety_chg_ma);
	if (ret < 0)
		pr_err("Failed to set safety register:%d\n", ret);
	
	ret = bq2415x_enable_term(bq, bq->enable_term);
	if (ret < 0)
		pr_err("Failed to %s termination:%d\n",
			bq->enable_term ? "enable" : "disable", ret);

	ret = bq2415x_set_vbatlow_volt(bq, bq->batlow_mv);
	if (ret < 0)
		pr_err("Failed to set vbatlow volt to %d,rc=%d\n",
					bq->batlow_mv, ret);

	bq2415x_set_term_current(bq, bq->iterm_ma);
	
	bq2415x_set_charge_profile(bq);

	if (bq->charge_enabled)
		ret = bq2415x_enable_charger(bq);
	else
		ret = bq2415x_disable_charger(bq);

	if (ret < 0)
		pr_err("Failed to %s charger:%d\n",
			bq->charge_enabled ? "enable" : "disable", ret);

	return 0;
}

static void bq2415x_dump_regs(struct bq2415x *bq)
{
	int ret;
	u8 addr;
	u8 val;

	for (addr = 0x00; addr <= 0x06; addr++) {
		msleep(2);
		ret = bq2415x_read_byte(bq, addr, &val);
		if (!ret)
			pr_err("Reg[%02X] = 0x%02X\n", addr, val);
	}

}


static const unsigned char * chg_state_str[] = {
	"Ready", "Charging", "Charge Done", "Charge Fault"
};

static const unsigned char * chg_fault_state_str[] = {
	"Normal", "VBUS OVP", "Sleep mode", "Bad Adapter",
	"Output ovp", "Thermal shutdown", "Timer fault",
	"No battery"
};

static const unsigned char * boost_fault_state_str[] = {
	"Normal", "VBUS OVP", "over load", "battery volt too low",
	"Battery OVP","Thermal shutdown", "Timer fault","N/A"
};

static void bq2415x_dump_status(struct bq2415x *bq)
{
	int ret;
	u8 val = 0;
	
	ret = bq2415x_read_byte(bq, BQ2415X_REG_00, &val);
	if (!ret) {
		bq->charge_state = (val & BQ2415X_STAT_MASK)
							>> BQ2415X_STAT_SHIFT;
		bq->boost_mode = !!(val & BQ2415X_BOOST_MASK);
		bq->otg_pin_status = !!(val & BQ2415X_OTG_MASK);
		bq->fault_status = (val & BQ2415X_FAULT_MASK)
							>> BQ2415X_FAULT_SHIFT;
		pr_err("Charge State:%s\n", chg_state_str[bq->charge_state]);
		pr_err("%s In boost mode\n", bq->boost_mode ? "":"Not");
		pr_err("otg pin:%s\n", bq->otg_pin_status ? "High" : "Low");
		pr_err("Fault Status:%s", bq->boost_mode ? 
					  boost_fault_state_str[bq->fault_status] 
					: chg_fault_state_str[bq->fault_status]);
	}
	
	ret = bq2415x_read_byte(bq, BQ2415X_REG_05, &val);
	if (!ret) {
		pr_err("DPM Status:%d\n", !!(val & BQ2415X_DPM_STATUS_MASK));
		pr_err("CD Status:%d\n", !!(val & BQ2415X_CD_STATUS_MASK));
	}

	bq2415x_dump_regs(bq);
}

static int bq2415x_charging_disable(struct bq2415x *bq, int reason, 
					int disable)
{

	int ret = 0;
	int disabled;

	mutex_lock(&bq->charging_disable_lock);

	disabled = bq->charging_disabled_status;

	pr_err("reason=%d requested_disable=%d disabled_status=%d\n",
					reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (disabled && bq->charge_enabled)
		ret = bq2415x_disable_charger(bq);
	else if (!disabled && !bq->charge_enabled)
		ret = bq2415x_enable_charger(bq);

	if (ret) {
		pr_err("Couldn't disable/enable charging for reason=%d ret=%d\n",
							ret, reason);
	} else {
		bq->charging_disabled_status = disabled;
		mutex_lock(&bq->data_lock);
		bq->charge_enabled = !disabled;
		mutex_unlock(&bq->data_lock);
	}
	mutex_unlock(&bq->charging_disable_lock);

	return ret;
}


static struct power_supply *get_bms_psy(struct bq2415x *bq)
{
	if (bq->bms_psy)
		return bq->bms_psy;
	bq->bms_psy = power_supply_get_by_name("bms");
	if (!bq->bms_psy)
		pr_debug("bms power supply not found\n");
	
	return bq->bms_psy;
}

static int bq2415x_get_batt_property(struct bq2415x *bq,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct power_supply *bms_psy = get_bms_psy(bq);

	int ret;
	
	if (!bms_psy)
		return -EINVAL;
	
	ret = bms_psy->get_property(bms_psy, psp, val);

	return ret;
}

static inline bool is_device_suspended(struct bq2415x *bq);
static int bq2415x_get_prop_charge_type(struct bq2415x *bq)
{
	u8 val = 0;
	if (is_device_suspended(bq))
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	bq2415x_read_byte(bq,  BQ2415X_REG_00, &val);
	val &= BQ2415X_STAT_MASK;
	val >>= BQ2415X_STAT_SHIFT;
	switch (val) {
	case BQ2415X_STAT_CHARGING:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case BQ2415X_STAT_CHGDONE:
	case BQ2415X_STAT_READY:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

static int bq2415x_get_prop_batt_present(struct bq2415x *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq2415x_get_batt_property(bq,
			POWER_SUPPLY_PROP_PRESENT, &batt_prop);
	if (!ret){
		mutex_lock(&bq->data_lock);
		bq->batt_present = batt_prop.intval;
		mutex_unlock(&bq->data_lock);
	}
	return ret;

}

static int bq2415x_get_prop_batt_full(struct bq2415x *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq2415x_get_batt_property(bq, 
			POWER_SUPPLY_PROP_STATUS, &batt_prop);
	if (!ret) {
		mutex_lock(&bq->data_lock);
		bq->batt_full = (batt_prop.intval == POWER_SUPPLY_STATUS_FULL);
		mutex_unlock(&bq->data_lock);
	}
	return ret;
}

static int bq2415x_get_prop_charge_status(struct bq2415x *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;
	u8 status;

	ret = bq2415x_get_batt_property(bq, 
			POWER_SUPPLY_PROP_STATUS, &batt_prop);
	if (!ret && batt_prop.intval == POWER_SUPPLY_STATUS_FULL)
		return POWER_SUPPLY_STATUS_FULL;

	ret = bq2415x_read_byte(bq, BQ2415X_REG_00, &status);
	if (ret) {
		return 	POWER_SUPPLY_STATUS_UNKNOWN;
	}

	mutex_lock(&bq->data_lock);
	bq->charge_state = (status & BQ2415X_STAT_MASK) >> BQ2415X_STAT_SHIFT;
	mutex_unlock(&bq->data_lock);

	if (bq->usb_present && bq->jeita_active 
			&& (bq->batt_warm || bq->batt_cool) 
			&& bq->charge_state == BQ2415X_STAT_CHGDONE)
		return POWER_SUPPLY_STATUS_FULL;

	switch(bq->charge_state) {
		case BQ2415X_STAT_CHARGING:
			return POWER_SUPPLY_STATUS_CHARGING;
		case BQ2415X_STAT_CHGDONE:
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
		case BQ2415X_STAT_READY:
			return POWER_SUPPLY_STATUS_DISCHARGING;
		default:
			return 	POWER_SUPPLY_STATUS_UNKNOWN;
	}

}

static int bq2415x_get_prop_health(struct bq2415x *bq)
{
	int ret;
	union power_supply_propval batt_prop = {0,};

	if (bq->software_jeita_supported) {
		if (bq->jeita_active) {
			if (bq->batt_hot) 
				ret = POWER_SUPPLY_HEALTH_OVERHEAT;
			else if (bq->batt_warm)
				ret = POWER_SUPPLY_HEALTH_WARM;
			else if (bq->batt_cool)
				ret = POWER_SUPPLY_HEALTH_COOL;
			else if (bq->batt_cold)
				ret = POWER_SUPPLY_HEALTH_COLD;
		} else {
			ret = POWER_SUPPLY_HEALTH_GOOD;
		}
	} else {/* get health status from gauge */
		ret = bq2415x_get_batt_property(bq, 
					POWER_SUPPLY_PROP_HEALTH, &batt_prop);
		if (!ret)
			ret = batt_prop.intval;
		else
			ret = POWER_SUPPLY_HEALTH_UNKNOWN;
	}
	return ret;
}


static enum power_supply_property bq2415x_charger_props[] = {

	POWER_SUPPLY_PROP_CHARGE_TYPE, 
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,

	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,

	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
};

void static runin_work(struct bq2415x *bq, int batt_capacity)
{
	int rc;

	printk("%s:BatteryTestStatus_enable = %d bq->usb_present = %d \n",
			__func__,BatteryTestStatus_enable,bq->usb_present);

	if (/*!bq->usb_present || */!BatteryTestStatus_enable) {
		if (bq->in_hiz) {
			rc = bq2415x_exit_hiz_mode(bq);
			if (rc) {
				dev_err(bq->dev, "Couldn't enable charge rc=%d\n", rc);
			} else {
				pr_err("Exit Hiz Successfully\n");
				bq->in_hiz = false;
			}
		}
		return;
	}

	if (batt_capacity >= 80) {
		pr_debug("bq2415x_get_prop_batt_capacity > 80\n");
		//rc = bq2415x_charging_disable(bq, USER, true);
		if (!bq->in_hiz) {
			rc = bq2415x_enter_hiz_mode(bq);
			if (rc) {
				dev_err(bq->dev, "Couldn't disenable charge rc=%d\n", rc);
			} else {
				pr_err("Enter Hiz Successfully\n");
				bq->in_hiz = true;
			}
		}
	} else if (batt_capacity < 60) {
		pr_debug("bq2415x_get_prop_batt_capacity < 60\n");
		//rc = bq2415x_charging_disable(bq, USER, false);
		if (bq->in_hiz) {
			rc = bq2415x_exit_hiz_mode(bq);
			if (rc) {
				dev_err(bq->dev, "Couldn't enable charge rc=%d\n", rc);
			} else {
				pr_err("Exit Hiz Successfully\n");
				bq->in_hiz = false;
			}
		} 
	}
}

static int bq2415x_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{

	struct bq2415x *bq = container_of(psy, struct bq2415x, batt_psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2415x_get_prop_charge_type(bq);
		pr_debug("POWER_SUPPLY_PROP_CHARGE_TYPE:%d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq->charge_enabled;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = 3080;
		break;
	
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq2415x_get_prop_charge_status(bq);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq2415x_get_prop_health(bq);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		bq2415x_get_batt_property(bq, psp, val);
		runin_work(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = bq->therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_TECHNOLOGY:
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		return bq2415x_get_batt_property(bq, psp, val);
	default:
		return -EINVAL;

	}

	return 0;
}

static int bq2415x_system_temp_level_set(struct bq2415x *bq, int);

static int bq2415x_charger_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct bq2415x *bq = container_of(psy,
				struct bq2415x, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		bq2415x_charging_disable(bq, USER, !val->intval);

		power_supply_changed(&bq->batt_psy);
		power_supply_changed(bq->usb_psy);
		pr_info("POWER_SUPPLY_PROP_CHARGING_ENABLED: %s\n", 
				val->intval ? "enable" : "disable");
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		bq2415x_system_temp_level_set(bq, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq2415x_charger_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int bq2415x_update_charging_profile(struct bq2415x *bq)
{
	int ret;
	int chg_ma;
	int chg_mv;
	int icl;
	int therm_ma;

	union power_supply_propval prop = {0,};


	if (!bq->usb_present) 
		return 0;

	ret = bq->usb_psy->get_property(bq->usb_psy, 
				POWER_SUPPLY_PROP_TYPE, &prop);
	
	if (ret < 0) {
		pr_err("couldn't read USB TYPE property, ret=%d\n", ret);
		return ret;
	}
	pr_err("charge type = %d\n", prop.intval);
	mutex_lock(&bq->profile_change_lock);
	if (bq->jeita_active) {
		chg_ma = bq->jeita_ma;
		chg_mv = bq->jeita_mv;
	} else {
		if (prop.intval == POWER_SUPPLY_TYPE_USB_DCP
			|| prop.intval == POWER_SUPPLY_TYPE_USB_CDP) {
			chg_ma = bq->chg_ma;
			chg_mv = bq->chg_mv;
		} else {
			chg_ma = 500;
			chg_mv = bq->chg_mv;
		}
	}
	
	icl = bq->usb_psy_ma;
	if (bq->usb_psy_ma < chg_ma) {
		chg_ma = bq->usb_psy_ma;
	}

	if (bq->therm_lvl_sel > 0
			&& bq->therm_lvl_sel < (bq->thermal_levels - 1))
		/*
		 * consider thermal limit only when it is active and not at
		 * the highest level
		 */
		therm_ma = bq->thermal_mitigation[bq->therm_lvl_sel];
	else
		therm_ma = chg_ma;

	chg_ma = min(therm_ma, chg_ma);

	pr_err("charge volt = %d, charge curr = %d, input curr limit = %d\n",
				chg_mv, chg_ma, icl);

	ret = bq2415x_set_input_current_limit(bq, icl);
	if (ret < 0)
		pr_err("couldn't set input current limit, ret=%d\n", ret);

	ret = bq2415x_set_input_volt_limit(bq, bq->ivl_mv);
	if (ret < 0)
		pr_err("couldn't set input voltage limit, ret=%d\n", ret);

	ret = bq2415x_set_chargevoltage(bq, chg_mv);
	if (ret < 0)
		pr_err("couldn't set charge voltage ret=%d\n", ret);

	ret = bq2415x_set_chargecurrent(bq, chg_ma);
	if (ret < 0)
		pr_err("couldn't set charge current, ret=%d\n", ret);
	
	mutex_unlock(&bq->profile_change_lock);

	return 0;
}


static int bq2415x_system_temp_level_set(struct bq2415x *bq,
							int lvl_sel)
{
	int ret = 0;
	int prev_therm_lvl;

	pr_err("lvl_sel=%d, bq->therm_lvl_sel = %d\n", lvl_sel, bq->therm_lvl_sel);
	if (BatteryTestStatus_enable)
		return 0;

	if (!bq->thermal_mitigation) {
		pr_err("Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		pr_err("Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= bq->thermal_levels) {
		pr_err("Unsupported level selected %d forcing %d\n", lvl_sel,
				bq->thermal_levels - 1);
		lvl_sel = bq->thermal_levels - 1;
	}

	if (lvl_sel == bq->therm_lvl_sel)
		return 0;

	prev_therm_lvl = bq->therm_lvl_sel;
	bq->therm_lvl_sel = lvl_sel;

	ret = bq2415x_update_charging_profile(bq);
	if (ret)
		pr_err("Couldn't set USB current ret = %d\n", ret);

	return ret;
}


static void bq2415x_external_power_changed(struct power_supply *psy)
{
	struct bq2415x *bq = container_of(psy, struct bq2415x, batt_psy);
	
	union power_supply_propval prop = {0,};
	int ret, current_limit = 0;

	
	ret = bq->usb_psy->get_property(bq->usb_psy, 
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (ret < 0)
		pr_err("could not read USB current_max property, ret=%d\n", ret);
	else
		current_limit = prop.intval / 1000;
	
	pr_err("current_limit = %d\n", current_limit);
	
	if (bq->usb_psy_ma != current_limit) {
		bq->usb_psy_ma = current_limit;
		bq2415x_update_charging_profile(bq);
	}

	ret = bq->usb_psy->get_property(bq->usb_psy, 
				POWER_SUPPLY_PROP_ONLINE, &prop);
	if (ret < 0)
		pr_err("could not read USB ONLINE property, ret=%d\n", ret);
	else
		pr_info("usb online status =%d\n", prop.intval);
	
	ret = 0;
	bq2415x_get_prop_charge_status(bq);
	if (bq->usb_present /*&& bq->charge_state != CHARGE_STATE_IDLE*//* && bq->charge_enabled *//*!bq->charging_disabled_status*/
				/*&& bq->usb_psy_ma != 0*/) {
		if (prop.intval == 0){
			pr_err("set usb online\n");
			ret = power_supply_set_online(bq->usb_psy, true);
		}
	} else {
		if (prop.intval == 1) {
			pr_err("set usb offline\n");
			ret = power_supply_set_online(bq->usb_psy, false);
		}
	}

	if (ret < 0)
		pr_info("could not set usb online state, ret=%d\n", ret);

}


static int bq2415x_psy_register(struct bq2415x *bq)
{
	int ret;

	bq->batt_psy.name = "battery";
	bq->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	bq->batt_psy.properties = bq2415x_charger_props;
	bq->batt_psy.num_properties = ARRAY_SIZE(bq2415x_charger_props);
	bq->batt_psy.get_property = bq2415x_charger_get_property;
	bq->batt_psy.set_property = bq2415x_charger_set_property;
	bq->batt_psy.external_power_changed = bq2415x_external_power_changed;
	bq->batt_psy.property_is_writeable = bq2415x_charger_is_writeable;

	ret = power_supply_register(bq->dev, &bq->batt_psy);
	if (ret < 0) {
		pr_err("failed to register batt_psy:%d\n", ret);
		return ret;
	}

	return 0;
}

static void bq2415x_psy_unregister(struct bq2415x *bq)
{
	power_supply_unregister(&bq->batt_psy);
}


static int bq2415x_otg_regulator_enable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2415x *bq = rdev_get_drvdata(rdev);

	ret = bq2415x_enable_otg(bq, true);
	if (ret) {
		pr_err("Couldn't enable OTG mode ret=%d\n", ret);
	} else {
		bq->otg_enabled = true;
		pr_info("bq2415x OTG mode Enabled!\n");
	}
	
	return ret;
}


static int bq2415x_otg_regulator_disable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2415x *bq = rdev_get_drvdata(rdev);

	ret = bq2415x_enable_otg(bq, false);
	if (ret) {
		pr_err("Couldn't disable OTG mode, ret=%d\n", ret);
	} else {
		bq->otg_enabled = false;
		pr_info("bq2415x OTG mode Disabled\n");
	}
	
	return ret;
}


static int bq2415x_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct bq2415x *bq = rdev_get_drvdata(rdev);
	int ret;
	u8 status;
	u8 enabled;

	ret = bq2415x_read_byte(bq, BQ2415X_REG_01, &status);
	if (ret)
		return ret;
	enabled = ((status & BQ2415X_OPA_MODE_MASK) >> BQ2415X_OPA_MODE_SHIFT);
	
	return (enabled == BQ2415X_BOOST_MODE) ? 1 : 0;
	
}


struct regulator_ops bq2415x_otg_reg_ops = {
	.enable		= bq2415x_otg_regulator_enable,
	.disable	= bq2415x_otg_regulator_disable,
	.is_enabled = bq2415x_otg_regulator_is_enable,
};

static int bq2415x_regulator_init(struct bq2415x *bq)
{
	int ret = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(bq->dev, bq->dev->of_node);
	if (!init_data) {
		dev_err(bq->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		bq->otg_vreg.rdesc.owner = THIS_MODULE;
		bq->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		bq->otg_vreg.rdesc.ops = &bq2415x_otg_reg_ops;
		bq->otg_vreg.rdesc.name = init_data->constraints.name;
		pr_info("regualtor name = %s\n", bq->otg_vreg.rdesc.name);

		cfg.dev = bq->dev;
		cfg.init_data = init_data;
		cfg.driver_data = bq;
		cfg.of_node = bq->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		bq->otg_vreg.rdev = regulator_register(
					&bq->otg_vreg.rdesc, &cfg);
		if (IS_ERR(bq->otg_vreg.rdev)) {
			ret = PTR_ERR(bq->otg_vreg.rdev);
			bq->otg_vreg.rdev = NULL;
			if (ret != -EPROBE_DEFER)
				dev_err(bq->dev,
					"OTG reg failed, rc=%d\n", ret);
		}
	}

	return ret;
}


static int bq2415x_parse_jeita_dt(struct device *dev, struct bq2415x* bq)
{
    struct device_node *np = dev->of_node;
	int ret;

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-hot-degc",
					&bq->batt_hot_degc);
    if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-hot-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-warm-degc",
					&bq->batt_warm_degc);
    if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-warm-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-cool-degc",
					&bq->batt_cool_degc);
    if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-cool-degc\n");
		return ret;
	}
	ret = of_property_read_u32(np,"ti,bq2415x,jeita-cold-degc",
					&bq->batt_cold_degc);
    if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-cold-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-hot-hysteresis",
					&bq->hot_temp_hysteresis);
    if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-hot-hysteresis\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-cold-hysteresis",
					&bq->cold_temp_hysteresis);
    if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-cold-hysteresis\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-cool-ma",
					&bq->batt_cool_ma);
    if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-cool-ma\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-cool-mv",
					&bq->batt_cool_mv);
    if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-cool-mv\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-warm-ma",
					&bq->batt_warm_ma);
    if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-warm-ma\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-warm-mv",
					&bq->batt_warm_mv);
    if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-warm-mv\n");
		return ret;
	}

	bq->software_jeita_supported = 
		of_property_read_bool(np,"ti,bq2415x,software-jeita-supported");

	return 0;
}

static void bq2415x_init_jeita(struct bq2415x *bq)
{

	bq->batt_temp = -EINVAL;

	/* set default value in case of dts read fail */
	bq->batt_hot_degc = 600;
	bq->batt_warm_degc = 450;
	bq->batt_cool_degc = 100;
	bq->batt_cold_degc = 0;
	
	bq->hot_temp_hysteresis = 50;
	bq->cold_temp_hysteresis = 50;

	bq->batt_cool_ma = 400;
	bq->batt_cool_mv = 4100;
	bq->batt_warm_ma = 400;
	bq->batt_warm_mv = 4100;

	bq->software_jeita_supported = true;

	/* DTS setting will overwrite above default value */

	bq2415x_parse_jeita_dt(&bq->client->dev, bq);
}

static void bq2415x_check_jeita(struct bq2415x *bq)
{

	int ret;
	bool last_hot, last_warm, last_cool, last_cold;
	bool chg_disabled_jeita, jeita_hot_cold;
	union power_supply_propval batt_prop = {0,};
	
	ret = bq2415x_get_batt_property(bq,
					POWER_SUPPLY_PROP_TEMP, &batt_prop);
	if (!ret)
		bq->batt_temp = batt_prop.intval;
	
	if (bq->batt_temp == -EINVAL) 
		return;

	last_hot = bq->batt_hot;
	last_warm = bq->batt_warm;
	last_cool = bq->batt_cool;
	last_cold = bq->batt_cold;

	if (bq->batt_temp >= bq->batt_hot_degc) {/* HOT */
		if (!bq->batt_hot) {
			bq->batt_hot  = true;
			bq->batt_warm = false;
			bq->batt_cool = false;
			bq->batt_cold = false;
			bq->jeita_ma = 0;
			bq->jeita_mv = 0;
		}
	} else if (bq->batt_temp >= bq->batt_warm_degc) {/* WARM */
		if (!bq->batt_hot
			||(bq->batt_temp < bq->batt_hot_degc - bq->hot_temp_hysteresis)){
			bq->batt_hot  = false;
			bq->batt_warm = true;
			bq->batt_cool = false;
			bq->batt_cold = false;
			bq->jeita_mv = bq->batt_warm_mv;
			bq->jeita_ma = bq->batt_warm_ma;
		}
	} else if (bq->batt_temp < bq->batt_cold_degc) {/* COLD */
		if (!bq->batt_cold) {
			bq->batt_hot  = false;
			bq->batt_warm = false;
			bq->batt_cool = false;
			bq->batt_cold = true;
			bq->jeita_ma = 0;
			bq->jeita_mv = 0;
		}
	} else if (bq->batt_temp < bq->batt_cool_degc) {/* COOL */
		if (!bq->batt_cold ||
			(bq->batt_temp > bq->batt_cold_degc + bq->cold_temp_hysteresis)) {
			bq->batt_hot  = false;
			bq->batt_warm = false;
			bq->batt_cool = true;
			bq->batt_cold = false;
			bq->jeita_mv = bq->batt_cool_mv;
			bq->jeita_ma = bq->batt_cool_ma;
		}
	} else {/* NORMAL */
		bq->batt_hot  = false;
		bq->batt_warm = false;
		bq->batt_cool = false;
		bq->batt_cold = false;
	}

	bq->jeita_active = bq->batt_cool || bq->batt_hot ||
			   bq->batt_cold || bq->batt_warm;
	
	if ((last_cold != bq->batt_cold) || (last_warm != bq->batt_warm) ||
		(last_cool != bq->batt_cool) || (last_hot != bq->batt_hot)) {
		bq2415x_update_charging_profile(bq);
		power_supply_changed(&bq->batt_psy);
		power_supply_changed(bq->usb_psy);
	} else if (bq->batt_hot || bq->batt_cold) { /*continuely update event */
		power_supply_changed(&bq->batt_psy);
		power_supply_changed(bq->usb_psy);
	}

	jeita_hot_cold = bq->jeita_active && (bq->batt_hot || bq->batt_cold);
	chg_disabled_jeita = !!(bq->charging_disabled_status & JEITA);
	if (jeita_hot_cold ^ chg_disabled_jeita)
		bq2415x_charging_disable(bq, JEITA, jeita_hot_cold);

}

static void bq2415x_check_batt_pres(struct bq2415x *bq)
{
	int ret = 0;
	bool chg_disabled_pres;

	ret = bq2415x_get_prop_batt_present(bq);
	if (!ret) {
		chg_disabled_pres = !!(bq->charging_disabled_status & BATT_PRES);
		if (chg_disabled_pres ^ !bq->batt_present) {
			ret = bq2415x_charging_disable(bq, BATT_PRES, !bq->batt_present);
			if (ret) {
				pr_err("failed to %s charging, ret = %d\n", 
						bq->batt_present ? "disable" : "enable",
						ret);
			}
			power_supply_changed(&bq->batt_psy);
			power_supply_changed(bq->usb_psy);
		}
	}

}

static void bq2415x_check_batt_full(struct bq2415x *bq)
{
	int ret = 0;
	bool chg_disabled_fc;

	ret = bq2415x_get_prop_batt_full(bq);
	if (!ret) {
		chg_disabled_fc = !!(bq->charging_disabled_status & BATT_FC);
		if (chg_disabled_fc ^ bq->batt_full) {
			ret = bq2415x_charging_disable(bq, BATT_FC, bq->batt_full);
			if (ret) {
				pr_err("failed to %s charging, ret = %d\n", 
						bq->batt_full ? "disable" : "enable",
						ret);
			}
			power_supply_changed(&bq->batt_psy);
			power_supply_changed(bq->usb_psy);
		}
	}
}


static int calculate_jeita_poll_interval(struct bq2415x* bq)
{
	int interval;

	if (bq->batt_hot || bq->batt_cold)
		interval = 5;
	else if (bq->batt_warm || bq->batt_cool)
		interval = 10;
	else
		interval = 15;
	return interval;
}

#define	FG_LOG_INTERVAL		120
static void bq2415x_dump_fg_reg(struct bq2415x *bq)
{
	union power_supply_propval val = {0,};
	static int dump_cnt;

	if (++dump_cnt >= (FG_LOG_INTERVAL / calculate_jeita_poll_interval(bq))) {
		dump_cnt = 0;
		val.intval = 0;
		bq->bms_psy->set_property(bq->bms_psy, 
				POWER_SUPPLY_PROP_UPDATE_NOW, &val); 
	}
}


static enum alarmtimer_restart bq2415x_jeita_alarm_cb(struct alarm *alarm,
							ktime_t now)
{
	struct bq2415x *bq = container_of(alarm, 
				struct bq2415x, jeita_alarm);
	unsigned long ns;

	bq2415x_stay_awake(&bq->bq2415x_ws, WAKEUP_SRC_JEITA);
	schedule_delayed_work(&bq->charge_jeita_work, HZ/2);

	ns = calculate_jeita_poll_interval(bq) * 1000000000LL;
	alarm_forward_now(alarm, ns_to_ktime(ns));
	return ALARMTIMER_RESTART;
}

static void bq2415x_charge_jeita_workfunc(struct work_struct *work)
{
	struct bq2415x *bq = container_of(work, 
				struct bq2415x, charge_jeita_work.work);

	bq2415x_check_batt_pres(bq);
	bq2415x_check_batt_full(bq);
	bq2415x_dump_fg_reg(bq);

	bq2415x_check_jeita(bq);
	bq2415x_dump_status(bq);
	bq2415x_relax(&bq->bq2415x_ws, WAKEUP_SRC_JEITA);
}


static void bq2415x_discharge_jeita_workfunc(struct work_struct *work)
{
	struct bq2415x *bq = container_of(work, 
				struct bq2415x, discharge_jeita_work.work);

	bq2415x_check_batt_pres(bq);
	bq2415x_check_batt_full(bq);
	bq2415x_dump_fg_reg(bq);

	bq2415x_check_jeita(bq);
	schedule_delayed_work(&bq->discharge_jeita_work,
				calculate_jeita_poll_interval(bq) * HZ);
}

static int show_registers(struct seq_file *m, void *data)
{
	struct bq2415x *bq = m->private;
	u8 addr;
	int ret;
	u8 val;
	
	for (addr = 0x0; addr <= 0x0B; addr++) {
		ret = bq2415x_read_byte(bq, addr, &val);
		if (!ret)
			seq_printf(m, "Reg[%02X] = 0x%02X\n", addr, val);
	}
	return 0;	
}


static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct bq2415x *bq = inode->i_private;
	
	return single_open(file, show_registers, bq);
}


static const struct file_operations reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= reg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debugfs_entry(struct bq2415x *bq)
{
	bq->debug_root = debugfs_create_dir("bq2415x", NULL);
	if (!bq->debug_root)
		pr_err("Failed to create debug dir\n");
	
	if (bq->debug_root) {
		
		debugfs_create_file("registers", S_IFREG | S_IRUGO,
						bq->debug_root, bq, &reg_debugfs_ops);

		debugfs_create_x32("charging_disable_status", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->charging_disabled_status));

		debugfs_create_x32("fault_status", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->fault_status));

		debugfs_create_x32("charge_state", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->charge_state));							

		debugfs_create_x32("skip_reads",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  bq->debug_root,
					  &(bq->skip_reads));
		debugfs_create_x32("skip_writes",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  bq->debug_root,
					  &(bq->skip_writes));
	}	
}

static void bq2415x_vbus_changed_workfunc(struct work_struct *work)
{
	struct bq2415x *bq = container_of(work,
				struct bq2415x, vbus_changed_work.work);
	int ret;	
	
	ret = qpnp_pon_get_cblpwr_status(&bq->power_good);
	if (ret)
		return;

	if(!bq->power_good) {
	    if(bq->usb_present) {
			bq->usb_present = false;
			power_supply_set_present(bq->usb_psy, bq->usb_present);
		}

		if (bq->software_jeita_supported) {
			alarm_try_to_cancel(&bq->jeita_alarm);
		}
		
		schedule_delayed_work(&bq->discharge_jeita_work,
					calculate_jeita_poll_interval(bq) * HZ);

		pr_err("usb removed, set usb present = %d\n", bq->usb_present);
	}
	else if (bq->power_good && !bq->usb_present) {
		bq->usb_present = true;
		msleep(10);/*for cdp detect*/
		power_supply_set_present(bq->usb_psy, bq->usb_present);

		cancel_delayed_work(&bq->discharge_jeita_work);

		if (bq->software_jeita_supported) { 
			ret = alarm_start_relative(&bq->jeita_alarm, 
						ns_to_ktime(1 * 1000000000LL));
			if (ret) 
				pr_err("start alarm for JEITA detection failed, ret=%d\n",
							ret);
		}
		
		pr_err("usb plugged in, set usb present = %d\n", bq->usb_present);
	}
	
	power_supply_changed(&bq->batt_psy);

}

void bq2415x_cblpwr_changed(void)
{
	if (g_bq)
		schedule_delayed_work(&g_bq->vbus_changed_work,0);
}
EXPORT_SYMBOL(bq2415x_cblpwr_changed);

static void determine_initial_status(struct bq2415x *bq)
{
	int ret;
	u8 status = 0;
	ret = bq2415x_get_hiz_mode(bq, &status);
	if (!ret) 
		bq->in_hiz = !!status;

	bq2415x_cblpwr_changed();
}

static int bq2415x_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct bq2415x *bq;
	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	bool temp_pg;

	int ret;
	
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		dev_dbg(&client->dev, "bms supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}
	/* make sure power on module is ready, 
	 * we need it to get adapter present status*/
	ret = qpnp_pon_get_cblpwr_status(&temp_pg);
	if (ret == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	bq = devm_kzalloc(&client->dev, sizeof(struct bq2415x), GFP_KERNEL);
	if (!bq) {
		pr_err("Out of memory\n");
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->usb_psy = usb_psy;
	bq->bms_psy = bms_psy;

	bq->client = client;
	i2c_set_clientdata(client, bq);
	
	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->profile_change_lock);
	mutex_init(&bq->charging_disable_lock);
	mutex_init(&bq->irq_complete);

	bq->resume_completed = true;
	bq->irq_waiting = false;
	
	ret = bq2415x_detect_device(bq);
	if(ret) {
		pr_err("No bq2415x device found!\n");
		return -ENODEV;
	}
	
	bq2415x_init_jeita(bq);

	if (client->dev.of_node) {
		ret = bq2415x_parse_dt(&client->dev, bq);
		if (ret) {
			pr_err("No platform data provided.\n");
			return -EINVAL;
		}
	}

	if (gpio_is_valid(bq->gpio_cd)) {
		ret = devm_gpio_request(&client->dev, bq->gpio_cd, "bq2415x_cd");
		if (ret) {
			pr_err("Failed to request chip disable gpio %d:, err: %d\n", bq->gpio_cd, ret);
			return ret;
		}
		gpio_direction_output(bq->gpio_cd, 0);
	}

	ret = bq2415x_init_device(bq);
	if (ret) {
		pr_err("Failed to init device\n");
		return ret;
	}

	
	ret = bq2415x_psy_register(bq);
	if (ret)
		return ret;
	ret = bq2415x_regulator_init(bq);
	if (ret) {
		pr_err("Couldn't initialize bq2415x regulator ret=%d\n", ret);
		return ret;
	}

	bq2415x_wakeup_src_init(bq);

	device_init_wakeup(bq->dev, 1);	

	INIT_DELAYED_WORK(&bq->charge_jeita_work, bq2415x_charge_jeita_workfunc);
	INIT_DELAYED_WORK(&bq->discharge_jeita_work, bq2415x_discharge_jeita_workfunc);
	INIT_DELAYED_WORK(&bq->vbus_changed_work, bq2415x_vbus_changed_workfunc);

	g_bq = bq;

	alarm_init(&bq->jeita_alarm, ALARM_BOOTTIME, bq2415x_jeita_alarm_cb);

	create_debugfs_entry(bq);

	ret = sysfs_create_group(&bq->dev->kobj, &bq2415x_attr_group);
	if (ret) {
		dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);
	}


	determine_initial_status(bq);


	pr_err("bq2415x probe successfully, Part Num:%d, Revision:%d\n!", 
				bq->part_no, bq->revision);
	
	return 0;
}

static inline bool is_device_suspended(struct bq2415x *bq)
{
	return !bq->resume_completed;
}

static int bq2415x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2415x *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);

	return 0;
}

static int bq2415x_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2415x *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int bq2415x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2415x *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	mutex_unlock(&bq->irq_complete);
#if 0

	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		mutex_unlock(&bq->irq_complete);
	} else {
		mutex_unlock(&bq->irq_complete);
	}
#endif
	power_supply_changed(&bq->batt_psy);

	return 0;
}

static int bq2415x_charger_remove(struct i2c_client *client)
{
	struct bq2415x *bq = i2c_get_clientdata(client);

	alarm_try_to_cancel(&bq->jeita_alarm);

	cancel_delayed_work_sync(&bq->charge_jeita_work);
	cancel_delayed_work_sync(&bq->discharge_jeita_work);
	cancel_delayed_work_sync(&bq->vbus_changed_work);

	regulator_unregister(bq->otg_vreg.rdev);
	
	bq2415x_psy_unregister(bq);

	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->profile_change_lock);
	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->irq_complete);

	debugfs_remove_recursive(bq->debug_root);
	sysfs_remove_group(&bq->dev->kobj, &bq2415x_attr_group);


	return 0;
}


static void bq2415x_charger_shutdown(struct i2c_client *client)
{
}

static struct of_device_id bq2415x_charger_match_table[] = {
	{.compatible = "ti,bq24157",},
	{},
};
MODULE_DEVICE_TABLE(of,bq2415x_charger_match_table);

static const struct i2c_device_id bq2415x_charger_id[] = {
	{ "bq24157-charger", BQ24157 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2415x_charger_id);

static const struct dev_pm_ops bq2415x_pm_ops = {
	.resume		= bq2415x_resume,
	.suspend_noirq = bq2415x_suspend_noirq,
	.suspend	= bq2415x_suspend,
};
static struct i2c_driver bq2415x_charger_driver = {
	.driver 	= {
		.name 	= "bq2415x-charger",
		.owner 	= THIS_MODULE,
		.of_match_table = bq2415x_charger_match_table,
		.pm		= &bq2415x_pm_ops,
	},
	.id_table	= bq2415x_charger_id,
	
	.probe		= bq2415x_charger_probe,
	.remove		= bq2415x_charger_remove,
	.shutdown	= bq2415x_charger_shutdown,
	
};

module_i2c_driver(bq2415x_charger_driver);

MODULE_DESCRIPTION("TI BQ2415x Charger Driver");
MODULE_LICENSE("GPL2");
MODULE_AUTHOR("Texas Instruments");
