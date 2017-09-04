/*
 * BQ2570x battery charging driver
 *
 * Copyright (C) 2017 Texas Instruments *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#define pr_fmt(fmt)	"bq2597x: %s: " fmt, __func__

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
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>

#include "bq25970_reg.h"
/*#include "bq2597x.h"*/

#if 1
#undef pr_debug
#define pr_debug pr_err
#undef pr_info
#define pr_info pr_err
#undef dev_dbg
#define dev_dbg dev_err
#else
#undef pr_info
#define pr_info pr_debug
#endif

enum {
	ADC_IBUS,
	ADC_VBUS,
	ADC_VAC,
	ADC_VOUT,
	ADC_VBAT,
	ADC_IBAT,
	ADC_TBUS,
	ADC_TBAT,
	ADC_TDIE,
	ADC_MAX_NUM,
};

enum {
	BQ25970 = 0x00,
};


#define	BAT_OVP_ALARM		BIT(7)
#define BAT_OCP_ALARM		BIT(6)
#define	BUS_OVP_ALARM		BIT(5)
#define	BUS_OCP_ALARM		BIT(4)
#define	BAT_UCP_ALARM		BIT(3)
#define	VBUS_INSERT		BIT(2)
#define VBAT_INSERT		BIT(1)
#define	ADC_DONE		BIT(0)

#define BAT_OVP_FAULT		BIT(7)
#define BAT_OCP_FAULT		BIT(6)
#define BUS_OVP_FAULT		BIT(5)
#define BUS_OCP_FAULT		BIT(4)
#define TBUS_TBAT_ALARM		BIT(3)
#define TS_BAT_FAULT		BIT(2)
#define	TS_BUS_FAULT		BIT(1)
#define	TS_DIE_FAULT		BIT(0)

/*below used for comm with other module*/
#define	BAT_OVP_FAULT_SHIFT			0
#define	BAT_OCP_FAULT_SHIFT			1
#define	BUS_OVP_FAULT_SHIFT			2
#define	BUS_OCP_FAULT_SHIFT			3
#define	BAT_THERM_FAULT_SHIFT			4
#define	BUS_THERM_FAULT_SHIFT			5
#define	DIE_THERM_FAULT_SHIFT			6

#define	BAT_OVP_FAULT_MASK		(1 << BAT_OVP_FAULT_SHIFT)
#define	BAT_OCP_FAULT_MASK		(1 << BAT_OCP_FAULT_SHIFT)
#define	BUS_OVP_FAULT_MASK		(1 << BUS_OVP_FAULT_SHIFT)
#define	BUS_OCP_FAULT_MASK		(1 << BUS_OCP_FAULT_SHIFT)
#define	BAT_THERM_FAULT_MASK		(1 << BAT_THERM_FAULT_SHIFT)
#define	BUS_THERM_FAULT_MASK		(1 << BUS_THERM_FAULT_SHIFT)
#define	DIE_THERM_FAULT_MASK		(1 << DIE_THERM_FAULT_SHIFT)

#define	BAT_OVP_ALARM_SHIFT			0
#define	BAT_OCP_ALARM_SHIFT			1
#define	BUS_OVP_ALARM_SHIFT			2
#define	BUS_OCP_ALARM_SHIFT			3
#define	BAT_THERM_ALARM_SHIFT			4
#define	BUS_THERM_ALARM_SHIFT			5
#define	DIE_THERM_ALARM_SHIFT			6
#define BAT_UCP_ALARM_SHIFT			7

#define	BAT_OVP_ALARM_MASK		(1 << BAT_OVP_ALARM_SHIFT)
#define	BAT_OCP_ALARM_MASK		(1 << BAT_OCP_ALARM_SHIFT)
#define	BUS_OVP_ALARM_MASK		(1 << BUS_OVP_ALARM_SHIFT)
#define	BUS_OCP_ALARM_MASK		(1 << BUS_OCP_ALARM_SHIFT)
#define	BAT_THERM_ALARM_MASK		(1 << BAT_THERM_ALARM_SHIFT)
#define	BUS_THERM_ALARM_MASK		(1 << BUS_THERM_ALARM_SHIFT)
#define	DIE_THERM_ALARM_MASK		(1 << DIE_THERM_ALARM_SHIFT)
#define	BAT_UCP_ALARM_MASK		(1 << BAT_UCP_ALARM_SHIFT)
/*end*/

struct bq2597x_cfg {
	bool bat_ovp_disable;
	bool bat_ocp_disable;
	bool bat_ovp_alm_disable;
	bool bat_ocp_alm_disable;

	int bat_ovp_th;
	int bat_ovp_alm_th;
	int bat_ocp_th;
	int bat_ocp_alm_th;

	bool bus_ovp_alm_disable;
	bool bus_ocp_disable;
	bool bus_ocp_alm_disable;

	int bus_ovp_th;
	int bus_ovp_alm_th;
	int bus_ocp_th;
	int bus_ocp_alm_th;

	bool bat_ucp_alm_disable;

	int bat_ucp_alm_th;
	int ac_ovp_th;

	bool bat_therm_disable;
	bool bus_therm_disable;
	bool die_therm_disable;

	int bat_therm_th; /*in %*/
	int bus_therm_th; /*in %*/
	int die_therm_th; /*in degC*/

};

struct bq2597x {
	struct device *dev;
	struct i2c_client *client;

	int part_no;
	int revision;

	struct mutex data_lock;
	struct mutex i2c_rw_lock;
	struct mutex charging_disable_lock;
	struct mutex irq_complete;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	bool batt_present;
	bool vbus_present;

	bool usb_present;
	bool charge_enabled;	/* Register bit status */

	/* ADC reading */
	int vbat_volt;
	int vbus_volt;
	int vout_volt;
	int vac_volt;

	int ibat_curr;
	int ibus_curr;

	int bat_temp;
	int bus_temp;
	int die_temp;

	/* alarm/fault status */
	bool bat_ovp_fault;
	bool bat_ocp_fault;
	bool bus_ovp_fault;
	bool bus_ocp_fault;

	bool bat_ovp_alarm;
	bool bat_ocp_alarm;
	bool bus_ovp_alarm;
	bool bus_ocp_alarm;

	bool bat_ucp_alarm;

	bool bat_therm_alarm;
	bool bus_therm_alarm;
	bool die_therm_alarm;

	bool bat_therm_fault;
	bool bus_therm_fault;
	bool die_therm_fault;

	bool therm_shutdown_flag;
	bool therm_shutdown_stat;

	int  prev_alarm;
	int  prev_fault;

	int chg_ma;
	int chg_mv;

	int charge_state;

	struct bq2597x_cfg *cfg;

	int skip_writes;
	int skip_reads;

	struct bq2597x_platform_data *platform_data;

	struct delayed_work monitor_work;

	struct dentry *debug_root;

	struct power_supply *pd_psy;
	struct power_supply fc2_psy;
};

/************************************************************************/

static int __bq2597x_read_byte(struct bq2597x *bq, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8) ret;

	return 0;
}

static int __bq2597x_write_byte(struct bq2597x *bq, int reg, u8 val)
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

static int __bq2597x_read_word(struct bq2597x *bq, u8 reg, u16 *data)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(bq->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u16) ret;

	return 0;
}

static int bq2597x_read_byte(struct bq2597x *bq, u8 reg, u8 *data)
{
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2597x_read_byte(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2597x_write_byte(struct bq2597x *bq, u8 reg, u8 data)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2597x_write_byte(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2597x_read_word(struct bq2597x *bq, u8 reg, u16 *data)
{
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2597x_read_word(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2597x_update_bits(struct bq2597x *bq, u8 reg,
				    u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2597x_read_byte(bq, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq2597x_write_byte(bq, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

/*********************************************************************/

static int bq2597x_enable_charge(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_CHG_ENABLE;
	else
		val = BQ2597X_CHG_DISABLE;

	val <<= BQ2597X_CHG_EN_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_0C,
				BQ2597X_CHG_EN_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_charge);

static int bq2597x_check_charge_enabled(struct bq2597x *bq, bool *enabled)
{
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_0C, &val);
	if (!ret)
		*enabled = !!(val & BQ2597X_CHG_EN_MASK);
	return ret;
}

static int bq2597x_enable_wdt(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_WATCHDOG_ENABLE;
	else
		val = BQ2597X_WATCHDOG_DISABLE;

	val <<= BQ2597X_WATCHDOG_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_0B,
				BQ2597X_WATCHDOG_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_wdt);

static int bq2597x_set_wdt(struct bq2597x *bq, int ms)
{
	int ret;
	u8 val;

	if (ms == 500)
		val = BQ2597X_WATCHDOG_0P5S;
	else if (ms == 1000)
		val = BQ2597X_WATCHDOG_1S;
	else if (ms == 5000)
		val = BQ2597X_WATCHDOG_5S;
	else if (ms == 30000)
		val = BQ2597X_WATCHDOG_30S;
	else
		val = BQ2597X_WATCHDOG_30S;

	val <<= BQ2597X_WATCHDOG_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_0B,
				BQ2597X_WATCHDOG_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_wdt);

static int bq2597x_enable_batovp(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BAT_OVP_ENABLE;
	else
		val = BQ2597X_BAT_OVP_DISABLE;

	val <<= BQ2597X_BAT_OVP_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_00,
				BQ2597X_BAT_OVP_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_batovp);

static int bq2597x_set_batovp_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BAT_OVP_BASE)
		threshold = BQ2597X_BAT_OVP_BASE;

	val = (threshold - BQ2597X_BAT_OVP_BASE) / BQ2597X_BAT_OVP_LSB;

	val <<= BQ2597X_BAT_OVP_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_00,
				BQ2597X_BAT_OVP_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_batovp_th);

static int bq2597x_enable_batovp_alarm(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BAT_OVP_ALM_ENABLE;
	else
		val = BQ2597X_BAT_OVP_ALM_DISABLE;

	val <<= BQ2597X_BAT_OVP_ALM_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_01,
				BQ2597X_BAT_OVP_ALM_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_batovp_alarm);

static int bq2597x_set_batovp_alarm_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BAT_OVP_ALM_BASE)
		threshold = BQ2597X_BAT_OVP_ALM_BASE;

	val = (threshold - BQ2597X_BAT_OVP_ALM_BASE) / BQ2597X_BAT_OVP_ALM_LSB;

	val <<= BQ2597X_BAT_OVP_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_01,
				BQ2597X_BAT_OVP_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_batovp_alarm_th);

static int bq2597x_enable_batocp(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BAT_OCP_ENABLE;
	else
		val = BQ2597X_BAT_OCP_DISABLE;

	val <<= BQ2597X_BAT_OCP_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_02,
				BQ2597X_BAT_OCP_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_batocp);

static int bq2597x_set_batocp_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BAT_OCP_BASE)
		threshold = BQ2597X_BAT_OCP_BASE;

	val = (threshold - BQ2597X_BAT_OCP_BASE) / BQ2597X_BAT_OCP_LSB;

	val <<= BQ2597X_BAT_OCP_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_02,
				BQ2597X_BAT_OCP_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_batocp_th);

static int bq2597x_enable_batocp_alarm(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BAT_OCP_ALM_ENABLE;
	else
		val = BQ2597X_BAT_OCP_ALM_DISABLE;

	val <<= BQ2597X_BAT_OCP_ALM_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_03,
				BQ2597X_BAT_OCP_ALM_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_batocp_alarm);

static int bq2597x_set_batocp_alarm_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BAT_OCP_ALM_BASE)
		threshold = BQ2597X_BAT_OCP_ALM_BASE;

	val = (threshold - BQ2597X_BAT_OCP_ALM_BASE) / BQ2597X_BAT_OCP_ALM_LSB;

	val <<= BQ2597X_BAT_OCP_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_03,
				BQ2597X_BAT_OCP_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_batocp_alarm_th);


static int bq2597x_set_busovp_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BUS_OVP_BASE)
		threshold = BQ2597X_BUS_OVP_BASE;

	val = (threshold - BQ2597X_BUS_OVP_BASE) / BQ2597X_BUS_OVP_LSB;

	val <<= BQ2597X_BUS_OVP_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_06,
				BQ2597X_BUS_OVP_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_busovp_th);

static int bq2597x_enable_busovp_alarm(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BUS_OVP_ALM_ENABLE;
	else
		val = BQ2597X_BUS_OVP_ALM_DISABLE;

	val <<= BQ2597X_BUS_OVP_ALM_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_07,
				BQ2597X_BUS_OVP_ALM_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_busovp_alarm);

static int bq2597x_set_busovp_alarm_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BUS_OVP_ALM_BASE)
		threshold = BQ2597X_BUS_OVP_ALM_BASE;

	val = (threshold - BQ2597X_BUS_OVP_ALM_BASE) / BQ2597X_BUS_OVP_ALM_LSB;

	val <<= BQ2597X_BUS_OVP_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_07,
				BQ2597X_BUS_OVP_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_busovp_alarm_th);

static int bq2597x_enable_busocp(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BUS_OCP_ENABLE;
	else
		val = BQ2597X_BUS_OCP_DISABLE;

	val <<= BQ2597X_BUS_OCP_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_08,
				BQ2597X_BUS_OCP_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_busocp);


static int bq2597x_set_busocp_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BUS_OCP_BASE)
		threshold = BQ2597X_BUS_OCP_BASE;

	val = (threshold - BQ2597X_BUS_OCP_BASE) / BQ2597X_BUS_OCP_LSB;

	val <<= BQ2597X_BUS_OCP_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_08,
				BQ2597X_BUS_OCP_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_busocp_th);

static int bq2597x_enable_busocp_alarm(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BUS_OCP_ALM_ENABLE;
	else
		val = BQ2597X_BUS_OCP_ALM_DISABLE;

	val <<= BQ2597X_BUS_OCP_ALM_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_09,
				BQ2597X_BUS_OCP_ALM_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_busocp_alarm);

static int bq2597x_set_busocp_alarm_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BUS_OCP_ALM_BASE)
		threshold = BQ2597X_BUS_OCP_ALM_BASE;

	val = (threshold - BQ2597X_BUS_OCP_ALM_BASE) / BQ2597X_BUS_OCP_ALM_LSB;

	val <<= BQ2597X_BUS_OCP_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_09,
				BQ2597X_BUS_OCP_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_busocp_alarm_th);

static int bq2597x_enable_batucp_alarm(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BAT_UCP_ALM_ENABLE;
	else
		val = BQ2597X_BAT_UCP_ALM_DISABLE;

	val <<= BQ2597X_BAT_UCP_ALM_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_04,
				BQ2597X_BAT_UCP_ALM_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_batucp_alarm);

static int bq2597x_set_batucp_alarm_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_BAT_UCP_ALM_BASE)
		threshold = BQ2597X_BAT_UCP_ALM_BASE;

	val = (threshold - BQ2597X_BAT_UCP_ALM_BASE) / BQ2597X_BAT_UCP_ALM_LSB;

	val <<= BQ2597X_BAT_UCP_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_04,
				BQ2597X_BAT_UCP_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_batucp_alarm_th);

static int bq2597x_set_acovp_th(struct bq2597x *bq, int threshold)
{
	int ret;
	u8 val;

	if (threshold < BQ2597X_AC_OVP_BASE)
		threshold = BQ2597X_AC_OVP_BASE;

	val = (threshold - BQ2597X_AC_OVP_BASE) /  BQ2597X_AC_OVP_LSB;

	val <<= BQ2597X_AC_OVP_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_05,
				BQ2597X_AC_OVP_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2597x_set_acovp_th);

static int bq2597x_enable_bat_therm(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_TSBAT_ENABLE;
	else
		val = BQ2597X_TSBAT_DISABLE;

	val <<= BQ2597X_TSBAT_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_18,
				BQ2597X_TSBAT_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_bat_therm);

/*
 * the input threshold is the raw value that would write to register directly.
 */
static int bq2597x_set_bat_therm_th(struct bq2597x *bq, u8 threshold)
{
	int ret;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_29, threshold);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_bat_therm_th);

static int bq2597x_enable_bus_therm(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_TSBUS_ENABLE;
	else
		val = BQ2597X_TSBUS_DISABLE;

	val <<= BQ2597X_TSBUS_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_18,
				BQ2597X_TSBUS_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_bus_therm);

/*
 * the input threshold is the raw value that would write to register directly.
 */
static int bq2597x_set_bus_therm_th(struct bq2597x *bq, u8 threshold)
{
	int ret;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_28, threshold);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_bus_therm_th);


static int bq2597x_enable_die_therm(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_TDIE_ENABLE;
	else
		val = BQ2597X_TDIE_DISABLE;

	val <<= BQ2597X_TDIE_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_18,
				BQ2597X_TDIE_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_die_therm);

/*
 * please be noted that the unit here is degC
 */
static int bq2597x_set_die_therm_th(struct bq2597x *bq, u8 threshold)
{
	int ret;
	u8 val;

	/*BE careful, LSB is here is 1/LSB, so we use multiply here*/
	val = (threshold - BQ2597X_TDIE_ALM_BASE) * BQ2597X_TDIE_ALM_LSB;
	val <<= BQ2597X_TDIE_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_2A,
				BQ2597X_TDIE_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_die_therm_th);

static int bq2597x_enable_adc(struct bq2597x *bq, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_ADC_ENABLE;
	else
		val = BQ2597X_ADC_DISABLE;

	val <<= BQ2597X_ADC_EN_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_14,
				BQ2597X_ADC_EN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_adc);

static int bq2597x_set_adc_average(struct bq2597x *bq, bool avg)
{
	int ret;
	u8 val;

	if (avg)
		val = BQ2597X_ADC_AVG_ENABLE;
	else
		val = BQ2597X_ADC_AVG_DISABLE;

	val <<= BQ2597X_ADC_AVG_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_14,
				BQ2597X_ADC_AVG_MASK, val);
	return 0;
}
EXPORT_SYMBOL_GPL(bq2597x_set_adc_average);

static int bq2597x_set_adc_scanrate(struct bq2597x *bq, bool oneshot)
{
	int ret;
	u8 val;

	if (oneshot)
		val = BQ2597X_ADC_RATE_ONESHOT;
	else
		val = BQ2597X_ADC_RATE_CONTINOUS;

	val <<= BQ2597X_ADC_RATE_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_14,
				BQ2597X_ADC_EN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_adc_scanrate);

static int bq2597x_set_adc_bits(struct bq2597x *bq, int bits)
{
	int ret;
	u8 val;

	if (bits > 15)
		bits = 15;
	if (bits < 12)
		bits = 12;
	val = 15 - bits;

	val <<= BQ2597X_ADC_SAMPLE_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_14,
				BQ2597X_ADC_SAMPLE_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_adc_bits);

#define ADC_REG_BASE 0x16
static int bq2597x_get_adc_data(struct bq2597x *bq, int channel,  int *result)
{
	int ret;
	u16 val;
	u16 t;

	if (channel > ADC_MAX_NUM)
		return -EINVAL;

	ret = bq2597x_read_word(bq, ADC_REG_BASE + (channel << 1), &val);
	if (ret < 0)
		return ret;
	t = val & 0xFF;
	t <<= 8;
	t |= (val >> 8) & 0xFF; 
	*result = (int)t;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2597x_get_adc_data);

static int bq2597x_set_adc_scan(struct bq2597x *bq, int channel, bool enable)
{
	int ret;
	u8 reg;
	u8 mask;
	u8 shift;
	u8 val;

	if (channel > ADC_MAX_NUM)
		return -EINVAL;

	if (channel == ADC_IBUS) {
		reg = BQ2597X_REG_14;
		shift = BQ2597X_IBUS_ADC_DIS_SHIFT;
		mask = BQ2597X_IBUS_ADC_DIS_MASK;
	} else {
		reg = BQ2597X_REG_15;
		shift = 8 - channel;
		mask = 1 << shift;
	}

	if (enable)
		val = 0 << shift;
	else
		val = 1 << shift;

	ret = bq2597x_update_bits(bq, reg, mask, val);

	return ret;
}

static int bq2597x_set_alarm_int_mask(struct bq2597x *bq, u8 mask)
{
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_0F, &val);
	if (ret)
		return ret;

	val |= mask;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_0F, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_alarm_int_mask);

static int bq2597x_clear_alarm_int_mask(struct bq2597x *bq, u8 mask)
{
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_0F, &val);
	if (ret)
		return ret;

	val &= ~mask;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_0F, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_clear_alarm_int_mask);

static int bq2597x_set_fault_int_mask(struct bq2597x *bq, u8 mask)
{
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_12, &val);
	if (ret)
		return ret;

	val |= mask;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_12, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_fault_int_mask);

static int bq2597x_clear_fault_int_mask(struct bq2597x *bq, u8 mask)
{
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_12, &val);
	if (ret)
		return ret;

	val &= ~mask;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_12, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_clear_fault_int_mask);

/*
 *
 *
 */

static int bq2597x_detect_device(struct bq2597x *bq)
{
	int ret;
	u8 data;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_13, &data);
	if (ret == 0) {
		bq->part_no = (data & BQ2597X_DEV_ID_MASK);
		bq->part_no >>= BQ2597X_DEV_ID_SHIFT;
		bq->revision = (data & BQ2597X_DEV_REV_MASK);
		bq->revision >>= BQ2597X_DEV_REV_SHIFT;
	}

	return ret;
}

static int bq2597x_parse_dt(struct bq2597x *bq, struct device *dev)
{
	int ret;
	struct device_node *np = dev->of_node;

	bq->cfg = devm_kzalloc(dev, sizeof(struct bq2597x_cfg),
					GFP_KERNEL);

	if (!bq->cfg) {
		pr_err("Out of memory\n");
		return -ENOMEM;
	}

	bq->cfg->bat_ovp_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-ovp-disable");
	bq->cfg->bat_ocp_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-ocp-disable");
	bq->cfg->bat_ovp_alm_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-ovp-alarm-disable");
	bq->cfg->bat_ocp_alm_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-ocp-alarm-disable");
	bq->cfg->bus_ocp_disable = of_property_read_bool(np,
			"ti,bq2597x,bus-ocp-disable");
	bq->cfg->bus_ovp_alm_disable = of_property_read_bool(np,
			"ti,bq2597x,bus-ovp-alarm-disable");
	bq->cfg->bus_ocp_alm_disable = of_property_read_bool(np,
			"ti,bq2597x,bus-ocp-alarm-disable");
	bq->cfg->bat_ucp_alm_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-ucp-alarm-disable");
	bq->cfg->bat_therm_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-therm-disable");
	bq->cfg->bus_therm_disable = of_property_read_bool(np,
			"ti,bq2597x,bus-therm-disable");
	bq->cfg->die_therm_disable = of_property_read_bool(np,
			"ti,bq2597x,die-therm-disable");

	ret = of_property_read_u32(np, "ti,bq2597x,bat-ovp-threshold",
			&bq->cfg->bat_ovp_th);
	if (ret) {
		pr_err("failed to read bat-ovp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bat-ovp-alarm-threshold",
			&bq->cfg->bat_ovp_alm_th);
	if (ret) {
		pr_err("failed to read bat-ovp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bat-ocp-threshold",
			&bq->cfg->bat_ocp_th);
	if (ret) {
		pr_err("failed to read bat-ocp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bat-ocp-alarm-threshold",
			&bq->cfg->bat_ocp_alm_th);
	if (ret) {
		pr_err("failed to read bat-ocp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bus-ovp-threshold",
			&bq->cfg->bus_ovp_th);
	if (ret) {
		pr_err("failed to read bus-ovp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bus-ovp-alarm-threshold",
			&bq->cfg->bus_ovp_alm_th);
	if (ret) {
		pr_err("failed to read bus-ovp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bus-ocp-threshold",
			&bq->cfg->bus_ocp_th);
	if (ret) {
		pr_err("failed to read bus-ocp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bus-ocp-alarm-threshold",
			&bq->cfg->bus_ocp_alm_th);
	if (ret) {
		pr_err("failed to read bus-ocp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bat-ucp-alarm-threshold",
			&bq->cfg->bat_ucp_alm_th);
	if (ret) {
		pr_err("failed to read bat-ucp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bat-therm-threshold",
			&bq->cfg->bat_therm_th);
	if (ret) {
		pr_err("failed to read bat-therm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bus-therm-threshold",
			&bq->cfg->bus_therm_th);
	if (ret) {
		pr_err("failed to read bus-therm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,die-therm-threshold",
			&bq->cfg->die_therm_th);
	if (ret) {
		pr_err("failed to read die-therm-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2597x,ac-ovp-threshold",
			&bq->cfg->ac_ovp_th);
	if (ret) {
		pr_err("failed to read ac-ovp-threshold\n");
		return ret;
	}

	return 0;
}

static int bq2597x_init_protection(struct bq2597x *bq)
{
	int ret;

	ret = bq2597x_enable_batovp(bq, !bq->cfg->bat_ovp_disable);
	pr_info("%s bat ovp %s\n",
		bq->cfg->bat_ovp_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_batocp(bq, !bq->cfg->bat_ocp_disable);
	pr_info("%s bat ocp %s\n",
		bq->cfg->bat_ocp_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_batovp_alarm(bq, !bq->cfg->bat_ovp_alm_disable);
	pr_info("%s bat ovp alarm %s\n",
		bq->cfg->bat_ovp_alm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_batocp_alarm(bq, !bq->cfg->bat_ocp_alm_disable);
	pr_info("%s bat ocp alarm %s\n",
		bq->cfg->bat_ocp_alm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_batucp_alarm(bq, !bq->cfg->bat_ucp_alm_disable);
	pr_info("%s bat ocp alarm %s\n",
		bq->cfg->bat_ucp_alm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_busovp_alarm(bq, !bq->cfg->bus_ovp_alm_disable);
	pr_info("%s bus ovp alarm %s\n",
		bq->cfg->bus_ovp_alm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_busocp(bq, !bq->cfg->bus_ocp_disable);
	pr_info("%s bus ocp %s\n",
		bq->cfg->bus_ocp_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_busocp_alarm(bq, !bq->cfg->bus_ocp_alm_disable);
	pr_info("%s bus ocp alarm %s\n",
		bq->cfg->bus_ocp_alm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_bat_therm(bq, !bq->cfg->bat_therm_disable);
	pr_info("%s bat therm %s\n",
		bq->cfg->bat_therm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_bus_therm(bq, !bq->cfg->bus_therm_disable);
	pr_info("%s bus therm %s\n",
		bq->cfg->bus_therm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_die_therm(bq, !bq->cfg->die_therm_disable);
	pr_info("%s die therm %s\n",
		bq->cfg->die_therm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = bq2597x_set_batovp_th(bq, bq->cfg->bat_ovp_th);
	pr_info("set bat ovp th %d %s\n", bq->cfg->bat_ovp_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_batovp_alarm_th(bq, bq->cfg->bat_ovp_alm_th);
	pr_info("set bat ovp alarm threshold %d %s\n", bq->cfg->bat_ovp_alm_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_batocp_th(bq, bq->cfg->bat_ocp_th);
	pr_info("set bat ocp threshold %d %s\n", bq->cfg->bat_ocp_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_batocp_alarm_th(bq, bq->cfg->bat_ocp_alm_th);
	pr_info("set bat ocp alarm threshold %d %s\n", bq->cfg->bat_ocp_alm_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_busovp_th(bq, bq->cfg->bus_ovp_th);
	pr_info("set bus ovp threshold %d %s\n", bq->cfg->bus_ovp_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_busovp_alarm_th(bq, bq->cfg->bus_ovp_alm_th);
	pr_info("set bus ovp alarm threshold %d %s\n", bq->cfg->bus_ovp_alm_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_busocp_th(bq, bq->cfg->bus_ocp_th);
	pr_info("set bus ocp threshold %d %s\n", bq->cfg->bus_ocp_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_busocp_alarm_th(bq, bq->cfg->bus_ocp_alm_th);
	pr_info("set bus ocp alarm th %d %s\n", bq->cfg->bus_ocp_alm_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_batucp_alarm_th(bq, bq->cfg->bat_ucp_alm_th);
	pr_info("set bat ucp threshold %d %s\n", bq->cfg->bat_ucp_alm_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_bat_therm_th(bq, bq->cfg->bat_therm_th);
	pr_info("set die therm threshold %d %s\n", bq->cfg->bat_therm_th,
		!ret ? "successfully" : "failed");
	ret = bq2597x_set_bus_therm_th(bq, bq->cfg->bus_therm_th);
	pr_info("set bus therm threshold %d %s\n", bq->cfg->bus_therm_th,
		!ret ? "successfully" : "failed");
	ret = bq2597x_set_die_therm_th(bq, bq->cfg->die_therm_th);
	pr_info("set die therm threshold %d %s\n", bq->cfg->die_therm_th,
		!ret ? "successfully" : "failed");

	ret = bq2597x_set_acovp_th(bq, bq->cfg->ac_ovp_th);
	pr_info("set ac ovp threshold %d %s\n", bq->cfg->ac_ovp_th,
		!ret ? "successfully" : "failed");

	return 0;
}

static int bq2597x_init_adc(struct bq2597x *bq)
{

	bq2597x_set_adc_scanrate(bq, false);
	bq2597x_set_adc_bits(bq, 13);
	bq2597x_set_adc_average(bq, true);
	bq2597x_set_adc_scan(bq, ADC_IBUS, true);
	bq2597x_set_adc_scan(bq, ADC_VBUS, true);
	bq2597x_set_adc_scan(bq, ADC_VOUT, true);
	bq2597x_set_adc_scan(bq, ADC_VBAT, true);
	bq2597x_set_adc_scan(bq, ADC_IBAT, true);
	bq2597x_set_adc_scan(bq, ADC_TBUS, true);
	bq2597x_set_adc_scan(bq, ADC_TBAT, true);
	bq2597x_set_adc_scan(bq, ADC_TDIE, true);
	bq2597x_set_adc_scan(bq, ADC_VAC, true);

	bq2597x_enable_adc(bq, true);

	return 0;
}

static int bq2597x_init_int_src(struct bq2597x *bq)
{
	int ret;
	/*TODO:be careful ts bus and ts bat alarm bit mask is in
	 *	fault mask register, so you need call
	 *	bq2597x_set_fault_int_mask for tsbus and tsbat alarm*/
	ret = bq2597x_set_alarm_int_mask(bq, ADC_DONE
		/*			| BAT_UCP_ALARM*/
					| BAT_OVP_ALARM);
	if (ret) {
		pr_err("failed to set alarm mask:%d\n", ret);
		return ret;
	}
#if 0
	ret = bq2597x_set_fault_int_mask(bq, TS_BUS_FAULT);
	if (ret) {
		pr_err("failed to set fault mask:%d\n", ret);
		return ret;
	}
#endif
	return ret;
}

static int bq2597x_init_device(struct bq2597x *bq)
{
	bq2597x_enable_wdt(bq, false);
	bq2597x_init_protection(bq);
	bq2597x_init_adc(bq);
	bq2597x_init_int_src(bq);

	return 0;
}


static int bq2597x_set_present(struct bq2597x *bq, bool present)
{
	bq->usb_present = present;

	if (present)
		bq2597x_init_device(bq);
	return 0;
}


static ssize_t bq2597x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bq2597x *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret ;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq25970");
	for (addr = 0x0; addr <= 0x2A; addr++) {
		ret = bq2597x_read_byte(bq, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
						"Reg[%.2X] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bq2597x_store_register(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2597x *bq = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf,"%x %x",&reg, &val);
	if (ret == 2 && reg <= 0x2A) {
		bq2597x_write_byte(bq,(unsigned char)reg,(unsigned char)val);
	}

	return count;
}


static DEVICE_ATTR(registers, 0660, bq2597x_show_registers, bq2597x_store_register);

static struct attribute *bq2597x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2597x_attr_group = {
	.attrs = bq2597x_attributes,
};

static enum power_supply_property bq2597x_charger_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_STATUS,

	POWER_SUPPLY_PROP_TI_BATTERY_PRESENT,
	POWER_SUPPLY_PROP_TI_VBUS_PRESENT,
	POWER_SUPPLY_PROP_TI_BATTERY_VOLTAGE,
	POWER_SUPPLY_PROP_TI_BATTERY_CURRENT,
	POWER_SUPPLY_PROP_TI_BATTERY_TEMPERATURE,
	POWER_SUPPLY_PROP_TI_BUS_VOLTAGE,
	POWER_SUPPLY_PROP_TI_BUS_CURRENT,
	POWER_SUPPLY_PROP_TI_BUS_TEMPERATURE,
	POWER_SUPPLY_PROP_TI_DIE_TEMPERATURE,
	POWER_SUPPLY_PROP_TI_ALARM_STATUS,
	POWER_SUPPLY_PROP_TI_FAULT_STATUS,


};

static void bq2597x_update_status(struct bq2597x *bq);

static int bq2597x_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct bq2597x *bq = container_of(psy, struct bq2597x, fc2_psy);
	int ret;
	int result;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		bq2597x_check_charge_enabled(bq, &bq->charge_enabled);
		val->intval = bq->charge_enabled;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq->usb_present;
		break;
	case POWER_SUPPLY_PROP_TI_BATTERY_PRESENT:
		val->intval = bq->batt_present;
		break;
	case POWER_SUPPLY_PROP_TI_VBUS_PRESENT:
		val->intval = bq->vbus_present;
		break;
	case POWER_SUPPLY_PROP_TI_BATTERY_VOLTAGE:
		ret = bq2597x_get_adc_data(bq, ADC_VBAT, &result);
		if (!ret)
			bq->vbat_volt = result;

		val->intval = bq->vbat_volt;
		break;
	case POWER_SUPPLY_PROP_TI_BATTERY_CURRENT:
		ret = bq2597x_get_adc_data(bq, ADC_IBAT, &result);
		if (!ret)
			bq->ibat_curr = result;

		val->intval = bq->ibat_curr;
		break;
	case POWER_SUPPLY_PROP_TI_BATTERY_TEMPERATURE:
		ret = bq2597x_get_adc_data(bq, ADC_TBAT, &result);
		if (!ret)
			bq->bat_temp = result;

		val->intval = bq->bat_temp;
		break;
	case POWER_SUPPLY_PROP_TI_BUS_VOLTAGE:
		ret = bq2597x_get_adc_data(bq, ADC_VBUS, &result);
		if (!ret)
			bq->vbus_volt = result;

		val->intval = bq->vbus_volt;
		break;
	case POWER_SUPPLY_PROP_TI_BUS_CURRENT:
		ret = bq2597x_get_adc_data(bq, ADC_IBUS, &result);
		if (!ret)
			bq->ibus_curr = result;

		val->intval = bq->ibus_curr;
		break;
	case POWER_SUPPLY_PROP_TI_BUS_TEMPERATURE:
		ret = bq2597x_get_adc_data(bq, ADC_TBUS, &result);
		if (!ret)
			bq->bus_temp = result;

		val->intval = bq->bus_temp;
		break;
	case POWER_SUPPLY_PROP_TI_DIE_TEMPERATURE:
		ret = bq2597x_get_adc_data(bq, ADC_TDIE, &result);
		if (!ret)
			bq->die_temp = result;

		val->intval = bq->die_temp;
		break;
	case POWER_SUPPLY_PROP_TI_ALARM_STATUS:
		/* call bq2597x_update_status to get recent status/flag
		 * alarm/fault clear would not trigger interrupt, so to
		 * to read to again to reflect current status.
		 * */
		bq2597x_update_status(bq);

		val->intval = ((bq->bat_ovp_alarm << BAT_OVP_ALARM_SHIFT)
			| (bq->bat_ocp_alarm << BAT_OCP_ALARM_SHIFT)
			| (bq->bat_ucp_alarm << BAT_UCP_ALARM_SHIFT)
			| (bq->bus_ovp_alarm << BUS_OVP_ALARM_SHIFT)
			| (bq->bus_ocp_alarm << BUS_OCP_ALARM_SHIFT)
			| (bq->bat_therm_alarm << BAT_THERM_ALARM_SHIFT)
			| (bq->bus_therm_alarm << BUS_THERM_ALARM_SHIFT)
			| (bq->die_therm_alarm << DIE_THERM_ALARM_SHIFT));
		break;

	case POWER_SUPPLY_PROP_TI_FAULT_STATUS:
		bq2597x_update_status(bq);
		val->intval = ((bq->bat_ovp_fault << BAT_OVP_FAULT_SHIFT)
			| (bq->bat_ocp_fault << BAT_OCP_FAULT_SHIFT)
			| (bq->bus_ovp_fault << BUS_OVP_FAULT_SHIFT)
			| (bq->bus_ocp_fault << BUS_OCP_FAULT_SHIFT)
			| (bq->bat_therm_fault << BAT_THERM_FAULT_SHIFT)
			| (bq->bus_therm_fault << BUS_THERM_FAULT_SHIFT)
			| (bq->die_therm_fault << DIE_THERM_FAULT_SHIFT));
		break;


	default:
		return -EINVAL;

	}

	return 0;
}

static int bq2597x_charger_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct bq2597x *bq = container_of(psy,
				struct bq2597x, fc2_psy);
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	//	if (bq->usb_present) {
			bq2597x_enable_charge(bq, val->intval);
			bq2597x_check_charge_enabled(bq, &bq->charge_enabled);
			pr_info("POWER_SUPPLY_PROP_CHARGING_ENABLED: %s\n",
					val->intval ? "enable" : "disable");
	//	}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		bq2597x_set_present(bq, !!val->intval);
		pr_info("set present :%d\n", val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq2597x_charger_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int bq2597x_psy_register(struct bq2597x *bq)
{
	int ret;

	bq->fc2_psy.name = "bq2597x";
	bq->fc2_psy.type = POWER_SUPPLY_TYPE_MAINS;
	bq->fc2_psy.properties = bq2597x_charger_props;
	bq->fc2_psy.num_properties = ARRAY_SIZE(bq2597x_charger_props);
	bq->fc2_psy.get_property = bq2597x_charger_get_property;
	bq->fc2_psy.set_property = bq2597x_charger_set_property;
/*	bq->fc2_psy.external_power_changed = bq2597x_external_power_changed;*/
	bq->fc2_psy.property_is_writeable = bq2597x_charger_is_writeable;

	ret = power_supply_register(bq->dev, &bq->fc2_psy);
	if (ret < 0) {
		pr_err("failed to register fc2_psy:%d\n", ret);
		return ret;
	}

	pr_info("power supply register successfully\n");

	return 0;
}

static void bq2597x_dump_reg(struct bq2597x *bq)
{

	int ret;
	u8 val;
	u8 addr;

	for (addr = 0x00; addr < 0x2B; addr++) {
		ret = bq2597x_read_byte(bq, addr, &val);
		if (!ret)
			pr_err("Reg[%02X] = 0x%02X\n", addr, val);
	}

}
/*
 * update alarm or fault status, PD policy manager will process the events
 */
static void bq2597x_update_status(struct bq2597x *bq)
{
	int ret;
	u8 flag = 0;
	u8 stat = 0;
	bool changed = false;

	mutex_lock(&bq->data_lock);
	/*read to clear alarm flag*/
	bq2597x_read_byte(bq, BQ2597X_REG_0E, &flag);

	ret = bq2597x_read_byte(bq, BQ2597X_REG_0D, &stat);
	if (!ret && stat != bq->prev_alarm) {
		changed = true;
		bq->prev_alarm = stat;
		bq->bat_ovp_alarm = !!(stat & BAT_OVP_ALARM);
		bq->bat_ocp_alarm = !!(stat & BAT_OCP_ALARM);
		bq->bus_ovp_alarm = !!(stat & BUS_OVP_ALARM);
		bq->bus_ocp_alarm = !!(stat & BUS_OCP_ALARM);
		bq->batt_present  = !!(stat & VBAT_INSERT);
		bq->vbus_present  = !!(stat & VBUS_INSERT);
		bq->bat_ucp_alarm = !!(stat & BAT_UCP_ALARM);
	}

	ret = bq2597x_read_byte(bq, BQ2597X_REG_11, &flag);
	if (!ret && flag != bq->prev_fault) {
		changed = true;
		bq->prev_fault = flag;
		bq->bat_ovp_fault = !!(flag & BAT_OVP_FAULT);
		bq->bat_ocp_fault = !!(flag & BAT_OCP_FAULT);
		bq->bus_ovp_fault = !!(flag & BUS_OVP_FAULT);
		bq->bus_ocp_fault = !!(flag & BUS_OCP_FAULT);
		bq->bat_therm_fault = !!(flag & TS_BAT_FAULT);
		bq->bus_therm_fault = !!(flag & TS_BUS_FAULT);

		bq->bat_therm_alarm = !!(flag & TBUS_TBAT_ALARM);
		bq->bus_therm_alarm = !!(flag & TBUS_TBAT_ALARM);
	}

	mutex_unlock(&bq->data_lock);
	
	if (changed)
		power_supply_changed(&bq->fc2_psy);

}



static irqreturn_t bq2597x_charger_interrupt(int irq, void *dev_id)
{
	struct bq2597x *bq = dev_id;

	pr_err("enter");

	mutex_lock(&bq->irq_complete);
	bq->irq_waiting = true;
	if (!bq->resume_completed) {
		dev_dbg(bq->dev, "IRQ triggered before device-resume\n");
		if (!bq->irq_disabled) {
			disable_irq_nosync(irq);
			bq->irq_disabled = true;
		}
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	bq->irq_waiting = false;

	/* TODO */
	bq2597x_update_status(bq);

	bq2597x_dump_reg(bq);

	mutex_unlock(&bq->irq_complete);

	pr_err("alarm stat=0x%02X, fault flag =0x%02X\n", bq->prev_alarm, bq->prev_fault);

	return IRQ_HANDLED;
}


static void determine_initial_status(struct bq2597x *bq)
{
	bq2597x_charger_interrupt(bq->client->irq, bq);
}



static int show_registers(struct seq_file *m, void *data)
{
	struct bq2597x *bq = m->private;
	u8 addr;
	int ret;
	u8 val;

	for (addr = 0x0; addr <= 0x2B; addr++) {
		ret = bq2597x_read_byte(bq, addr, &val);
		if (!ret)
			seq_printf(m, "Reg[%02X] = 0x%02X\n", addr, val);
	}
	return 0;
}


static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct bq2597x *bq = inode->i_private;

	return single_open(file, show_registers, bq);
}


static const struct file_operations reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= reg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debugfs_entry(struct bq2597x *bq)
{
	bq->debug_root = debugfs_create_dir("bq2597x", NULL);
	if (!bq->debug_root)
		pr_err("Failed to create debug dir\n");

	if (bq->debug_root) {
		debugfs_create_file("registers",
					S_IFREG | S_IRUGO,
					bq->debug_root, bq, &reg_debugfs_ops);

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


static int bq2597x_charger_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct bq2597x *bq;
	struct power_supply *pd_psy;

	int ret;
#if 0
	pd_psy = power_supply_get_by_name("usb_pd");
	if (!pd_psy) {
		dev_dbg(&client->dev, "usb pd power supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}
#endif
	bq = devm_kzalloc(&client->dev, sizeof(struct bq2597x), GFP_KERNEL);
	if (!bq) {
		pr_err("Out of memory\n");
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->pd_psy = pd_psy;

	bq->client = client;
	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->charging_disable_lock);
	mutex_init(&bq->irq_complete);

	bq->resume_completed = true;
	bq->irq_waiting = false;

	ret = bq2597x_detect_device(bq);
	if (ret) {
		pr_err("No bq2597x device found!\n");
		return -ENODEV;
	}

	if (client->dev.of_node) {
		ret = bq2597x_parse_dt(bq, &client->dev);
		if (ret)
			return -EIO;
	}

	ret = bq2597x_init_device(bq);
	if (ret) {
		pr_err("Failed to init device\n");
		return ret;
	}

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, bq2597x_charger_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"bq2597x charger irq", bq);
		if (ret < 0) {
			pr_err("request irq for irq=%d failed, ret =%d\n",
							client->irq, ret);
			goto err_1;
		}
		enable_irq_wake(client->irq);
	}

	ret = bq2597x_psy_register(bq);
	if (ret)
		return ret;

	device_init_wakeup(bq->dev, 1);
	create_debugfs_entry(bq);

	ret = sysfs_create_group(&bq->dev->kobj, &bq2597x_attr_group);
	if (ret) {
		pr_err("failed to register sysfs. err: %d\n", ret);
		goto err_1;
	}


	determine_initial_status(bq);

	pr_info("bq2597x probe successfully, Part Num:%d, Revision:%d\n!",
				bq->part_no, bq->revision);

	return 0;

err_1:
	power_supply_unregister(&bq->fc2_psy);
	return ret;
}


static inline bool is_device_suspended(struct bq2597x *bq)
{
	return !bq->resume_completed;
}

static int bq2597x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2597x *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);
	pr_err("Suspend successfully!");

	return 0;
}

static int bq2597x_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2597x *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int bq2597x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2597x *bq = i2c_get_clientdata(client);


	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&bq->irq_complete);
		bq2597x_charger_interrupt(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}

	power_supply_changed(&bq->fc2_psy);
	pr_err("Resume successfully!");

	return 0;
}
static int bq2597x_charger_remove(struct i2c_client *client)
{
	struct bq2597x *bq = i2c_get_clientdata(client);


	bq2597x_enable_adc(bq, false);

	power_supply_unregister(&bq->fc2_psy);

	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->irq_complete);

	debugfs_remove_recursive(bq->debug_root);

	sysfs_remove_group(&bq->dev->kobj, &bq2597x_attr_group);

	return 0;
}


static void bq2597x_charger_shutdown(struct i2c_client *client)
{
	pr_info("Shutdown Successfully\n");
}

static struct of_device_id bq2597x_charger_match_table[] = {
	{.compatible = "ti,bq25970-charger",},
	{},
};
MODULE_DEVICE_TABLE(of, bq2597x_charger_match_table);

static const struct i2c_device_id bq2597x_charger_id[] = {
	{ "bq25970-charger", BQ25970 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2597x_charger_id);

static const struct dev_pm_ops bq2597x_pm_ops = {
	.resume		= bq2597x_resume,
	.suspend_noirq = bq2597x_suspend_noirq,
	.suspend	= bq2597x_suspend,
};
static struct i2c_driver bq2597x_charger_driver = {
	.driver		= {
		.name	= "bq2597x-charger",
		.owner	= THIS_MODULE,
		.of_match_table = bq2597x_charger_match_table,
		.pm	= &bq2597x_pm_ops,
	},
	.id_table	= bq2597x_charger_id,

	.probe		= bq2597x_charger_probe,
	.remove		= bq2597x_charger_remove,
	.shutdown	= bq2597x_charger_shutdown,
};

module_i2c_driver(bq2597x_charger_driver);

MODULE_DESCRIPTION("TI BQ2597x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");

