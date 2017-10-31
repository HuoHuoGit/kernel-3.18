/*
 * TUSB422 Power Delivery
 *
 * Author: Brian Quach <brian.quach@ti.com>
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#define pr_fmt(fmt)	"[FC2-PM]: %s: " fmt, __func__

#include "tcpm.h"
#include "tusb422_common.h"
#include "usb_pd.h"
#include "usb_pd_policy_engine.h"
#include "usb_pd_protocol.h"
#include "version.h"
#include "usb_pd_policy_manager.h"
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#ifdef pr_debug
#undef pr_debug
#define pr_debug pr_err
#endif

#ifdef CONFIG_TUSB422
	#include <linux/string.h>
#else
	#include <string.h>
#endif

#define PDO_DUAL_ROLE_POWER_BIT           ((uint32_t)0x01 << 29)
#define SRC_PDO_USB_SUSPEND_BIT           ((uint32_t)0x01 << 28)
#define SNK_PDO_HIGHER_CAPABILITY_BIT     ((uint32_t)0x01 << 28)
#define PDO_EXTERNALLY_POWERED_BIT        ((uint32_t)0x01 << 27)
#define PDO_USB_COMM_CAPABLE_BIT          ((uint32_t)0x01 << 26)
#define PDO_DUAL_ROLE_DATA_BIT            ((uint32_t)0x01 << 25)
#define SRC_PDO_UNCHUNKED_EXT_MSG_SUP_BIT ((uint32_t)0x01 << 24)


#define BATT_MAX_CHG_VOLT		4400
#define BATT_FAST_CHG_CURR		6000
#define	BUS_OVP_THRESHOLD		12000
#define	BUS_OVP_ALARM_THRESHOLD		9500

#define BUS_VOLT_INIT_UP		200		

#define BAT_VOLT_LOOP_LMT		BATT_MAX_CHG_VOLT
#define BAT_CURR_LOOP_LMT		BATT_FAST_CHG_CURR
#define BUS_VOLT_LOOP_LMT		BUS_OVP_THRESHOLD


static const struct sys_config sys_config = {
	.bat_volt_lp_lmt		= BAT_VOLT_LOOP_LMT,
	.bat_curr_lp_lmt		= BAT_CURR_LOOP_LMT + 1000,
	.bus_volt_lp_lmt		= BUS_VOLT_LOOP_LMT,
	.bus_curr_lp_lmt		= BAT_CURR_LOOP_LMT >> 1,

	.fc2_taper_current		= 2000,
	.flash2_policy.down_steps	= -1,
	.flash2_policy.volt_hysteresis	= 50,

	.min_vbat_start_flash2		= 3500,
};

static adapter_t adapter;
static pm_t pm_state;

static usb_pd_port_config_t pd_port_config[NUM_TCPC_DEVICES];  

usb_pd_port_config_t* usb_pd_pm_get_config(unsigned int port)
{
	return &pd_port_config[port];
}

uint32_t get_data_object(uint8_t *obj_data)
{
	return((((uint32_t)obj_data[3]) << 24) | 
		   (((uint32_t)obj_data[2]) << 16) | 
		   (((uint32_t)obj_data[1]) << 8) | 
		   (((uint32_t)obj_data[0]))); 
}


//------------------------------------------------------------------------------
// calc_power and calc_current
//
//   Voltage is in 50mV units.
//   Current is in 10mA units.
//   Power is in 250mW units.
//------------------------------------------------------------------------------
#if 0
static uint16_t calc_power(uint32_t voltage_50mv, uint32_t current_10ma)
{
	return (voltage_50mv * current_10ma) / 500;
}

static uint16_t calc_current(uint32_t power_250mw, uint32_t voltage_50mv)
{
	return (power_250mw * 500) / voltage_50mv;
}

#endif

typedef struct
{
	enum supply_type_t supply_type;
	uint16_t min_voltage;
	uint16_t max_voltage;
	uint16_t max_current;
	uint16_t max_power;
	bool cap_mismatch;
} pdo_offer_t;


static void usb_pd_pm_update_sw_status(void)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pm_state.sw_psy) {
		pm_state.sw_psy = power_supply_get_by_name("bq2589x");
		if (!pm_state.sw_psy) {
			pr_err("sw_psy not found\n");
			return;
		}
	}

	ret = pm_state.sw_psy->get_property(pm_state.sw_psy, POWER_SUPPLY_PROP_INPUT_SUSPEND, &val);
	if (!ret)
		pm_state.bq2589x.charge_hized = val.intval;

	ret = pm_state.sw_psy->get_property(pm_state.sw_psy, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		pm_state.bq2589x.charge_enabled = val.intval;

}

static void usb_pd_pm_update_fc_status(void)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pm_state.fc_psy) {
		pm_state.fc_psy = power_supply_get_by_name("bq2597x");
		if (!pm_state.fc_psy) {
			pr_err("fc_psy not found\n");
			return;
		}
	}

	ret = pm_state.fc_psy->get_property(pm_state.fc_psy, POWER_SUPPLY_PROP_TI_BATTERY_VOLTAGE, &val);
	if (!ret)
		pm_state.bq2597x.vbat_volt = val.intval; 

	ret = pm_state.fc_psy->get_property(pm_state.fc_psy, POWER_SUPPLY_PROP_TI_BATTERY_CURRENT, &val);
	if (!ret)
		pm_state.bq2597x.ibat_curr = val.intval; 

	ret = pm_state.fc_psy->get_property(pm_state.fc_psy, POWER_SUPPLY_PROP_TI_BUS_VOLTAGE, &val);
	if (!ret)
		pm_state.bq2597x.vbus_volt = val.intval; 

	ret = pm_state.fc_psy->get_property(pm_state.fc_psy, POWER_SUPPLY_PROP_TI_BUS_CURRENT, &val);
	if (!ret)
		pm_state.bq2597x.ibus_curr = val.intval; 

	ret = pm_state.fc_psy->get_property(pm_state.fc_psy, POWER_SUPPLY_PROP_TI_BUS_TEMPERATURE, &val);
	if (!ret)
		pm_state.bq2597x.bus_temp = val.intval; 

	ret = pm_state.fc_psy->get_property(pm_state.fc_psy, POWER_SUPPLY_PROP_TI_BATTERY_TEMPERATURE, &val);
	if (!ret)
		pm_state.bq2597x.bat_temp = val.intval; 

	ret = pm_state.fc_psy->get_property(pm_state.fc_psy, POWER_SUPPLY_PROP_TI_DIE_TEMPERATURE, &val);
	if (!ret)
		pm_state.bq2597x.die_temp = val.intval; 

	ret = pm_state.fc_psy->get_property(pm_state.fc_psy, POWER_SUPPLY_PROP_TI_BATTERY_PRESENT, &val);
	if (!ret)
		pm_state.bq2597x.batt_pres = val.intval;

	ret = pm_state.fc_psy->get_property(pm_state.fc_psy, POWER_SUPPLY_PROP_TI_VBUS_PRESENT, &val);
	if (!ret)
		pm_state.bq2597x.vbus_pres = val.intval;

	ret = pm_state.fc_psy->get_property(pm_state.fc_psy, POWER_SUPPLY_PROP_CHARGE_ENABLED, &val);
	if (!ret)
		pm_state.bq2597x.charge_enabled = val.intval;
	
	ret = pm_state.fc_psy->get_property(pm_state.fc_psy, POWER_SUPPLY_PROP_TI_ALARM_STATUS, &val);
	if (!ret) {
		pm_state.bq2597x.bat_ovp_alarm = !!(val.intval & BAT_OVP_ALARM_MASK); 
		pm_state.bq2597x.bat_ocp_alarm = !!(val.intval & BAT_OCP_ALARM_MASK); 
		pm_state.bq2597x.bus_ovp_alarm = !!(val.intval & BUS_OVP_ALARM_MASK); 
		pm_state.bq2597x.bus_ocp_alarm = !!(val.intval & BUS_OCP_ALARM_MASK); 
		pm_state.bq2597x.bat_ucp_alarm = !!(val.intval & BAT_UCP_ALARM_MASK); 
		pm_state.bq2597x.bat_therm_alarm = !!(val.intval & BAT_THERM_ALARM_MASK);
		pm_state.bq2597x.bus_therm_alarm = !!(val.intval & BUS_THERM_ALARM_MASK);
		pm_state.bq2597x.die_therm_alarm = !!(val.intval & DIE_THERM_ALARM_MASK);
	}

	ret = pm_state.fc_psy->get_property(pm_state.fc_psy, POWER_SUPPLY_PROP_TI_FAULT_STATUS, &val);
	if (!ret) {
		pm_state.bq2597x.bat_ovp_fault = !!(val.intval & BAT_OVP_FAULT_MASK); 
		pm_state.bq2597x.bat_ocp_fault = !!(val.intval & BAT_OCP_FAULT_MASK); 
		pm_state.bq2597x.bus_ovp_fault = !!(val.intval & BUS_OVP_FAULT_MASK); 
		pm_state.bq2597x.bus_ocp_fault = !!(val.intval & BUS_OCP_FAULT_MASK); 
		pm_state.bq2597x.bat_therm_fault = !!(val.intval & BAT_THERM_FAULT_MASK);
		pm_state.bq2597x.bus_therm_fault = !!(val.intval & BUS_THERM_FAULT_MASK);
		pm_state.bq2597x.die_therm_fault = !!(val.intval & DIE_THERM_FAULT_MASK);
	}

}


static int usb_pd_pm_enable_fc(bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pm_state.fc_psy) {
		pm_state.fc_psy = power_supply_get_by_name("bq2597x");
		if (!pm_state.fc_psy) {
			return -ENODEV;
		}
	}

	val.intval = enable;
	ret = pm_state.fc_psy->set_property(pm_state.fc_psy, 
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	
	return ret;
}

static int usb_pd_pm_enable_sw(bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pm_state.sw_psy) {
		pm_state.sw_psy = power_supply_get_by_name("bq2589x");
		if (!pm_state.sw_psy) {
			return -ENODEV;
		}
	}

	val.intval = enable;
	ret = pm_state.sw_psy->set_property(pm_state.sw_psy, 
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	
	return ret;
}

static int usb_pd_pm_enable_sw_hiz(bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pm_state.sw_psy) {
		pm_state.sw_psy = power_supply_get_by_name("bq2589x");
		if (!pm_state.sw_psy) {
			return -ENODEV;
		}
	}

	val.intval = enable;
	ret = pm_state.sw_psy->set_property(pm_state.sw_psy, 
			POWER_SUPPLY_PROP_INPUT_SUSPEND, &val);
	
	return ret;
}

static int usb_pd_pm_check_fc_enabled(void)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pm_state.fc_psy) {
		pm_state.fc_psy = power_supply_get_by_name("bq2597x");
		if (!pm_state.fc_psy) {
			return -ENODEV;
		}
	}

	ret = pm_state.fc_psy->get_property(pm_state.fc_psy, 
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		pm_state.bq2597x.charge_enabled = !!val.intval;
	
	return ret;
}

static int usb_pd_pm_check_sw_enabled(void)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pm_state.sw_psy) {
		pm_state.sw_psy = power_supply_get_by_name("bq2589x");
		if (!pm_state.sw_psy) {
			return -ENODEV;
		}
	}

	ret = pm_state.sw_psy->get_property(pm_state.sw_psy, 
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		pm_state.bq2589x.charge_enabled = !!val.intval;
	
	return ret;
}

static int usb_pd_pm_check_sw_hized(void)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pm_state.sw_psy) {
		pm_state.sw_psy = power_supply_get_by_name("bq2589x");
		if (!pm_state.sw_psy) {
			return -ENODEV;
		}
	}

	ret = pm_state.sw_psy->get_property(pm_state.sw_psy, 
			POWER_SUPPLY_PROP_INPUT_SUSPEND, &val);
	if (!ret)
		pm_state.bq2589x.charge_hized = !!val.intval;
	
	return ret;
}



static void usb_pd_pm_retrieve_src_pdo(unsigned int port)
{
	usb_pd_port_t *dev = usb_pd_pe_get_device(port);
	uint8_t *pdo_data = dev->rx_msg_buf;
	uint32_t pdo;
	int max_volt;
	int max_curr;
	int i;

	dev->rx_src_pdo_num = dev->rx_msg_data_len >> 2;
	//reset max_volt, max_curr to 0 in case of keeping value from previous adapter
	dev->apdo_max_volt = 0;
	dev->apdo_max_curr = 0;

	adapter.pps_supported = false;

	for (i = 0; i < dev->rx_src_pdo_num; i++) {
		pdo = get_data_object(&pdo_data[i << 2]);
		INFO("PDO[%u] = 0x%08x\n", i , pdo);

		dev->rx_src_pdo[i].pdo = pdo;
		
		if (i == 0)
		{
			dev->remote_externally_powered = (pdo & PDO_EXTERNALLY_POWERED_BIT) ? true : false;
			INFO("Remote SRC_CAPS externally powered bit is %s.\n", (dev->remote_externally_powered) ? "set" : "not set");
		}

		if ((enum supply_type_t )PDO_SUPPLY_TYPE(pdo) == SUPPLY_TYPE_AUGMENTED) {
			max_volt = APDO_MAX_VOLTAGE(pdo) * 100;
			max_curr = APDO_MAX_CURRENT(pdo) * 50;
			pr_debug("APDO supported: %u, max_volt:%u, max_curr:%u",
					i, max_volt, max_curr);
			/*
			 * if volt is satisfied, we prefer PDO with higher current capability
			 */
			if (max_volt >= 11000 && max_curr > dev->apdo_max_curr) {
				dev->apdo_max_volt = max_volt;
				dev->apdo_max_curr = max_curr;
				dev->apdo_idx = i;
				adapter.pps_supported = true;
			}
		}
	}
}

void usb_pd_pm_select_default_5v(unsigned int port)
{
	usb_pd_port_t *dev = usb_pd_pe_get_device(port);
		// No acceptable PDO was found. Use first PDO (vSafe5V).
	dev->object_position = 1;
	dev->selected_pdo = dev->rx_src_pdo[0].pdo;;
	dev->selected_snk_pdo_idx = 0;

	CRIT("PDO-%u 0x%08x selected\n", dev->object_position, dev->selected_pdo);
	DEBUG("selected_snk_pdo_idx: %u\n", dev->selected_snk_pdo_idx);
}

void usb_pd_pm_evaluate_src_caps(unsigned int port)
{

	usb_pd_pm_retrieve_src_pdo(port);
	usb_pd_pm_select_default_5v(port);

	return;
}

void usb_pd_pm_evaluate_pps_status(unsigned int port)
{

	u16 temp;
	usb_pd_port_t *dev = usb_pd_pe_get_device(port);

	temp = dev->rx_msg_buf[3] << 8 | dev->rx_msg_buf[2];

	adapter.volt = temp * 20;	/*to mV*/
	adapter.curr = (int)dev->rx_msg_buf[4] * 50; /*to mA*/
	adapter.ptf = (dev->rx_msg_buf[5] & 0x06) >> 1;
	adapter.omf = !!(dev->rx_msg_buf[5] & 0x08);

	CRIT("Adapter volt:%u, current:%u\n", adapter.volt, adapter.curr);
}

/*PD PPS mode */
static void usb_pd_pm_switch_to_ardo(unsigned int port)
{
	usb_pd_port_t *dev = usb_pd_pe_get_device(port);

	pm_state.request_volt = pm_state.bq2597x.vbat_volt * 2 + BUS_VOLT_INIT_UP;
	pm_state.request_current = min(dev->apdo_max_curr, pm_state.ibus_lmt_curr);

	dev->selected_pdo = dev->rx_src_pdo[dev->apdo_idx].pdo;
	dev->object_position = dev->apdo_idx + 1;
}

static uint8_t get_volt_increase_steps(uint16_t vbat)
{
	return 1;
}

static int usb_pd_pm_flash2_charge(unsigned int port)
{
    usb_pd_port_t *dev = usb_pd_pe_get_device(port);

    int steps;
    int sw_ctrl_steps = 0;
    int hw_ctrl_steps = 0;
    int step_vbat = 0;
/*    int step_vbus = 0;*/
    int step_ibus = 0;
    int step_ibat = 0;
    static int ibus_limit;

    if (ibus_limit == 0)
	ibus_limit = pm_state.ibus_lmt_curr * 110 / 100;

    if (pm_state.bq2597x.vbat_volt > sys_config.bat_volt_lp_lmt - 50)
	ibus_limit = pm_state.ibus_lmt_curr * 90 / 100;
    else if (pm_state.bq2597x.vbat_volt < sys_config.bat_volt_lp_lmt - 250)
	ibus_limit = pm_state.ibus_lmt_curr * 110 / 100;
 
    if (pm_state.bq2597x.vbat_volt > sys_config.bat_volt_lp_lmt)
	step_vbat = sys_config.flash2_policy.down_steps;
    else if (pm_state.bq2597x.vbat_volt < sys_config.bat_volt_lp_lmt - 5)
	step_vbat = get_volt_increase_steps(pm_state.bq2597x.vbat_volt);

    if (pm_state.bq2597x.ibat_curr < sys_config.bat_curr_lp_lmt )
        step_ibat = get_volt_increase_steps(pm_state.bq2597x.vbat_volt);
    else if (pm_state.bq2597x.ibat_curr > sys_config.bat_curr_lp_lmt + 100)
        step_ibat = sys_config.flash2_policy.down_steps;

    if (pm_state.bq2597x.ibus_curr < ibus_limit + 50)
	step_ibus = get_volt_increase_steps(pm_state.bq2597x.vbat_volt);
    else if (pm_state.bq2597x.ibus_curr > ibus_limit + 100)
	step_ibus = sys_config.flash2_policy.down_steps;

    sw_ctrl_steps = min(min(step_vbat, step_ibus), step_ibat);

    if (pm_state.bq2597x.bat_ocp_alarm 
		/*|| pm_state.bq2597x.bat_ovp_alarm */
		|| pm_state.bq2597x.bus_ocp_alarm
		|| pm_state.bq2597x.bus_ovp_alarm 
		/*|| pm_state.bq2597x.tbat_temp > 60 
		  || pm_state.bq2597x.tbus_temp > 50*/) {
        hw_ctrl_steps = sys_config.flash2_policy.down_steps;
    } else {
	hw_ctrl_steps = get_volt_increase_steps(pm_state.bq2597x.vbat_volt);
    }

    if (pm_state.bq2597x.bat_therm_fault ) // battery overheat, stop charge
        return -1;
    else if (pm_state.bq2597x.bus_therm_fault 
		|| pm_state.bq2597x.die_therm_fault)
        return -2; // goto switch mode, and never go to flash charge
    else if (pm_state.bq2597x.bat_ocp_fault 
		|| pm_state.bq2597x.bus_ocp_fault 
		||pm_state.bq2597x.bat_ovp_fault 
		|| pm_state.bq2597x.bus_ovp_fault)
        return 2; // go to switch, and try to ramp up if ok

    if (pm_state.bq2597x.vbat_volt > sys_config.bat_volt_lp_lmt - 50 &&
            pm_state.bq2597x.ibat_curr < sys_config.fc2_taper_current)
        return 1; // goto switch, never go to flash charge


    steps = min(sw_ctrl_steps, hw_ctrl_steps);

    pr_debug("step_vbat=%d, step_ibat=%d, step_ibus = %d, hw_ctrl_steps=%d, adjust step= %d\n", 
		step_vbat, step_ibat, step_ibus,hw_ctrl_steps, steps);

    pm_state.request_volt = pm_state.request_volt + steps * 20;

    if (pm_state.request_volt > dev->apdo_max_volt)
	pm_state.request_volt = dev->apdo_max_volt;

    if (pm_state.request_volt > adapter.volt + 500)
	pm_state.request_volt = adapter.volt + 500;

    return 0;
}

const unsigned char *pm_state_str[] = {
	"PD_PM_STATE_ENTRY",
	"PD_PM_STATE_DISCONNECT",
	"PD_PM_STATE_SW_ENTRY",
	"PD_PM_STATE_SW_ENTRY_2",
	"PD_PM_STATE_SW_ENTRY_3",
	"PD_PM_STATE_SW_LOOP",
	"PD_PM_STATE_FLASH2_ENTRY",
	"PD_PM_STATE_FLASH2_ENTRY_1",
	"PD_PM_STATE_FLASH2_ENTRY_2",
	"PD_PM_STATE_FLASH2_ENTRY_3",
	"PD_PM_STATE_FLASH2_GET_PPS_STATUS",
	"PD_PM_STATE_FLASH2_TUNE",
	"PD_PM_STATE_STOP_CHARGE",
};

static void usb_pd_pm_move_state(pm_sm_state_t state)
{
#if 1
    pr_debug("pm_state change:%s -> %s\n", 
		pm_state_str[pm_state.state], pm_state_str[state]);
    pm_state.state_log[pm_state.log_idx] = pm_state.state;
    pm_state.log_idx++;
    pm_state.log_idx %= PM_STATE_LOG_MAX;
#endif
    pm_state.state = state;
}

void usb_pd_pm_statemachine(unsigned int port)
{
    usb_pd_port_t *dev = usb_pd_pe_get_device(port);
    int ret;
    static int tune_vbus_retry;

    if (!pm_state.bq2597x.vbus_pres || !dev->explicit_contract)
        pm_state.state = PD_PM_STATE_DISCONNECT;
    else if (pm_state.state == PD_PM_STATE_DISCONNECT){
        usb_pd_pm_move_state(PD_PM_STATE_ENTRY);
    }

    switch (pm_state.state) {
    case PD_PM_STATE_DISCONNECT:
        if (pm_state.bq2597x.charge_enabled) {
            usb_pd_pm_enable_fc(false);
            usb_pd_pm_check_fc_enabled();
        }

	if (!pm_state.bq2589x.charge_enabled) {
	    usb_pd_pm_enable_sw(true);
	    usb_pd_pm_check_sw_enabled();
	}
	
	if (pm_state.bq2589x.charge_hized) {
	    usb_pd_pm_enable_sw_hiz(false);
	    usb_pd_pm_check_sw_hized();
	}

        pm_state.sw_from_flash2 = false;
	pm_state.sw_fc2_init_fail = false;
        break;

    case PD_PM_STATE_ENTRY:
	pr_debug("PPS supported:%d, vbat_volt:%d\n", adapter.pps_supported, pm_state.bq2597x.vbat_volt);
        if (!adapter.pps_supported 
		|| pm_state.bq2597x.vbat_volt < sys_config.min_vbat_start_flash2
                || pm_state.sw_from_flash2
		|| pm_state.sw_fc2_init_fail) {
	    pr_err("Start switch charge due to: pps_supported = %d, vbat_volt = %d, sw_from_flash2 = %d\n",
		adapter.pps_supported, pm_state.bq2597x.vbat_volt, pm_state.sw_from_flash2);
            usb_pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
	} else if (pm_state.bq2597x.vbat_volt > sys_config.bat_volt_lp_lmt - 100) {
	    pr_err("battery volt-%d is too high, start switch charging directly\n", 
			pm_state.bq2597x.vbat_volt);
            pm_state.sw_near_cv = true;
            usb_pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
        } else {
	    pr_err("battery volt-%d is ok, start flash charging\n", 
			pm_state.bq2597x.vbat_volt);
            usb_pd_pm_move_state(PD_PM_STATE_FLASH2_ENTRY);
        }
        break;

    case PD_PM_STATE_SW_ENTRY:
        if (pm_state.bq2597x.charge_enabled) {
            usb_pd_pm_enable_fc(false);
            usb_pd_pm_check_fc_enabled();
        }

        if (!pm_state.bq2597x.charge_enabled)
            usb_pd_pm_move_state(PD_PM_STATE_SW_ENTRY_2);

        break;

    case PD_PM_STATE_SW_ENTRY_2:
        if (*dev->current_state == PE_SNK_READY){
            usb_pd_pm_select_default_5v(port);
           // Build RDO based on policy manager response.
            usb_pd_policy_manager_request(port, PD_POLICY_MNGR_REQ_SEL_CAPABILITY);
            usb_pd_pm_move_state(PD_PM_STATE_SW_ENTRY_3);
        }
	break;

    case PD_PM_STATE_SW_ENTRY_3:
        if (*dev->current_state == PE_SNK_READY){
		pr_err("enable sw charger and check enable\n");
		usb_pd_pm_enable_sw(true);
		usb_pd_pm_check_sw_enabled();
		usb_pd_pm_enable_sw_hiz(false);
		usb_pd_pm_check_sw_hized();
		if (pm_state.bq2589x.charge_enabled && !pm_state.bq2589x.charge_hized) 
			usb_pd_pm_move_state(PD_PM_STATE_SW_LOOP);
	}
       break;

    case PD_PM_STATE_SW_LOOP:
        if (adapter.pps_supported && !pm_state.sw_from_flash2 && !pm_state.sw_near_cv) {
            if (pm_state.bq2597x.vbat_volt > sys_config.min_vbat_start_flash2) {
		pr_err("battery volt: %d is ok, proceeding to flash charging...\n",
			pm_state.bq2597x.vbat_volt);
                usb_pd_pm_move_state(PD_PM_STATE_FLASH2_ENTRY);
                break;
            }
        }
        break;

    case PD_PM_STATE_FLASH2_ENTRY:
        if (pm_state.bq2589x.charge_enabled) {
            usb_pd_pm_enable_sw(false);
	    usb_pd_pm_check_sw_enabled();
        }
	
        if (!pm_state.bq2589x.charge_hized) {
            usb_pd_pm_enable_sw_hiz(true);
	    usb_pd_pm_check_sw_hized();
        }

        if (!pm_state.bq2589x.charge_enabled && pm_state.bq2589x.charge_hized)
            usb_pd_pm_move_state(PD_PM_STATE_FLASH2_ENTRY_1);

        break;

    case PD_PM_STATE_FLASH2_ENTRY_1:
        if (*dev->current_state == PE_SNK_READY) {
            usb_pd_pm_switch_to_ardo(port);
            usb_pd_policy_manager_request(port, PD_POLICY_MNGR_REQ_SEL_CAPABILITY);
            usb_pd_pm_move_state(PD_PM_STATE_FLASH2_ENTRY_2);
	    tune_vbus_retry = 0;
        }
        break;

    /*tune adapter voltage to acceptable range:2xVBat + 200 +/- 50*/
    case PD_PM_STATE_FLASH2_ENTRY_2:
        if (*dev->current_state == PE_SNK_READY) {
	    msleep(50);
	    usb_pd_pm_update_fc_status();
            if (pm_state.bq2597x.vbus_volt < (pm_state.bq2597x.vbat_volt * 2 + BUS_VOLT_INIT_UP - 50)) {
		tune_vbus_retry++;
		pm_state.request_volt += 20;
		usb_pd_policy_manager_request(port, PD_POLICY_MNGR_REQ_SEL_CAPABILITY);
	    } else if (pm_state.bq2597x.vbus_volt > (pm_state.bq2597x.vbat_volt * 2 + BUS_VOLT_INIT_UP + 50)) {
		tune_vbus_retry++;
		pm_state.request_volt -= 20;
		usb_pd_policy_manager_request(port, PD_POLICY_MNGR_REQ_SEL_CAPABILITY);
	    } else {
		pr_err("inital adapter volt tune ok, retry %d times\n", tune_vbus_retry);
                usb_pd_pm_move_state(PD_PM_STATE_FLASH2_ENTRY_3);
		break;
	    }
	    if (tune_vbus_retry > 20) {
		pr_err("Failed to tune adapter volt into valid range, charge with switching charger\n");
		pm_state.sw_fc2_init_fail = true;
		usb_pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
	    }	
        }
        break;
    case PD_PM_STATE_FLASH2_ENTRY_3:
        if (*dev->current_state == PE_SNK_READY) {
	    pr_err("bq2589x.charge_hized:%d\n", pm_state.bq2589x.charge_hized);
	    /*check switch charge is disabled again, volt tune up in PD_PM_STATE_FLASH2_ENTRY_2
	     * sometimes trigger an adapter plugin event, which make sw charger exit hiz mode*/
	    if (!pm_state.bq2589x.charge_hized) {
		usb_pd_pm_enable_sw_hiz(true);
		usb_pd_pm_check_sw_hized();
	    }
            if (pm_state.bq2589x.charge_hized && !pm_state.bq2597x.charge_enabled) {
                usb_pd_pm_enable_fc(true);
                usb_pd_pm_check_fc_enabled();
                if (pm_state.bq2597x.charge_enabled)
                    usb_pd_pm_move_state(PD_PM_STATE_FLASH2_GET_PPS_STATUS);
            }
        }
        break;
    case PD_PM_STATE_FLASH2_GET_PPS_STATUS:
        if (*dev->current_state == PE_SNK_READY) {
            usb_pd_policy_manager_request(port, PD_POLICY_MNGR_REQ_GET_PPS_STATUS);
            usb_pd_pm_move_state(PD_PM_STATE_FLASH2_TUNE);
        }
        break;
    case PD_PM_STATE_FLASH2_TUNE:
        if (pm_state.bq2597x.vbat_volt < sys_config.min_vbat_start_flash2 - 400){
            usb_pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
            break;
        }

        if (*dev->current_state == PE_SNK_READY) {
            ret = usb_pd_pm_flash2_charge(port);
            if (ret == -1) {
		pr_err("Move to stop charging:%d\n", ret);
                usb_pd_pm_move_state(PD_PM_STATE_STOP_CHARGE);
                break;
            } else if (ret == -2 || ret == 1) {
		pr_err("Move to switch charging:%d\n", ret);
                usb_pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
                pm_state.sw_from_flash2 = true;
                break;
            } else if (ret == 2) {
		pr_err("Move to switch charging, will try to recover to flash charging:%d\n", ret);
                usb_pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
            } else {// normal tune adapter output
                usb_pd_policy_manager_request(port, PD_POLICY_MNGR_REQ_SEL_CAPABILITY);
                usb_pd_pm_move_state(PD_PM_STATE_FLASH2_GET_PPS_STATUS);
            }
        }
        break;

    case PD_PM_STATE_STOP_CHARGE:
        if (pm_state.bq2597x.charge_enabled) {
            usb_pd_pm_enable_fc(false);
            usb_pd_pm_check_fc_enabled();
        }
        if (pm_state.bq2589x.charge_enabled) {
            usb_pd_pm_enable_sw(false);
	    usb_pd_pm_check_sw_enabled();
        }
        break;
    }

}

void build_src_caps(unsigned int port)
{
	unsigned int n;
	usb_pd_port_config_t *config = &pd_port_config[port];
	usb_pd_port_t *dev = usb_pd_pe_get_device(port);
	tcpc_device_t *typec_dev = tcpm_get_device(port);

	// Clear PDOs.
	for (n = 0; n < config->num_src_pdos; n++)
	{
		dev->src_pdo[n] = 0;
	}

	if (typec_dev->role == ROLE_DRP)
	{
		dev->src_pdo[0] |= PDO_DUAL_ROLE_POWER_BIT;
	}

	if (config->usb_suspend_supported)
	{
		dev->src_pdo[0] |= SRC_PDO_USB_SUSPEND_BIT;
	}

	if (dev->externally_powered)
	{
		dev->src_pdo[0] |= PDO_EXTERNALLY_POWERED_BIT;
	}

	if (config->usb_comm_capable)
	{
		dev->src_pdo[0] |= PDO_USB_COMM_CAPABLE_BIT;
	}

	if (config->dual_role_data)
	{
		dev->src_pdo[0] |= PDO_DUAL_ROLE_DATA_BIT;
	}

	if (config->unchunked_msg_support)
	{
		dev->src_pdo[0] |= SRC_PDO_UNCHUNKED_EXT_MSG_SUP_BIT;
	}

	for (n = 0; n < config->num_src_pdos; n++)
	{
		// Note: Source PDOs my need to be hidden or modified based on voltage 
		// and current limits of the cable.     

		// PDO(n)[31:30] = SourceCapabilities.PDO(n).SupplyType.
		dev->src_pdo[n] |= ((uint32_t)(config->src_caps[n].SupplyType) & 0x03) << 30;

		if (config->src_caps[n].SupplyType == SUPPLY_TYPE_FIXED)
		{
			// PDO(n)[21:20] = SourceCapabilities.PDO(n).PeakCurrent.
			dev->src_pdo[n] |= ((uint32_t)(config->src_caps[n].PeakI) & 0x03) << 20;
		}
		else /* Variable or Battery */
		{
			// PDO(n)[29:20] = SourceCapabilities.PDO(n).MaximumVoltage.
			dev->src_pdo[n] |= ((uint32_t)(config->src_caps[n].MaxV) & 0x3FF) << 20;
		}   

		// PDO(n)[19:10] = SourceCapabilities.PDO(n).MinimumVoltage.
		dev->src_pdo[n] |= ((uint32_t)(config->src_caps[n].MinV) & 0x3FF) << 10;

		if (config->src_caps[n].SupplyType == SUPPLY_TYPE_BATTERY)
		{
			// PDO(n)[9:0] = SourceCapabilities.PDO(n).MaximumPower.
			dev->src_pdo[n] |= ((uint32_t)(config->src_caps[n].MaxPower) & 0x3FF);
		}
		else /* Fixed or Variable */
		{
			// PDO(n)[9:0] = SourceCapabilities.PDO(n).MaximumCurrent.
			dev->src_pdo[n] |= ((uint32_t)(config->src_caps[n].MaxI) & 0x3FF);
		}		 
	}

	return;
}


void build_snk_caps(unsigned int port)
{
	unsigned int n;
	usb_pd_port_config_t *config = &pd_port_config[port];
	usb_pd_port_t *dev = usb_pd_pe_get_device(port);
	tcpc_device_t *typec_dev = tcpm_get_device(port);

	// Clear PDOs.
	for (n = 0; n < config->num_snk_pdos; n++)
	{
		dev->snk_pdo[n] = 0;
	}

	if (typec_dev->role == ROLE_DRP)
	{
		dev->snk_pdo[0] |= PDO_DUAL_ROLE_POWER_BIT;
	}

	if (config->higher_capability)
	{
		dev->snk_pdo[0] |= SNK_PDO_HIGHER_CAPABILITY_BIT;
	}

	if (dev->externally_powered)
	{
		dev->snk_pdo[0] |= PDO_EXTERNALLY_POWERED_BIT;
	}

	if (config->usb_comm_capable)
	{
		dev->snk_pdo[0] |= PDO_USB_COMM_CAPABLE_BIT;
	}

	if (config->dual_role_data)
	{
		dev->snk_pdo[0] |= PDO_DUAL_ROLE_DATA_BIT;
	}

	dev->snk_pdo[0] |= ((uint32_t)config->fast_role_swap_support & 0x03) << 23;

	for (n = 0; n < config->num_snk_pdos; n++)
	{
		// SinkPDO(n)(31:30) = pSupplyType(n).
		dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].SupplyType) & 0x03) << 30;

		// SinkPDO(n)(19:10) = pMinimumVoltage(n).
		dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].MinV) & 0x3FF) << 10;

		if (config->snk_caps[n].SupplyType == SUPPLY_TYPE_BATTERY ||
		    config->snk_caps[n].SupplyType == SUPPLY_TYPE_VARIABLE)
		{
			// SinkPDO(n)(29:20) = pMaximumVoltage(n).
			dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].MaxV) & 0x3FF) << 20;
		}
		else if (config->snk_caps[n].SupplyType == SUPPLY_TYPE_AUGMENTED) 
		{
			dev->snk_pdo[n] |= (uint32_t)0x00 << 28; //PPS
		}



		if (config->snk_caps[n].SupplyType == SUPPLY_TYPE_BATTERY)
		{
			// SinkPDO(n)(9:0) = pOperationalPower(n).
			dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].OperationalPower) & 0x3FF);
		}
		else if (config->snk_caps[n].SupplyType == SUPPLY_TYPE_FIXED ||
		    config->snk_caps[n].SupplyType == SUPPLY_TYPE_VARIABLE) /* Fixed or Variable */
		{
			// SinkPDO(n)(9:0) = pOperationalCurrent(n).
			dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].OperationalCurrent) & 0x3FF);
		}
		else /*Augmented Programmable Power Supply*/
		{
			dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].MaxV & 0xFF) << 17);
			dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].MinV & 0xFF) << 8);
			dev->snk_pdo[n] |= ((uint32_t)(config->snk_caps[n].MaxOperatingCurrent & 0x7F) << 0);
		}
	}

	return;
}

typedef enum
{
	RDO_GIVEBACK_FLAG         = ((uint32_t)1 << 27),
	RDO_CAPABILITY_MISMATCH   = ((uint32_t)1 << 26),
	RDO_USB_COMM_CAPABLE      = ((uint32_t)1 << 25),
	RDO_NO_USB_SUSPEND        = ((uint32_t)1 << 24),
	RDO_UNCHUNKED_MSG_SUPPORT = ((uint32_t)1 << 23)
} rdo_bits_t;

void build_rdo(unsigned int port)
{
	uint32_t rdo = 0;
	usb_pd_port_t *dev = usb_pd_pe_get_device(port);
	usb_pd_port_config_t *config = usb_pd_pm_get_config(port);

	rdo |= ((uint32_t)dev->object_position) << 28;

	if (config->giveback_flag)
	{
		rdo |= RDO_GIVEBACK_FLAG;
	}

	if ((dev->object_position == 1) && config->higher_capability)
	{
		// Sink requires voltage greater than 5V.
		rdo |= RDO_CAPABILITY_MISMATCH;
	}

	if (config->usb_comm_capable)
	{
		rdo |= RDO_USB_COMM_CAPABLE;
	}

	if (config->no_usb_suspend)
	{
		rdo |= RDO_NO_USB_SUSPEND;
	}

	if (config->unchunked_msg_support)
	{
		rdo |= RDO_UNCHUNKED_MSG_SUPPORT;
	}

	if (PDO_SUPPLY_TYPE(dev->selected_pdo) == SUPPLY_TYPE_BATTERY)
	{
		if (PDO_MAX_CURRENT_OR_POWER(dev->selected_pdo) < config->snk_caps[dev->selected_snk_pdo_idx].MaxOperatingPower)
		{
			// Sink requires higher power for full operation.
			rdo |= RDO_CAPABILITY_MISMATCH;

			rdo |= PDO_MAX_CURRENT_OR_POWER(dev->selected_pdo) << 10;
		}
		else
		{
			rdo |= ((uint32_t)config->snk_caps[dev->selected_snk_pdo_idx].OperationalPower & 0x3FF) << 10;
		}

		if (config->giveback_flag)
		{
			rdo |= config->snk_caps[dev->selected_snk_pdo_idx].MinOperatingPower & 0x3FF;
		}
		else
		{
			rdo |= config->snk_caps[dev->selected_snk_pdo_idx].MaxOperatingPower & 0x3FF;
		}
	}
	else if (PDO_SUPPLY_TYPE(dev->selected_pdo) == SUPPLY_TYPE_FIXED ||
		PDO_SUPPLY_TYPE(dev->selected_pdo) == SUPPLY_TYPE_VARIABLE) /* Fixed or Variable supply */
	{
		if (PDO_MAX_CURRENT_OR_POWER(dev->selected_pdo) < config->snk_caps[dev->selected_snk_pdo_idx].MaxOperatingCurrent)
		{
			// Sink requires higher current for full operation.
			rdo |= RDO_CAPABILITY_MISMATCH;

			rdo |= PDO_MAX_CURRENT_OR_POWER(dev->selected_pdo) << 10;
		}
		else
		{
			rdo |= ((uint32_t)config->snk_caps[dev->selected_snk_pdo_idx].OperationalCurrent & 0x3FF) << 10;
		}

		if (config->giveback_flag)
		{
			rdo |= config->snk_caps[dev->selected_snk_pdo_idx].MinOperatingCurrent & 0x3FF;
		}
		else
		{
			rdo |= config->snk_caps[dev->selected_snk_pdo_idx].MaxOperatingCurrent & 0x3FF;
		}
	}
	else
	{
		rdo |= (((uint32_t)pm_state.request_volt / 20) & 0x7FF) << 9;
		rdo |= (pm_state.request_current / 50) & 0x7F;
	}

	dev->rdo = rdo;

	return;
}

static void usb_pd_pm_workfunc(struct work_struct *work)
{

	usb_pd_pm_update_sw_status();
	usb_pd_pm_update_fc_status();

	usb_pd_pm_statemachine(0);

	schedule_delayed_work(&pm_state.pm_work, msecs_to_jiffies(50));
}

void usb_pd_print_version(void)
{
	CRIT(" ________  _________  ____ ___  ___\n"); 
	CRIT("/_  __/ / / / __/ _ )/ / /|_  ||_  |\n");
	CRIT(" / / / /_/ /\\ \\/ _  /_  _/ __// __/\n"); 
	CRIT("/_/  \\____/___/____/ /_//____/____/\n"); 
	CRIT("\n");
	PRINT("PD Stack v%u.%02u\n", PD_LIB_VERSION_MAJOR, PD_LIB_VERSION_MINOR);
	CRIT("=====================================\n");
	return;
}


void usb_pd_init(const usb_pd_port_config_t *port_config)
{
	unsigned int port;

	usb_pd_prl_init();

	memcpy(pd_port_config, port_config, sizeof(pd_port_config));

	for (port = 0; port < NUM_TCPC_DEVICES; port++)
	{
		usb_pd_pe_init(port, &pd_port_config[port]);
	}

	pm_state.fc_psy = power_supply_get_by_name("bq2597x");
	pm_state.sw_psy = power_supply_get_by_name("bq2589x");

	pm_state.ibus_lmt_curr = sys_config.bus_curr_lp_lmt;

	INIT_DELAYED_WORK(&pm_state.pm_work, usb_pd_pm_workfunc);

	schedule_delayed_work(&pm_state.pm_work, msecs_to_jiffies(100));

	return;
}
