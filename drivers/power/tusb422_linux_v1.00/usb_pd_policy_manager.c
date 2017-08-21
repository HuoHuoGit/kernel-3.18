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


#define BATT_MAX_VOLT			4200
#define BATT_MAX_CURRENT		4000
#define	BUS_OVP_THRESHOLD		10000
#define	BUS_OVP_ALARM_THRESHOLD		9500


static const struct sys_config sys_config = {
	.bq2597x.bat_ovp_th	= BATT_MAX_VOLT + 100,
	.bq2597x.bat_ocp_th	= BATT_MAX_CURRENT + 2000,
	.bq2597x.bus_ovp_th	= BUS_OVP_THRESHOLD,
	.bq2597x.bus_ocp_th	= (BATT_MAX_CURRENT >> 1 ) + 750,

	.bq2597x.bat_ovp_alarm_th	= BATT_MAX_VOLT,
	.bq2597x.bat_ocp_alarm_th	= BATT_MAX_CURRENT + 1000,
	.bq2597x.bus_ovp_alarm_th	= BUS_OVP_ALARM_THRESHOLD,
	.bq2597x.bus_ocp_alarm_th	= (BATT_MAX_CURRENT >> 1 ) + 550,

	.bq2597x.bat_ucp_alarm_th	= 2000,

	.max4_policy.down_steps = -1,
	.max4_policy.volt_hysteresis = 50,

	.min_vbat_start_maxchg4 = 3500,
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

	ret = pm_state.sw_psy->get_property(pm_state.sw_psy, POWER_SUPPLY_PROP_CHARGE_ENABLED, &val);
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
	//		pr_err("fc_psy not found\n");
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
		pm_state.sw_psy = power_supply_get_by_name("bq2597x");
		if (!pm_state.sw_psy) {
	//		pr_err("sw_psy not found\n");
			return -ENODEV;
		}
	}

	val.intval = enable;
	ret = pm_state.sw_psy->set_property(pm_state.sw_psy, 
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	
	return ret;
}

static int usb_pd_pm_check_fc_enabled(void)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pm_state.fc_psy) {
		pm_state.fc_psy = power_supply_get_by_name("bq2597x");
		if (!pm_state.fc_psy) {
			pr_err("fc_psy not found\n");
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
			pr_err("sw_psy not found\n");
			return -ENODEV;
		}
	}

	ret = pm_state.sw_psy->get_property(pm_state.sw_psy, 
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		pm_state.bq2589x.charge_enabled = !!val.intval;
	
	return ret;
}


//------------------------------------------------------------------------------------------
// Returns true if the new_offer is better than the current_offer, else returns false.
//------------------------------------------------------------------------------------------
#if 0
static bool better_offer(pdo_offer_t *new_offer, pdo_offer_t *current_offer, pdo_priority_t pdo_priority)
{
	// Compare offers based on user defined priority (voltage, current, or power).
	if (pdo_priority == PRIORITY_VOLTAGE)
	{
		if (new_offer->min_voltage > current_offer->min_voltage)
		{
			return true;
		}
		else if (new_offer->min_voltage == current_offer->min_voltage)
		{
			if (new_offer->max_voltage > current_offer->max_voltage)
			{
				return true;
			}
		}
	}
	else if (pdo_priority == PRIORITY_CURRENT)
	{
		if (new_offer->max_current > current_offer->max_current)
		{
			return true;
		}
		else if (new_offer->max_current == current_offer->max_current)
		{
			if (new_offer->min_voltage > current_offer->min_voltage)
			{
				return true;
			}
		}
	}
	else /* PRIORITY_POWER */
	{
		if (new_offer->max_power > current_offer->max_power)
		{
			return true;
		}
		else if (new_offer->max_power == current_offer->max_power)
		{
			if (new_offer->min_voltage > current_offer->min_voltage)
			{
				return true;
			}
		}
	}

	// Both offers are equal in terms of user defined priority
	// so pick based on supply type:  Fixed > Variable > Battery.
	if (new_offer->supply_type != current_offer->supply_type)
	{
		if (new_offer->supply_type == SUPPLY_TYPE_FIXED)
		{
			return true;
		}
		else if (current_offer->supply_type == SUPPLY_TYPE_FIXED)
		{
			return false;
		}
		else if (new_offer->supply_type == SUPPLY_TYPE_VARIABLE)
		{
			return true;
		}
	}

	return false;
}
#endif
#if 1
static void usb_pd_pm_retrieve_src_pdo(unsigned int port)
{
	usb_pd_port_t *dev = usb_pd_pe_get_device(port);
	uint8_t *pdo_data = dev->rx_msg_buf;
	uint32_t pdo;
	int i;

	dev->rx_src_pdo_num = dev->rx_msg_data_len >> 2;

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
			dev->apdo_idx = i;
			adapter.pps_supported = true;
		}
	}

}
#endif

#if 1
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
#endif

void usb_pd_pm_evaluate_src_caps(unsigned int port)
{
#if 1
	usb_pd_pm_retrieve_src_pdo(port);

	 usb_pd_pm_select_default_5v(port);
#endif
#if 0
	usb_pd_port_t *dev = usb_pd_pe_get_device(port);
	uint8_t num_offered_pdos;
	uint8_t src_pdo_idx;
	uint8_t *pdo_data = dev->rx_msg_buf;
	uint32_t pdo;
	pdo_offer_t offer[2];
	uint8_t     offer_idx = 0;
	pdo_offer_t *selected_offer;
	pdo_offer_t *new_offer;

	// Divide the Rx'd msg length by 4 to get the number of source PDOs offered.
	num_offered_pdos = dev->rx_msg_data_len >> 2;

	dev->rx_src_pdo_num = num_offered_pdos;

	// Initialize RDO object position to zero (invalid value).
	dev->object_position = 0;

	// Initialize offer pointers.
	new_offer = &offer[offer_idx];
	selected_offer = NULL;

	// Evaluate each PDO offered in source caps.
	for (src_pdo_idx = 0; src_pdo_idx < num_offered_pdos; src_pdo_idx++)
	{
		// Using get_data_object() instead of casting to 32-bit pointer 
		// in case pdo_data pointer is not 4-byte aligned.
		pdo = get_data_object(&pdo_data[src_pdo_idx << 2]);
		dev->rx_src_pdo[src_pdo_idx].pdo = pdo;

		INFO("PDO[%u] = 0x%08x\n", src_pdo_idx, pdo);

		if (src_pdo_idx == 0)
		{
			dev->remote_externally_powered = (pdo & PDO_EXTERNALLY_POWERED_BIT) ? true : false;
			INFO("Remote SRC_CAPS externally powered bit is %s.\n", (dev->remote_externally_powered) ? "set" : "not set");
		}

		// Extract the offer params from the source PDO.
		new_offer->supply_type = (enum supply_type_t)PDO_SUPPLY_TYPE(pdo);

		if (new_offer->supply_type == SUPPLY_TYPE_AUGMENTED){
		    dev->apdo_idx = src_pdo_idx;
		    adapter.pps_supported = true;
		}

		CRIT("PDO-%u %s, ", src_pdo_idx + 1, 
			 (new_offer->supply_type == SUPPLY_TYPE_FIXED) ? "Fixed" : 
			 (new_offer->supply_type == SUPPLY_TYPE_VARIABLE) ? "Vari" :
			 (new_offer->supply_type == SUPPLY_TYPE_BATTERY) ? "Batt" : "PPS");

		CRIT("%u - ", PDO_VOLT_TO_MV(new_offer->min_voltage));
		CRIT("%u mV, ", PDO_VOLT_TO_MV(new_offer->max_voltage));
		CRIT("%u mA, ", PDO_CURR_TO_MA(new_offer->max_current));
		CRIT("%u mW\n", PDO_PWR_TO_MW(new_offer->max_power));
	} /* End: Source PDO loop */

	if (selected_offer == NULL)
	{
		// No acceptable PDO was found. Use first PDO (vSafe5V).
		dev->object_position = 1;
		dev->selected_pdo = get_data_object(&pdo_data[0]);
		dev->selected_snk_pdo_idx = 0;
	}
	CRIT("PDO-%u 0x%08x selected\n", dev->object_position, dev->selected_pdo);

#endif
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

	pm_state.request_volt = pm_state.bq2597x.vbat_volt * 215 / 100;
	pm_state.request_current = 3000;

	dev->selected_pdo = dev->rx_src_pdo[dev->apdo_idx].pdo;
	dev->object_position = dev->apdo_idx + 1;
}

#if 0
/*Normal PD mode */
static void usb_pd_pm_switch_to_rdo(unsigned int port)
{
    usb_pd_port_config_t *config = usb_pd_pm_get_config(port);
    usb_pd_port_t *dev = usb_pd_pe_get_device(port);
    uint8_t snk_pdo_idx;
    uint8_t src_pdo_idx;
    uint32_t pdo;
    bool acceptable_pdo;
    pdo_offer_t offer[2];
    uint8_t     offer_idx = 0;
    pdo_offer_t *selected_offer;
    pdo_offer_t *new_offer;

    // Initialize RDO object position to zero (invalid value).
    dev->object_position = 0;

    // Initialize offer pointers.
    new_offer = &offer[offer_idx];
    selected_offer = NULL;

    // Evaluate each PDO offered in source caps.
    for (src_pdo_idx = 0; src_pdo_idx < dev->rx_src_pdo_num; src_pdo_idx++)
    {
        pdo = dev->rx_src_pdo[src_pdo_idx].pdo;

        // Extract the offer params from the source PDO.
        new_offer->supply_type = (enum supply_type_t)PDO_SUPPLY_TYPE(pdo);

        if (new_offer->supply_type == SUPPLY_TYPE_AUGMENTED)
            continue;

        new_offer->min_voltage = PDO_MIN_VOLTAGE(pdo);

        if (new_offer->supply_type == SUPPLY_TYPE_FIXED)
        {
            new_offer->max_voltage = new_offer->min_voltage;
        }
        else
        {
            new_offer->max_voltage = PDO_MAX_VOLTAGE(pdo);
        }

        if (new_offer->supply_type == SUPPLY_TYPE_BATTERY)
        {
            new_offer->max_power = PDO_MAX_CURRENT_OR_POWER(pdo);
            new_offer->max_current = calc_current(new_offer->max_power, new_offer->max_voltage);
        }
        else
        {
            new_offer->max_current = PDO_MAX_CURRENT_OR_POWER(pdo);
            new_offer->max_power = calc_power(new_offer->max_voltage, new_offer->max_current);
        }

        CRIT("PDO-%u %s, ", src_pdo_idx + 1,
             (new_offer->supply_type == SUPPLY_TYPE_FIXED) ? "Fixed" :
             (new_offer->supply_type == SUPPLY_TYPE_VARIABLE) ? "Vari" : "Batt");
        CRIT("%u - ", PDO_VOLT_TO_MV(new_offer->min_voltage));
        CRIT("%u mV, ", PDO_VOLT_TO_MV(new_offer->max_voltage));
        CRIT("%u mA, ", PDO_CURR_TO_MA(new_offer->max_current));
        CRIT("%u mW\n", PDO_PWR_TO_MW(new_offer->max_power));

        for (snk_pdo_idx = 0; snk_pdo_idx < config->num_snk_pdos; snk_pdo_idx++)
        {
            // Make sure the SrcCap and SinkCap supply type match:
            // A Fixed Supply SinkCap PDO can only consider a Fixed Supply SrcCap PDO
            // A Variable Supply SinkCap PDO can consider a Fixed or Variable Supply SrcCap PDO
            // A Battery Supply SinkCap PDO can consider any Supply SrcCap PDO
            if (config->snk_caps[snk_pdo_idx].SupplyType == SUPPLY_TYPE_AUGMENTED)
                continue;

            if ((new_offer->supply_type != config->snk_caps[snk_pdo_idx].SupplyType) &&
                (config->snk_caps[snk_pdo_idx].SupplyType != SUPPLY_TYPE_BATTERY) &&
                ((new_offer->supply_type == SUPPLY_TYPE_BATTERY) &&
                 (config->snk_caps[snk_pdo_idx].SupplyType == SUPPLY_TYPE_VARIABLE)))
            {
                continue;
            }

            if (config->snk_caps[snk_pdo_idx].SupplyType == SUPPLY_TYPE_FIXED)
            {
                config->snk_caps[snk_pdo_idx].MaxV = config->snk_caps[snk_pdo_idx].MinV;
            }

            acceptable_pdo = false;

            // Determine whether PDO is acceptable.
            if ((new_offer->max_voltage <= config->snk_caps[snk_pdo_idx].MaxV) &&
                (new_offer->min_voltage >= config->snk_caps[snk_pdo_idx].MinV))
            {
                if (config->snk_caps[snk_pdo_idx].SupplyType == SUPPLY_TYPE_BATTERY)
                {
                    if (new_offer->max_power >= config->snk_caps[snk_pdo_idx].OperationalPower)
                    {
                        acceptable_pdo = true;

                        // Check for capability mismatch.
                        new_offer->cap_mismatch = (new_offer->max_power < config->snk_caps[snk_pdo_idx].MaxOperatingPower) ? true : false;
                    }
                }
                else /* Fixed or Variable Sink Cap */
                {
                    if (new_offer->max_current >= config->snk_caps[snk_pdo_idx].OperationalCurrent)
                    {
                        acceptable_pdo = true;

                        // Check for capability mismatch.
                        new_offer->cap_mismatch = (new_offer->max_current < config->snk_caps[snk_pdo_idx].MaxOperatingCurrent) ? true : false;
                    }
                }
            }

            // Select this PDO if
            // Firstly, it is acceptable
            // [AND] Secondly,
            // no PDO has been selected yet [OR]
            // the capability is a better match  [OR]
            // it is a better offer based on user-defined priority.
            if (acceptable_pdo &&
                ((selected_offer == NULL) ||
                 (selected_offer->cap_mismatch && !new_offer->cap_mismatch) ||
                  better_offer(new_offer, selected_offer, config->pdo_priority)))
            {
                selected_offer = new_offer;
                dev->selected_pdo = pdo;
                dev->object_position = src_pdo_idx + 1;
                dev->selected_snk_pdo_idx = snk_pdo_idx;

                // Switch pointer to current offer.
                offer_idx ^= 1;
                new_offer = &offer[offer_idx];
            }
        } /* End: Sink PDO loop */
    } /* End: Source PDO loop */

    if (selected_offer == NULL)
    {
        // No acceptable PDO was found. Use first PDO (vSafe5V).
        dev->object_position = 1;
        dev->selected_pdo = dev->rx_src_pdo[0].pdo;
        dev->selected_snk_pdo_idx = 0;
    }

    CRIT("PDO-%u 0x%08x selected\n", dev->object_position, dev->selected_pdo);
    DEBUG("selected_snk_pdo_idx: %u\n", dev->selected_snk_pdo_idx);

    return;

}
#endif

static uint8_t get_volt_increase_steps(uint16_t vbat)
{
	return 1;
}

static int usb_pd_pm_maxchg4_charge(unsigned int port)
{
    int steps = 0;

    if (pm_state.bq2597x.ibat_curr < sys_config.bq2597x.bat_ocp_alarm_th - 900)
        steps = get_volt_increase_steps(pm_state.bq2597x.vbat_volt);
    else if (pm_state.bq2597x.ibat_curr > sys_config.bq2597x.bat_ocp_alarm_th - 800)
        steps = sys_config.max4_policy.down_steps;

    if (pm_state.bq2597x.vbat_volt > sys_config.bq2597x.bat_ovp_alarm_th)
        steps = sys_config.max4_policy.down_steps;

    if (pm_state.bq2597x.vbus_volt > sys_config.bq2597x.bus_ovp_alarm_th - 20)
        steps = sys_config.max4_policy.down_steps;

    if (pm_state.bq2597x.bat_ocp_alarm /*|| pm_state.bq2597x.bat_ovp_alarm */|| pm_state.bq2597x.bus_ocp_alarm
            || pm_state.bq2597x.bus_ovp_alarm /*|| pm_state.bq2597x.tbat_temp > 60 || pm_state.bq2597x.tbus_temp > 50*/) {
        steps = sys_config.max4_policy.down_steps;
    }

    if (pm_state.bq2597x.bat_therm_fault ) // battery overheat, stop charge
        return -1;
    else if (pm_state.bq2597x.bus_therm_fault || pm_state.bq2597x.die_therm_fault)
        return -2; // goto switch mode, and never go to flash charge
    else if (pm_state.bq2597x.bat_ocp_fault || pm_state.bq2597x.bus_ocp_fault ||pm_state.bq2597x.bat_ovp_fault || pm_state.bq2597x.bus_ovp_fault) {
//        steps = sys_config.max4_policy.down_steps * 2;
        return 2; // go to switch, and try to ramp up if ok
    }

    if (pm_state.bq2597x.vbat_volt > sys_config.bq2597x.bat_ovp_alarm_th - 50 &&
            pm_state.bq2597x.ibat_curr < sys_config.bq2597x.bat_ucp_alarm_th)
        return 1; // goto switch, never go to flash charge

    pm_state.request_volt = adapter.volt + steps * 20;

    if (pm_state.request_volt < pm_state.bq2597x.vbat_volt * 2)
        pm_state.request_volt = pm_state.bq2597x.vbat_volt * 2 + 40; //cable loss

    return 0;
}

const unsigned char *pm_state_str[] = {
	"PD_PM_STATE_ENTRY",
	"PD_PM_STATE_DISCONNECT",
	"PD_PM_STATE_SW_ENTRY",
	"PD_PM_STATE_SW_ENTRY_2",
	"PD_PM_STATE_SW_LOOP",
	"PD_PM_STATE_MAXCHG4_ENTRY",
	"PD_PM_STATE_MAXCHG4_ENTRY_1",
	"PD_PM_STATE_MAXCHG4_ENTRY_2",
	"PD_PM_STATE_MAXCHG4_ENTRY_3",
	"PD_PM_STATE_MAXCHG4_GET_PPS_STATUS",
	"PD_PM_STATE_MAXCHG4_TUNE",
	"PD_PM_STATE_STOP_CHARGE",
};

static void usb_pd_pm_move_state(pm_sm_state_t state)
{
    pr_err("pm_state change:%s -> %s\n", 
		pm_state_str[pm_state.state], pm_state_str[state]);

    pm_state.state_log[pm_state.log_idx] = pm_state.state;
    pm_state.log_idx++;
    pm_state.log_idx %= PM_STATE_LOG_MAX;
    pm_state.state = state;
}

void usb_pd_pm_statemachine(unsigned int port)
{
    usb_pd_port_t *dev = usb_pd_pe_get_device(port);
    //tcpc_device_t *typec_dev = tcpm_get_device(port);
    int ret;


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
        if (pm_state.bq2589x.charge_enabled) {
            usb_pd_pm_enable_sw(false);
	    usb_pd_pm_check_sw_enabled();
        }
        pm_state.sw_from_maxchg4 = false;
        break;

    case PD_PM_STATE_ENTRY:
	pr_err("PPS supported:%d, vbat_volt:%d\n", adapter.pps_supported, pm_state.bq2597x.vbat_volt);
        if (!adapter.pps_supported 
		|| pm_state.bq2597x.vbat_volt < sys_config.min_vbat_start_maxchg4
                || pm_state.sw_from_maxchg4)
            usb_pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
        else if (pm_state.bq2597x.vbat_volt > sys_config.bq2597x.bat_ovp_alarm_th - 100) {
            pm_state.sw_near_cv = true;
            usb_pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
        } else {
            usb_pd_pm_move_state(PD_PM_STATE_MAXCHG4_ENTRY);
        }
        break;

    case PD_PM_STATE_SW_ENTRY:
        if (pm_state.bq2597x.charge_enabled) {
            usb_pd_pm_enable_fc(false);
            usb_pd_pm_check_fc_enabled();
        }
        if (!pm_state.bq2597x.charge_enabled) {
            usb_pd_pm_enable_sw(true);
	    usb_pd_pm_check_sw_enabled();
        }
        if (!pm_state.bq2597x.charge_enabled && pm_state.bq2589x.charge_enabled)
            usb_pd_pm_move_state(PD_PM_STATE_SW_ENTRY_2);
        break;

    case PD_PM_STATE_SW_ENTRY_2:
        if (*dev->current_state == PE_SNK_READY){
            //usb_pd_pm_switch_to_rdo(port);
           // usb_pd_pm_evaluate_src_caps(port);
            usb_pd_pm_select_default_5v(port);
           // Build RDO based on policy manager response.
            usb_pd_policy_manager_request(port, PD_POLICY_MNGR_REQ_SEL_CAPABILITY);
//            pm_run_pe_sm = true;
            usb_pd_pm_move_state(PD_PM_STATE_SW_LOOP);
        }
        break;
    case PD_PM_STATE_SW_LOOP:
        if (adapter.pps_supported && !pm_state.sw_from_maxchg4 && !pm_state.sw_near_cv) {
            if (pm_state.bq2597x.vbat_volt > sys_config.min_vbat_start_maxchg4) {
                usb_pd_pm_move_state(PD_PM_STATE_MAXCHG4_ENTRY);
                break;
            }
        }
        break;

    case PD_PM_STATE_MAXCHG4_ENTRY:
        if (pm_state.bq2589x.charge_enabled) {
            usb_pd_pm_enable_sw(false);
	    usb_pd_pm_check_sw_enabled();
        }
        if (!pm_state.bq2589x.charge_enabled)
            usb_pd_pm_move_state(PD_PM_STATE_MAXCHG4_ENTRY_1);

        break;

    case PD_PM_STATE_MAXCHG4_ENTRY_1:
        if (*dev->current_state == PE_SNK_READY) {
            usb_pd_pm_switch_to_ardo(port);
            usb_pd_policy_manager_request(port, PD_POLICY_MNGR_REQ_SEL_CAPABILITY);
  //          pm_run_pe_sm = true;
            usb_pd_pm_move_state(PD_PM_STATE_MAXCHG4_ENTRY_2);
        }
        break;

    case PD_PM_STATE_MAXCHG4_ENTRY_2:
        if (*dev->current_state == PE_SNK_READY) {
            if (pm_state.bq2597x.vbus_volt > (pm_state.bq2597x.vbat_volt * 205 / 100)
		&& pm_state.bq2597x.vbus_volt < (pm_state.bq2597x.vbat_volt * 230 / 100))
                usb_pd_pm_move_state(PD_PM_STATE_MAXCHG4_ENTRY_3);
            else
                usb_pd_pm_move_state(PD_PM_STATE_MAXCHG4_ENTRY_1);
        }
        break;
    case PD_PM_STATE_MAXCHG4_ENTRY_3:
        if (*dev->current_state == PE_SNK_READY) {
            if (!pm_state.bq2589x.charge_enabled && !pm_state.bq2597x.charge_enabled) {
                usb_pd_pm_enable_fc(true);
                usb_pd_pm_check_fc_enabled();
                if (pm_state.bq2597x.charge_enabled)
                    usb_pd_pm_move_state(PD_PM_STATE_MAXCHG4_GET_PPS_STATUS);
            }
        }
        break;
    case PD_PM_STATE_MAXCHG4_GET_PPS_STATUS:
        if (*dev->current_state == PE_SNK_READY) {
            usb_pd_policy_manager_request(port, PD_POLICY_MNGR_REQ_GET_PPS_STATUS);
 //           pm_run_pe_sm = true;
            usb_pd_pm_move_state(PD_PM_STATE_MAXCHG4_TUNE);
        }
        break;
    case PD_PM_STATE_MAXCHG4_TUNE:
        if (pm_state.bq2597x.vbat_volt < sys_config.min_vbat_start_maxchg4 - 400){
            usb_pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
            break;
        }

        if (*dev->current_state == PE_SNK_READY) {
            ret = usb_pd_pm_maxchg4_charge(port);
	    pr_err("usb_pd_pm_maxchg4_charge ret:%d\n", ret);
            if (ret == -1) {
                usb_pd_pm_move_state(PD_PM_STATE_STOP_CHARGE);
                break;
            } else if (ret == -2 || ret == 1) {
                usb_pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
                pm_state.sw_from_maxchg4 = true;
                break;
            } else if (ret == 2) {
                usb_pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
            } else {// normal tune adapter output
                usb_pd_policy_manager_request(port, PD_POLICY_MNGR_REQ_SEL_CAPABILITY);
                usb_pd_pm_move_state(PD_PM_STATE_MAXCHG4_GET_PPS_STATUS);
//                pm_run_pe_sm = true;
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


	INIT_DELAYED_WORK(&pm_state.pm_work, usb_pd_pm_workfunc);

	schedule_delayed_work(&pm_state.pm_work, msecs_to_jiffies(100));

	return;
}
