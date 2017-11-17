/*
 * usb_pd_policy_manager.h
 *
 *  Created on: Mar 27, 2017
 *      Author: a0220433
 */

#ifndef SRC_PDLIB_USB_PD_POLICY_MANAGER_H_
#define SRC_PDLIB_USB_PD_POLICY_MANAGER_H_
#include <linux/delay.h>
#include <linux/workqueue.h>



typedef struct {
    uint16_t    volt;
    uint16_t    curr;
    uint8_t     ptf;    // present temperature flag
    bool        omf;    // operation mode flag, set if current foldback
    bool        pps_supported;
}adapter_t;

typedef enum  {
    PD_PM_STATE_ENTRY,
    PD_PM_STATE_DISCONNECT,
    PD_PM_STATE_SW_ENTRY,
    PD_PM_STATE_SW_ENTRY_2,
    PD_PM_STATE_SW_ENTRY_3,
    PD_PM_STATE_SW_LOOP,
    PD_PM_STATE_FLASH2_ENTRY,
    PD_PM_STATE_FLASH2_ENTRY_1,
    PD_PM_STATE_FLASH2_ENTRY_2,
    PD_PM_STATE_FLASH2_ENTRY_3,
    PD_PM_STATE_FLASH2_GET_PPS_STATUS,
    PD_PM_STATE_FLASH2_TUNE,
    PD_PM_STATE_STOP_CHARGE,

}pm_sm_state_t;

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

#define VBAT_REG_STATUS_SHIFT			0
#define IBAT_REG_STATUS_SHIFT			1

#define VBAT_REG_STATUS_MASK		(1 << VBAT_REG_STATUS_SHIFT)
#define IBAT_REG_STATUS_MASK		(1 << VBAT_REG_STATUS_SHIFT)


struct flash2_policy {
	
	int down_steps;
	uint16_t volt_hysteresis;
};

#if 0
struct bq2597x_cfg {
	uint16_t bat_ovp_th;
	uint16_t bat_ocp_th;
	uint16_t bus_ovp_th;
	uint16_t bus_ocp_th;


	uint16_t bat_ucp_alarm_th;

	uint16_t bat_ovp_alarm_th;
	uint16_t bat_ocp_alarm_th;
	uint16_t bus_ovp_alarm_th;
	uint16_t bus_ocp_alarm_th;

	uint16_t ac_ovp_th;
	uint16_t ts_bus_fault_th;
	uint16_t ts_bat_fault_th;
	uint16_t ts_die_fault_th;

	/* software protection threshold*/
	int32_t sw_bat_ovp_th;
	int32_t sw_bat_ocp_th;
	int32_t sw_bus_ovp_th;
	int32_t sw_bus_ocp_th;
	int32_t sw_bat_ucp_th;
	int32_t taper_current;
};
#endif

struct bq2589x {
	bool charge_enabled;
	bool charge_hized;

};

struct bq2597x {
	bool charge_enabled;

	bool batt_pres;
	bool vbus_pres;

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

	bool vbat_reg;
	bool ibat_reg;

	int  vout_volt;
	int  vbat_volt;
	int  vbus_volt;
	int  ibat_curr;
	int  ibus_curr;
	
	int  bat_temp; 
	int  bus_temp;
	int  die_temp;
};


#define PM_STATE_LOG_MAX    32
typedef struct {
    bool        sw_is_charging;
    bool        flash2_is_charging;
    bool        sw_from_flash2;
    bool        sw_near_cv;
    bool	sw_fc2_init_fail;

    /* for RDO build */
    uint16_t    request_volt;
    uint16_t    request_current;

    uint16_t	ibus_lmt_curr;
    
    pm_sm_state_t     state; //state machine

    pm_sm_state_t state_log[PM_STATE_LOG_MAX];
    uint8_t     log_idx;

    struct bq2597x bq2597x;
    struct bq2589x bq2589x;

    struct power_supply *fc_psy;
    struct power_supply *sw_psy;

    struct delayed_work pm_work;

}pm_t;

struct sys_config {
	uint16_t bat_volt_lp_lmt; /*bat volt loop limit*/
	uint16_t bat_curr_lp_lmt;
	uint16_t bus_volt_lp_lmt;
	uint16_t bus_curr_lp_lmt;
	int32_t fc2_taper_current;

	struct flash2_policy flash2_policy;

	uint16_t min_vbat_start_flash2;
};

void usb_pd_pm_statemachine(unsigned int port);

#endif /* SRC_PDLIB_USB_PD_POLICY_MANAGER_H_ */
