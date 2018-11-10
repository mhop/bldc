/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils.h"
#include "comm_can.h"
#include "hw.h"
#include <math.h>
#include "commands.h"

// Settings
#define MAX_CAN_AGE						0.1
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				8

#define mean_filter_float(val, cnt)      \
do {                                     \
	static float filter_buffer[(cnt)];   \
	static int filter_ptr = 0;           \
                                         \
	filter_buffer[filter_ptr++] = (val); \
	if (filter_ptr>=(cnt)) {             \
		filter_ptr = 0;                  \
	}                                    \
	val = 0.0;                           \
	for (int i=0; i<(cnt); i++) {        \
		val += filter_buffer[i];         \
	}                                    \
	val /= (cnt);                        \
} while(0)

// Threads
static THD_FUNCTION(adc_thread, arg);
static THD_WORKING_AREA(adc_thread_wa, 1024);
// Private variables
static volatile adc_config config;
static volatile float ms_without_power = 0.0;
static volatile float decoded_level = 0.0;
static volatile float read_voltage = 0.0;
static volatile float decoded_level2 = 0.0;
static volatile float read_voltage2 = 0.0;
static volatile bool use_rx_tx_as_buttons = false;
static volatile bool stop_now = true;
static volatile bool is_running = false;

// PAS -----------------------
#define PAS_TIMER_FREQ   10000
#define PAS_MAGNET_COUNT 8
#define RpmToPasCounter(f) (PAS_TIMER_FREQ*60/((f)*PAS_MAGNET_COUNT))
#define PAS_TIMEOUT_MS 400
#define MsToPasCounter(ms) (PAS_TIMER_FREQ/1000*(ms))
#define motor_pole_pairs 22
#define tyre_circum_mm   220
#define kmh_to_erpm(v) (((v)*1000/60)/(tyre_circum_mm/100)*motor_pole_pairs)
#define GEAR_CNT 5
static const bool with_pas=true;
struct s_cpas {
	unsigned int wait_max;
	unsigned int backcnt_min;
	float erpm_min_move;
	float erpm_max_no_pedal;
	float pwr_brake_max;
	float pwr_pedal_min;
	float pwr_pedal_max;
	uint32_t cnt_period_min;
	uint32_t cnt_period_max;
	uint32_t cnt_period_max_pedal;
};
struct s_pas {
	icucnt_t t_on;
	uint32_t cnt_period;
	uint32_t cnt_off;
	bool updated;
};

static const struct s_cpas cpas = {
	.wait_max             = MS2ST(PAS_TIMEOUT_MS),
	.backcnt_min          = 3,
	.erpm_min_move        = kmh_to_erpm(2/10),
	.erpm_max_no_pedal    = kmh_to_erpm(6),
	.pwr_brake_max        = -1.00,
	.pwr_pedal_min        = +0.05,
	.pwr_pedal_max        = +0.50,
	.cnt_period_min       = RpmToPasCounter(95),
	.cnt_period_max       = RpmToPasCounter(5),
	.cnt_period_max_pedal = MsToPasCounter(PAS_TIMEOUT_MS),
};
static struct s_pas pas;

static void icuwidthcb(ICUDriver *icup);
static ICUConfig icucfg = {
		ICU_INPUT_ACTIVE_HIGH,
		PAS_TIMER_FREQ,
		icuwidthcb,
		NULL, //icuperiodcb,
		NULL,
		HW_ICU_CHANNEL,
		0
		};
static void icuwidthcb(ICUDriver *icup) {
	pas.cnt_off    = icuGetWidthX(icup);
	pas.cnt_period = icuGetPeriodX(icup);
	if(pas.cnt_period < cpas.cnt_period_max_pedal) {
		pas.t_on       = chVTGetSystemTimeX();
		pas.updated    = true;
	}
}
// end PAS -----------------------

void app_adc_configure(adc_config *conf) {
	config = *conf;
	ms_without_power = 0.0;
}

void app_adc_start(bool use_rx_tx) {
	use_rx_tx_as_buttons = use_rx_tx;
	stop_now = false;
	chThdCreateStatic(adc_thread_wa, sizeof(adc_thread_wa), NORMALPRIO, adc_thread, NULL);
}

void app_adc_stop(void) {
	if (with_pas && is_running) {
		icuStop(&HW_ICU_DEV);
		palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT);
	}
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

float app_adc_get_decoded_level(void) {
	return decoded_level;
}

float app_adc_get_voltage(void) {
	return read_voltage;
}

float app_adc_get_decoded_level2(void) {
	return decoded_level2;
}

float app_adc_get_voltage2(void) {
	return read_voltage2;
}

// ---------- PAS ---------------------------

/* Checks PAS-Sensor for pedaling, direction and cadence. returns power
 *  if pedaling forward: 
 *   power linear to cadence (torque-simulation).
 *   power can be boosted by throttle
 *  if not pedaling or pedaling backward:
 *   power by throttle up to cpas.erpm_max_no_pedal (6km/h)
 *   if faster than cpas.erpm_max_no_pedal: brake by throttle (if throttle was back to zero after pedaling)
 */
static float pas_check(const float p, const float erpm)
{
	static float pwr_pas=0.0;
	static unsigned int backcnt=0;
	float pwr=p, ret=p;
	static enum {thr_no, thr_power, thr_brake, thr_help} thr_state=thr_no;
	enum { ped_keep, ped_no, ped_forward, ped_backward } pedaling;
	const bool print=true;
	const bool cad2pwr=false;

	systime_t t = chVTGetSystemTimeX();
	pedaling=ped_keep;
	if (t-pas.t_on < cpas.wait_max) { // pedaling
		if (pas.updated) {
			if (pas.cnt_off < pas.cnt_period/2) { // backward
				if (++backcnt>=cpas.backcnt_min) pedaling=ped_backward;
				else                             pedaling=ped_no;
			} else { // forward
				backcnt=0;
				pedaling=ped_forward;
			} // forward
			pas.updated=false;
		}
	} else {
		backcnt=0;
		pedaling=ped_no;
	}

	if(pwr==0.0) thr_state=thr_no;
	switch (pedaling) {
		case ped_keep:
		break;
		case ped_backward:
		case ped_no: {
			pwr_pas=0;
			if (erpm > cpas.erpm_max_no_pedal) {
				if (thr_state==thr_no) thr_state=thr_brake; // if not in help-mode and throttle was back to zero after pedaling
				else pwr=0.0;
			} else {
				if(thr_state==thr_no) thr_state=thr_help;
			}
			if(thr_state==thr_brake) {
				pwr=-pwr; // use throttle for braking
			}
		}
		break;
		case ped_forward: {
			if (cad2pwr) {
				pwr_pas=utils_map_bound(pas.cnt_period,
								cpas.cnt_period_max, cpas.cnt_period_min,
								cpas.pwr_pedal_min,  cpas.pwr_pedal_max);
			} else {
				pwr_pas = (cpas.pwr_pedal_min+cpas.pwr_pedal_max)/2;
			}
			if(pwr!=0.0) thr_state=thr_power;
		}
		break;
	} // switch (pedaling)
	ret = utils_max_abs(pwr, pwr_pas); // returns pwr_pas if abs equal
	if (erpm<cpas.erpm_min_move) {
		ret=0.0; // not moving
	}
	
	if (print) { 
		static int n=0;
		if(++n>1000 || pedaling!=ped_keep) {
			commands_printf("upd:%d pwrpas:%d, pwr:%d erpm:%d thrs:%d ped:%d",
							pas.updated, (int)pwr_pas*100, (int)pwr*100, (int)erpm, thr_state, pedaling);
			n=0;
		}
	}
	return ret;
}

// --------------- end PAS ----------------	

static THD_FUNCTION(adc_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_ADC");

	// Set servo pin as an input with pullup
	if (with_pas) {
		icuStart(&HW_ICU_DEV, &icucfg);
		palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_ALTERNATE(HW_ICU_GPIO_AF));
		icuStartCapture(&HW_ICU_DEV);
		icuEnableNotifications(&HW_ICU_DEV);
	} else {
		if (use_rx_tx_as_buttons) {
			palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLUP);
			palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);
		} else {
			palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT);
		}
	}
	is_running = true;

	for(;;) {
		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (stop_now) {
			is_running = false;
			return;
		}

		// For safe start when fault codes occur
		if (mc_interface_get_fault() != FAULT_CODE_NONE) {
			ms_without_power = 0;
		}

		// Read the external ADC pin and convert the value to a voltage.
		float pwr = (float)ADC_Value[ADC_IND_EXT];
		pwr /= 4095;
		pwr *= V_REG;

		read_voltage = pwr;

		// Optionally apply a mean value filter
		if (config.use_filter) {
			mean_filter_float(pwr, FILTER_SAMPLES);
		}

		// Map the read voltage
		switch (config.ctrl_type) {
		case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		case ADC_CTRL_TYPE_DUTY_REV_CENTER:
		case ADC_CTRL_TYPE_PID_REV_CENTER:
			// Mapping with respect to center voltage
			if (pwr < config.voltage_center) {
				pwr = utils_map(pwr, config.voltage_start,
						config.voltage_center, 0.0, 0.5);
			} else {
				pwr = utils_map(pwr, config.voltage_center,
						config.voltage_end, 0.5, 1.0);
			}
			break;

		default:
			// Linear mapping between the start and end voltage
			pwr = utils_map(pwr, config.voltage_start, config.voltage_end, 0.0, 1.0);
			break;
		}

		// Truncate the read voltage
		utils_truncate_number(&pwr, 0.0, 1.0);

		// Optionally invert the read voltage
		if (config.voltage_inverted) {
			pwr = 1.0 - pwr;
		}

		decoded_level = pwr;

		// Read the external ADC pin and convert the value to a voltage.
#ifdef ADC_IND_EXT2
		float brake = (float)ADC_Value[ADC_IND_EXT2];
		brake /= 4095;
		brake *= V_REG;
#else
		float brake = 0.0;
#endif

		read_voltage2 = brake;

		// Optionally apply a mean value filter
		if (config.use_filter) {
			mean_filter_float(brake, FILTER_SAMPLES);
		}

		// Map and truncate the read voltage
		brake = utils_map(brake, config.voltage2_start, config.voltage2_end, 0.0, 1.0);
		utils_truncate_number(&brake, 0.0, 1.0);

		// Optionally invert the read voltage
		if (config.voltage2_inverted) {
			brake = 1.0 - brake;
		}

		decoded_level2 = brake;

		// Read the button pins
		bool cc_button = false;
		bool rev_button = false;
		if (!with_pas) {
			if (use_rx_tx_as_buttons) {
				cc_button = !palReadPad(HW_UART_TX_PORT, HW_UART_TX_PIN);
				if (config.cc_button_inverted) {
					cc_button = !cc_button;
				}
				rev_button = !palReadPad(HW_UART_RX_PORT, HW_UART_RX_PIN);
				if (config.rev_button_inverted) {
					rev_button = !rev_button;
				}
			} else {
				// When only one button input is available, use it differently depending on the control mode
				if (config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON ||
				    config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER ||
				    config.ctrl_type == ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON ||
					config.ctrl_type == ADC_CTRL_TYPE_DUTY_REV_BUTTON ||
					config.ctrl_type == ADC_CTRL_TYPE_PID_REV_BUTTON) {
					rev_button = !palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
					if (config.rev_button_inverted) {
						rev_button = !rev_button;
					}
				} else {
					cc_button = !palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
					if (config.cc_button_inverted) {
						cc_button = !cc_button;
					}
				}
			}
		}

		// All pins and buttons are still decoded for debugging, even
		// when output is disabled.
		if (app_is_output_disabled()) {
			continue;
		}

		switch (config.ctrl_type) {
		case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		case ADC_CTRL_TYPE_DUTY_REV_CENTER:
		case ADC_CTRL_TYPE_PID_REV_CENTER:
			// Scale the voltage and set 0 at the center
			pwr *= 2.0;
			pwr -= 1.0;
			break;

		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
			pwr -= brake;
			break;

		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON:
		case ADC_CTRL_TYPE_DUTY_REV_BUTTON:
		case ADC_CTRL_TYPE_PID_REV_BUTTON:
			// Invert the voltage if the button is pressed
			if (rev_button) {
				pwr = -pwr;
			}
			break;

		default:
			break;
		}

		// Filter RPM to avoid glitches
		const float rpm_now = mc_interface_get_rpm();
		float rpm_filtered=rpm_now;
		mean_filter_float(rpm_filtered, RPM_FILTER_SAMPLES);

		// PAS-Sensor
		if (with_pas) pwr=pas_check(pwr, rpm_filtered);

		// Apply deadband
		utils_deadband(&pwr, config.hyst, 1.0);

		// Apply throttle curve
		pwr = utils_throttle_curve(pwr, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

		// Apply ramping
		static systime_t last_time = 0;
		static float pwr_ramp = 0.0;
		float ramp_time = fabsf(pwr) > fabsf(pwr_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

		if (fabsf(pwr) > 0.001) {
			ramp_time = fminf(config.ramp_time_pos, config.ramp_time_neg);
		}

		if (ramp_time > 0.01) {
			const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);
			utils_step_towards(&pwr_ramp, pwr, ramp_step);
			last_time = chVTGetSystemTimeX();
			pwr = pwr_ramp;
		}

		float current_rel = 0.0;
		bool current_mode = false;
		bool current_mode_brake = false;
		const volatile mc_configuration *mcconf = mc_interface_get_configuration();
		bool send_duty = false;

		// Use the filtered and mapped voltage for control according to the configuration.
		switch (config.ctrl_type) {
		case ADC_CTRL_TYPE_CURRENT:
		case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
			current_mode = true;
			if ((pwr >= 0.0 && rpm_now > 0.0) || (pwr < 0.0 && rpm_now < 0.0)) {
				current_rel = pwr;
			} else {
				current_rel = pwr;
			}

			if (fabsf(pwr) < 0.001) {
				ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
			}
			break;

        case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
			current_mode = true;
			if (pwr >= 0.0) {
				current_rel = pwr;
			} else {
				current_rel = fabsf(pwr);
				current_mode_brake = true;
			}

			if (pwr < 0.001) {
				ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
			}

			if ((config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC ||
			    config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER) && rev_button) {
				current_rel = -current_rel;
			}
			break;

		case ADC_CTRL_TYPE_DUTY:
		case ADC_CTRL_TYPE_DUTY_REV_CENTER:
		case ADC_CTRL_TYPE_DUTY_REV_BUTTON:
			if (fabsf(pwr) < 0.001) {
				ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
			}

			if (!(ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start)) {
				mc_interface_set_duty(utils_map(pwr, -1.0, 1.0, -mcconf->l_max_duty, mcconf->l_max_duty));
				send_duty = true;
			}
			break;

		case ADC_CTRL_TYPE_PID:
		case ADC_CTRL_TYPE_PID_REV_CENTER:
		case ADC_CTRL_TYPE_PID_REV_BUTTON:
			if ((pwr >= 0.0 && rpm_now > 0.0) || (pwr < 0.0 && rpm_now < 0.0)) {
				current_rel = pwr;
			} else {
				current_rel = pwr;
			}

			if (!(ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start)) {
				float speed = 0.0;
				if (pwr >= 0.0) {
					speed = pwr * mcconf->l_max_erpm;
				} else {
					speed = pwr * fabsf(mcconf->l_min_erpm);
				}

				mc_interface_set_pid_speed(speed);
				send_duty = true;
			}

			if (fabsf(pwr) < 0.001) {
				ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
			}
			break;

		default:
			continue;
		}

		// If safe start is enabled and the output has not been zero for long enough
		if (ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start) {
			static int pulses_without_power_before = 0;
			if (ms_without_power == pulses_without_power_before) {
				ms_without_power = 0;
			}
			pulses_without_power_before = ms_without_power;
			mc_interface_set_brake_current(timeout_get_brake_current());

			if (config.multi_esc) {
				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						comm_can_set_current_brake(msg->id, timeout_get_brake_current());
					}
				}
			}

			continue;
		}

		// Reset timeout
		timeout_reset();

		// If c is pressed and no throttle is used, maintain the current speed with PID control
		static bool was_pid = false;

		if (current_mode && cc_button && fabsf(pwr) < 0.001) {
			static float pid_rpm = 0.0;

			if (!was_pid) {
				was_pid = true;
				pid_rpm = rpm_filtered;
			}

			mc_interface_set_pid_speed(pid_rpm);

			// Send the same duty cycle to the other controllers
			if (config.multi_esc) {
				float current = mc_interface_get_tot_current_directional_filtered();

				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						comm_can_set_current(msg->id, current);
					}
				}
			}

			continue;
		}

		was_pid = false;

		// Find lowest RPM (for traction control)
		float rpm_local = mc_interface_get_rpm();
		float rpm_lowest = rpm_local;
		if (config.multi_esc) {
			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					float rpm_tmp = msg->rpm;

					if (fabsf(rpm_tmp) < fabsf(rpm_lowest)) {
						rpm_lowest = rpm_tmp;
					}
				}
			}
		}

		// Optionally send the duty cycles to the other ESCs seen on the CAN-bus
		if (send_duty && config.multi_esc) {
			float duty = mc_interface_get_duty_cycle_now();

			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					comm_can_set_duty(msg->id, duty);
				}
			}
		}

		if (current_mode) {
			if (current_mode_brake) {
				mc_interface_set_brake_current_rel(current_rel);

				// Send brake command to all ESCs seen recently on the CAN bus
				if (config.multi_esc) {
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg *msg = comm_can_get_status_msg_index(i);

						if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
							comm_can_set_current_brake_rel(msg->id, current_rel);
						}
					}
				}
			} else {
				float current_out = current_rel;
				bool is_reverse = false;
				if (current_out < 0.0) {
					is_reverse = true;
					current_out = -current_out;
					current_rel = -current_rel;
					rpm_local = -rpm_local;
					rpm_lowest = -rpm_lowest;
				}

				// Traction control
				if (config.multi_esc) {
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg *msg = comm_can_get_status_msg_index(i);

						if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
							if (config.tc) {
								float rpm_tmp = msg->rpm;
								if (is_reverse) {
									rpm_tmp = -rpm_tmp;
								}

								float diff = rpm_tmp - rpm_lowest;
								current_out = utils_map(diff, 0.0, config.tc_max_diff, current_rel, 0.0);
							}

							if (is_reverse) {
								comm_can_set_current_rel(msg->id, -current_out);
							} else {
								comm_can_set_current_rel(msg->id, current_out);
							}
						}
					}

					if (config.tc) {
						float diff = rpm_local - rpm_lowest;
						current_out = utils_map(diff, 0.0, config.tc_max_diff, current_rel, 0.0);
					}
				}

				if (is_reverse) {
					mc_interface_set_current_rel(-current_out);
				} else {
					mc_interface_set_current_rel(current_out);
				}
			}
		}
	}
}
