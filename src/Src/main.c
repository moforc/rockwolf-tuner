/* ============================================================================
 * ROCKWOLF HARD RULE (v46, learned the expensive way on 2026-09-15):
 * NEVER erase or program flash while the ESC is armed or the motor is
 * commutating. On STM32F051 the commutation ISR executes from flash; a page
 * erase/program stalls the CPU for ~20-40 ms, commutation freezes mid-step,
 * and a brushless bridge can put two FETs on one leg into conduction at once
 * (shoot-through) — that destroyed an ESC in testing (v45).
 * Samples accumulate in RAM only. Any run-log flush must be issued from the
 * MAIN LOOP, after throttle has been zero AND commutation has been stopped for
 * >= 200 ms, and it must call allOff() before the first erase. Buffer-full
 * stops sampling; it never flushes.
 * ==========================================================================*/

/* AM32- multi-purpose brushless controller firmware for the stm32f051 */

//===========================================================================
//=============================== Changelog =================================
//===========================================================================
/*
 * 1.54 Changelog;
 * --Added firmware name to targets and firmware version to main
 * --added two more dshot to beacons 1-3 currently working
 * --added KV option to firmware, low rpm power protection is based on KV
 * --start power now controls minimum idle power as well as startup strength.
 * --change default timing to 22.5
 * --Lowered default minimum idle setting to 1.5 percent duty cycle, slider
range from 1-2.
 * --Added dshot commands to save settings and reset ESC.
 *
 *1.56 Changelog.
 * -- added check to stall protection to wait until after 40 zero crosses to fix
high startup throttle hiccup.
 * -- added TIMER 1 update interrupt and PWM changes are done once per pwm
period
 * -- reduce commutation interval averaging length
 * -- reduce false positive filter level to 2 and eliminate threshold where
filter is stopped.
 * -- disable interrupt before sounds
 * -- disable TIM1 interrupt during stepper sinusoidal mode
 * -- add 28us delay for dshot300
 * -- report 0 rpm until the first 10 successful steps.
 * -- move serial ADC telemetry calculations and desync check to 10Khz
interrupt.
 *
 * 1.57
 * -- remove spurious commutations and rpm data at startup by polling for longer
interval on startup
 *
 * 1.58
 * -- move signal timeout to 10khz routine and set armed timeout to one quarter
second 2500 / 10000
 * 1.59
 * -- moved comp order definitions to target.h
 * -- fixed update version number if older than new version
 * -- cleanup, moved all input and output to IO.c
 * -- moved comparator functions to comparator.c
 * -- removed ALOT of useless variables
 * -- added siskin target
 * -- moved pwm changes to 10khz routine
 * -- moved basic functions to functions.c
 * -- moved peripherals setup to periherals.c
 * -- added crawler mode settings
 *
 * 1.60
 * -- added sine mode hysteresis
 * -- increased power in stall protection and lowered start rpm for crawlers
 * -- removed onehot125 from crawler mode
 * -- reduced maximum startup power from 400 to 350
 * -- change minimum duty cycle to DEAD_TIME
 * -- version and name moved to permanent spot in FLASH memory, thanks mikeller
 *
 * 1.61
 * -- moved duty cycle calculation to 10khz and added max change option.
 * -- decreased maximum interval change to 25%
 * -- reduce wait time on fast acceleration (fast_accel)
 * -- added check in interrupt for early zero cross
 *
 * 1.62
 * --moved control to 10khz loop
 * --changed condition for low rpm filter for duty cycle from || to &&
 * --introduced max deceleration and set it to 20ms to go from 100 to 0
 * --added configurable servo throttle ranges
 *
 *
 *1.63
 *-- increase time for zero cross error detection below 250us commutation
interval
 *-- increase max change a low rpm x10
 *-- set low limit of throttle ramp to a lower point and increase upper range
 *-- change desync event from full restart to just lower throttle.

 *1.64
 * --added startup check for continuous high signal, reboot to enter bootloader.
 *-- added brake on stop from eeprom
 *-- added stall protection from eeprom
 *-- added motor pole divider for sinusoidal and low rpm power protection
 *-- fixed dshot commands, added confirmation beeps and removed blocking
behavior
 *--
 *1.65
 *-- Added 32 millisecond telemetry output
 *-- added low voltage cutoff , divider value and cutoff voltage needs to be
added to eeprom
 *-- added beep to indicate cell count if low voltage active
 *-- added current reading on pa3 , conversion factor needs to be added to
eeprom
 *-- fixed servo input capture to only read positive pulse to handle higher
refresh rates.
 *-- disabled oneshot 125.
 *-- extended servo range to match full output range of receivers
 *-- added RC CAR style reverse, proportional brake on first reverse , double
tap to change direction
 *-- added brushed motor control mode
 *-- added settings to EEPROM version 1
 *-- add gimbal control option.
 *--
 *1.66
 *-- move idwg init to after input tune
 *-- remove reset after save command -- dshot
 *-- added wraith32 target
 *-- added average pulse check for signal detection
 *--
 *1.67
 *-- Rework file structure for multiple MCU support
 *-- Add g071 mcu
 *--
 *1.68
 *--increased allowed average pulse length to avoid double startup
 *1.69
 *--removed line re-enabling comparator after disabling.
 *1.70 fix dshot for Kiss FC
 *1.71 fix dshot for Ardupilot / Px4 FC
 *1.72 Fix telemetry output and add 1 second arming.
 *1.73 Fix false arming if no signal. Remove low rpm throttle protection below
300kv *1.74 Add Sine Mode range and drake brake strength adjustment *1.75
Disable brake on stop for PWM_ENABLE_BRIDGE Removed automatic brake on stop on
neutral for RC car proportional brake. Adjust sine speed and stall protection
speed to more closely match makefile fixes from Cruwaller Removed gd32 build,
until firmware is functional *1.76 Adjust g071 PWM frequency, and startup power
to be same frequency as f051. Reduce number of polling back emf checks for g071
 *1.77 increase PWM frequency range to 8-48khz
 *1.78 Fix bluejay tunes frequency and speed.
           Fix g071 Dead time
           Increment eeprom version
 *1.79 Add stick throttle calibration routine
           Add variable for telemetry interval
 *1.80 -Enable Comparator blanking for g071 on timer 1 channel 4
           -add hardware group F for Iflight Blitz
           -adjust parameters for pwm frequency
           -add sine mode power variable and eeprom setting
           -fix telemetry rpm during sine mode
           -fix sounds for extended pwm range
           -Add adjustable braking strength when driving
 *1.81 -Add current limiting PID loop
           -fix current sense scale
           -Increase brake power on maximum reverse ( car mode only)
           -Add HK and Blpwr targets
           -Change low kv motor throttle limit
           -add reverse speed threshold changeover based on motor kv
           -doubled filter length for motors under 900kv
*1.82  -Add speed control pid loop.
*1.83  -Add stall protection pid loop.
           -Improve sine mode transition.
           -decrease speed step re-entering sine mode
           -added fixed duty cycle and speed mode build option
           -added rpm_controlled by input signal ( to be added to config tool )
*1.84  -Change PID value to int for faster calculations
           -Enable two channel brushed motor control for dual motors
           -Add current limit max duty cycle
*1.85  -fix current limit not allowing full rpm on g071 or low pwm frequency
                -remove unused brake on stop conditional
*1.86  - create do-once in sine mode instead of setting pwm mode each time.
*1.87  - fix fixed mode max rpm limits
*1.88  - Fix stutter on sine mode re-entry due to position reset
*1.89  - Fix drive by rpm mode scaling.
           - Fix dshot px4 timings
*1.90  - Disable comp interrupts for brushed mode
           - Re-enter polling mode after prop strike or desync
           - add G071 "N" variant
           - add preliminary Extended Dshot
*1.91  - Reset average interval time on desync only after 100 zero crosses
*1.92  - Move g071 comparator blanking to TIM1 OC5
           - Increase ADC read frequency and current sense filtering
           - Add addressable LED strip for G071 targets
*1.93  - Optimization for build process
       - Add firmware file name to each target hex file
       -fix extended telemetry not activating dshot600
       -fix low voltage cuttoff timeout
*1.94  - Add selectable input types
*1.95  - reduce timeout to 0.5 seconds when armed
*1.96  - Improved erpm accuracy dshot and serial telemetry, thanks Dj-Uran
             - Fix PID loop integral.
                 - add overcurrent low voltage cuttoff to brushed mode.
*1.97    - enable input pullup
*1.98    - Dshot erpm rounding compensation.
*1.99    - Add max duty cycle change to individual targets ( will later become
an settings option)
                 - Fix dshot telemetry delay f4 and e230 mcu
*2.00    - Cleanup of target structure
*2.01    - Increase 10khztimer to 20khz, increase max duty cycle change.
*2.02	 - Increase startup power for inverted output targets.
*2.03    - Move chime from dshot direction change commands to save command.
*2.04    - Fix current protection, max duty cycle not increasing
                 - Fix double startup chime
                 - Change current averaging method for more precision
                 - Fix startup ramp speed adjustment
*2.05		 - Fix ramp tied to input frequency
*2.06    - fix input pullups
         - Remove half xfer insterrupt from servo routine
                                 - update running brake and brake on stop
*2.07    - Dead time change f4a
*2.08		 - Move zero crosss timing
*2.09    - filter out short zero crosses
*2.10    - Polling only below commutation intverval of 1500-2000us
				 - fix tune frequency again
*2.11    - RC-Car mode fix
*2.12    - Reduce Advance on hard braking
*2.13    - Remove Input capture filter for dshot2400
         - Change dshot 300 speed detection threshold 
*2.14    - Reduce G071 zero cross checks
         - Assign all mcu's duty cycle resolution 2000 steps
*2.15    - Enforce 1/2 commutation interval as minimum for g071
         - Revert timing change on braking
				 - Add per target over-ride option to max duty cycle change.
				 - todo fix signal detection
*2.16    - add L431 
				 - add variable auto timing
				 - add droneCAN
*/
#include "main.h"
#include "ADC.h"
#include "IO.h"
#include "common.h"
#include "comparator.h"
#include "dshot.h"
#include "eeprom.h"
#include "functions.h"
#include "peripherals.h"
#include "phaseouts.h"
#include "serial_telemetry.h"
#include "kiss_telemetry.h"
#include "signal.h"
#include "sounds.h"
#include "targets.h"
#include <stdint.h>
#include <string.h>
#include <assert.h>

#ifndef NXP
#ifdef USE_LED_STRIP
#include "WS2812.h"
#endif
#endif

#ifdef USE_CRSF_INPUT
#include "crsf.h"
#endif

#if DRONECAN_SUPPORT
#include "DroneCAN/DroneCAN.h"
#endif

#include <version.h>

void zcfoundroutine(void);

// firmware build options !! fixed speed and duty cycle modes are not to be used
// with sinusoidal startup !!

//#define FIXED_DUTY_MODE  // bypasses signal input and arming, uses a set duty
// cycle. For pumps, slot cars etc 
//#define FIXED_DUTY_MODE_POWER 100     //
// 0-100 percent not used in fixed speed mode

// #define FIXED_SPEED_MODE  // bypasses input signal and runs at a fixed rpm
// using the speed control loop PID 
//#define FIXED_SPEED_MODE_RPM  1000  //
// intended final rpm , ensure pole pair numbers are entered correctly in config
// tool.

// #define BRUSHED_MODE         // overrides all brushless config settings,
// enables two channels for brushed control 
//#define GIMBAL_MODE     // also
// sinusoidal_startup needs to be on, maps input to sinusoidal angle.

//===========================================================================
//=============================  Defaults =============================
//===========================================================================

uint8_t drive_by_rpm = 0;
uint32_t MAXIMUM_RPM_SPEED_CONTROL = 10000;
uint32_t MINIMUM_RPM_SPEED_CONTROL = 1000;

// assign speed control PID values values are x10000
fastPID speedPid = { // commutation speed loop time
    .Kp = 10,
    .Ki = 0,
    .Kd = 100,
    .integral_limit = 10000,
    .output_limit = 50000
};

fastPID currentPid = { // 1khz loop time
    .Kp = 400,
    .Ki = 0,
    .Kd = 1000,
    .integral_limit = 20000,
    .output_limit = 100000
};

fastPID stallPid = { // 1khz loop time
    .Kp = 1,
    .Ki = 0,
    .Kd = 50,
    .integral_limit = 10000,
    .output_limit = 50000
};

EEprom_t eepromBuffer;
volatile uint32_t polling_mode_changeover;
volatile uint8_t ramp_divider;
volatile uint8_t max_ramp_startup = RAMP_SPEED_STARTUP;
volatile uint8_t max_ramp_low_rpm = RAMP_SPEED_LOW_RPM;
volatile uint8_t max_ramp_high_rpm = RAMP_SPEED_HIGH_RPM;
char send_esc_info_flag;
uint32_t eeprom_address = EEPROM_START_ADD; 
uint16_t prop_brake_duty_cycle = 0;
uint16_t ledcounter = 0;
uint16_t ramp_count;
uint32_t process_time = 0;
uint32_t start_process = 0;
uint16_t one_khz_loop_counter = 0;
uint16_t target_e_com_time_high;
uint16_t target_e_com_time_low;
volatile uint8_t compute_dshot_flag = 0;
uint8_t crsf_input_channel = 1;
uint8_t crsf_output_PWM_channel = 2;
uint8_t telemetry_interval_ms = 30;
uint8_t temp_advance;
uint16_t motor_kv = 2000;
uint8_t dead_time_override = DEAD_TIME;
uint16_t stall_protect_target_interval = TARGET_STALL_PROTECTION_INTERVAL;
uint16_t enter_sine_angle = 180;
char do_once_sinemode = 0;
uint8_t auto_advance_level;
volatile uint8_t zero_throttle_brake_active;
volatile uint8_t temp_comp_pwm;
uint8_t brake_countdown;

//============================= Servo Settings ==============================
uint16_t servo_low_threshold = 1100; // anything below this point considered 0
uint16_t servo_high_threshold = 1900; // anything above this point considered 2000 (max)
uint16_t servo_neutral = 1500;
uint8_t servo_dead_band = 100;

//========================= Battery Cuttoff Settings ========================
char LOW_VOLTAGE_CUTOFF = 0; // Turn Low Voltage CUTOFF on or off
uint16_t rw_vref_base = 0;       // v60: VREFINT count while at rest (low current)
uint16_t rw_vref_peak = 0;       // v60: VREFINT count at the run's peak current
uint16_t rw_lvc_latch_v = 0;     // v59: LVC's own low-current pack sample (0.01 V units)
uint16_t rw_lvc_neutral_ms = 0;  // v59: ms of continuous neutral before that sample is taken
uint16_t low_cell_volt_cutoff = 330; // 3.3volts per cell

//=========================== END EEPROM Defaults ===========================

#ifndef RW_FW_TAG
#define RW_FW_TAG ""
#endif
const char filename[30] __attribute__((section(".file_name"))) = FILE_NAME RW_FW_TAG;
_Static_assert(sizeof(FIRMWARE_NAME) <=13,"Firmware name too long");   // max 12 character firmware name plus NULL 

// move these to targets folder or peripherals for each mcu
uint16_t ADC_CCR = 30;
uint16_t current_angle = 90;
uint16_t desired_angle = 90;
char return_to_center = 0;
uint16_t target_e_com_time = 0;
int16_t Speed_pid_output;
char use_speed_control_loop = 0;
int32_t input_override = 0;
int16_t use_current_limit_adjust = 2000;
char use_current_limit = 0;
int32_t trap_limit_ca = 0; // ROCKWOLF v27: trap-mode current ceiling, 10mA units
int32_t stall_protection_adjust = 0;
uint32_t MCU_Id = 0;
uint32_t REV_Id = 0;

uint16_t armed_timeout_count;
uint16_t reverse_speed_threshold = 1500;
#if DRONECAN_SUPPORT
uint32_t desync_happened = 0;
#else
uint8_t desync_happened = 0;
#endif
char maximum_throttle_change_ramp = 1;

char crawler_mode = 0; // no longer used //
uint16_t velocity_count = 0;
uint16_t velocity_count_threshold = 75;

char low_rpm_throttle_limit = 1;

uint16_t low_voltage_count = 0;
uint16_t telem_ms_count;

uint16_t VOLTAGE_DIVIDER = TARGET_VOLTAGE_DIVIDER; // 100k upper and 10k lower resistor in divider
uint16_t
    battery_voltage; // scale in volts * 10.  1260 is a battery voltage of 12.60
char cell_count = 0;
char brushed_direction_set = 0;
// ROCKWOLF v5: runtime brushed/brushless mode flag (EEPROM reserved byte 16 == 1 -> brushed)
uint8_t rw_brushed = 0;

uint16_t tenkhzcounter = 0;
int32_t consumed_current = 0;
int32_t smoothed_raw_current = 0;
int16_t actual_current = 0;

char lowkv = 0;

uint16_t min_startup_duty = 120;
uint16_t sin_mode_min_s_d = 120;
char bemf_timeout = 10;

char startup_boost = 50;
char reversing_dead_band = 1;

uint16_t low_pin_count = 0;

uint8_t max_duty_cycle_change = 2;
char fast_accel = 1;
char fast_deccel = 0;
uint16_t last_duty_cycle = 0;
uint16_t duty_cycle_setpoint = 0;
char play_tone_flag = 0;

typedef enum { GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET } GPIO_PinState;

uint16_t startup_max_duty_cycle = 200;
uint16_t minimum_duty_cycle = DEAD_TIME;
uint16_t stall_protect_minimum_duty = DEAD_TIME;
char desync_check = 0;
char low_kv_filter_level = 20;

uint16_t tim1_arr = TIM1_AUTORELOAD; // current auto reset value
uint16_t TIMER1_MAX_ARR = TIM1_AUTORELOAD; // maximum auto reset register value
uint16_t duty_cycle_maximum = 2000; // restricted by temperature or low rpm throttle protect
uint16_t low_rpm_level = 20; // thousand erpm used to set range for throttle resrictions
uint16_t high_rpm_level = 70; //
uint16_t throttle_max_at_low_rpm = 400;
uint16_t throttle_max_at_high_rpm = 2000;

uint16_t commutation_intervals[6] = { 0 };
volatile uint32_t average_interval = 0;
uint32_t last_average_interval;
int e_com_time;

uint16_t ADC_smoothed_input = 0;
volatile int16_t degrees_celsius;
int16_t converted_degrees;
uint8_t temperature_offset;
#ifdef NXP	// raw temperature uses two 16-bit values
uint16_t ADC_raw_temp[2] = {0};
#else
uint16_t ADC_raw_temp;
#endif
uint16_t ADC_raw_volts;
uint16_t ADC_raw_vrefint;   // v60: raw VREFINT count (VDDA reference check)
uint16_t ADC_raw_current;
uint16_t ADC_raw_input;
uint16_t ADC_raw_ntc;
uint8_t PROCESS_ADC_FLAG = 0;
volatile char send_telemetry = 0;
char telemetry_done = 0;
char prop_brake_active = 0;

volatile char dshot_telemetry = 0;

uint8_t last_dshot_command = 0;
char old_routine = 1;
uint16_t adjusted_input = 0;

#define TEMP30_CAL_VALUE ((uint16_t*)((uint32_t)0x1FFFF7B8))
#define TEMP110_CAL_VALUE ((uint16_t*)((uint32_t)0x1FFFF7C2))

uint16_t smoothedcurrent = 0;
const uint8_t numReadings = 50; // the readings from the analog input
uint8_t readIndex = 0; // the index of the current reading
uint32_t total = 0;
uint16_t readings[50];

uint8_t bemf_timeout_happened = 0;
uint8_t changeover_step = 5;
uint8_t filter_level = 5;
uint8_t running = 0;
uint16_t advance = 0;
uint8_t advancedivisor = 6;
volatile char rising = 1;

////Space Vector PWM ////////////////
// const int pwmSin[] ={128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166,
// 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221,
// 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248,
// 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255,
// 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253,
// 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243,
// 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248,
// 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255,
// 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253,
// 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243,
// 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203,
// 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147,
// 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86,
// 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18,
// 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2,
// 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8,
// 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9,
// 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
// 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16,
// 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78,
// 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};

////Sine Wave PWM ///////////////////
int16_t pwmSin[] = {
    180, 183, 186, 189, 193, 196, 199, 202, 205, 208, 211, 214, 217, 220, 224,
    227, 230, 233, 236, 239, 242, 245, 247, 250, 253, 256, 259, 262, 265, 267,
    270, 273, 275, 278, 281, 283, 286, 288, 291, 293, 296, 298, 300, 303, 305,
    307, 309, 312, 314, 316, 318, 320, 322, 324, 326, 327, 329, 331, 333, 334,
    336, 337, 339, 340, 342, 343, 344, 346, 347, 348, 349, 350, 351, 352, 353,
    354, 355, 355, 356, 357, 357, 358, 358, 359, 359, 359, 360, 360, 360, 360,
    360, 360, 360, 360, 360, 359, 359, 359, 358, 358, 357, 357, 356, 355, 355,
    354, 353, 352, 351, 350, 349, 348, 347, 346, 344, 343, 342, 340, 339, 337,
    336, 334, 333, 331, 329, 327, 326, 324, 322, 320, 318, 316, 314, 312, 309,
    307, 305, 303, 300, 298, 296, 293, 291, 288, 286, 283, 281, 278, 275, 273,
    270, 267, 265, 262, 259, 256, 253, 250, 247, 245, 242, 239, 236, 233, 230,
    227, 224, 220, 217, 214, 211, 208, 205, 202, 199, 196, 193, 189, 186, 183,
    180, 177, 174, 171, 167, 164, 161, 158, 155, 152, 149, 146, 143, 140, 136,
    133, 130, 127, 124, 121, 118, 115, 113, 110, 107, 104, 101, 98, 95, 93,
    90, 87, 85, 82, 79, 77, 74, 72, 69, 67, 64, 62, 60, 57, 55,
    53, 51, 48, 46, 44, 42, 40, 38, 36, 34, 33, 31, 29, 27, 26,
    24, 23, 21, 20, 18, 17, 16, 14, 13, 12, 11, 10, 9, 8, 7,
    6, 5, 5, 4, 3, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 5, 5,
    6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 18, 20, 21, 23,
    24, 26, 27, 29, 31, 33, 34, 36, 38, 40, 42, 44, 46, 48, 51,
    53, 55, 57, 60, 62, 64, 67, 69, 72, 74, 77, 79, 82, 85, 87,
    90, 93, 95, 98, 101, 104, 107, 110, 113, 115, 118, 121, 124, 127, 130,
    133, 136, 140, 143, 146, 149, 152, 155, 158, 161, 164, 167, 171, 174, 177
};

// int sin_divider = 2;
int16_t phase_A_position;
int16_t phase_B_position;
int16_t phase_C_position;
// ROCKWOLF v4: fractional sine phase accumulator (1/256 electrical degree units)
uint32_t sine_phase_accum = 0;
uint16_t sine_phase_inc = 256; // per 100us tick
// ROCKWOLF v6: closed-loop sine current (torque) regulation
int32_t sine_amp_trim = 0; // signed amplitude trim, 256 = one sine_mode_power step
uint16_t sine_amp_now = 1280; // effective amplitude scale applied (256..2560)
uint8_t rw_sine_target = 0; // bus current target at top of sine range, 0.05A units; 0 = off
uint8_t rw_sine_mode = 0; // ROCKWOLF v22: sine coverage - 0 normal (~35%), 1 extended (~60%), 2 full sine (no handoff)
uint8_t rw_stall_boost = 5; // ROCKWOLF v28: max stall-protection boost, x30 duty counts (stock 5 = 150); 0 disables
uint8_t rw_stall_resp = 1; // ROCKWOLF v28: stall-protection reaction rate (stallPid Kp, stock 1)
uint8_t rw_trap_mult = 24; // ROCKWOLF v29: trap cap as multiple of CTT, 0.1x units (15-40 = 1.5-4.0x)
int8_t rw_trap_start = 0; // ROCKWOLF v30: handoff speed bias, -5..+5 (0 = stock exit speed)
uint8_t rw_kick_timer = 0; // ROCKWOLF v31: brushed kickstart pulse countdown, ms
int32_t rw_br_assist = 0; // ROCKWOLF v31: brushed torque-assist integrator
uint32_t rw_br_slew = 0; // ROCKWOLF v32: brushed soft-start slew limit (rises at the Ramp rate)
uint8_t rw_br_ms = 0; // ROCKWOLF v33: 1ms tick flag for the brushed current limiter
uint8_t rw_arm_diag_done = 0; // ROCKWOLF v35: one-shot arming-state snapshot
uint16_t rw_arm_ms = 0;
// ROCKWOLF v38: run recorder — FIXED WINDOW design. The moment the ESC arms,
// a RW_RUNLOG_WINDOW_MS window opens: V (0.1 V) + current (0.1 A) are sampled
// every 50 ms into RAM for the whole window (throttle or not), then the log is
// auto-flushed to page 30 (0x7800-0x7BDF) with a confirmation tone. The page
// erase wipes the .file_name section at 0x7BE0, so a full name copy is kept
// in .rodata and rewritten on every flush.
// v46: HARD RULE — no flash erase/program while armed or commutating.
// (v45 erased the SETTINGS page while the motor was live: on F051 the
// commutation ISR runs from flash, a page erase stalls the CPU ~40 ms,
// commutation froze mid-step and the bridge took shoot-through damage.)
// Samples accumulate in RAM ONLY. The flush runs once, from the MAIN LOOP,
// after throttle zero AND commutation stopped >= 200 ms, with all FETs off.
// Page 30 layout (log + firmware name, both rewritten by the one erase):
//   log header 0x7800-0x781F, samples 0x7820-0x7BDF, name 0x7BE0-0x7BFF
#define RW_RUNLOG_BASE 0x08007800  // log page 30
#define RW_RUNLOG_INTERVAL_MS   50
#define RW_RUNLOG_MAX_SAMPLES   320     // v53: 320*3 = 960 B (V, I, throttle) = 13 s at 41.7 ms
#define RW_RUNLOG_WINDOW_MS     20000   // stop sampling this long after arming
// v56: a crawl run is STOP-AND-GO. The old 1 s neutral end-of-run treated every
// pause as the end of the run, so a run was cut at the first reposition. The end
// is now (a) the buffer filling after evidence of a real run, or (b) this many ms
// of CONTINUOUS neutral - a deliberate stop, not a pause - or (c) falling back to
// waiting if the buffer filled without any throttle ever being commanded.
#define RW_RUNLOG_END_IDLE_MS   2000    // v56: continuous neutral that means "run over" (operator call: 2 s)
#define RW_RUNLOG_MIN_MOVED     8       // v56: stored off-neutral samples = a real run
// v48 buffer: EXACTLY the page-30 layout, written in ONE save_flash_nolib call
// (proven path — the hand-rolled flash sequence used by v46/v47 never landed a
// byte on this hardware, and in v47 it erased page 31 and left it empty).
//   b[0..31]    log header (incl. flush marker, stage, attempted/stored counters)
//   b[32..991]  samples: 480 x (0.1 V, 0.1 A)
//   b[992..1023] firmware name (.file_name at 0x7BE0 — page is erased, so it
//                must be rewritten in the same call or the name is lost)
#define RW_RUNLOG_HDR_LEN       32
#define RW_RUNLOG_SAMPLE_OFF    32
#define RW_RUNLOG_NAME_OFF      992
#define RW_RUNLOG_BUF_LEN       1024
uint8_t rw_runlog_buf[RW_RUNLOG_BUF_LEN];
const char rw_name_copy[32] __attribute__((used)) = FILE_NAME RW_FW_TAG;
uint16_t rw_runlog_o = 32;     // v48: samples start after the 32-byte header
uint16_t rw_runlog_attempts = 0; // v48: sampler ticks (attempted stores) for the header
uint16_t rw_runlog_idx = 0;
uint8_t rw_runlog_active = 0;  // window open (armed, not yet flushed)
uint8_t rw_runlog_done = 0;    // this power cycle already recorded a run
uint8_t rw_runlog_flush_request = 0; // v46: set by the ISR, serviced ONLY by the main loop
uint16_t rw_runlog_stop_ms = 0;      // v46: ms of continuous commutation-stop
uint8_t rw_runlog_full = 0;
uint8_t rw_runlog_tick = 0;
uint16_t rw_runlog_win_ms = 0; // window countdown (ms since arming)
uint16_t rw_runlog_idle_ms = 0; // consecutive ms at zero throttle
uint16_t rw_runlog_moved = 0;  // v56: samples stored with the stick off neutral (run evidence)
uint8_t rw_runlog_wrapped = 0; // v57: the buffer wrapped - it holds the LAST 320 samples, not the first
uint8_t rw_runlog_khz = 0;     // 10 kHz divider (1 ms ticks)
// v50 beacons: 4-byte markers programmed into the log page's still-erased area
// (0x7B00 window-opened, 0x7B04 flush-requested). save_flash_nolib only erases
// when the address is page-aligned, and 0x7B00 % 1024 = 768, so these are pure
// program operations: no erase, no name loss, no page-31 touch. They exist so a
// "no tone, no log" run can never again be confused with "the trigger never
// fired" — the capture now shows exactly which hop is broken.
static uint8_t rw_beacon[4];
uint8_t rw_beacon1_done = 0;
uint8_t rw_beacon2_done = 0;
uint8_t rw_arm_diag_khz = 0;   // v42: 10 kHz divider for the arming snapshot
uint16_t rw_runlog_max_i = 0;      // events: max current (0.1 A)
uint16_t rw_runlog_max_i_t = 0;
uint16_t rw_runlog_min_v = 0xFFFF; // min voltage (0.1 V)
uint16_t rw_runlog_min_v_t = 0;
uint16_t rw_runlog_max_thr = 0;    // max throttle (0-255)
uint16_t rw_runlog_max_thr_t = 0;
uint8_t sine_cur_div = 0;
int32_t sine_trim_accum = 0; // v10: fine integrator so sub-8-count errors still integrate
// ROCKWOLF v10 diagnostic: capture peak measured current once per boot into eeprom byte 15
uint16_t rw_peak_current = 0;
uint16_t rw_run_ticks = 0;
uint8_t rw_cur_captured = 0;
uint32_t rw_sine_cur_sum = 0; // v11: average current while sine drive active
uint16_t rw_sine_cur_cnt = 0;
uint16_t rw_max_amp = 0; // v11: max effective sine amplitude reached (which way did the loop rail?)
uint16_t rw_min_amp = 0xFFFF; // v17: minimum amplitude reached - how far the loop wound down
// ROCKWOLF v13: boot-time auto-zero of the current channel. Many sense amps idle
// at mid-rail; the stock CURRENT_OFFSET 0 assumption made the reading peg high.
uint32_t rw_zero_acc = 0;
uint16_t rw_zero_cnt = 0;
int32_t rw_cur_zero_cmv = -1; // calibrated idle level, mV*100; -1 = not yet calibrated
uint16_t rw_zero_settle_ms = 0; // v58: boot settling time before the auto-zero is taken (once)
uint16_t step_delay = 100;
char stepper_sine = 0;
char forward = 1;
uint16_t gate_drive_offset = DEAD_TIME;

uint8_t stuckcounter = 0;
uint16_t k_erpm;
uint16_t e_rpm; // electrical revolution /100 so,  123 is 12300 erpm

uint16_t adjusted_duty_cycle;

uint8_t bad_count = 0;
uint8_t bad_count_threshold = CPU_FREQUENCY_MHZ / 24;
uint8_t dshotcommand;
uint16_t armed_count_threshold = 1000;

volatile char armed = 0;
uint16_t zero_input_count = 0;

uint16_t input = 0;
volatile uint16_t newinput = 0;
volatile char inputSet = 0;
char dshot = 0;
volatile char servoPwm = 0;
volatile uint32_t zero_crosses;

volatile uint8_t zcfound = 0;

volatile uint8_t bemfcounter;
uint8_t min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS;
uint8_t min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS;

volatile uint16_t lastzctime;
volatile uint16_t thiszctime;

volatile uint16_t duty_cycle = 0;
char step = 1;
volatile uint32_t commutation_interval = 12500;
volatile uint16_t waitTime = 0;
uint16_t signaltimeout = 0;
uint8_t ubAnalogWatchdogStatus = RESET;

#if defined(NEED_INPUT_READY) || defined(NXP)
volatile char input_ready = 0;
#endif

int32_t doPidCalculations(struct fastPID* pidnow, int actual, int target)
{

    pidnow->error = actual - target;
    pidnow->integral = pidnow->integral + pidnow->error * pidnow->Ki;
    if (pidnow->integral > pidnow->integral_limit) {
        pidnow->integral = pidnow->integral_limit;
    }
    if (pidnow->integral < -pidnow->integral_limit) {
        pidnow->integral = -pidnow->integral_limit;
    }

    pidnow->derivative = pidnow->Kd * (pidnow->error - pidnow->last_error);
    pidnow->last_error = pidnow->error;

    pidnow->pid_output = pidnow->error * pidnow->Kp + pidnow->integral + pidnow->derivative;

    if (pidnow->pid_output > pidnow->output_limit) {
        pidnow->pid_output = pidnow->output_limit;
    }
    if (pidnow->pid_output < -pidnow->output_limit) {
        pidnow->pid_output = -pidnow->output_limit;
    }
    return pidnow->pid_output;
}

void loadEEpromSettings()
{
    read_flash_bin(eepromBuffer.buffer, eeprom_address, sizeof(eepromBuffer.buffer));
    rw_brushed = (eepromBuffer.reserved_eeprom_3[2] == 1); // ROCKWOLF v5 dual-mode select
    rw_stall_boost = eepromBuffer.can.debug_rate; // ROCKWOLF v28: byte 182
    if (rw_stall_boost > 10) { // erased flash = stock
        rw_stall_boost = 5;
    }
    rw_stall_resp = eepromBuffer.can.term_enable; // ROCKWOLF v28: byte 183
    if (rw_stall_resp < 1 || rw_stall_resp > 10) {
        rw_stall_resp = 1;
    }
    stallPid.Kp = rw_stall_resp;
    rw_trap_mult = (uint8_t)eepromBuffer.can.reserved[0]; // ROCKWOLF v29: byte 184
    if (rw_trap_mult < 15 || rw_trap_mult > 40) { // erased flash = stock 2.4x
        rw_trap_mult = 24;
    }
    { // ROCKWOLF v30: byte 185, stored 0-10 = -5..+5
        uint8_t ts = (uint8_t)eepromBuffer.can.reserved[1];
        rw_trap_start = (ts <= 10) ? ((int8_t)ts - 5) : 0;
    }
    rw_sine_target = (uint8_t)eepromBuffer.reserved_eeprom_3[0]; // ROCKWOLF v9: sine current target, 0.05A units
    if (rw_sine_target > 250) { // cap 12.5A; treats erased-flash 0xFF as OFF
        rw_sine_target = 0;
    }
    if(eepromBuffer.eeprom_version < 3){ // eeprom versions less than 3 had a firmware name string in these bytes 
      eepromBuffer.max_ramp = 160;    // 0.1% per ms to 25% per ms 
      eepromBuffer.minimum_duty_cycle = 1; // 0.2% to 51 percent
      eepromBuffer.disable_stick_calibration = 0; // 
      eepromBuffer.absolute_voltage_cutoff = 10;  // voltage level 1 to 100 in 0.5v increments
      eepromBuffer.current_P = 100; // 0-255
      eepromBuffer.current_I = 0; // 0-255
      eepromBuffer.current_D = 100; // 0-255
      eepromBuffer.active_brake_power = 0; // 1-5 percent duty cycle
      eepromBuffer.brake_on_zero_throttle = 0;
      eepromBuffer.reserved_eeprom_3[0] = 0; //14-16  for crsf input
      eepromBuffer.reserved_eeprom_3[1] = 0;
      eepromBuffer.reserved_eeprom_3[2] = 0;
    }
    if(eepromBuffer.brake_on_zero_throttle > 9){ // byte 13 held a firmware name character (0x30 or similar) before eeprom version 4
      eepromBuffer.brake_on_zero_throttle = 0;
    }
    // eepromBuffer.advance_level can either be set to 0-3 with config tools less than 1.90 or 10-42 with 1.90 or above 
    if (eepromBuffer.advance_level > 42 || (eepromBuffer.advance_level < 10 && eepromBuffer.advance_level > 3)){
        temp_advance = 16;
    }
    if (eepromBuffer.advance_level < 4) {         // old format needs to be converted to 0-32 range
        temp_advance = (eepromBuffer.advance_level<<3);
        eepromBuffer.advance_level = temp_advance + 10;
    }
    if (eepromBuffer.advance_level < 43 && eepromBuffer.advance_level > 9 ) { // new format subtract 10 from advance
        temp_advance = eepromBuffer.advance_level - 10;
    }

    if (eepromBuffer.pwm_frequency < 145 && eepromBuffer.pwm_frequency > 7) {
      int divider = eepromBuffer.pwm_frequency * 100 / 6;
      TIMER1_MAX_ARR =   TIM1_AUTORELOAD * 400 / divider;
      SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    } else {
      tim1_arr = TIM1_AUTORELOAD;
      SET_AUTO_RELOAD_PWM(tim1_arr);
    }
    if(eepromBuffer.minimum_duty_cycle < 51 && eepromBuffer.minimum_duty_cycle > 0){
    minimum_duty_cycle = eepromBuffer.minimum_duty_cycle * 10;
    }else{
    minimum_duty_cycle = 0;
    }
    if (eepromBuffer.startup_power < 151 && eepromBuffer.startup_power > 49) {
            min_startup_duty = minimum_duty_cycle + eepromBuffer.startup_power;
    } else {
        min_startup_duty = minimum_duty_cycle;
    }
    startup_max_duty_cycle = minimum_duty_cycle + 400;  

    motor_kv = (eepromBuffer.motor_kv * 20) + 20; // ROCKWOLF v26: 20KV steps (was 40) - fine resolution for micro motors
#ifdef THREE_CELL_MAX
		motor_kv =  motor_kv / 2;
#endif
#ifdef ONE_TWO_CELL_MAX
		motor_kv =  motor_kv / 16;
#endif
    setVolume(2);
    if (eepromBuffer.eeprom_version > 0) { // these commands weren't introduced until eeprom version 1.
#ifdef CUSTOM_RAMP

#else
        if (eepromBuffer.beep_volume > 11) {
            setVolume(5);
        } else {
            setVolume(eepromBuffer.beep_volume);
        }
#endif
        servo_low_threshold = (eepromBuffer.servo.low_threshold * 2) + 750; // anything below this point considered 0
        servo_high_threshold = (eepromBuffer.servo.high_threshold * 2) + 1750; // anything above this point considered 2000 (max)
        servo_neutral = (eepromBuffer.servo.neutral) + 1374;
        servo_dead_band = eepromBuffer.servo.dead_band;

        low_cell_volt_cutoff = eepromBuffer.low_cell_volt_cutoff + 250; // 2.5 to 3.5 volts per cell range
        
        
#ifndef HAS_HALL_SENSORS
        eepromBuffer.use_hall_sensors = 0;
#endif

        // ROCKWOLF v23: value is now literal percent-of-stick sine coverage, 5-100.
        // 100 = full sine, no trap handoff at all.
        if (eepromBuffer.sine_mode_changeover_thottle_level < 5 || eepromBuffer.sine_mode_changeover_thottle_level > 100) {
            eepromBuffer.sine_mode_changeover_thottle_level = 25;
        }
        rw_sine_mode = (eepromBuffer.sine_mode_changeover_thottle_level >= 100) ? 2 : 0;
        if (eepromBuffer.drag_brake_strength == 0 || eepromBuffer.drag_brake_strength > 10) { // drag brake 1-10
            eepromBuffer.drag_brake_strength = 10;
        }

        if (eepromBuffer.driving_brake_strength == 0 || eepromBuffer.driving_brake_strength > 9) { // motor brake 1-9
            eepromBuffer.driving_brake_strength = 10;
        }

        if(eepromBuffer.driving_brake_strength < 10){
            dead_time_override = DEAD_TIME + (150 - (eepromBuffer.driving_brake_strength * 10));
            if (dead_time_override > 200) {
                dead_time_override = 200;
            }
        min_startup_duty = min_startup_duty + dead_time_override;
        minimum_duty_cycle = minimum_duty_cycle + dead_time_override;
        throttle_max_at_low_rpm = throttle_max_at_low_rpm + dead_time_override;
        startup_max_duty_cycle = startup_max_duty_cycle + dead_time_override;
#ifdef STMICRO
        TIM1->BDTR |= dead_time_override;
#endif
#ifdef ARTERY
        TMR1->brk |= dead_time_override;
#endif
#ifdef GIGADEVICES
        TIMER_CCHP(TIMER0) |= dead_time_override;
#endif
#ifdef NXP
    	for (int submodule = 0; submodule <= 2; submodule++) {
    		FLEXPWM0->SM[submodule].DTCNT0 = PWM_DTCNT0_DTCNT0(dead_time_override);	//PWMA deadtime
    		FLEXPWM0->SM[submodule].DTCNT1 = PWM_DTCNT1_DTCNT1(dead_time_override);	//PWMB deadtime
    	}
#endif
#ifdef WCH
            TIM1->BDTR |= dead_time_override;
#endif
        }
        if (eepromBuffer.limits.temperature < 70 || eepromBuffer.limits.temperature > 140) {
            eepromBuffer.limits.temperature = 255;
        }

        if (eepromBuffer.limits.current > 0 && eepromBuffer.limits.current <= 100) {
            use_current_limit = 1;
            trap_limit_ca = eepromBuffer.limits.current * 2 * 100;
        }
        // ROCKWOLF v27: auto trap limit derived from CTT - sprint zone gets
        // ~2.4x the crawl torque budget; a manually-set stock limit wins if stricter
        if (rw_sine_target) {
            int32_t auto_ca = ((int32_t)rw_sine_target * rw_trap_mult) / 2; // CTT x multiplier, 10mA units
            if (trap_limit_ca == 0 || auto_ca < trap_limit_ca) {
                trap_limit_ca = auto_ca;
            }
            use_current_limit = 1;
        }
        
        // ROCKWOLF v27: sanitize - erased-flash 0xFF gains would make the
        // auto-enabled trap limiter wildly aggressive
        currentPid.Kp = ((eepromBuffer.current_P > 200) ? 100 : eepromBuffer.current_P) * 2;
        currentPid.Ki = (eepromBuffer.current_I > 200) ? 0 : eepromBuffer.current_I;
        currentPid.Kd = ((eepromBuffer.current_D > 200) ? 100 : eepromBuffer.current_D) * 2;
        
        if (eepromBuffer.sine_mode_power == 0 || eepromBuffer.sine_mode_power > 10) {
            eepromBuffer.sine_mode_power = 5;
        }

        // unsinged int cant be less than 0
        if (eepromBuffer.input_type < 10) {
            switch (eepromBuffer.input_type) {
            case AUTO_IN:
                dshot = 0;
                servoPwm = 0;
                EDT_ARMED = 1;
                break;
            case DSHOT_IN:
                // ROCKWOLF v39: DShot input removed. Fall back to servo PWM so a
                // unit with DShot stored in EEPROM still drives on a normal RC
                // receiver instead of sitting dead.
                servoPwm = 1;
                break;
            case SERVO_IN:
                servoPwm = 1;
                break;
            case SERIAL_IN:
                break;
            case EDTARM_IN:
                // ROCKWOLF v39: extended DShot telemetry removed along with DShot.
                // Same graceful fallback as DSHOT_IN.
                servoPwm = 1;
                break;
            };
        } else {
            dshot = 0;
            servoPwm = 0;
            EDT_ARMED = 1;
        }
        
        if(eepromBuffer.max_ramp < 10){
          ramp_divider = 9;
          max_ramp_startup = eepromBuffer.max_ramp;
          max_ramp_low_rpm = eepromBuffer.max_ramp;
          max_ramp_high_rpm = eepromBuffer.max_ramp;
        }else{
          ramp_divider = 0;
          if((eepromBuffer.max_ramp / 10) < max_ramp_startup){
            max_ramp_startup = eepromBuffer.max_ramp / 10;
          }
          if((eepromBuffer.max_ramp / 10) < max_ramp_low_rpm){
            max_ramp_low_rpm = eepromBuffer.max_ramp / 10;
          }
          if((eepromBuffer.max_ramp / 10) < max_ramp_high_rpm){
            max_ramp_high_rpm = eepromBuffer.max_ramp / 10;
          }
        }
        
        if (motor_kv < 300) {
            low_rpm_throttle_limit = 0;
        }
        low_rpm_level = motor_kv / 100 / (32 / eepromBuffer.motor_poles);
        high_rpm_level = motor_kv / 12 / (32 / eepromBuffer.motor_poles);				
    }
    reverse_speed_threshold = map(motor_kv, 300, 3000, 1000, 500);
    if (eepromBuffer.bi_direction){
      polling_mode_changeover = POLLING_MODE_THRESHOLD / 2;
    }else{
      polling_mode_changeover = POLLING_MODE_THRESHOLD;
    }
    temp_comp_pwm = eepromBuffer.comp_pwm;
}

void saveEEpromSettings()
{
    save_flash_nolib(eepromBuffer.buffer, sizeof(eepromBuffer.buffer), eeprom_address);
}

// ROCKWOLF v37: fill the 16-byte boot-failure trace (buffer offsets 192..207)
// with a snapshot of the arming path: pulse window actually seen this boot,
// frame-trust state, timeout counters and a consecutive-failed-boot counter.
// Called only from the 3-second un-armed snapshot, then saved with the
// settings. Wiped by the next settings save — same lifetime as bytes 186/187.
void rw_trace_write(uint8_t saw_pulse)
{
    uint8_t *t = &eepromBuffer.buffer[RW_TRACE_OFF];
    uint8_t prev_boot = t[1];
    /* v62: "consecutive failed-arm boots" means boots that had a talking receiver and
       still did not arm. A boot with no pulse ever seen is a bench read or a wiring
       fault - the ESC cannot distinguish them - so it is recorded in the trace but must
       not inflate this counter. prev_boot stays 0xFF ("none") until a real arming
       failure happens. */
    uint8_t boot = saw_pulse ? ((prev_boot == 0xFF) ? 1 : ((prev_boot < 255) ? (prev_boot + 1) : 255))
                             : prev_boot;
    uint16_t ai = (adjusted_input > 2047) ? 2047 : adjusted_input;
    t[0]  = RW_TRACE_MAGIC;
    t[1]  = boot;                                    // consecutive failed-arm boots
    t[2]  = (inputSet ? 1 : 0)
          | ((zero_input_count > 30) ? 2 : 0)
          | (servoPwm ? 4 : 0)
          | ((signaltimeout > 1000) ? 8 : 0)
          | (rw_brushed ? 16 : 0);                   // same flags as byte 187
    t[3]  = rw_trace_good_frames;                    // frame-trust counter (0-8)
    t[4]  = (uint8_t)rw_trace_pulse_last;            // last in-range pulse, lo
    t[5]  = (uint8_t)(rw_trace_pulse_last >> 8);
    t[6]  = (uint8_t)rw_trace_pulse_min;             // min pulse seen this boot (0xFFFF = none)
    t[7]  = (uint8_t)(rw_trace_pulse_min >> 8);
    t[8]  = (uint8_t)rw_trace_pulse_max;             // max pulse seen this boot
    t[9]  = (uint8_t)(rw_trace_pulse_max >> 8);
    t[10] = (uint8_t)(ai >> 4);                      // matches byte 186
    t[11] = (uint8_t)(zero_input_count > 255 ? 255 : zero_input_count);
    t[12] = (uint8_t)signaltimeout;                  // lo
    t[13] = (uint8_t)(signaltimeout >> 8);
    // v43: the two spare bytes now carry the arming-gate evidence —
    // t[14] = cell_count (the gate flag: must be 0 for the gate to open)
    // t[15] = armed_timeout_count >> 8 (progress of the 1-second arming counter)
    t[14] = (uint8_t)(cell_count > 254 ? 254 : cell_count);
    t[15] = (uint8_t)(armed_timeout_count >> 8);
}

// ROCKWOLF v38: flash write for the run log. Deliberately NOT using
// save_flash_nolib (its stack copy of the whole payload would overflow the
// stack with a 2 KB buffer); programs half-words straight from the RAM buffer.
// Called from the 1 kHz block after 2 s at zero throttle: erase pages 29+30
// and write the whole recorded run (samples + 8-byte header) in one pass.
// Runs while armed but stopped, so the ~100 ms stall is harmless.
// v48: the ONLY place the run log touches flash. ONE page, ONE call.
//
// Why this replaced the hand-rolled v46/v47 sequences: across five builds the
// hand-written erase/program code never landed a single byte, and v47a/v47b
// left the SETTINGS page (0x7C00) erased and empty — the ESC came back as
// "FRESH EEPROM" with every diagnostic gone. save_flash_nolib() is the path the
// firmware already uses for every settings save (peak currents, arming
// snapshots) and it demonstrably writes on this hardware: it clears EOP after
// each operation and checks the error flags, which the hand-rolled copy never
// did. It also erases the page when the address is page-aligned.
//
// Safety: called ONLY from the main loop, after throttle zero AND commutation
// stopped >= 200 ms, with allOff() first (see the HARD RULE at the top). The
// page erased is page 30 — the log page. Page 31 (settings) is never touched
// by the flush, so a power loss mid-flush can no longer wipe the settings.
// The name at 0x7BE0 shares page 30, so it is rewritten in the same call.
static void rw_beacon_write(uint32_t addr, uint8_t stage, uint8_t a, uint8_t b)
{
    rw_beacon[0] = 0xA5;
    rw_beacon[1] = stage;
    rw_beacon[2] = a;
    rw_beacon[3] = b;
    save_flash_nolib(rw_beacon, 4, addr);
}
void rw_runlog_flush()
{
    uint16_t i;
    uint8_t *b = rw_runlog_buf;
    uint8_t ok = 1;
    allOff();
    SET_DUTY_CYCLE_ALL(0);
    // ---- header (page offset 0..31) ----
    b[0] = 'R'; b[1] = 'W'; b[2] = 'L'; b[3] = 'G';
    b[4] = (uint8_t)((rw_runlog_full ? 0x80 : 0) | 8);   // format v8: 3-byte samples V/I/throttle, throttle = 7-bit magnitude | dir<<7, volts 0.1 V
    b[5] = RW_RUNLOG_INTERVAL_MS;
    b[6] = (uint8_t)rw_runlog_idx;      b[7] = (uint8_t)(rw_runlog_idx >> 8);
    b[8] = (uint8_t)rw_runlog_max_i;    b[9] = (uint8_t)(rw_runlog_max_i >> 8);
    b[10] = (uint8_t)rw_runlog_max_i_t; b[11] = (uint8_t)(rw_runlog_max_i_t >> 8);
    b[12] = (uint8_t)rw_runlog_min_v;   b[13] = (uint8_t)(rw_runlog_min_v >> 8);
    b[14] = (uint8_t)rw_runlog_min_v_t; b[15] = (uint8_t)(rw_runlog_min_v_t >> 8);
    b[16] = (uint8_t)rw_runlog_max_thr;
    b[17] = rw_runlog_wrapped ? 1 : 0;   // v57: 1 = the buffer wrapped (log holds the LAST samples)
    b[18] = (uint8_t)rw_runlog_max_thr_t; b[19] = (uint8_t)(rw_runlog_max_thr_t >> 8);
    b[20] = 0xA5;   // flush-attempt marker (0xA5 = this page was written by a flush)
    b[21] = 0xF0;   // stage: F0 = complete; D0 = read-back verify FAILED; E0 = never ran
    b[22] = (uint8_t)((rw_runlog_win_ms / 100) > 255 ? 255 : (rw_runlog_win_ms / 100));
    b[23] = (rw_runlog_idle_ms >= 500 && rw_runlog_win_ms >= 12000) ? 1 : 2;   // v57: no buffer-full end any more
    b[24] = (uint8_t)rw_runlog_attempts; b[25] = (uint8_t)(rw_runlog_attempts >> 8);
    b[26] = (uint8_t)rw_runlog_win_ms;   b[27] = (uint8_t)(rw_runlog_win_ms >> 8);
    /* v60: VREFINT counts - the 3.3 V rail at rest and at the run's peak current.
       A rising count under load means VDDA is drooping, which inflates EVERY
       ratiometric reading (pack voltage and current alike). 0xFFFF = not recorded. */
    b[28] = (uint8_t)rw_vref_base;  b[29] = (uint8_t)(rw_vref_base >> 8);
    b[30] = (uint8_t)rw_vref_peak;  b[31] = (uint8_t)(rw_vref_peak >> 8);
    // ---- name must be rewritten in the same call (same page) ----
    for (i = 0; i < 32; i++) {
        b[RW_RUNLOG_NAME_OFF + i] = ((const uint8_t*)rw_name_copy)[i];
    }
    // ---- ONE erase + ONE write of the whole page, via the proven path ----
    save_flash_nolib(b, RW_RUNLOG_BUF_LEN, RW_RUNLOG_BASE);
    // ---- read back what should be there; the TONE reports the result ----
    for (i = 0; i < RW_RUNLOG_HDR_LEN; i++) {
        if (*(volatile uint8_t*)(RW_RUNLOG_BASE + i) != b[i]) { ok = 0; }
    }
    for (i = 0; i < 32; i++) {
        if (*(volatile uint8_t*)(0x08007BE0U + i) != ((const uint8_t*)rw_name_copy)[i]) { ok = 0; }
    }
    if (ok) {
        playChangedTone();      // one beep  = log written and verified
    } else {
        delayMillis(300);
        playChangedTone();      // two beeps = written but READ-BACK FAILED
    }
}

uint16_t getSmoothedCurrent()
{
    total = total - readings[readIndex];
    readings[readIndex] = ADC_raw_current;
    total = total + readings[readIndex];
    readIndex = readIndex + 1;
    if (readIndex >= numReadings) {
        readIndex = 0;
    }
    smoothedcurrent = total / numReadings;
    return smoothedcurrent;
}

void getBemfState()
{
    uint8_t current_state = 0;
#if defined(MCU_F031) || defined(MCU_G031)
    if (step == 1 || step == 4) {
        current_state = PHASE_C_EXTI_PORT->IDR & PHASE_C_EXTI_PIN;
    }
    if (step == 2 || step == 5) { //        in phase two or 5 read from phase A Pf1
        current_state = PHASE_A_EXTI_PORT->IDR & PHASE_A_EXTI_PIN;
    }
    if (step == 3 || step == 6) { // phase B pf0
        current_state = PHASE_B_EXTI_PORT->IDR & PHASE_B_EXTI_PIN;
    }
#else
    //Get current comparator output level
    current_state = !getCompOutputLevel(); // polarity reversed
#endif
    if (rising) {
        if (current_state) {
            bemfcounter++;
        } else {
            bad_count++;
            if (bad_count > bad_count_threshold) {
                bemfcounter = 0;
            }
        }
    } else {
        if (!current_state) {
            bemfcounter++;
        } else {
            bad_count++;
            if (bad_count > bad_count_threshold) {
                bemfcounter = 0;
            }
        }
    }
}

void commutate()
{
    if (forward == 1) {
        step++;
        if (step > 6) {
            step = 1;
            desync_check = 1;
        }
        rising = step % 2;
    } else {
        step--;
        if (step < 1) {
            step = 6;
            desync_check = 1;
        }
        rising = !(step % 2);
    }
#ifdef INVERTED_EXTI
    rising = !rising;
#endif
    __disable_irq(); // don't let dshot interrupt
    if (!prop_brake_active) {
        comStep(step);
    }
    __enable_irq();
    changeCompInput();
#ifndef NO_POLLING_START
	if (average_interval > polling_mode_changeover + 500) {
      old_routine = 1;
   }
#endif
    bemfcounter = 0;
    zcfound = 0;
    commutation_intervals[step - 1] = commutation_interval; // just used to calulate average
    
#ifdef USE_PULSE_OUT
	if(step == 1 || step == 4  ){
    WRITE_REG(RPM_PULSE_PORT->ODR, READ_REG(RPM_PULSE_PORT->ODR) ^ RPM_PULSE_PIN);
	}
#endif
}

/*
 * @brief 	Called by the COM_TIMER interrupt handler after the set wait time
 * 			This computes how much to advance in a commutation step.
 * 			This disables the COM_TIMER interrupt.
 * 			Then it enables the comparator to generate its interrupt.
 */
void PeriodElapsedCallback()
{
    DISABLE_COM_TIMER_INT(); // disable interrupt
    commutate();
    commutation_interval = ((commutation_interval)+((lastzctime + thiszctime) >> 1))>>1;
  	if (!eepromBuffer.auto_advance) {
	  advance = (commutation_interval * temp_advance) >> 6; // 60 divde 64 0.9375 degree increments
	} else {
	  advance = (commutation_interval * auto_advance_level) >> 6; // 60 divde 64 0.9375 degree increments
    }
    waitTime = (commutation_interval >> 1) - advance;
    if (!old_routine) {
        enableCompInterrupts(); // enable comp interrupt
    }
    if (zero_crosses < 10000) {
        zero_crosses++;
    }
}

/*
 * @brief 	Called by the comparator interrupt handler.
 * 			Disables the comparator interrupt.
 * 			Enables the COM_TIMER and sets it to generate an interrupt after the wait time.
 */
void interruptRoutine()
{
//   if (average_interval > 125) {
//        if ((INTERVAL_TIMER_COUNT < 125) && (duty_cycle < 600) && (zero_crosses < 500)) { // should be impossible, desync?exit anyway
//           return;
//        }
//        stuckcounter++; // stuck at 100 interrupts before the main loop happens
//                        // again.
//        if (stuckcounter > 100) {
//            maskPhaseInterrupts();
//            zero_crosses = 0;
//            return;
//        }
//    }
        for (int i = 0; i < filter_level; i++) {
#if defined(MCU_F031) || defined(MCU_G031)
            if (((current_GPIO_PORT->IDR & current_GPIO_PIN) == !(rising))) {
#else
            if (getCompOutputLevel() == rising) {
#endif
                return;
            }
        }
    __disable_irq();
    maskPhaseInterrupts();
    lastzctime = thiszctime;
    thiszctime = INTERVAL_TIMER_COUNT;  
    SET_INTERVAL_TIMER_COUNT(0);
    SET_AND_ENABLE_COM_INT(waitTime+1); // enable COM_TIMER interrupt
    __enable_irq();
}

void startMotor()
{
    if (running == 0) {
        commutate();
        commutation_interval = 10000;
        SET_INTERVAL_TIMER_COUNT(5000);
        running = 1;
    }
    enableCompInterrupts();
}

void setInput()
{
    if (eepromBuffer.bi_direction) {
        if (dshot == 0) {
            if (eepromBuffer.rc_car_reverse) {
                if (newinput > (1000 + (servo_dead_band << 1))) {
                    if (forward == eepromBuffer.dir_reversed) {
                        adjusted_input = 0;
                        //               if (running) {
                        prop_brake_active = 1;
                        if (return_to_center) {
                            forward = 1 - eepromBuffer.dir_reversed;
                            prop_brake_active = 0;
                            return_to_center = 0;
                        }
                    }
                    if (prop_brake_active == 0) {
                        return_to_center = 0;
                        adjusted_input = map(newinput, 1000 + (servo_dead_band << 1), 2000, 47, 2047);
                    }
                }
                if (newinput < (1000 - (servo_dead_band << 1))) {
                    if (forward == (1 - eepromBuffer.dir_reversed)) {
                        adjusted_input = 0;
                        prop_brake_active = 1;
                        if (return_to_center) {
                            forward = eepromBuffer.dir_reversed;
                            prop_brake_active = 0;
                            return_to_center = 0;
                        }
                    }
                    if (prop_brake_active == 0) {
                        return_to_center = 0;
                        adjusted_input = map(newinput, 0, 1000 - (servo_dead_band << 1), 2047, 47);
                    }
                }
                if (newinput >= (1000 - (servo_dead_band << 1)) && newinput <= (1000 + (servo_dead_band << 1))) {
                    adjusted_input = 0;
                    if (prop_brake_active) {
                        prop_brake_active = 0;
                        return_to_center = 1;
                    }
                }
            } else {
                if (newinput > (1000 + (servo_dead_band << 1))) {
                    if (forward == eepromBuffer.dir_reversed) {
                        if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) || stepper_sine) {
                            forward = 1 - eepromBuffer.dir_reversed;
                            zero_crosses = 0;
                            old_routine = 1;
                            maskPhaseInterrupts();
                            brushed_direction_set = 0;
                        } else {
                            newinput = 1000;
                        }
                    }
                    adjusted_input = map(newinput, 1000 + (servo_dead_band << 1), 2000, 47, 2047);
                }
                if (newinput < (1000 - (servo_dead_band << 1))) {
                    if (forward == (1 - eepromBuffer.dir_reversed)) {
                        if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) || stepper_sine) {
                            zero_crosses = 0;
                            old_routine = 1;
                            forward = eepromBuffer.dir_reversed;
                            maskPhaseInterrupts();
                            brushed_direction_set = 0;
                        } else {
                            newinput = 1000;
                        }
                    }
                    adjusted_input = map(newinput, 0, 1000 - (servo_dead_band << 1), 2047, 47);
                }

                if (newinput >= (1000 - (servo_dead_band << 1)) && newinput <= (1000 + (servo_dead_band << 1))) {
                    adjusted_input = 0;
                    brushed_direction_set = 0;
                }
            }
        }
        if (dshot) {
                     if (eepromBuffer.rc_car_reverse) {
                         if (newinput > 1047) {
                         if (forward == eepromBuffer.dir_reversed) {
                         adjusted_input = 0;
                         prop_brake_active = 1;
                         if (return_to_center) {
                             forward = 1 - eepromBuffer.dir_reversed;
                             prop_brake_active = 0;
                             return_to_center = 0;
                         }
                     }
                     if (prop_brake_active == 0) {
                         return_to_center = 0;
                         adjusted_input = ((newinput - 1048) * 2 + 47) - reversing_dead_band;
                     }
                     }
                     if (newinput <= 1047 && newinput > 47) {
                     if (forward == (1 - eepromBuffer.dir_reversed)) {
                         adjusted_input = 0;
                         prop_brake_active = 1;
                         if (return_to_center) {
                             forward = eepromBuffer.dir_reversed;
                             prop_brake_active = 0;
                             return_to_center = 0;
                         }
                     }
                     if (prop_brake_active == 0) {
                         return_to_center = 0;
                         adjusted_input = ((newinput - 48) * 2 + 47) - reversing_dead_band;
                     }
                     }
                     if (newinput < 48) {
                     adjusted_input = 0;
                     if (prop_brake_active) {
                         prop_brake_active = 0;
                         return_to_center = 1;
                     }
                 }
                         } else {
            if (newinput > 1047) {

                if (forward == eepromBuffer.dir_reversed) {
                    if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) || stepper_sine) {
                        forward = 1 - eepromBuffer.dir_reversed;
                        zero_crosses = 0;
                        old_routine = 1;
                        maskPhaseInterrupts();
                        brushed_direction_set = 0;
                     } else {
                        newinput = 0;
                    }
                }
                adjusted_input = ((newinput - 1048) * 2 + 47) - reversing_dead_band;
            }
            if (newinput <= 1047 && newinput > 47) {
                if (forward == (1 - eepromBuffer.dir_reversed)) {
                    if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) || stepper_sine) {
                        zero_crosses = 0;
                        old_routine = 1;
                        forward = eepromBuffer.dir_reversed;
                        maskPhaseInterrupts();
                        brushed_direction_set = 0;
                     } else {
                        newinput = 0;
                    }
                }
                adjusted_input = ((newinput - 48) * 2 + 47) - reversing_dead_band;
            }
            if (newinput < 48) {
                adjusted_input = 0;
                brushed_direction_set = 0;
                }
            }
        }
    } else {
        adjusted_input = newinput;
    }
    if (!rw_brushed) { // RW dual-mode (was #ifndef BRUSHED_MODE)
    if ((bemf_timeout_happened > bemf_timeout) && eepromBuffer.stuck_rotor_protection) {
        allOff();
        maskPhaseInterrupts();
        input = 0;
        bemf_timeout_happened = 102;
#ifdef USE_RGB_LED
        setIndividualRGBLed(1, 0, 0);
#endif
    } else {
#ifdef FIXED_DUTY_MODE
        input = FIXED_DUTY_MODE_POWER * 20 + 47;
#else
        if (eepromBuffer.use_sine_start) {
            if (adjusted_input < 30) { // dead band ?
                input = 0;
            }
            if (rw_sine_mode == 2) {
                // ROCKWOLF v22 full-sine mode: entire stick maps inside the sine
                // stepper region (input < 137), so the trap handoff never occurs
                if (adjusted_input > 30) {
                    input = map(adjusted_input, 30, 2047, 47, 136);
                }
            } else {
                // ROCKWOLF v25: label-accurate coverage. Sine exits at internal
                // input 137, so map the sine band to end exactly there at the
                // labeled stick percent (stock mapped to 160, putting the real
                // exit at ~80% of the label).
                uint16_t rw_span = (uint16_t)(((uint32_t)eepromBuffer.sine_mode_changeover_thottle_level * 2047) / 100);
                if (adjusted_input > 30 && adjusted_input < rw_span) {
                    input = map(adjusted_input, 30, rw_span, 47, 137);
                }
                if (adjusted_input >= rw_span) {
                    input = map(adjusted_input, rw_span, 2047, 137, 2047);
                }
            }
        } else {
            if (use_speed_control_loop) {
                if (drive_by_rpm) {
                    target_e_com_time = 60000000 / map(adjusted_input, 47, 2047, MINIMUM_RPM_SPEED_CONTROL, MAXIMUM_RPM_SPEED_CONTROL) / (eepromBuffer.motor_poles / 2);
                    if (adjusted_input < 47) { // dead band ?
                        input = 0;
                        speedPid.error = 0;
                        input_override = 0;
                    } else {
                        input = (uint16_t)(input_override / 10000); // speed control pid override
                        if (input > 2047) {
                            input = 2047;
                        }
                        if (input < 48) {
                            input = 48;
                        }
                    }
                } else {

                    input = (uint16_t)(input_override / 10000); // speed control pid override
                    if (input > 2047) {
                        input = 2047;
                    }
                    if (input < 48) {
                        input = 48;
                    }
                }
            } else {

                input = adjusted_input;
            }
        }
#endif
    }
    } // RW dual-mode end
    if (!rw_brushed) { // RW dual-mode (was #ifndef BRUSHED_MODE)
if (!stepper_sine && armed) {
        if (input >= 47 + (80 * eepromBuffer.use_sine_start)) {
            if (running == 0) {
                allOff();
                if (!old_routine) {
                    startMotor();
                }
                running = 1;
                last_duty_cycle = min_startup_duty;
            }

            if (eepromBuffer.use_sine_start) {
                duty_cycle_setpoint = map(input, 137, 2047, minimum_duty_cycle+130, 2000);
            } else {
                duty_cycle_setpoint = map(input, 47, 2047, minimum_duty_cycle, 2000);
            }

            if (!eepromBuffer.rc_car_reverse) {
                prop_brake_active = 0;
            }
        }

        if (input < 47 + (80 * eepromBuffer.use_sine_start)) {
            if (play_tone_flag != 0) {
                switch (play_tone_flag) {
									
                case 1:
                    playDefaultTone();
                    break;
                case 2:
                    playChangedTone();
                    break;
                case 3:
                    playBeaconTune3();
                    break;
                case 4:
                    playInputTune2();
                    break;
                case 5:
                    playDefaultTone();
                    break;
                }
                play_tone_flag = 0;
            }

            if (!eepromBuffer.comp_pwm) {
                duty_cycle_setpoint = 0;
                if (!running) {
                    old_routine = 1;
                    zero_crosses = 0;
                    if (eepromBuffer.brake_on_stop) {
                        fullBrake();
                    } else {
                        if (!prop_brake_active) {
                            allOff();
                        }
                    }
                }
                if (eepromBuffer.rc_car_reverse && prop_brake_active) {
#ifndef PWM_ENABLE_BRIDGE

                  if (dshot == 0) prop_brake_duty_cycle = (getAbsDif(1000, newinput) + 1000);
                    if (dshot)  {
                        if (newinput <= 1047 && newinput > 47) prop_brake_duty_cycle = ((newinput - 48) * 2 + 47) - reversing_dead_band;
                        if (newinput > 1047) prop_brake_duty_cycle = ((newinput - 1048) * 2 + 47) - reversing_dead_band;
                    }
                    if (prop_brake_duty_cycle >= (1999)) {

                        fullBrake();
                    } else {
                        proportionalBrake();
                    }
#endif
                }
            } else {
                if (!running) {

                    old_routine = 1;
                    zero_crosses = 0;
                    bad_count = 0;
                    if (eepromBuffer.brake_on_stop > 0) {
                        if (!eepromBuffer.use_sine_start) {
#ifndef PWM_ENABLE_BRIDGE
                          if(eepromBuffer.brake_on_stop == 1){
                             prop_brake_duty_cycle =  eepromBuffer.drag_brake_strength * 200;
                              if (prop_brake_duty_cycle >= (1999)) {
                                fullBrake();
                              } else {
                                proportionalBrake();
                                prop_brake_active = 1;
                              }
                           }
#else
                            // todo add proportional braking for pwm/enable style bridge.
#endif
                        }
                    } else {
                        allOff();
                    }
                    duty_cycle_setpoint = 0;
                }

                phase_A_position = ((step - 1) * 60) + enter_sine_angle;
                if (phase_A_position > 359) {
                    phase_A_position -= 360;
                }
                // ROCKWOLF v4: keep the fractional accumulator in sync on sine re-entry
                sine_phase_accum = ((uint32_t)phase_A_position) << 8;
                phase_B_position = phase_A_position + 119;
                if (phase_B_position > 359) {
                    phase_B_position -= 360;
                }
                phase_C_position = phase_A_position + 239;
                if (phase_C_position > 359) {
                    phase_C_position -= 360;
                }

                if (eepromBuffer.use_sine_start == 1) {
                    stepper_sine = 1;
                }
                duty_cycle_setpoint = 0;
            }
        }
        if (!prop_brake_active) {
            if (input >= 47 && (zero_crosses < (uint32_t)(30 >> eepromBuffer.stall_protection))) {
                if (duty_cycle_setpoint < min_startup_duty) {
                    duty_cycle_setpoint = min_startup_duty;
                }
                if (duty_cycle_setpoint > startup_max_duty_cycle) {
                    duty_cycle_setpoint = startup_max_duty_cycle;
                }
            }

            if (duty_cycle_setpoint > duty_cycle_maximum) {
                duty_cycle_setpoint = duty_cycle_maximum;
            }
            if (use_current_limit) {
                if (duty_cycle_setpoint > use_current_limit_adjust) {
                    duty_cycle_setpoint = use_current_limit_adjust;
                }
            }

            if (stall_protection_adjust > 0 && input > 47) {

                duty_cycle_setpoint = duty_cycle_setpoint + (uint16_t)(stall_protection_adjust/10000);
            }
        }
    }
    } // RW dual-mode end
}

void tenKhzRoutine()
{ // 20khz as of 2.00 to be renamed
    duty_cycle = duty_cycle_setpoint;
    tenkhzcounter++;
    ledcounter++;
    ramp_count++;
    one_khz_loop_counter++;
    // ROCKWOLF v38: fixed-window run recorder. The window opens the moment the
    // ESC arms and auto-flushes when it expires — no throttle-dependent logic.
    // Lives HERE (unconditional 10 kHz) and keeps the ADC block alive at 1 kHz
    // for the whole window so V/I stay fresh even at rest.
    if (armed && !rw_runlog_active && !rw_runlog_done) {
        rw_runlog_active = 1;
        rw_runlog_idx = 0;
        rw_runlog_o = 32;   // v48: first sample goes after the header
        rw_runlog_full = 0;
        rw_runlog_tick = 0;
        rw_runlog_win_ms = 0;
        rw_runlog_max_i = 0;
        rw_runlog_min_v = 0xFFFF;
        rw_runlog_max_thr = 0;
        rw_runlog_moved = 0;   // v56: no run evidence yet
        rw_vref_base = ADC_raw_vrefint;   // v60: reference at rest, before any load
        rw_vref_peak = ADC_raw_vrefint;
        rw_runlog_wrapped = 0; // v57
        rw_beacon1_done = 0;   // v50
        rw_beacon2_done = 0;
    }
    if (rw_runlog_active) {
        rw_runlog_khz++;
        if (rw_runlog_khz >= 10) {
            rw_runlog_khz = 0;
            PROCESS_ADC_FLAG = 1; // keep the ADC block running all window long
            rw_runlog_win_ms++;
            // v46: the ISR NEVER touches flash. It only records how long the
            // motor has been stopped and, when the run looks over, raises a
            // request that the MAIN LOOP services from a safe context.
            if (running) {
                rw_runlog_stop_ms = 0;
            } else if (rw_runlog_stop_ms < 60000) {
                rw_runlog_stop_ms++;
            }
            if (adjusted_input <= 48) {
                rw_runlog_idle_ms++;
            } else {
                rw_runlog_idle_ms = 0;
            }
            // v49: TIME + STOPPED only. The old trigger additionally required
            // rw_runlog_idx != 0, so a dry sampler produced no request, no
            // flush, no tone and no log — a state indistinguishable from a
            // broken flash path. The main loop separately demanded a centred
            // stick, which a stopped-but-not-centred ESC never satisfies.
            // The safety condition is the motor being STOPPED (HARD RULE at the
            // top of this file), not the throttle position.
            // v53b: 'running' is USELESS as a safety test in sine mode. The sine
            // stepper drives open-loop and the block that raises 'running'
            // (if (!stepper_sine && armed)) is skipped while stepper_sine is set,
            // so 'running' stays 0 through an entire crawl - the old test fired
            // the flush mid-crawl and cut the drive with allOff().
            // NEUTRAL THROTTLE ALONE IS SUFFICIENT: neither the sine stepper (it
            // only runs with the stick inside the sine band) nor commutation (it
            // needs the stick above it) can drive at adjusted_input <= 48.
            // Rejected alternatives: duty_cycle == 0 is settings-dependent (the
            // drag brake drives prop_brake_duty_cycle, not duty_cycle, and the
            // sine stepper does not drive duty_cycle either - so it is neither a
            // working drive test nor certain to be satisfiable), and
            // !stepper_sine deadlocks (stepper_sine is only cleared in the trap
            // handoff, Src/main.c:3012 - there is no clear-on-release path).
            // v53c: END-OF-RUN TRIGGER. A bare "any release ends the log" would
            // truncate at the pause BETWEEN phases (forward stop before reverse),
            // so the neutral must PERSIST: 1 s at neutral means the run is over,
            // while a phase change is 0.2-0.5 s. The old 4000 ms floor was a
            // floor, not a ceiling - it only decided the EARLIEST flush, and on
            // v51b it happened to fire at 4.0 s because the (bogus) stop_ms test
            // was already satisfied mid-crawl. v53d lowered it to 2 s; v55 raises
            // it to 3 s: the window opens the instant `armed` goes true, which is
            // the SAME moment the arming tone is queued, so the arming tune eats
            // most of the floor and 2 s left almost no time to get on the trigger
            // before a zero-stick flush burned the connection (one window per
            // power cycle). Still a floor only - while driving idle_ms is 0 and no
            // request can be raised, and buffer-full raises one too.
            /* v56 evidence gate, v57 wrap. A window that never saw a throttle
               command still must not burn the log. A full buffer no longer ends the
               run either: the sampler WRAPS, so the log always holds the most recent
               RW_RUNLOG_MAX_SAMPLES samples and the run ends where the run ends -
               2 s of continuous neutral after at least RW_RUNLOG_MIN_MOVED off-neutral
               samples. Nothing is flushed while you are driving, so the old
               zero-stick flush and the lost tail are both gone. */
            if (rw_runlog_moved >= RW_RUNLOG_MIN_MOVED &&
                rw_runlog_win_ms >= 3000 && rw_runlog_idle_ms >= RW_RUNLOG_END_IDLE_MS) {
                rw_runlog_flush_request = 1;   // serviced by the main loop only
            }
        }
    }
    // ROCKWOLF v42: arming snapshot. v35 put this in the ADC block, which is
    // gated by PROCESS_ADC_FLAG — and that flag is only set while driving (or by
    // a PID path that does not run un-armed at boot), so the diagnostic could
    // NEVER fire in the one case it exists for. Moved here, unconditional.
    if (!rw_arm_diag_done) {
        rw_arm_diag_khz++;
        if (rw_arm_diag_khz >= 10) { // 1 ms ticks
            rw_arm_diag_khz = 0;
            rw_arm_ms++;
            if (rw_arm_ms > 2500) { // snapshot before the 3s unarmed self-reset
                rw_arm_diag_done = 1;
                /* v62: write ALWAYS, count only when a receiver was talking.
                   v61 suppressed the snapshot when no pulse was ever seen - which threw
                   away the case most worth seeing: a dead BEC, a signal lead on the
                   wrong pin or a reversed plug produce NO pulses, and that customer got
                   silence. The discriminator cannot be pulses, and it cannot be pack
                   voltage either: a bench read through the dongle with the battery
                   attached also shows full pack voltage (measured on every capture of
                   2026-09-17, 7.9-8.0 V). The ESC genuinely cannot tell a powered bench
                   read from a receiver that is not talking.
                   So: keep the data, and stop the COUNTER from conflating them. The
                   trace is written on every un-armed boot; "consecutive failed-arm
                   boots" increments only when a pulse was actually seen, which is what
                   makes a boot an ARMING failure rather than a read or a wiring fault.
                   The tool adjudicates the no-pulse case with the pack voltage it can
                   see (bench read if there is no pack, wiring check if there is). */
                if (!armed) {
                    uint16_t ai = (adjusted_input > 2047) ? 2047 : adjusted_input;
                    eepromBuffer.can.reserved[2] = (uint8_t)(ai >> 4); // throttle seen
                    eepromBuffer.can.reserved[3] = (inputSet ? 1 : 0)
                        | ((zero_input_count > 30) ? 2 : 0)
                        | (servoPwm ? 4 : 0)
                        | ((signaltimeout > 1000) ? 8 : 0)
                        | (rw_brushed ? 16 : 0);
                    rw_trace_write(inputSet || rw_trace_pulse_max != 0xFFFF); // v62: param = a receiver was talking
                    saveEEpromSettings();
                }
            }
        }
    }

    if (!armed) {
        if (cell_count == 0) {
            if (inputSet) {
                if (adjusted_input == 0) {
                    armed_timeout_count++;
                    if (armed_timeout_count > LOOP_FREQUENCY_HZ) { // one second
                        if (zero_input_count > 30) {
                            armed = 1;
#ifdef USE_LED_STRIP
                            //	send_LED_RGB(0,0,0);
                            delayMicros(1000);
                            send_LED_RGB(0, 255, 0);
#endif
#ifdef USE_RGB_LED
                            setIndividualRGBLed(0,1,0);
#endif
                            if ((cell_count == 0) && eepromBuffer.low_voltage_cut_off == 1) {
                                cell_count = battery_voltage / 370;
                                for (int i = 0; i < cell_count; i++) {
                                    playInputTune();
                                    delayMillis(100);
                                    RELOAD_WATCHDOG_COUNTER();
                                }
                            } else {
#ifdef MCU_AT415
															play_tone_flag = 4;
#else
															playInputTune();
#endif
                            }
                            if (!servoPwm && !dshot) {
                                eepromBuffer.rc_car_reverse = 0;
                            }
                        } else {
                            inputSet = 0;
                            armed_timeout_count = 0;
                        }
                    }
                } else {
                    armed_timeout_count = 0;
                }
            }
        }
    }

    if (eepromBuffer.telemetry_on_interval) {
        telem_ms_count++;
        if (telem_ms_count > ((telemetry_interval_ms - 1 + eepromBuffer.telemetry_on_interval) * 20)) {
            // telemetry_on_interval = 1 is a boolean, but it can also be 2 or more to indicate an identifier
            // by making the interval just slightly different with an unique identifier, we can guarantee that many ESCs can communicate on just one signal
            // there will be some collisions but not as many as if two ESCs always tried to talk at once.
            send_telemetry = 1;
            telem_ms_count = 0;
        }
    }

    if (!rw_brushed) { // RW dual-mode (was #ifndef BRUSHED_MODE)

    if (!stepper_sine) {
#ifndef CUSTOM_RAMP
        if (old_routine && running) {
	//				send_LED_RGB(255, 0, 0);
            maskPhaseInterrupts();
            getBemfState();
            if (!zcfound) {
                if (rising) {
                    if (bemfcounter > min_bemf_counts_up) {
                        zcfound = 1;
                        zcfoundroutine();
                    }
                } else {
                    if (bemfcounter > min_bemf_counts_down) {
                        zcfound = 1;
                        zcfoundroutine();
                    }
                }
            }
        }
#endif
        if (one_khz_loop_counter > PID_LOOP_DIVIDER) { // 1khz PID loop
            PROCESS_ADC_FLAG = 1; // set flag to do new adc read at lower priority
            one_khz_loop_counter = 0;
            if (use_current_limit && running) {
                use_current_limit_adjust -= (int16_t)(doPidCalculations(&currentPid, actual_current,
                                                          trap_limit_ca)
                    / 10000);
                if (use_current_limit_adjust < minimum_duty_cycle) {
                    use_current_limit_adjust = minimum_duty_cycle;
                }
                if (use_current_limit_adjust > 2000) {
                    use_current_limit_adjust = 2000;
                }
            }
            if (eepromBuffer.stall_protection && running && rw_stall_boost) { // this boosts throttle as the rpm gets lower, for crawlers
                                               // and rc cars only, do not use for multirotors.
                stall_protection_adjust += (doPidCalculations(&stallPid, commutation_interval,
                                               stall_protect_target_interval));
                // ROCKWOLF v28: boost ceiling is user-set (x30 duty counts)
                if (stall_protection_adjust > (int32_t)rw_stall_boost * 30 * 10000) {
                    stall_protection_adjust = (int32_t)rw_stall_boost * 30 * 10000;
                }
                if (stall_protection_adjust <= 0) {
                    stall_protection_adjust = 0;
                }
            }
            if (use_speed_control_loop && running) {
                input_override += doPidCalculations(&speedPid, e_com_time, target_e_com_time);
                if (input_override > 2047 * 10000) {
                    input_override = 2047 * 10000;
                }
                if (input_override < 0) {
                    input_override = 0;
                }
                if (zero_crosses < 100) {
                    speedPid.integral = 0;
                }
            }
        }
        if (ramp_count > ramp_divider) {
          ramp_count = 0;
#ifdef VOLTAGE_BASED_RAMP
            uint16_t voltage_based_max_change = map(battery_voltage, 800, 2200, 10, 1);
            if (average_interval > 200) {
                max_duty_cycle_change = voltage_based_max_change;
            } else {
                max_duty_cycle_change = voltage_based_max_change * 3;
            }
#else
            if (zero_crosses < 150 || last_duty_cycle < 150) {   
                max_duty_cycle_change = max_ramp_startup;
            } else {
                if (average_interval > 500) {
                    max_duty_cycle_change = max_ramp_low_rpm;
                } else {
                    max_duty_cycle_change = max_ramp_high_rpm;
                }
            }
          
#endif
#ifdef CUSTOM_RAMP
   //         max_duty_cycle_change = eepromBuffer[30];
#endif
            if ((duty_cycle - last_duty_cycle) > max_duty_cycle_change) {
                duty_cycle = last_duty_cycle + max_duty_cycle_change;

            }
            if ((last_duty_cycle - duty_cycle) > max_duty_cycle_change) {
                duty_cycle = last_duty_cycle - max_duty_cycle_change;
            }
            }else{
             duty_cycle = last_duty_cycle;
            }

        if ((armed && running) && input > 47) {
          if(zero_throttle_brake_active){
            zero_throttle_brake_active = 0;
            temp_comp_pwm = eepromBuffer.comp_pwm;
          }else{
            adjusted_duty_cycle = ((duty_cycle * tim1_arr) / 2000) + 1;
        }
        } else {
          if(running && input < 47){ // brake on zero throttle behavior while motor is still rotating
            if(eepromBuffer.brake_on_zero_throttle == 1){   // coast on 0 throttle
              temp_comp_pwm = 0;                            // tracks rpm until stopped 
              zero_throttle_brake_active = 1;
            }
              if(eepromBuffer.brake_on_zero_throttle == 2){   // motor brake on 0 throttle    
              temp_comp_pwm = 1;                             // tracks rpm until stopped
              zero_throttle_brake_active = 1;
            }
              if((eepromBuffer.brake_on_zero_throttle > 2) && (eepromBuffer.brake_on_zero_throttle < 10)){   // brake on 0 throttle after 2 + x seconds
              if(zero_throttle_brake_active == 0){
                brake_countdown = eepromBuffer.brake_on_zero_throttle - 2;  // brake countdown decremented in 10khz routine
                tenkhzcounter = 10000; 
              }
              if((brake_countdown == 0) && (zero_throttle_brake_active == 1)){
                zero_crosses = 0;                          // after countdown forces the brake on stop behavior 
                running = 0;                               // stops tracking rpm
              }
              zero_throttle_brake_active = 1;
              }
              adjusted_duty_cycle = ((duty_cycle * tim1_arr) / 2000);
          } else{  // input less than 47 and not running, normal brake on stop behavior
            if (prop_brake_active) {
              adjusted_duty_cycle =  tim1_arr - ((prop_brake_duty_cycle * tim1_arr) / 2000);
            } else {
              if((eepromBuffer.brake_on_stop == 2) && armed){  // require arming for active brake
                comStep(2);
                adjusted_duty_cycle = DEAD_TIME + ((eepromBuffer.active_brake_power * tim1_arr) / 2000)* 10;
            }else{
                adjusted_duty_cycle = ((duty_cycle * tim1_arr) / 2000);
            }
            }
          }
        }
        last_duty_cycle = duty_cycle;
        SET_AUTO_RELOAD_PWM(tim1_arr);
        SET_DUTY_CYCLE_ALL(adjusted_duty_cycle);
    }
    } // RW dual-mode end
#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
    if (getInputPinState()) {
        signaltimeout++;
        if (signaltimeout > LOOP_FREQUENCY_HZ) {
            NVIC_SystemReset();
        }
    } else {
        signaltimeout = 0;
    }
#else
    signaltimeout++;

#endif
}

void processDshot()
{
    // ROCKWOLF v39: DShot decode + telemetry REMOVED to reclaim flash.
    // The input DMA path itself stays: receiveDshotDma()/dma_buffer are shared
    // with servo PWM and with detectInput(), so nothing here touches them.
    // This function is the input entry point on NEED_INPUT_READY targets
    // (F051 included), so setInput() MUST still be called.
    setInput();
}

// ROCKWOLF v4: interpolated table lookup - 1/256-degree waveform resolution
static inline int32_t sineLookupInterp(int16_t pos, uint16_t frac)
{
    int16_t nxt = (pos >= 359) ? 0 : (pos + 1);
    return pwmSin[pos] + (((int32_t)(pwmSin[nxt] - pwmSin[pos]) * frac) >> 8);
}

void advanceincrement()
{
    // ROCKWOLF v4: micro-stepped sine drive. The phase lives in a fractional
    // accumulator (1/256 deg); each call advances by sine_phase_inc, so both
    // position and speed are essentially continuous instead of 1-degree /
    // integer-microsecond quantized.
    if (!forward) {
        sine_phase_accum += sine_phase_inc;
        if (sine_phase_accum >= (360UL << 8)) {
            sine_phase_accum -= (360UL << 8);
        }
    } else {
        sine_phase_accum -= sine_phase_inc;
        if (sine_phase_accum >= (360UL << 8)) { // unsigned underflow
            sine_phase_accum += (360UL << 8);
        }
    }
    phase_A_position = (int16_t)(sine_phase_accum >> 8);
    phase_B_position = phase_A_position + 120;
    if (phase_B_position > 359) {
        phase_B_position -= 360;
    }
    phase_C_position = phase_A_position + 240;
    if (phase_C_position > 359) {
        phase_C_position -= 360;
    }
    uint16_t frac = sine_phase_accum & 0xFF;
    int32_t vA = sineLookupInterp(phase_A_position, frac);
    int32_t vB = sineLookupInterp(phase_B_position, frac);
    int32_t vC = sineLookupInterp(phase_C_position, frac);
#ifdef GIMBAL_MODE
    setPWMCompare1(((2 * vA) + gate_drive_offset) * TIMER1_MAX_ARR / 2000);
    setPWMCompare2(((2 * vB) + gate_drive_offset) * TIMER1_MAX_ARR / 2000);
    setPWMCompare3(((2 * vC) + gate_drive_offset) * TIMER1_MAX_ARR / 2000);
#else
    // ROCKWOLF v6: effective amplitude = sine_mode_power feedforward plus
    // closed-loop current trim, clamped to the stock power-1..power-10 envelope
    // so the loop can never drive harder than sine_mode_power=10 would.
    int32_t amp = (int32_t)eepromBuffer.sine_mode_power * 256 + sine_amp_trim;
    // v15: floor at 2.5% (was 10%) - tiny motors can exceed small current
    // targets even at 10% drive near stall, leaving the loop railed with no
    // authority. Torque is current; a deep floor is safe under regulation.
    if (amp < 64) {
        amp = 64;
    }
    if (amp > 2560) {
        amp = 2560;
    }
    sine_amp_now = (uint16_t)amp;
    setPWMCompare1(
        (((((2 * vA / SINE_DIVIDER) + gate_drive_offset) * TIMER1_MAX_ARR / 2000) * amp) / 2560));
    setPWMCompare2(
        (((((2 * vB / SINE_DIVIDER) + gate_drive_offset) * TIMER1_MAX_ARR / 2000) * amp) / 2560));
    setPWMCompare3(
        (((((2 * vC / SINE_DIVIDER) + gate_drive_offset) * TIMER1_MAX_ARR / 2000) * amp) / 2560));
#endif
}

void zcfoundroutine()
{ // only used in polling mode, blocking routine.
    thiszctime = INTERVAL_TIMER_COUNT;
    SET_INTERVAL_TIMER_COUNT(0);
    commutation_interval = (thiszctime + (3 * commutation_interval)) / 4;
    advance = (temp_advance * commutation_interval) >> 6; //   7.5 degree increments
    waitTime = commutation_interval / 2 - advance;
    while ((INTERVAL_TIMER_COUNT) < (waitTime)) {
        if (zero_crosses < 5) {
            break;
        }
    }
#ifdef MCU_GDE23
    TIMER_CAR(COM_TIMER) = waitTime;
#endif
#ifdef STMICRO
    COM_TIMER->ARR = waitTime;
#endif
#ifdef MCU_AT32
		COM_TIMER->pr = waitTime;
#endif
#ifdef NXP
//	COM_TIMER->MSR[0] = waitTime;
	COM_TIMER->MR[0] = waitTime;
#endif

    commutate();
    bemfcounter = 0;
    bad_count = 0;

    zero_crosses++;
#ifdef NO_POLLING_START     // changes to interrupt mode after 2 zero crosses, does not re-enter
       if (zero_crosses > 2) {
            old_routine = 0;
            enableCompInterrupts(); // enable interrupt
        }
#else
    if (eepromBuffer.stall_protection || eepromBuffer.rc_car_reverse) {
        if (zero_crosses >= 20 && commutation_interval <= 2000) {
            old_routine = 0;
            enableCompInterrupts(); // enable interrupt
        }
    } else {
       if (commutation_interval < polling_mode_changeover) {
            old_routine = 0;
            enableCompInterrupts(); // enable interrupt
        }
    }
 #endif
}

void runBrushedLoop()
{

    uint16_t brushed_duty_cycle = 0;

    if (brushed_direction_set == 0 && adjusted_input > 48) {
        if (forward) {
            allOff();
            delayMicros(10);
            twoChannelForward();
        } else {
            allOff();
            delayMicros(10);
            twoChannelReverse();
        }
        brushed_direction_set = 1;
        rw_kick_timer = 100; // ROCKWOLF v31: kickstart window on every start from stop
        prop_brake_active = 0;
    }

    // ROCKWOLF v31: progressive (squared) throttle curve - the bottom half of
    // the stick becomes a finesse zone instead of instant power
    {
        int32_t bx = adjusted_input - 48;
        if (bx < 0) {
            bx = 0;
        }
        if (bx > 1999) {
            bx = 1999;
        }
        int32_t bmax = TIMER1_MAX_ARR - (TIMER1_MAX_ARR / 20);
        brushed_duty_cycle = (uint16_t)(((bx * bx) / 1999) * bmax / 1999);
    }
    // ROCKWOLF v31: torque assist (CTT on) - adds up to +100% of commanded duty
    // when measured current is below the throttle-proportional target; breaks
    // neo-magnet cogging loose and holds torque through binds, backs off free
    if (rw_sine_target && brushed_duty_cycle > 0) {
        uint32_t br_assist = (uint32_t)(rw_br_assist / 8);
        if (br_assist > brushed_duty_cycle) {
            br_assist = brushed_duty_cycle;
        }
        brushed_duty_cycle += (uint16_t)br_assist;
    }
    // ROCKWOLF v32: soft-start slew - duty may only rise as fast as the Ramp
    // setting allows (the kickstart below is exempt on purpose)
    if (brushed_duty_cycle > rw_br_slew) {
        brushed_duty_cycle = (uint16_t)rw_br_slew;
    }
    // ROCKWOLF v31: kickstart - brief punch out of the magnetic detent,
    // strength from the Startup Power setting (50-150 -> ~8-25% duty)
    if (rw_kick_timer && brushed_duty_cycle > 0) {
        uint16_t kick = (uint16_t)(((uint32_t)TIMER1_MAX_ARR * eepromBuffer.startup_power) / 600);
        if (brushed_duty_cycle < kick) {
            brushed_duty_cycle = kick;
        }
    }

    if (degrees_celsius > eepromBuffer.limits.temperature) {
        duty_cycle_maximum = map(degrees_celsius, eepromBuffer.limits.temperature,
            eepromBuffer.limits.temperature + 20, TIMER1_MAX_ARR / 2, 1);
    } else {
        duty_cycle_maximum = TIMER1_MAX_ARR - 50;
    }
    if (brushed_duty_cycle > duty_cycle_maximum) {
        brushed_duty_cycle = duty_cycle_maximum;
    }

    if (use_current_limit) {
        // ROCKWOLF v33: update the limiter PID at 1kHz only - it ran at full
        // main-loop rate here (~30x too fast for its gains), which made it
        // slam power in audible bursts at high rpm once CTT auto-enabled it
        if (rw_br_ms) {
            rw_br_ms = 0;
            use_current_limit_adjust -= (int16_t)(doPidCalculations(&currentPid, actual_current,
                                                      trap_limit_ca)
                / 10000);
            if (use_current_limit_adjust < minimum_duty_cycle) {
                use_current_limit_adjust = minimum_duty_cycle;
            }
            if (use_current_limit_adjust > 2000) {
                use_current_limit_adjust = 2000;
            }
        }
        if (brushed_duty_cycle > use_current_limit_adjust) {
            brushed_duty_cycle = use_current_limit_adjust;
        }
    }
    if ((brushed_duty_cycle > 0) && armed) {
        SET_DUTY_CYCLE_ALL(brushed_duty_cycle);
    } else {
        // ROCKWOLF v31: brushed drag brake - short the motor through the low
        // sides at the set strength instead of freewheeling (hill hold)
        if (eepromBuffer.brake_on_stop == 1 && armed) {
            uint16_t pb = eepromBuffer.drag_brake_strength * 200;
            adjusted_duty_cycle = tim1_arr - ((pb * tim1_arr) / 2000);
            proportionalBrake();
            SET_DUTY_CYCLE_ALL(adjusted_duty_cycle);
            prop_brake_active = 1;
        } else {
            SET_DUTY_CYCLE_ALL(0);
        }
        brushed_direction_set = 0;
    }
}



/*
  check device info from the bootloader, confirming pin code and eeprom location
 */
static void checkDeviceInfo(void)
{
#ifdef NXP
    uint32_t pflashBlockBase  = 0U;
    uint32_t pflashTotalSize  = 0U;
    uint32_t pflashSectorSize = 0U;

    //Get flash properties
    FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashBlockBaseAddr, &pflashBlockBase);
    FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashSectorSize, &pflashSectorSize);
    FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashTotalSize, &pflashTotalSize);
#else
#define DEVINFO_MAGIC1 0x5925e3da
#define DEVINFO_MAGIC2 0x4eb863d9

    const struct devinfo {
        uint32_t magic1;
        uint32_t magic2;
        const uint8_t deviceInfo[9];
    } *devinfo = (struct devinfo *)(0x1000 - 32);
    if (devinfo->magic1 != DEVINFO_MAGIC1 ||
        devinfo->magic2 != DEVINFO_MAGIC2) {
        // bootloader does not support this feature, nothing to do
        return;
    }
    // change eeprom_address based on the code in the bootloaders device info
    switch (devinfo->deviceInfo[4]) {
        case 0x1f:
            eeprom_address = 0x08007c00;
            break;
        case 0x35:
            eeprom_address = 0x0800f800;
            break;
        case 0x2b:
            eeprom_address = 0x0801f800;
            break;
    }
#endif

    // TODO: check pin code and reboot to bootloader if incorrect

}

int main(void)
{

#ifdef NXP
    initCorePeripherals();
    checkDeviceInfo();
    loadEEpromSettings();
    enableCorePeripherals();
    initAfterJump();
#else
    initAfterJump();
    checkDeviceInfo();
    initCorePeripherals();
    enableCorePeripherals();
    loadEEpromSettings();
#endif

    // ROCKWOLF v44: boot-time name repair, moved to the FRONT of main().
    // v40 ran this after receiveDshotDma() had already armed the servo input
    // DMA — and because the bootloader drops the .file_name tail on every
    // flash, the name is blank on the first boot after flashing, i.e. exactly
    // the boot used for an arm test. The repair's page-erase stall (20-40 ms)
    // then landed while input capture was live and before the detection state
    // machine ran, leaving input detection desynced: "arms on v38, never on
    // v40+". Do it here instead: no peripheral is running yet, nothing to
    // disturb. One erase, only when the name is actually missing.
    if (*(volatile uint32_t*)0x08007BE0U == 0xFFFFFFFFU) {
        // v48: same proven write path as everything else. Address is not
        // page-aligned (0x7BE0 % 1024 = 992), so save_flash_nolib programs
        // without erasing — correct here: the guard above proves it is 0xFF.
        save_flash_nolib((uint8_t*)rw_name_copy, 32, 0x08007BE0U);
    }

    if (VERSION_MAJOR != eepromBuffer.version.major || VERSION_MINOR != eepromBuffer.version.minor || EEPROM_VERSION > eepromBuffer.eeprom_version) {
        eepromBuffer.version.major = VERSION_MAJOR;
        eepromBuffer.version.minor = VERSION_MINOR;
        eepromBuffer.eeprom_version = EEPROM_VERSION;
        saveEEpromSettings();
    }
    
    if (eepromBuffer.dir_reversed == 1) {
        forward = 0;
    } else {
        forward = 1;
    }
    tim1_arr = TIMER1_MAX_ARR;
    if (!eepromBuffer.comp_pwm) {
        eepromBuffer.use_sine_start = 0; // sine start requires complementary pwm.
    }

    if (eepromBuffer.rc_car_reverse) { // overrides a whole lot of things!
        throttle_max_at_low_rpm = 1000;
        eepromBuffer.bi_direction = 1;
        eepromBuffer.use_sine_start = 0;
        low_rpm_throttle_limit = 1;
        eepromBuffer.variable_pwm = 0;
        eepromBuffer.brake_on_zero_throttle = 0;
        eepromBuffer.comp_pwm = 0;
        temp_comp_pwm = 0;
        eepromBuffer.stuck_rotor_protection = 0;
        minimum_duty_cycle = minimum_duty_cycle + 50;
        stall_protect_minimum_duty = stall_protect_minimum_duty + 50;
        min_startup_duty = min_startup_duty + 50;
    }

#ifdef MCU_F031
    GPIOF->BSRR = LL_GPIO_PIN_6; // uncomment to take bridge out of standby mode
                                 // and set oc level
    GPIOF->BRR = LL_GPIO_PIN_7; // out of standby mode
    GPIOA->BRR = LL_GPIO_PIN_11;
#endif
#ifdef MCU_G031
    GPIOA->BRR = LL_GPIO_PIN_11;
    GPIOA->BSRR = LL_GPIO_PIN_12;    // Pa12 attached to enable on dev board
#endif
#ifdef USE_LED_STRIP
    send_LED_RGB(125, 0, 0);
#endif
#ifdef USE_RGB_LED
     setIndividualRGBLed(1,0,0);
#endif

#ifdef USE_CRSF_INPUT
    inputSet = 1;
    playStartupTune();
    MX_IWDG_Init();
    LL_IWDG_ReloadCounter(IWDG);
#else
#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
    MX_IWDG_Init();
    RELOAD_WATCHDOG_COUNTER();
    inputSet = 1;
    armed = 1;
    adjusted_input = 48;
    newinput = 48;
		comStep(2);
#ifdef FIXED_SPEED_MODE
    use_speed_control_loop = 1;
    eepromBuffer.use_sine_start = 0;
    target_e_com_time = 60000000 / FIXED_SPEED_MODE_RPM / (eepromBuffer.motor_poles / 2);
    input = 48;
#endif

#else
    if (rw_brushed) { // RW dual-mode boot init
    // bi_direction = 1;
    commutation_interval = 5000;
    eepromBuffer.use_sine_start = 0;
    maskPhaseInterrupts();
    // v34: melody-in-brushed-boot reverted - it hung boot on the bench.
    // Brushed uses its proven startup tune until the interaction is understood.
    playBrushedStartupTune();
    } else { // RW dual-mode
 #ifdef MCU_AT415
    play_tone_flag = 5;
 #else
    playStartupTune();
	#endif
    } // RW dual-mode end
    zero_input_count = 0;
    MX_IWDG_Init();
    RELOAD_WATCHDOG_COUNTER();
#ifdef GIMBAL_MODE
    eepromBuffer.bi_direction = 1;
    eepromBuffer.use_sine_start = 1;
#endif

#ifdef USE_ADC_INPUT
    armed_count_threshold = 5000;
    inputSet = 1;

#else
    // checkForHighSignal();     // will reboot if signal line is high for 10ms
    receiveDshotDma();
    if (drive_by_rpm) {
        use_speed_control_loop = 1;
    }
#endif

#endif // end fixed duty mode ifdef
#endif // end crsf input

#ifdef MCU_F051
    MCU_Id = DBGMCU->IDCODE &= 0xFFF;
    REV_Id = DBGMCU->IDCODE >> 16;

    if (REV_Id >= 4096) {
        temperature_offset = 0;
    } else {
        temperature_offset = 230;
    }

#endif
#ifdef NEUTRONRC_G071
    setInputPullDown();
#else
    setInputPullUp();
#endif

#ifdef USE_STARTUP_BOOST
  min_startup_duty = min_startup_duty + 200 + ((eepromBuffer.pwm_frequency * 100)/24);
  minimum_duty_cycle = minimum_duty_cycle + 50 + ((eepromBuffer.pwm_frequency * 50 )/24);
  startup_max_duty_cycle = startup_max_duty_cycle + 400;
#endif

    while (1) {
        // ROCKWOLF v46: the ONLY flash write of the run recorder, and the only
        // place it can ever be issued. Requires: a pending request, throttle at
        // zero, and commutation stopped for >= 200 ms. Not armed-only: a stopped
        // motor with FETs off is the one state where a flash stall is harmless.
        // v50 beacons (main loop, motor stopped — same safety condition as the flush)
        // v53d: 1000 ms, matching the trigger. A forward->reverse thumb TRANSIT
        // measures 285 ms of true neutral (2026-09-15 14:17 capture) - only 15 ms
        // under the old 300 ms gate, so a slightly slower flick with the buffer
        // full would have fired the flush mid-transit and cut the drive.
        if (rw_runlog_active && adjusted_input <= 48 && rw_runlog_idle_ms >= 1000) {
            if (!rw_beacon1_done) {
                rw_beacon_write(0x08007B00, 1,
                                (uint8_t)(rw_runlog_win_ms / 100),
                                (uint8_t)(rw_runlog_win_ms >> 8));
                rw_beacon1_done = 1;
            } else if (rw_runlog_flush_request && !rw_beacon2_done) {
                rw_beacon_write(0x08007B04, 2,
                                (uint8_t)(rw_runlog_win_ms / 100),
                                (uint8_t)(rw_runlog_stop_ms / 100));
                rw_beacon2_done = 1;
            }
        }
        if (rw_runlog_flush_request) {
            // v53d: neutral 1000 ms - the SAME persistence as the trigger, so a
            // 285 ms centre transit can never satisfy the gate (see the beacon note)
            if (adjusted_input <= 48 && rw_runlog_idle_ms >= 1000) {
                rw_runlog_flush();
                rw_runlog_flush_request = 0;
                rw_runlog_active = 0;
                rw_runlog_done = 1;
            }
        }
e_com_time = ((commutation_intervals[0] + commutation_intervals[1] + commutation_intervals[2] + commutation_intervals[3] + commutation_intervals[4] + commutation_intervals[5]) + 4) >> 1; // COMMUTATION INTERVAL IS 0.5US INCREMENTS 

#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
        setInput();
#endif

#ifdef NEED_INPUT_READY
 #ifdef MCU_F031
    if (input_ready) {
    setInput(); 
    input_ready = 0;
    }
#else
    if (input_ready) {
     processDshot();
     input_ready = 0;
     }
#endif
#endif
if(zero_crosses < 5){
    if(eepromBuffer.bi_direction){
     min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS + 1;
     min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS + 1;
   }else{
     min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS * 2;
     min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS * 2;
   }
}else{
	  min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS;
	  min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS;
}

       RELOAD_WATCHDOG_COUNTER();

        if (eepromBuffer.variable_pwm == 1) {      // uses range defined by pwm frequency setting
            tim1_arr = map(commutation_interval, 96, 200, TIMER1_MAX_ARR / 2,
                TIMER1_MAX_ARR);
        }
        if (eepromBuffer.variable_pwm == 2) {      // uses automatic range   
          if(average_interval < 250 && average_interval > 100){
            tim1_arr = average_interval * (CPU_FREQUENCY_MHZ/9);
          }
          if(average_interval < 100 && average_interval > 0){
            tim1_arr = 100 * (CPU_FREQUENCY_MHZ/9);
         }
          if((average_interval >= 250) || (average_interval == 0)){
              tim1_arr = 250 * (CPU_FREQUENCY_MHZ/9);
          } 
        }
        if (signaltimeout > (LOOP_FREQUENCY_HZ >> 1)) { // half second timeout when armed;
            if (armed) {
                allOff();
                armed = 0;
                input = 0;
                inputSet = 0;
                zero_input_count = 0;
                SET_DUTY_CYCLE_ALL(0);
                resetInputCaptureTimer();
                for (int i = 0; i < 64; i++) {
                    dma_buffer[i] = 0;
                }
                // v46: deliberately NOT flushed here. This path runs while the
                // ESC is armed; a flash erase now can freeze commutation and
                // destroy the bridge (v45 did exactly that). The RAM log is lost
                // on this reset — acceptable, and the safe behaviour.
                NVIC_SystemReset();
            }
            if (signaltimeout > 30000) { // 3.0 second when not armed (v41: let the 2.5s arming snapshot fire first)
                allOff();
                armed = 0;
                input = 0;
                inputSet = 0;
                zero_input_count = 0;
                SET_DUTY_CYCLE_ALL(0);
                resetInputCaptureTimer();
                for (int i = 0; i < 64; i++) {
                    dma_buffer[i] = 0;
                }
                NVIC_SystemReset();
            }
        }
#ifdef USE_CUSTOM_LED
        if ((input >= 47) && (input < 1947)) {
            if (ledcounter > (2000 >> forward)) {
                GPIOB->BSRR = LL_GPIO_PIN_3;
            } else {
                GPIOB->BRR = LL_GPIO_PIN_3;
            }
            if (ledcounter > (4000 >> forward)) {
                ledcounter = 0;
            }
        }
        if (input > 1947) {
            GPIOB->BSRR = LL_GPIO_PIN_3;
        }
        if (input < 47) {
            GPIOB->BRR = LL_GPIO_PIN_3;
        }
#endif

        if (tenkhzcounter > LOOP_FREQUENCY_HZ) { // 1s sample interval 10000
            consumed_current += (actual_current << 16) / 360;
            tenkhzcounter = 0;
            if(brake_countdown > 0){
              brake_countdown--;
            }
        }

    if (!rw_brushed) { // RW dual-mode (was #ifndef BRUSHED_MODE)

        if ((zero_crosses > 1000) || (adjusted_input == 0)) {
            bemf_timeout_happened = 0;
        }
        if (zero_crosses > 100 && adjusted_input < 200) {
            bemf_timeout_happened = 0;
        }
        if (eepromBuffer.use_sine_start && adjusted_input < 160) {
            bemf_timeout_happened = 0;
        }

        if (crawler_mode) {
            if (adjusted_input < 400) {
                bemf_timeout_happened = 0;
            }
        } else {
            if (adjusted_input < 150) { // startup duty cycle should be low enough to not burn motor
                bemf_timeout = 100;
            } else {
                bemf_timeout = 10;
            }
        }
    } // RW dual-mode end
        average_interval = e_com_time / 3;
        if (desync_check && zero_crosses > 10) {
            if ((getAbsDif(last_average_interval, average_interval) > average_interval >> 1) && (average_interval < 2000)) { // throttle resitricted before zc 20.
                zero_crosses = 0;
                desync_happened++;
                if ((!eepromBuffer.bi_direction && (input > 47)) || commutation_interval > 1000) {
                    running = 0;
                }
                old_routine = 1;
                if (zero_crosses > 100) {
                    average_interval = 5000;
                }
                last_duty_cycle = min_startup_duty / 2;
            }
            desync_check = 0;
            //	}
            last_average_interval = average_interval;
        }

#if !defined(MCU_G031) && !defined(NEED_INPUT_READY)
#ifdef NXP
	if (dshot_telemetry && (commutation_interval > DSHOT_PRIORITY_THRESHOLD)) {
		NVIC_SetPriority(IC_DMA_IRQ_NAME, 0);
		NVIC_SetPriority(COM_TIMER_IRQ, 1);
		NVIC_SetPriority(COMP0_IRQ, 1);
		NVIC_SetPriority(COMP1_IRQ, 1);
	} else {
		NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
		NVIC_SetPriority(COM_TIMER_IRQ, 0);
		NVIC_SetPriority(COMP0_IRQ, 0);
		NVIC_SetPriority(COMP1_IRQ, 0);
	}
#else
        if (dshot_telemetry && (commutation_interval > DSHOT_PRIORITY_THRESHOLD)) {
             NVIC_SetPriority(IC_DMA_IRQ_NAME, 0);
             NVIC_SetPriority(COM_TIMER_IRQ, 1);
             NVIC_SetPriority(COMPARATOR_IRQ, 1);
         } else {
             NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
             NVIC_SetPriority(COM_TIMER_IRQ, 0);
             NVIC_SetPriority(COMPARATOR_IRQ, 0);
         }
#endif
#endif
        if (send_telemetry) {
#ifdef USE_SERIAL_TELEMETRY
            makeTelemPackage((int8_t)degrees_celsius, battery_voltage, actual_current,
                (uint16_t)(consumed_current >> 16), e_rpm);
            send_telem_DMA(10);
            send_telemetry = 0;
#endif
        } else if(send_esc_info_flag ) {
           makeInfoPacket();
           send_telem_DMA(49);
           send_esc_info_flag = 0;
        }
        if (PROCESS_ADC_FLAG == 1) { // for adc and telemetry set adc counter at 1khz loop rate
#if defined(STMICRO)
            ADC_DMA_Callback();
            LL_ADC_REG_StartConversion(ADC1);
#ifdef USE_ADC_1_2
          LL_ADC_REG_StartConversion(ADC2);
#endif          
            converted_degrees = __LL_ADC_CALC_TEMPERATURE(3300, ADC_raw_temp, LL_ADC_RESOLUTION_12B);
#endif
#ifdef MCU_GDE23
            ADC_DMA_Callback();
            // converted_degrees = (1.43 - ADC_raw_temp * 3.3 / 4096) * 1000 / 4.3 + 25;
            converted_degrees = ((int32_t)(357.5581395348837f * (1 << 16)) - ADC_raw_temp * (int32_t)(0.18736373546511628f * (1 << 16))) >> 16;
            adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
#endif
#ifdef ARTERY
            ADC_DMA_Callback();
            adc_ordinary_software_trigger_enable(ADC1, TRUE);
    #ifdef USE_NTC
            converted_degrees = getNTCDegrees(ADC_raw_ntc);
    #else     
            converted_degrees = getConvertedDegrees(ADC_raw_temp);
    #endif
#endif
#ifdef NXP
            //Call ADC_DMA callback to get raw data
            ADC_DMA_Callback();

            //Convert temperature data to actual temperature in degrees Celsius
            converted_degrees = computeTemperature(ADC_raw_temp[0], ADC_raw_temp[1]);

            //Start ADC conversion
            startADCConversion();
#endif
#ifdef WCH
            startADCConversion( );
            converted_degrees = getConvertedDegrees(ADC_raw_temp);
#endif
            degrees_celsius = converted_degrees;
#ifdef NXP
            //MCXA has 16-bit ADC data
            battery_voltage = ((7 * battery_voltage) + ((ADC_raw_volts * 3300 / 65535 * VOLTAGE_DIVIDER) / 100)) / 8;
            smoothed_raw_current = getSmoothedCurrent();
            //Actual current is in 10mA, so 1 = 10mA
            actual_current = (((smoothed_raw_current * 3300 / 65535) - CURRENT_OFFSET) * 100) / (MILLIVOLT_PER_AMP);
#else
            battery_voltage = ((7 * battery_voltage) + ((ADC_raw_volts * 3300 / 4095 * VOLTAGE_DIVIDER) / 100)) >> 3;
            smoothed_raw_current = getSmoothedCurrent();
            // ROCKWOLF v13: auto-zero - average the channel's idle level for the
            // first ~half second after boot (motor guaranteed stopped), then
            // subtract it, instead of trusting the compile-time CURRENT_OFFSET.
            /* v58: the mid-run re-take (v56) is REMOVED. It could land while the
               motor was still coasting - `running` is 0 right through a sine crawl
               and a coast - which captured a LOADED level as the zero and then
               clamped every later current reading to 0: the v57 capture shows 320
               stored samples of 0.0 A through a full-stick run while the header's
               max_i still held the 1.0 A seen at the start, and the voltage dipped
               7.8 V at half stick (so current really flowed). Calibration now runs
               ONCE, at boot, where the motor is guaranteed stopped and unarmed. */
            if (rw_cur_zero_cmv < 0) {
                /* v58: settle first. Averaging the first ~0.5 s after boot captured
                   a not-yet-settled channel and left a 32 mV (1.0 A at 32 mV/A)
                   offset - measured as min=max=mean 1.0 A at a dead stop. */
                if (rw_zero_settle_ms < 1000) {
                    if (rw_zero_settle_ms < 60000) { rw_zero_settle_ms++; }
                } else if (input < 48 && !running) {
                    rw_zero_acc += (uint32_t)(smoothed_raw_current * 3300 / 41);
                    rw_zero_cnt++;
                    if (rw_zero_cnt >= 512) {
                        rw_cur_zero_cmv = (int32_t)(rw_zero_acc / 512);
                        // stash idle level (mV, 16-bit) for the tuner diagnostic;
                        // persisted by the next run-capture save, no extra flash write
                        uint16_t zmv = (uint16_t)(rw_cur_zero_cmv / 100);
                        eepromBuffer.can.require_arming = (uint8_t)(zmv & 0xFF);
                        eepromBuffer.can.telem_rate = (uint8_t)(zmv >> 8);
                    }
                } else {
                    /* the guard broke mid-average: start over rather than mixing a
                       spinning motor into the zero (v58) */
                    rw_zero_acc = 0; rw_zero_cnt = 0;
                }
                actual_current = 0;
            } else {
                actual_current = ((smoothed_raw_current * 3300 / 41) - rw_cur_zero_cmv) / (MILLIVOLT_PER_AMP);
            }
#endif
            if (actual_current < 0) {
                actual_current = 0;
            }
            // ROCKWOLF v38: run recorder — sample V/I every 50 ms for the whole
            // arming-to-flush window (the 10 kHz routine opens the window and
            // keeps this ADC block alive at 1 kHz all window long).
            if (rw_runlog_active) {
                rw_runlog_tick++;
                if (rw_runlog_tick >= RW_RUNLOG_INTERVAL_MS) {
                    rw_runlog_tick = 0;
                    rw_runlog_attempts++;      // v48: attempted vs stored (analyst request)
                    {
                        /* v57: WRAP. Once the buffer is full the write pointer goes
                           back to the first sample slot and overwrites the OLDEST
                           sample, so the page always holds the last 320 samples.
                           The helper recovers the order: oldest buffer index =
                           attempts mod count, and header byte 17 says it wrapped. */
                        if (rw_runlog_o > RW_RUNLOG_NAME_OFF - 3) { rw_runlog_o = 32; }
                        uint8_t cv = (uint8_t)(battery_voltage / 10);   // v52: 0.1 V units (was the low byte of a 0.01 V value, which wrapped every 2.56 V)
                        uint8_t ci = (actual_current < 0) ? 0 : (uint8_t)(actual_current > 2550 ? 255 : (actual_current / 10));
                        // v54: throttle byte carries MAGNITUDE + DIRECTION.
                        // 7 bits of magnitude (0-127, was 0-255) and bit 7 = the
                        // 'forward' state, because a 3D crawler log is useless
                        // without knowing which way the motor was being driven -
                        // the reverse half of the first two-direction log was
                        // indistinguishable from forward. Costs 0.8% of stick
                        // resolution, which no crawl tune can feel.
                        uint8_t th = (uint8_t)((adjusted_input >> 4) | (forward ? 0x00 : 0x80));
                        rw_runlog_buf[rw_runlog_o] = cv;
                        rw_runlog_buf[rw_runlog_o + 1] = ci;
                        rw_runlog_buf[rw_runlog_o + 2] = th;   // v53: throttle position (adjusted_input >> 3)
                        {   /* v57: events are recorded as PHYSICAL buffer indices */
                            uint16_t sidx = (uint16_t)((rw_runlog_o - 32) / 3);
                            if (ci > (uint8_t)rw_runlog_max_i) { rw_runlog_max_i = ci; rw_runlog_max_i_t = sidx; rw_vref_peak = ADC_raw_vrefint; }   // v60
                            if (cv < (uint8_t)rw_runlog_min_v) { rw_runlog_min_v = cv; rw_runlog_min_v_t = sidx; }
                            if ((th & 0x7F) > (uint8_t)rw_runlog_max_thr) { rw_runlog_max_thr = (th & 0x7F); rw_runlog_max_thr_t = sidx; }
                        }
                        if (adjusted_input > 48) { rw_runlog_moved++; }   // v56: run evidence
                        rw_runlog_o += 3;   // v53: 3 bytes per sample (V, I, throttle)
                        if (rw_runlog_idx < RW_RUNLOG_MAX_SAMPLES) { rw_runlog_idx++; }
                        else { rw_runlog_wrapped = 1; }
                    }
                }
            }
            // ROCKWOLF v31: brushed-mode helpers at 1kHz
            if (rw_brushed) {
                rw_br_ms = 1; // v33: 1kHz tick for the brushed limiter PID
                if (rw_kick_timer) {
                    rw_kick_timer--;
                }
                // v32: throttle ramp - the slew ceiling rises at the Ramp rate
                // (0.1%/ms units) while driving, resets when stopped
                if (adjusted_input > 48) {
                    uint32_t slew_step = ((uint32_t)TIMER1_MAX_ARR * eepromBuffer.max_ramp) / 10000;
                    if (slew_step == 0) {
                        slew_step = 1;
                    }
                    if (rw_br_slew < TIMER1_MAX_ARR) {
                        rw_br_slew += slew_step;
                    }
                } else {
                    rw_br_slew = 0;
                }
                if (rw_sine_target && adjusted_input > 48) {
                    // torque assist: integrate current shortfall vs a
                    // throttle-proportional target (10mA units)
                    int32_t br_tgt = map(adjusted_input, 48, 2047,
                        (int32_t)rw_sine_target * 1, (int32_t)rw_sine_target * 5);
                    int32_t br_err = br_tgt - actual_current;
                    rw_br_assist += br_err / 4;
                    if (rw_br_assist < 0) {
                        rw_br_assist = 0;
                    }
                    if (rw_br_assist > (2000 * 8)) {
                        rw_br_assist = 2000 * 8;
                    }
                } else {
                    rw_br_assist = 0;
                }
            }
            // ROCKWOLF v10 diagnostic: track peak current while driving; after
            // the first real run ends, store the peak once into eeprom byte 15
            // (10mA units, capped 250) so the tuner can read it back.
            if (input > 60) {
                if (rw_run_ticks < 60000) {
                    rw_run_ticks++;
                }
                if (actual_current > rw_peak_current) {
                    rw_peak_current = (uint16_t)actual_current;
                }
                if (stepper_sine && rw_sine_cur_cnt < 60000) { // v11: sine-mode average
                    rw_sine_cur_sum += (uint32_t)actual_current;
                    rw_sine_cur_cnt++;
                    if (sine_amp_now > rw_max_amp) {
                        rw_max_amp = sine_amp_now;
                    }
                    if (sine_amp_now < rw_min_amp) { // v17
                        rw_min_amp = sine_amp_now;
                    }
                }
            } else if (input < 48 && !rw_cur_captured && rw_run_ticks > 300 && !running) { // v46: also require commutation stopped
                // v11: peak stored in 0.1A units (cap 25.5A) at byte 15;
                // sine-mode average (0.1A units) borrows unused CAN byte 176
                uint16_t pk10 = rw_peak_current / 10;
                // cap at 254: 255 (0xFF) is the tuner's "never captured" sentinel
                eepromBuffer.reserved_eeprom_3[1] = (pk10 > 254) ? 254 : (uint8_t)pk10;
                if (rw_sine_cur_cnt > 0) {
                    uint32_t av10 = (rw_sine_cur_sum / rw_sine_cur_cnt) / 10;
                    eepromBuffer.can.can_node = (av10 > 254) ? 254 : (uint8_t)av10;
                    uint16_t amp10 = rw_max_amp / 10; // amp rail diagnostic
                    // cap 254: 255 (0xFF) is the tuner's never-captured sentinel
                    eepromBuffer.can.esc_index = (amp10 > 254) ? 254 : (uint8_t)amp10;
                    uint16_t ampmin10 = (rw_min_amp == 0xFFFF) ? 254 : (rw_min_amp / 10); // v17
                    eepromBuffer.can.require_zero_throttle = (ampmin10 > 254) ? 254 : (uint8_t)ampmin10;
                    // v18: what does firmware think the battery is? If this reads
                    // nonsense on a known 2S pack, the volt/current pins are swapped.
                    uint16_t bv10 = battery_voltage / 10; // 0.1V units
                    eepromBuffer.can.filter_hz = (bv10 > 254) ? 254 : (uint8_t)bv10;
                }
                saveEEpromSettings();
                rw_cur_captured = 1;
            }
             
            /* v59: LVC LATCH - protection never reads the pack under load.
               The channel reads HIGH under load (measured +101 mV/A across a loaded
               run, +113 mV/A at constant stick where duty is fixed), so a decision
               taken under load is taken on a number that is too high and the cut
               lands late. Deliberately NOT corrected with k*I: that would wire pack
               protection to the current channel, which read a flat 1.0 A offset
               (v54), then zero for an entire run (v57), then correct (v58) inside
               one morning - a sense fault would stop being a logging annoyance and
               become an unprotected pack. LVC instead samples only at LOW CURRENT:
               200 ms after the throttle returns to neutral the current has
               collapsed, so latch there and hold it. A crawler is at neutral
               constantly, so the latch stays fresh; an ESC that never sees neutral
               keeps the last good value, which fails safe. k*I is for the tuner
               display only, where a wrong number is cosmetic. */
            if (adjusted_input <= 48) {
                if (rw_lvc_neutral_ms < 60000) { rw_lvc_neutral_ms++; }
                if (rw_lvc_neutral_ms >= 200) { rw_lvc_latch_v = battery_voltage; }
            } else {
                rw_lvc_neutral_ms = 0;
            }
            uint16_t rw_lvc_v = (rw_lvc_latch_v != 0) ? rw_lvc_latch_v : battery_voltage;
            if (eepromBuffer.low_voltage_cut_off == 1) {  
                if (rw_lvc_v < (cell_count * low_cell_volt_cutoff)) {
                  low_voltage_count++;
                } else {
                  if(!LOW_VOLTAGE_CUTOFF){  // if set low cutoff has happened, require power cycle to reset
                    low_voltage_count = 0;
                  }
                }
            }
            if (eepromBuffer.low_voltage_cut_off == 2 ){   // absolute cut off
              if (rw_lvc_v <  (eepromBuffer.absolute_voltage_cutoff * 50)) {
                low_voltage_count++;    
                } else {
                  if(!LOW_VOLTAGE_CUTOFF){
                    low_voltage_count = 0;
                  }
                }
            }
            if (low_voltage_count > (10000 - (stepper_sine * 9900))) {      // 10 second wait before cut-off for low voltage
              LOW_VOLTAGE_CUTOFF = 1;
              input = 0;
              allOff();
              maskPhaseInterrupts();
              running = 0;
              zero_input_count = 0;
              armed = 0;
             }
           
            PROCESS_ADC_FLAG = 0;
#ifdef USE_ADC_INPUT
            if (ADC_raw_input < 10) {
                zero_input_count++;
            } else {
                zero_input_count = 0;
            }
#endif
        }
#ifdef USE_ADC_INPUT
        signaltimeout = 0;
        ADC_smoothed_input = (((10 * ADC_smoothed_input) + ADC_raw_input) / 11);
        newinput = ADC_smoothed_input / 2;
        if (newinput > 2000) {
            newinput = 2000;
        }
        input_ready = 1;       
#endif
        stuckcounter = 0;
        if (stepper_sine == 0) {

            e_rpm = running * (600000 / e_com_time); // in tens of rpm
            k_erpm = e_rpm / 10; // ecom time is time for one electrical revolution in microseconds

            if (low_rpm_throttle_limit) { // some hardware doesn't need this, its on
                                          // by default to keep hardware / motors
                                          // protected but can slow down the response
                                          // in the very low end a little.
                duty_cycle_maximum = map(k_erpm, low_rpm_level, high_rpm_level, throttle_max_at_low_rpm,
                    throttle_max_at_high_rpm); // for more performance lower the
                                               // high_rpm_level, set to a
                                               // consvervative number in source.
            }else{
							duty_cycle_maximum = 2000;
						}

            if (degrees_celsius > eepromBuffer.limits.temperature) {
              duty_cycle_maximum = map(degrees_celsius, eepromBuffer.limits.temperature - 10, eepromBuffer.limits.temperature + 10,
                throttle_max_at_high_rpm / 2, 1);
            }
            if (zero_crosses < 100 && commutation_interval > 500) {
              filter_level = 12;
            } else {
              filter_level = map(average_interval, 100, 500, 3, 12);
            }
            if (commutation_interval < 50) {
              filter_level = 2;
            }

            if (eepromBuffer.auto_advance) {
              auto_advance_level = map(duty_cycle, 100, 2000, 13, 23);
            }

            /**************** old routine*********************/
#ifdef CUSTOM_RAMP
            if (old_routine && running) {
                maskPhaseInterrupts();
                getBemfState();
                if (!zcfound) {
                    if (rising) {
                        if (bemfcounter > min_bemf_counts_up) {
                            zcfound = 1;
                            zcfoundroutine();
                        }
                    } else {
                        if (bemfcounter > min_bemf_counts_down) {
                            zcfound = 1;
                            zcfoundroutine();
                        }
                    }
                }
            }
#endif
            if (INTERVAL_TIMER_COUNT > 45000) {
              zero_throttle_brake_active = 0;   // reset zero throttle brake on back emf timeout (rotation stop)
              if(running){
                bemf_timeout_happened++;
                
                temp_comp_pwm = eepromBuffer.comp_pwm;
                maskPhaseInterrupts();
                old_routine = 1;
                if (input < 48) {
                    running = 0;
                    commutation_interval = 5000;
                }
                zero_crosses = 0;
                zcfoundroutine();
              }
            }
        } else { // stepper sine

#ifdef GIMBAL_MODE
            step_delay = 300;
            maskPhaseInterrupts();
            allpwm();
            if (newinput > 1000) {
                desired_angle = map(newinput, 1000, 2000, 180, 360);
            } else {
                desired_angle = map(newinput, 0, 1000, 0, 180);
            }
            if (current_angle > desired_angle) {
                forward = 1;
                advanceincrement();
                delayMicros(step_delay);
                current_angle--;
            }
            if (current_angle < desired_angle) {
                forward = 0;
                advanceincrement();
                delayMicros(step_delay);
                current_angle++;
            }
#else

            if (input > 48 && armed) {
                PROCESS_ADC_FLAG = 1;
                if (input > 48 && input < 137) { // sine wave stepper

                    if (do_once_sinemode) {
                        // disable commutation interrupt in case set
                        DISABLE_COM_TIMER_INT();
                        maskPhaseInterrupts();
                        SET_DUTY_CYCLE_ALL(0);
                        allpwm();
                        do_once_sinemode = 0;
                        sine_amp_trim = 0; // ROCKWOLF v6: fresh entry, no stale trim
                        sine_trim_accum = 0;
                        sine_cur_div = 0;
                    }
                    // ROCKWOLF v4: micro-stepped speed control - fixed 100us tick,
                    // speed set by fractional phase increment (1/256 deg units).
                    // Same physical speed range as the v2 stepper (7000/poles ..
                    // 540/poles us per degree), but with ~256x finer speed
                    // resolution at crawl.
                    // ROCKWOLF v8: progressive (squared) speed curve. A linear
                    // map puts even a barely-cracked stick well up the speed
                    // range; squaring the normalized stick keeps the lower half
                    // of the sine range at true crawl speed while still hitting
                    // full exit speed at the top for a clean handoff.
                    {
                        int32_t x = input - 48; // 0..72 across the stepper region
                        if (x < 0) {
                            x = 0;
                        }
                        if (x > 72) {
                            x = 72;
                        }
                        int32_t inc_min = (25600 * eepromBuffer.motor_poles) / 35000;
                        int32_t inc_max = (25600 * eepromBuffer.motor_poles) / 540;
                        if (rw_sine_mode == 2) {
                            inc_max = inc_max * 6; // full sine: max ceiling, whole stick is sine
                        } else {
                            // v25: top sine speed scales with coverage so rpm-per-stick
                            // stays constant - more coverage buys real speed range
                            // instead of diluting trigger sensitivity (35% = 1x baseline)
                            uint8_t lvl = eepromBuffer.sine_mode_changeover_thottle_level;
                            if (lvl < 20) {
                                lvl = 20;
                            }
                            inc_max = (inc_max * lvl) / 35;
                        }
                        sine_phase_inc = (uint16_t)(inc_min + ((x * x * (inc_max - inc_min)) / 5184));
                    }
                    if (sine_phase_inc < 4) {
                        sine_phase_inc = 4; // keep the accumulator moving
                    }
                    // ROCKWOLF v24: adaptive tick. At high full-sine speeds a 100us
                    // update rate makes the waveform coarse; shorten the tick (and
                    // scale the per-tick increment) so the sine stays smooth.
                    uint16_t rw_tick = 100;
                    uint8_t rw_div = 10;
                    if (sine_phase_inc > 1280) {
                        if (sine_phase_inc > 2560) {
                            rw_tick = 33;
                            rw_div = 30;
                        } else {
                            rw_tick = 50;
                            rw_div = 20;
                        }
                        sine_phase_inc = (uint16_t)(((uint32_t)sine_phase_inc * rw_tick) / 100);
                        if (sine_phase_inc < 4) {
                            sine_phase_inc = 4;
                        }
                    }
                    advanceincrement();
                    step_delay = 25600 / (uint16_t)(((uint32_t)sine_phase_inc * 100) / rw_tick); // telemetry equivalent
                    delayMicros(rw_tick);
                    // ROCKWOLF v6: closed-loop torque - trim sine amplitude to hold
                    // bus current at a throttle-proportional target. Compensates for
                    // BEMF growth with speed and battery sag, so torque stays flat
                    // through the ramp instead of fading toward the handoff.
                    if (rw_sine_target) {
                        if (++sine_cur_div >= rw_div) { // ~1kHz regardless of tick length
                            sine_cur_div = 0;
                            int32_t target_ca = map(input, 48, 137,
                                (int32_t)rw_sine_target * 1, (int32_t)rw_sine_target * 5); // 10mA units (v9: 0.05A steps)
                            int32_t cerr = target_ca - actual_current;
                            // v16: PI with slow integrator. The pure fast integrator
                            // swung rail-to-rail in ~100ms against the lagging
                            // (smoothed) current measurement - a limit cycle. The
                            // integrator now takes ~0.9s full-swing (can't outrun
                            // the lag) and a proportional term damps transients.
                            sine_trim_accum += cerr;
                            if (sine_trim_accum > (2304 * 64)) {
                                sine_trim_accum = 2304 * 64;
                            }
                            if (sine_trim_accum < (-2496 * 64)) { // reach amp floor 64 from power 10
                                sine_trim_accum = -2496 * 64;
                            }
                            sine_amp_trim = (sine_trim_accum / 64) + (cerr / 2);
                        }
                    } else {
                        sine_amp_trim = 0;
                    }
                    e_rpm = 600 / step_delay; // in hundreds so 33 e_rpm is 3300 actual erpm
                    e_com_time = step_delay * 360;

                } else {
                    do_once_sinemode = 1;
                    // ROCKWOLF v30: user-biased handoff speed. Slider shifts the
                    // exit target up (+) or down (-); the stepper ramps toward it
                    // from either direction so the rotor crosses into trap at a
                    // chosen speed instead of a fixed constant.
                    uint16_t exit_cap = (uint16_t)(320 + (int16_t)rw_trap_start * 48); // 80..560
                    uint16_t exit_min = (uint16_t)(240 + (int16_t)rw_trap_start * 36); // 60..420
                    if (sine_phase_inc < exit_cap) {
                        sine_phase_inc += 2;
                    } else if (sine_phase_inc > exit_cap + 4) {
                        sine_phase_inc -= 2; // arriving hot from high coverage: settle down to the target
                    }
                    uint32_t accum_before = sine_phase_accum;
                    advanceincrement();
                    delayMicros(100);
                    // wrap detection: fires exactly once per electrical rev even
                    // when a fractional step jumps past position 0
                    uint8_t crossed_zero = (!forward) ? (sine_phase_accum < accum_before)
                                                      : (sine_phase_accum > accum_before);
                    if (crossed_zero && sine_phase_inc >= exit_min) {
                        stepper_sine = 0;
                        running = 1;
                        old_routine = 1;
                        // v30: seed the trap commutation estimate to match the actual
                        // exit speed instead of assuming the stock constant
                        uint32_t seed = (9000UL * 240) / exit_min;
                        if (seed < 4500) {
                            seed = 4500;
                        }
                        if (seed > 27000) {
                            seed = 27000;
                        }
                        commutation_interval = (uint16_t)seed;
                        average_interval = commutation_interval;
                        last_average_interval = average_interval;
                        SET_INTERVAL_TIMER_COUNT(commutation_interval);
                        zero_crosses = 20;
                        prop_brake_active = 0;
                        step = changeover_step;
                        // comStep(step);// rising bemf on a same as position 0.
                        // ROCKWOLF v3: torque-continuous handoff - enter trap with
                        // duty scaled to the sine drive strength instead of bare minimum
                        // (v6: use the closed-loop effective amplitude, not the raw knob)
                        last_duty_cycle = stall_protect_minimum_duty + ((sine_amp_now * 12) >> 8);
                        commutate();
                        generatePwmTimerEvent();
                    }
                }

            } else {
                running = 0;
                do_once_sinemode = 1;
                sine_amp_trim = 0; // ROCKWOLF v6: reset current loop at zero throttle
                sine_trim_accum = 0;
                if (eepromBuffer.brake_on_stop == 1) {
#ifndef PWM_ENABLE_BRIDGE
                    prop_brake_duty_cycle =  eepromBuffer.drag_brake_strength * 200;
                    adjusted_duty_cycle =  tim1_arr - ((prop_brake_duty_cycle * tim1_arr) / 2000);
                    if(adjusted_duty_cycle < 100){
                      fullBrake();
                    }else{
                      proportionalBrake();
                      SET_DUTY_CYCLE_ALL(adjusted_duty_cycle);
                      prop_brake_active = 1;
                    } 
#else
                    // todo add braking for PWM /enable style bridges.
#endif
                } else if (eepromBuffer.brake_on_stop == 2){
                  comStep(2);
                  SET_DUTY_CYCLE_ALL(DEAD_TIME + ((eepromBuffer.active_brake_power * tim1_arr) / 2000)* 10);
                }else{
                   SET_DUTY_CYCLE_ALL(0);
                   allOff();
                }
                e_rpm = 0;
            }

#endif // gimbal mode
        } // stepper/sine mode end

        if (rw_brushed) { // RW dual-mode
        runBrushedLoop();
    } // RW dual-mode end
#if DRONECAN_SUPPORT
	DroneCAN_update();
#endif
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, tex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
