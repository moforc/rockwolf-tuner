/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "signal.h"
#include "IO.h"
#include "common.h"
#include "dshot.h"
#include "functions.h"
#include "serial_telemetry.h"
#include "sounds.h"
#include "targets.h"
int max_servo_deviation = 250;
int servorawinput;
uint16_t smallestnumber = 20000;
uint8_t enter_calibration_count = 0;
uint8_t calibration_required = 0;
uint8_t high_calibration_counts = 0;
uint8_t high_calibration_set = 0;
uint16_t last_high_threshold = 0;
uint8_t low_calibration_counts = 0;
uint16_t last_input = 0;
char output_timer_prescaler;
uint8_t buffersize = 32;
uint32_t average_signal_pulse;
uint8_t average_count;
uint32_t average_packet_length;
uint16_t dshot_frametime_high = 50000;
uint16_t dshot_frametime_low = 0;

void computeMSInput()
{

    int lastnumber = dma_buffer[0];
    for (int j = 1; j < 2; j++) {

        if (((dma_buffer[j] - lastnumber) < 1500) && ((dma_buffer[j] - lastnumber) > 0)) { // blank space

            newinput = map((dma_buffer[j] - lastnumber), 243, 1200, 0, 2000);
            break;
        }
        lastnumber = dma_buffer[j];
    }
}

// ROCKWOLF v36: noise-immune servo frame validation. A floating (unplugged)
// signal wire picks up junk that can decode as random in-range pulses and drive
// the motor. Real receivers send steady frames; require a run of consecutive
// plausible pulses that don't jump wildly before any pulse is trusted, and go
// back to distrust the moment they turn erratic (which also feeds the normal
// signal-loss timeout).
static uint16_t rw_last_pulse = 0;
static uint8_t rw_good_frames = 0;
// ROCKWOLF v37: per-boot pulse window stats for the boot-failure trace
// (written to the EEPROM trace on a failed-arm boot; read by RockwolfDiag.html)
uint16_t rw_trace_pulse_last = 0;
uint16_t rw_trace_pulse_min = 0xFFFF;
uint16_t rw_trace_pulse_max = 0;
uint8_t rw_trace_good_frames = 0;
#define RW_FRAMES_TO_TRUST 8
#define RW_MAX_JITTER 400 // us between consecutive frames (a real stick can't slam faster)

void computeServoInput()
{
    uint16_t rw_pulse = dma_buffer[1] - dma_buffer[0];
    if (rw_pulse > 800 && rw_pulse < 2200) {
        uint16_t rw_jit = (rw_pulse > rw_last_pulse) ? (rw_pulse - rw_last_pulse) : (rw_last_pulse - rw_pulse);
        rw_last_pulse = rw_pulse;
        rw_trace_pulse_last = rw_pulse; // v37 trace
        if (rw_pulse < rw_trace_pulse_min) { rw_trace_pulse_min = rw_pulse; }
        if (rw_pulse > rw_trace_pulse_max) { rw_trace_pulse_max = rw_pulse; }
        if (rw_jit > RW_MAX_JITTER) {
            rw_good_frames = 0; // erratic - start over
        } else if (rw_good_frames < RW_FRAMES_TO_TRUST) {
            rw_good_frames++;
        }
        rw_trace_good_frames = rw_good_frames; // v37 trace
        if (rw_good_frames < RW_FRAMES_TO_TRUST) {
            zero_input_count = 0; // not trusted yet: neither arms nor drives
            return;
        }
    } else {
        rw_good_frames = 0;
        rw_trace_good_frames = 0; // v37 trace
    }
    if (rw_pulse > 800 && rw_pulse < 2200) {
				signaltimeout = 0;
        if (calibration_required) {
            if (!high_calibration_set) {
                if (high_calibration_counts == 0) {
                    last_high_threshold = dma_buffer[1] - dma_buffer[0];
                }
                high_calibration_counts++;
                if (getAbsDif(last_high_threshold, servo_high_threshold) > 50) {
                    calibration_required = 0;
                } else {
                    servo_high_threshold = ((7 * servo_high_threshold + (dma_buffer[1] - dma_buffer[0])) >> 3);
                    if (high_calibration_counts > 50) {
                        servo_high_threshold = servo_high_threshold - 25;
                        eepromBuffer.servo.high_threshold = (servo_high_threshold - 1750) / 2;
                        high_calibration_set = 1;
                        playDefaultTone();
                    }
                }
                last_high_threshold = servo_high_threshold;
            }
            if (high_calibration_set) {
                if (dma_buffer[1] - dma_buffer[0] < 1250) {
                    low_calibration_counts++;
                    servo_low_threshold = ((7 * servo_low_threshold + (dma_buffer[1] - dma_buffer[0])) >> 3);
                }
                if (low_calibration_counts > 75) {
                    servo_low_threshold = servo_low_threshold + 25;
                    eepromBuffer.servo.low_threshold = (servo_low_threshold - 750) / 2;
                    calibration_required = 0;
                    saveEEpromSettings();
                    low_calibration_counts = 0;
                    playChangedTone();
                }
            }
            signaltimeout = 0;
        } else {
            if (eepromBuffer.bi_direction) {
                if (dma_buffer[1] - dma_buffer[0] <= servo_neutral) {
                    servorawinput = map((dma_buffer[1] - dma_buffer[0]),
                        servo_low_threshold, servo_neutral, 0, 1000);
                } else {
                    servorawinput = map((dma_buffer[1] - dma_buffer[0]), servo_neutral + 1,
                        servo_high_threshold, 1001, 2000);
                }
            } else {
                servorawinput = map((dma_buffer[1] - dma_buffer[0]), servo_low_threshold,
                    servo_high_threshold, 47, 2047);
                if (servorawinput <= 48) {
                    servorawinput = 0;
                }
            }
            signaltimeout = 0;
        }
    } else {
        zero_input_count = 0; // reset if out of range
    }

    if (servorawinput - newinput > max_servo_deviation) {
        newinput += max_servo_deviation;
    } else if (newinput - servorawinput > max_servo_deviation) {
        newinput -= max_servo_deviation;
    } else {
        newinput = servorawinput;
    }
}

void transfercomplete()
{
    // ROCKWOLF v39: DShot decode/telemetry removed. The input DMA path
    // (receiveDshotDma + dma_buffer) is shared with servo PWM and stays.
    if (inputSet == 0) {
        detectInput();
        receiveDshotDma();
        return;
    }
    if (inputSet == 1) {

        if (servoPwm == 1) {
            if (getInputPinState()) {
                buffersize = 3;
            } else {
                buffersize = 2;
                computeServoInput();
            }
            receiveDshotDma();
        }
        if (!armed) {
            if (adjusted_input == 0 && calibration_required == 0) { // note this in input..not newinput so it
                                                                    // will be adjusted be main loop
                zero_input_count++;
            } else {
              if(!eepromBuffer.disable_stick_calibration){
                zero_input_count = 0;
                if (adjusted_input > 1500) {
                    if (getAbsDif(adjusted_input, last_input) > 50) {
                        enter_calibration_count = 0;
                    } else {
                        enter_calibration_count++;
                    }

                    if (enter_calibration_count > 50 && (!high_calibration_set)) {
                        playBeaconTune3();
                        calibration_required = 1;
                        enter_calibration_count = 0;
                    }
                    last_input = adjusted_input;
                }
              }
            }
        }
    }
}

void checkDshot()
{
    if ((smallestnumber >= 1) && (smallestnumber < 4) && (average_signal_pulse < 60)) {
        ic_timer_prescaler = 0;
        if (CPU_FREQUENCY_MHZ > 100) {
            output_timer_prescaler = 1;
        } else {
            output_timer_prescaler = 0;
        }
        //	dshot_runout_timer = 1000;
        dshot = 1;
        buffer_padding = 14;
        buffersize = 32;
        inputSet = 1;
    }
    if ((smallestnumber >= 4) && (smallestnumber <= 8) && (average_signal_pulse < 100)) {
        dshot = 1;
        ic_timer_prescaler = 1;
        if (CPU_FREQUENCY_MHZ > 100) {
            output_timer_prescaler = 3;
        } else {
            output_timer_prescaler = 1;
        }
        buffer_padding = 7;
        buffersize = 32;
        inputSet = 1;
    }
}
void checkServo()
{
    if (smallestnumber > 200 && smallestnumber < 20000) {
        servoPwm = 1;
        ic_timer_prescaler = CPU_FREQUENCY_MHZ - 1;
        buffersize = 2;
        inputSet = 1;
    }
}

void detectInput()
{
    smallestnumber = 20000;
    average_signal_pulse = 0;
    int lastnumber = dma_buffer[0];
    for (int j = 1; j < 31; j++) {
        if (dma_buffer[j] - lastnumber > 0) {
            if ((dma_buffer[j] - lastnumber) < smallestnumber) {
                smallestnumber = dma_buffer[j] - lastnumber;
            }
            average_signal_pulse += (dma_buffer[j] - lastnumber);
        }
        lastnumber = dma_buffer[j];
    }
    average_signal_pulse = average_signal_pulse / 32;

    // ROCKWOLF v39: DShot detection removed - servo PWM is the only input mode.
    if (servoPwm == 1) {
        checkServo();
    }

    if (!dshot && !servoPwm) {
        checkServo();
    }
}
