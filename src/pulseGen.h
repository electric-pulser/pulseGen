/*!
 *  @file       pulseGen.h
 *  Project     Pulse generator for Arduino using hardware timer
 *  @brief      A Library to implement a pulse generator using hardware timer interruption. Supports ATmega168/328, ATmega16u4/32u4, ATmega2560, Teensy and Seedstudio XIAO M0
 *  @version    1.0.0
 *  @author     Electric Pulser
 *  @date       10/21/2022
 *  @license    MIT - (c) 2022 - Electric Pulser - electric-pulser@proton.me
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE. 
 */

#ifndef __U_PULSEGEN_H__
#define __U_PULSEGEN_H__

#include <Arduino.h>
#include <inttypes.h>
#include <digitalWriteFast.h>

#include "wavetable.h"

#define CLOCK_BASE_US   100

namespace pulsegen {

// pins setup for fastWrite usage
//#define PULSER1_PIN    3
//#define PULSER2_PIN    9
//#define PULSER3_PIN    4
#define PULSER1_PIN    9
#define PULSER2_PIN    8
#define PULSER3_PIN    7

// minimun for AVRs 
#define MIN_FREQUENCY  0.1
#define MAX_FREQUENCY 50000 // 2.5kHz x 100

// from 5khz on we get dutty problems...

// for high frequency blocking pulser
#define MIN_HIGH_FREQUENCY  0.3
#define MAX_HIGH_FREQUENCY 50000 // 50kHz

// XIAO TC3 library is legacy and bugged for setPeriod() only works initialize()
// https://forum.arduino.cc/t/i-have-trouble-using-arduinozeros-timer-tc3/1012469/17
// TODO: write port based on setup for registers instead of those libraries

// want a different avr clock support?
#define AVR_CLOCK_FREQ	16000000

#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)

#define MAX_PULSERS    3

#define DEFAULT_DUTTY            50 // 50%

// pulser id
typedef enum {
  PULSER_1,
  PULSER_2,
  PULSER_3,
  PULSER_4,
} PULSER_ID;

typedef enum {
  PAUSED,
  RUNNING,
  STOPED,
} RUN_STATE;

typedef struct
{
  bool active = false;
  RUN_STATE state = STOPED;
  float frequency;
  long freq_counter;
  long pulse_counter;
  long dutty;
  long dutty_ctrl;
} PULSER_CTRL;

class pulseGenClass {

  public:

    volatile uint32_t start_timer = 0;
    volatile uint8_t state = STOPED;
    volatile unsigned long _micros = 0;
    volatile bool _pulser_high_frequency_enabled = false;
    volatile PULSER_CTRL pulser[MAX_PULSERS];

		void (*onClockStartCallback)();
		void (*onClockStopCallback)();

		void setOnClockStartOutput(void (*callback)()) {
			onClockStartCallback = callback;
		}

		void setOnClockStopOutput(void (*callback)()) {
			onClockStopCallback = callback;
		}

    pulseGenClass();
    
    // external class control
    void start();
    void stop();
    void pause();

    void setPulser(PULSER_ID id, float hertz, uint8_t dutty);
    void setHighFreqPulser(float hertz, uint8_t dutty);
    float getFrequency(PULSER_ID id = PULSER_1);

    void pulser1Handler();
    void pulser2Handler();
    void pulser3Handler();
    void pulserHighFreqHandler();

    void setVaryingTime(bool state);
    void setVaryingTimePackSize(uint8_t size);
    void setVaryingTimePulsesInBundle(uint32_t pulses);
    void setVaryingTimePulsesInBundlePause(uint32_t pulses);
    void setVaryingTimePulsesInPack(uint32_t pulses);
    void setPwmModulation(bool state);
    void setPwmModulationType(uint8_t type);
    void setPwmModulationRate(uint32_t rate);
    void setMinDutty(uint8_t dutty);
    void setMaxDutty(uint8_t dutty);
    void setDuttyPwm(uint8_t dutty);
    void setPwmModFluxInvertion(bool invert_mod);

    // elapsed time support
    unsigned long micros();
    // total milliseconds since start of clock
    unsigned long millis();
    // for clock style usage
    // seconds overflow at 60
    uint8_t getNumberOfSeconds();
    // minutes overflow at 60
    uint8_t getNumberOfMinutes();
    // hours overflow at 60
    uint8_t getNumberOfHours();
    // days dont overflow
    uint8_t getNumberOfDays();

  private:
    
    bool setTimerFrequency(float hertz);
    void resetRuntimeVars();
    void resetVaryingTimePulsesVar();
    void setDuttyHelpers(float freq);

    // we can use a potentiometer to control pulse duration per sec
    // and/or use it to control frequency in manual mode.

    // features control
    volatile uint8_t _dutty = DEFAULT_DUTTY;
    volatile uint8_t _min_dutty = 1;   // 1% dutty
    volatile uint8_t _max_dutty = DEFAULT_DUTTY;
    // pwm dutty helper
    volatile uint32_t _dutty_interval_us;
    volatile uint32_t _dutty_interval_min_us;
    volatile uint32_t _dutty_interval_max_us;
    volatile uint16_t _dutty_interval_ms;
    volatile uint16_t _dutty_interval_min_ms;
    volatile uint16_t _dutty_interval_max_ms;
    volatile uint32_t _us;
    volatile uint16_t _ms;

    // pwm dutty modulation
    volatile bool _use_dutty_modulation_pulses = false;
    volatile uint8_t _use_dutty_modulation_type = 0;
    volatile uint8_t _dsp_dutty_modulation_rate = 10; 
    volatile uint32_t _dsp_dutty_modulation_rate_helper = 0; // pulses per wavetable step
    volatile bool _dsp_pwm_mod_flux_invertion = false;
    //volatile uint16_t _use_dutty_modulation_type = sawtooth | triangule | random;
    // time modulation
    volatile bool _use_varying_time_pulses = false;
    volatile uint32_t _varying_time_pulses_bundle_size_pulses = 2000; // how many pulses per 1 bundle?
    volatile uint32_t _varying_time_pulses_bundle_in_pulses_pause = 1000; // how many pulses pauses between bundles
    volatile uint32_t _varying_time_pulses_bundle_out_pulses_pause = 10000; // how many pulses pauses between group of bundles
    volatile uint32_t _varying_time_pulses_bundle_pack_size = 5;  // size of a pack of bundles

    // features control data
    volatile uint16_t _dsp_wavetable_ctrl = 0;
    volatile uint32_t _dsp_wavetable_ctrl_helper = 0;
    volatile int8_t _dsp_wavetable_ctrl_idx = 1;
    volatile bool _dsp_lock_flux_guard = false;

    volatile uint32_t _pulses_counter = 0;
    volatile uint32_t _pulses_bundle_counter = 0;
    volatile uint32_t _pause_bundle_counter = 0;
    volatile uint32_t _bundle_counter = 0;
    volatile uint32_t _pause_bundle_out_counter = 0;
};

}

extern pulsegen::pulseGenClass pulseGen;

extern "C" {
  extern volatile long _timer;
}

#endif /* __U_PULSEGEN_H__ */


