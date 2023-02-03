/*!
 *  @file       pulseGen.cpp
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
#include "pulseGen.h"

//
// Teensyduino
//
#if defined(TEENSYDUINO)
  IntervalTimer _pulseGenTimer1;
  IntervalTimer _pulseGenTimer2;
  IntervalTimer _pulseGenTimer3;
#endif
//
// Seedstudio XIAO M0
//
#if defined(SEEED_XIAO_M0)
  // 24 bits timer
  #include <TimerTCC0.h>
  // uses TimerTcc0
  // 16 bits timer buggued!
  //#include <TimerTC3.h>
  // uses TimerTc3
  //#include <TimerTC1.h>
  // uses TimerTc1
  //#include <TimerTC4.h>
  // uses TimerTc4
#endif
//
// ESP32 family
//
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  hw_timer_t * _pulseGenTimer1 = NULL;
  hw_timer_t * _pulseGenTimer2 = NULL;
  hw_timer_t * _pulseGenTimer3 = NULL;
  portMUX_TYPE _pulseGenTimerMux = portMUX_INITIALIZER_UNLOCKED;
  #define TIMER_ID	  0
  #define TIMER_ID_1	1
  #define TIMER_ID_2	2
#endif

//
// multicore archs
//
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  #define ATOMIC(X) portENTER_CRITICAL_ISR(&_pulseGenTimerMux); X; portEXIT_CRITICAL_ISR(&_pulseGenTimerMux);
//
// singlecore archs
//
#else
  #define ATOMIC(X) noInterrupts(); X; interrupts();
#endif

//
// TIMER_1 setup
//
#if defined(ARDUINO_ARCH_AVR)
void pulseGenInitTimer1()
{
  ATOMIC(
    // 16bits Timer1 init
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    TCNT1  = 0; // initialize counter value to 0
  #if CLOCK_BASE_US == 10
    // operates at 100000hz - 10us
    // set compare match register for 100000 Hz increments
    OCR1A = 159; // = 16000000 / (1 * 100000) - 1 (must be <65536)
    // Set CS12, CS11 and CS10 bits for 1 prescaler
    TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  #elif CLOCK_BASE_US == 100
    // operates at 10000hz - 100us
    // set compare match register for 10000 Hz increments
    OCR1A = 1599; // = 16000000 / (1 * 10000) - 1 (must be <65536)
    // Set CS12, CS11 and CS10 bits for 1 prescaler
    TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  #endif
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
  )
}

#else

  // forward declaration of ISR
  #if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
void ARDUINO_ISR_ATTR pulseGenISR1();
  #else
void pulseGenISR1();
  #endif

void pulseGenInitTimer1()
{
  const long init_clock = CLOCK_BASE_US;

  #if defined(TEENSYDUINO)
  _pulseGenTimer1.begin(pulseGenISR1, init_clock); 
  _pulseGenTimer1.priority(0);
  #endif

  #if defined(SEEED_XIAO_M0)
  TimerTcc0.initialize(init_clock);
  // attach to generic uclock ISR
  TimerTcc0.attachInterrupt(pulseGenISR1);
  #endif

  #if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  _pulseGenTimer1 = timerBegin(TIMER_ID, 80, true);
  // attach to generic uclock ISR
  timerAttachInterrupt(_pulseGenTimer1, &pulseGenISR1, true);
  // init clock tick time
  timerAlarmWrite(_pulseGenTimer1, init_clock, true); 
  // activate it!
  timerAlarmEnable(_pulseGenTimer1);
  #endif
}
#endif

//
// TIMER_2 setup
//
#if defined(ARDUINO_ARCH_AVR)
void pulseGenInitTimer2()
{
  ATOMIC(
  #if defined(__AVR_ATmega32U4__)	
    // avr general timer3 - 16bits
    TCCR3A = 0; // set entire TCCR1A register to 0
    TCCR3B = 0; // same for TCCR1B
    TCNT3  = 0; // initialize counter value to 0
    #if CLOCK_BASE_US == 10
    // set compare match register for 100000 Hz increments
    OCR3A = 159; // = 16000000 / (1 * 100000) - 1 (must be <65536)
    // Set CS12, CS11 and CS10 bits for 1 prescaler
    TCCR3B |= (0 << CS32) | (0 << CS31) | (1 << CS30);
    #elif CLOCK_BASE_US == 100
    // operates at 10000hz - 100us
    // set compare match register for 10000 Hz increments
    OCR3A = 1599; // = 16000000 / (1 * 10000) - 1 (must be <65536)
    // Set CS12, CS11 and CS10 bits for 1 prescaler
    TCCR3B |= (0 << CS32) | (0 << CS31) | (1 << CS30);
    #endif
    // turn on CTC mode
    TCCR3B |= (1 << WGM32);
    // enable timer compare interrupt
    TIMSK3 |= (1 << OCIE3A);
  #else
    // avr general timer2 - 8bits
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0;
    #if CLOCK_BASE_US == 10
    // 100000 Hz (16000000/((4+1)*32))
    OCR2A = 4;
    // Prescaler 32
    TCCR2B |= (1 << CS21) | (1 << CS20);
    #elif CLOCK_BASE_US == 100
    // 10000 Hz (16000000/((24+1)*64))
    OCR2A = 24;
    // Prescaler 64
    TCCR2B |= (1 << CS22);
    #endif
    // CTC
    TCCR2A |= (1 << WGM21);
    // Output Compare Match A Interrupt Enable
    TIMSK2 |= (1 << OCIE2A);
  #endif
  )
}
// ARM timers
#else

  // forward declaration of ISR2
  #if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
void ARDUINO_ISR_ATTR pulseGenISR2();
  #else
void pulseGenISR2();
  #endif

void pulseGenInitTimer2()
{
  const long init_clock = CLOCK_BASE_US;
  #if defined(TEENSYDUINO)
  _pulseGenTimer2.begin(pulseGenISR2, init_clock);
  _pulseGenTimer2.priority(70);
  #endif

  #if defined(SEEED_XIAO_M0) // xiao Tc3 is bugged!
  //TimerTc3.initialize(init_clock);
  // attach to generic uclock ISR
  //TimerTc3.attachInterrupt(pulseGenISR2);
  #endif

  #if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  _pulseGenTimer2 = timerBegin(TIMER_ID, 70, true);
  // attach to generic uclock ISR
  timerAttachInterrupt(_pulseGenTimer2, &pulseGenISR2, true);
  // init clock tick time
  timerAlarmWrite(_pulseGenTimer2, init_clock, true); 
  // activate it!
  timerAlarmEnable(_pulseGenTimer2);
  #endif
}
#endif


//
// TIMER_3 setup
//
#if defined(ARDUINO_ARCH_AVR)
void pulseGenInitTimer3()
{
  ATOMIC(
    // avr general timer0 - 8bits
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0 = 0;
  #if CLOCK_BASE_US == 10
    // 100000 Hz (16000000/((19+1)*8))
    OCR0A = 19;
    // Prescaler 8
    TCCR0B |= (1 << CS01);
  #elif CLOCK_BASE_US == 100
    // 10000 Hz (16000000/((24+1)*64))
    OCR0A = 24;
    // Prescaler 64
    TCCR0B |= (1 << CS01) | (1 << CS00);
  #endif
    // CTC
    TCCR0A |= (1 << WGM01);
    // Output Compare Match A Interrupt Enable
    TIMSK0 |= (1 << OCIE0A);
  )
}
// ARM timers
#else

  // forward declaration of ISR3
  #if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
void ARDUINO_ISR_ATTR pulseGenISR3();
  #else
void pulseGenISR3();
  #endif

void pulseGenInitTimer3()
{
  const long init_clock = CLOCK_BASE_US;

  #if defined(TEENSYDUINO)
  _pulseGenTimer3.begin(pulseGenISR3, init_clock);
  _pulseGenTimer3.priority(60);
  #endif

  #if defined(SEEED_XIAO_M0)
  // sorry no official support yet for other timers
  //TimerTC4.initialize(init_clock);
  // attach to generic uclock ISR
  //TimerTC4.attachInterrupt(pulseGenISR3);
  #endif

  #if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  _pulseGenTimer3 = timerBegin(TIMER_ID, 60, true);
  // attach to generic uclock ISR
  timerAttachInterrupt(_pulseGenTimer3, &pulseGenISR3, true);
  // init clock tick time
  timerAlarmWrite(_pulseGenTimer3, init_clock, true); 
  // activate it!
  timerAlarmEnable(_pulseGenTimer3);
  #endif
}
#endif

namespace pulsegen {

pulseGenClass::pulseGenClass()
{
  onClockStartCallback = nullptr;
  onClockStopCallback = nullptr;
}

void pulseGenClass::start() 
{
  if (state == STOPED) {
    ATOMIC(
      start_timer = 0;
      if (onClockStartCallback) {
        onClockStartCallback();
      } 
      state = RUNNING;
    )
  }
}

void pulseGenClass::stop()
{
  if (state == RUNNING) {
    ATOMIC(
      start_timer = 0;
      ////if (_pulser_high_frequency_enabled) {
        ////resetRuntimeVars();
        //resetVaryingTimePulsesVar();
      ////}
      if (onClockStopCallback) {
        onClockStopCallback();
      }
      state = STOPED;
    )
  }
}

void pulseGenClass::pause() 
{
  if (state == RUNNING) {
    ATOMIC(state = PAUSED)
  } else if (state == PAUSED) {
    ATOMIC(state = RUNNING)
  }
}

void pulseGenClass::setPwmMod(PULSER_ID id, float freq, uint8_t min_dutty, uint8_t max_dutty)
{
  long freq_interval_us = (1.0 / freq) * 1000000;
  long freq_base_us = (1.0 / pulser[id].frequency) * 1000000;

  ATOMIC(
    pulser[id].min_dutty = (freq_base_us * (min_dutty / 100.00)) / CLOCK_BASE_US;
    pulser[id].max_dutty = (freq_base_us * (max_dutty / 100.00)) / CLOCK_BASE_US;

    pulser[id].pwm_mod_freq_counter = (freq_interval_us / CLOCK_BASE_US) / WAVETABLE_SIZE;
    pulser[id].pwm_mod_freq = pulser[id].pwm_mod_freq_counter;
    pulser[id].pwm_mod_ctrl = 0;
    pulser[id].pwm_mod = true;
  )
}

void pulseGenClass::setPulser(PULSER_ID id, float hertz, uint8_t dutty) 
{
  if (hertz < MIN_FREQUENCY || hertz > MAX_FREQUENCY)
    return;

  long freq_interval_us = (1.0 / hertz) * 1000000;

  ATOMIC(
    pulser[id].frequency = hertz;
    pulser[id].dutty = (freq_interval_us * (dutty / 100.00)) / CLOCK_BASE_US;
    pulser[id].dutty_ctrl = pulser[id].dutty;
    pulser[id].freq_counter = freq_interval_us / CLOCK_BASE_US;
    pulser[id].pulse_counter = pulser[id].freq_counter;
  )

  if (pulser[id].active == false) {
    switch(id) {
      case PULSER_1:
          // freq function pins setup
          pinMode(PULSER1_PIN, OUTPUT);
          digitalWrite(PULSER1_PIN, LOW);
          // init hardware timer
          pulseGenInitTimer1();
        break;
      case PULSER_2:
          // freq function pins setup
          pinMode(PULSER2_PIN, OUTPUT);
          digitalWrite(PULSER2_PIN, LOW);
          // init hardware timer
          pulseGenInitTimer2();
        break;
      case PULSER_3:
          // freq function pins setup
          pinMode(PULSER3_PIN, OUTPUT);
          digitalWrite(PULSER3_PIN, LOW);
          // init hardware timer
          pulseGenInitTimer3();
        break;
    }
    // set it active after timer setup finished, avoid reentering here
    pulser[id].active = true;
  } 

}

// this is a block option to go further on high frequencies
void pulseGenClass::setHighFreqPulser() 
{
  if (pulser[PULSER_1].active == false) {
    // freq function pins setup
    pinMode(PULSER1_PIN, OUTPUT);
    digitalWrite(PULSER1_PIN, LOW);
    // in case we want to use pin change on dutty mod
    pinMode(PULSER2_PIN, OUTPUT);
    digitalWrite(PULSER2_PIN, LOW);
    // init hardware timer
    _pulser_high_frequency_enabled = true;
    state = STOPED;
    pulseGenInitTimer1();
  }

  pulser[PULSER_1].active = true;
}

bool pulseGenClass::setFrequency(float hertz) 
{
  if (hertz < MIN_HIGH_FREQUENCY || hertz > MAX_HIGH_FREQUENCY)
    return false;

  pulser[PULSER_1].dutty = _dutty;

  if (setTimerFrequency(hertz)) {
    ATOMIC(pulser[PULSER_1].frequency = hertz)
    setDuttyHelpers(hertz);
    return true;
  }

  return false;
}

void pulseGenClass::setHelperPulsePin(uint8_t pin)
{
  ATOMIC(_pulser_high_frequency_pin = pin)
}

// this is only used by high frequency pulser option
// only timer1
bool pulseGenClass::setTimerFrequency(float hertz) 
{

//
// ARV's implementation
//  
#if defined(ARDUINO_ARCH_AVR)
  uint32_t ocr;
  uint8_t tccr = 0;

  // TIMER_1
  // 16bits avr timer setup
  if ((ocr = AVR_CLOCK_FREQ / ( hertz * 1 )) < 65535) {
    // Set CS12, CS11 and CS10 bits for 1 prescaler
    tccr |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  } else if ((ocr = AVR_CLOCK_FREQ / ( hertz * 8 )) < 65535) {
    // Set CS12, CS11 and CS10 bits for 8 prescaler
    tccr |= (0 << CS12) | (1 << CS11) | (0 << CS10);
  } else if ((ocr = AVR_CLOCK_FREQ / ( hertz * 64 )) < 65535) {
    // Set CS12, CS11 and CS10 bits for 64 prescaler
    tccr |= (0 << CS12) | (1 << CS11) | (1 << CS10);
  } else if ((ocr = AVR_CLOCK_FREQ / ( hertz * 256 )) < 65535) {
    // Set CS12, CS11 and CS10 bits for 256 prescaler
    tccr |= (1 << CS12) | (0 << CS11) | (0 << CS10);
  } else if ((ocr = AVR_CLOCK_FREQ / ( hertz * 1024 )) < 65535) {
    // Set CS12, CS11 and CS10 bits for 1024 prescaler
    tccr |= (1 << CS12) | (0 << CS11) | (1 << CS10);
  } else {
    // frequency not achiavable
    return false;
  }
  ATOMIC(
    TCCR1B = 0;
    OCR1A = ocr-1;
    TCCR1B |= (1 << WGM12);
    TCCR1B |= tccr;
  )

  return true;

//
// ARM's implementation
//  
#else
  long tick_us_interval = (1.0 / hertz) * 1000000;

  #if defined(TEENSYDUINO)
  _pulseGenTimer1.update(tick_us_interval);
  return true;
  #endif

  #if defined(SEEED_XIAO_M0)
  TimerTcc0.setPeriod(tick_us_interval);    
  return true;
  #endif

  #if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  timerAlarmWrite(_pulseGenTimer1, tick_us_interval, true);
  return true;
  #endif
#endif
}

float pulseGenClass::getFrequency(PULSER_ID id) 
{
  return pulser[id].frequency;
}

// elapsed time support
unsigned long pulseGenClass::micros()
{
  if (pulseGen._pulser_high_frequency_enabled == false) {
    unsigned long micros;
    ATOMIC(micros = _micros)
    return micros;
  } else {
    return micros();
  }
}

unsigned long pulseGenClass::millis()
{
  return pulseGen.micros()/1000;
}

uint8_t pulseGenClass::getNumberOfSeconds()
{
  if (state == STOPED) return 0;
  return ((pulseGen.millis() - start_timer) / 1000) % SECS_PER_MIN;
}

uint8_t pulseGenClass::getNumberOfMinutes()
{
  if (state == STOPED) return 0;
  return (((pulseGen.millis() - start_timer) / 1000) / SECS_PER_MIN) % SECS_PER_MIN;
}

uint8_t pulseGenClass::getNumberOfHours()
{
  if (state == STOPED) return 0;
  return (((pulseGen.millis() - start_timer) / 1000) % SECS_PER_DAY) / SECS_PER_HOUR;
}

uint8_t pulseGenClass::getNumberOfDays()
{
  if (state == STOPED) return 0;
  return ((pulseGen.millis() - start_timer) / 1000) / SECS_PER_DAY;
}

void pulseGenClass::resetRuntimeVars()
{
  ATOMIC(
    _use_varying_time_pulses = false;
    _use_dutty_modulation_pulses = false;
    _dsp_pwm_mod_flux_invertion = false;
    _max_dutty = DEFAULT_DUTTY;
    _dutty = DEFAULT_DUTTY;
    _dsp_wavetable_ctrl = 0;
    _dsp_wavetable_ctrl_helper = 0;
    _dsp_wavetable_ctrl_idx = 1;
    _dsp_lock_flux_guard = false;
    _use_dutty_modulation_type = 0;
    _dsp_dutty_modulation_rate = 10; 
    _dsp_dutty_modulation_rate_helper = 0;
    _pulser_high_frequency_pin = 0;
  )
}

void pulseGenClass::resetVaryingTimePulsesVar()
{
  _pulses_bundle_counter = 0;
  _pause_bundle_counter = 0;
  _bundle_counter = 0;
  _pause_bundle_out_counter = 0;
  _pulses_counter = 0;
}

// free cpu from realtime calculus
void pulseGenClass::setDuttyHelpers(float freq)
{
  uint32_t freq_interval_us = (1.00 / freq) * 1000000;
  uint32_t dsp_dutty_modulation_rate = map(_dsp_dutty_modulation_rate, 0, 100, freq, 1);
  
  ATOMIC(
    _dutty_interval_us = (freq_interval_us * (_dutty / 100.00)) - US_SHIFT;
    _dutty_interval_min_us = freq_interval_us * (_min_dutty / 100.00);
    _dutty_interval_max_us = freq_interval_us * (_max_dutty / 100.00);
    _dutty_interval_ms = _dutty_interval_us / 1000;
    _dutty_interval_min_ms = _dutty_interval_min_us / 1000;
    _dutty_interval_max_ms = _dutty_interval_max_us / 1000;
    _dsp_dutty_modulation_rate_helper = dsp_dutty_modulation_rate;
    //_dsp_wavetable_ctrl_helper = 0;
    //_dsp_wavetable_ctrl = 0;
  )
}

void pulseGenClass::setVaryingTime(bool state)
{
  ATOMIC(_use_varying_time_pulses = state)
}

void pulseGenClass::setVaryingTimePackSize(uint8_t size)
{
  ATOMIC(_varying_time_pulses_bundle_pack_size = size)
}

void pulseGenClass::setVaryingTimePulsesInBundle(uint32_t pulses)
{
  ATOMIC(_varying_time_pulses_bundle_size_pulses = pulses)
}

void pulseGenClass::setVaryingTimePulsesInBundlePause(uint32_t pulses)
{
  ATOMIC(_varying_time_pulses_bundle_in_pulses_pause = pulses)
}

void pulseGenClass::setVaryingTimePulsesInPack(uint32_t pulses)
{
  ATOMIC(_varying_time_pulses_bundle_out_pulses_pause = pulses)
}

void pulseGenClass::setPwmModulation(bool state)
{
  ATOMIC(_use_dutty_modulation_pulses = state)
}

void pulseGenClass::setPwmModulationType(uint8_t type)
{
  ATOMIC(_use_dutty_modulation_type = type)
}

void pulseGenClass::setPwmModulationRate(uint32_t rate)
{
  ATOMIC(_dsp_dutty_modulation_rate = rate)
}

void pulseGenClass::setMinDutty(uint8_t dutty)
{
  ATOMIC(_min_dutty = dutty)
}

void pulseGenClass::setMaxDutty(uint8_t dutty)
{
  ATOMIC(_max_dutty = dutty)
}

void pulseGenClass::setDuttyPwm(uint8_t dutty)
{
  ATOMIC(_dutty = dutty)
}

void pulseGenClass::setPwmModFluxInvertion(bool invert_mod)
{
  ATOMIC(_dsp_pwm_mod_flux_invertion = invert_mod)
}

// all timers operate at 10us frequency(100khz)
void pulseGenClass::pulser1Handler()
{
  if (pulseGen.pulser[pulsegen::PULSER_1].pulse_counter == pulseGen.pulser[pulsegen::PULSER_1].freq_counter) {
    if (pulseGen.state == RUNNING)
      digitalWriteFast(PULSER1_PIN, HIGH);

    if (pulseGen.pulser[pulsegen::PULSER_1].pwm_mod == false) {
      // set dutty ctrl
      pulseGen.pulser[pulsegen::PULSER_1].dutty_ctrl = pulseGen.pulser[pulsegen::PULSER_1].dutty;
    } else {
      // modulated PWM?
      pulseGen.pulser[pulsegen::PULSER_1].dutty_ctrl = map(_WAVETABLE_SINE[pulseGen.pulser[pulsegen::PULSER_1].pwm_mod_ctrl], 0, 255, pulseGen.pulser[pulsegen::PULSER_1].min_dutty, pulseGen.pulser[pulsegen::PULSER_1].max_dutty);
    }
  }

  // modulated PWM?
  if (pulseGen.pulser[pulsegen::PULSER_1].pwm_mod == true) {
    if(--pulseGen.pulser[pulsegen::PULSER_1].pwm_mod_freq == 0) {
      if (++pulseGen.pulser[pulsegen::PULSER_1].pwm_mod_ctrl == WAVETABLE_SIZE) {
        pulseGen.pulser[pulsegen::PULSER_1].pwm_mod_ctrl = 0;
      }
      pulseGen.pulser[pulsegen::PULSER_1].pwm_mod_freq = pulseGen.pulser[pulsegen::PULSER_1].pwm_mod_freq_counter;
    }
  }
  
  if (pulseGen.pulser[pulsegen::PULSER_1].dutty_ctrl-- == 0) {
    digitalWriteFast(PULSER1_PIN, LOW);
  }

  if (--pulseGen.pulser[pulsegen::PULSER_1].pulse_counter == 0) {
    pulseGen.pulser[pulsegen::PULSER_1].pulse_counter = pulseGen.pulser[pulsegen::PULSER_1].freq_counter;
  }
}

void pulseGenClass::pulser2Handler()
{
  if (pulseGen.pulser[pulsegen::PULSER_2].pulse_counter == pulseGen.pulser[pulsegen::PULSER_2].freq_counter) {
    if (pulseGen.state == RUNNING)
      digitalWriteFast(PULSER2_PIN, HIGH);

    if (pulseGen.pulser[pulsegen::PULSER_2].pwm_mod == false) {
      // set dutty ctrl
      pulseGen.pulser[pulsegen::PULSER_2].dutty_ctrl = pulseGen.pulser[pulsegen::PULSER_2].dutty;
    } else {
      // modulated PWM?
      pulseGen.pulser[pulsegen::PULSER_2].dutty_ctrl = map(_WAVETABLE_SINE[pulseGen.pulser[pulsegen::PULSER_2].pwm_mod_ctrl], 0, 255, pulseGen.pulser[pulsegen::PULSER_2].min_dutty, pulseGen.pulser[pulsegen::PULSER_2].max_dutty);
    }
  }

  // modulated PWM?
  if (pulseGen.pulser[pulsegen::PULSER_2].pwm_mod == true) {
    if(--pulseGen.pulser[pulsegen::PULSER_2].pwm_mod_freq == 0) {
      if (++pulseGen.pulser[pulsegen::PULSER_2].pwm_mod_ctrl == WAVETABLE_SIZE) {
        pulseGen.pulser[pulsegen::PULSER_2].pwm_mod_ctrl = 0;
      }
      pulseGen.pulser[pulsegen::PULSER_2].pwm_mod_freq = pulseGen.pulser[pulsegen::PULSER_2].pwm_mod_freq_counter;
    }
  }
  
  if (pulseGen.pulser[pulsegen::PULSER_2].dutty_ctrl-- == 0) {
    digitalWriteFast(PULSER2_PIN, LOW);
  }

  if (--pulseGen.pulser[pulsegen::PULSER_2].pulse_counter == 0) {
    pulseGen.pulser[pulsegen::PULSER_2].pulse_counter = pulseGen.pulser[pulsegen::PULSER_2].freq_counter;
  }
}

void pulseGenClass::pulser3Handler()
{
  if (pulseGen.pulser[pulsegen::PULSER_3].pulse_counter == pulseGen.pulser[pulsegen::PULSER_3].freq_counter) {
    if (pulseGen.state == RUNNING)
      digitalWriteFast(PULSER3_PIN, HIGH);

    if (pulseGen.pulser[pulsegen::PULSER_3].pwm_mod == false) {
      // set dutty ctrl
      pulseGen.pulser[pulsegen::PULSER_3].dutty_ctrl = pulseGen.pulser[pulsegen::PULSER_3].dutty;
    } else {
      // modulated PWM?
      pulseGen.pulser[pulsegen::PULSER_3].dutty_ctrl = map(_WAVETABLE_SINE[pulseGen.pulser[pulsegen::PULSER_3].pwm_mod_ctrl], 0, 255, pulseGen.pulser[pulsegen::PULSER_3].min_dutty, pulseGen.pulser[pulsegen::PULSER_3].max_dutty);
    }
  }

  // modulated PWM?
  if (pulseGen.pulser[pulsegen::PULSER_3].pwm_mod == true) {
    if(--pulseGen.pulser[pulsegen::PULSER_3].pwm_mod_freq == 0) {
      if (++pulseGen.pulser[pulsegen::PULSER_3].pwm_mod_ctrl == WAVETABLE_SIZE) {
        pulseGen.pulser[pulsegen::PULSER_3].pwm_mod_ctrl = 0;
      }
      pulseGen.pulser[pulsegen::PULSER_3].pwm_mod_freq = pulseGen.pulser[pulsegen::PULSER_3].pwm_mod_freq_counter;
    }
  }
  
  if (pulseGen.pulser[pulsegen::PULSER_3].dutty_ctrl-- == 0) {
    digitalWriteFast(PULSER3_PIN, LOW);
  }

  if (--pulseGen.pulser[pulsegen::PULSER_3].pulse_counter == 0) {
    pulseGen.pulser[pulsegen::PULSER_3].pulse_counter = pulseGen.pulser[pulsegen::PULSER_3].freq_counter;
  }
}

void pulseGenClass::pulserHighFreqHandler()
{
  if (pulseGen.state != RUNNING)
    return;

  if (_use_varying_time_pulses) {
    // creating solo bundles of varying_time_pulses_bundle_size_pulses pulses size
    if(_pulses_bundle_counter >= _varying_time_pulses_bundle_size_pulses) {
      if (_pause_bundle_counter <= _varying_time_pulses_bundle_in_pulses_pause) {
        if (_pause_bundle_counter == _varying_time_pulses_bundle_in_pulses_pause) {
          _pulses_bundle_counter = 0;
          _pause_bundle_counter = 0;
          ++_bundle_counter;
        } else {
          ++_pause_bundle_counter;
          // holdon no pulses for now
          return;
        }
      }
    }
  
    if (_bundle_counter >= _varying_time_pulses_bundle_pack_size) {
      if (_pause_bundle_out_counter <= _varying_time_pulses_bundle_out_pulses_pause) {
        if (_pause_bundle_out_counter == _varying_time_pulses_bundle_out_pulses_pause) {
          _bundle_counter = 0;
          _pause_bundle_out_counter = 0;
        } else {
          ++_pause_bundle_out_counter;
          // holdon no pulses for now
          return;
        }
      }
    }
  }

  // the pulse    
  // wait the dutty time to be blocked
  _us = _dutty_interval_us;

  digitalWriteFast(_pulser_high_frequency_pin ? PULSER2_PIN : PULSER1_PIN, HIGH);
  //digitalWriteFast(PULSER1_PIN, HIGH);

  // milliseconds
  if (_dutty_interval_ms > 0) {
    for (uint16_t i=0; i < _dutty_interval_ms; i++) {
      delayMicroseconds(1000);
      _us -= 1000;
    }
  }
  // less than 1 millisecond precision
  if (_us > 0) {
    delayMicroseconds(_us);
  }

  digitalWriteFast(_pulser_high_frequency_pin ? PULSER2_PIN : PULSER1_PIN, LOW);
  //digitalWriteFast(PULSER1_PIN, LOW);
  
  ++_pulses_bundle_counter;
  ++_pulses_counter;

  // variable dutty?
  if (_use_dutty_modulation_pulses) {

    // TODO: create a array of waveforms to be selected by the user
    // ...
    
    if (_pulses_counter - _dsp_wavetable_ctrl_helper >= _dsp_dutty_modulation_rate_helper) {
    
      if (_dsp_wavetable_ctrl >= WAVETABLE_SIZE) {
        _dsp_wavetable_ctrl = 0;
      }
    
      if (_dsp_wavetable_ctrl == 0 && _dsp_pwm_mod_flux_invertion) {
        // change direction at min dutty? TODO: check it here!
        _pulser_high_frequency_pin = !_pulser_high_frequency_pin;
        //invertFluxDirection(true);
      }
    
      if (_dutty_interval_ms > 0) {
        _dutty_interval_ms = map(_WAVETABLE_SINE[_dsp_wavetable_ctrl], 0, 255, _dutty_interval_min_ms, _dutty_interval_max_ms);
      }
      if (_dutty_interval_us  > 0) {
        _dutty_interval_us = map(_WAVETABLE_SINE[_dsp_wavetable_ctrl], 0, 255, _dutty_interval_min_us, _dutty_interval_max_us);
      }
    
      _dsp_wavetable_ctrl_helper = _pulses_counter;
      ++_dsp_wavetable_ctrl;
    }
  }

}

} // end namespace pulseGen

pulsegen::pulseGenClass pulseGen;

//
// PULSER_1 INTERRUPT HANDLER 
// 
#if defined(ARDUINO_ARCH_AVR)
ISR(TIMER1_COMPA_vect)
#elif defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
void ARDUINO_ISR_ATTR pulseGenISR1()
#else
void pulseGenISR1() 
#endif
{
  if (pulseGen._pulser_high_frequency_enabled == false) {
    // use timer1 for timming counting
    pulseGen._micros += CLOCK_BASE_US;
    pulseGen.pulser1Handler();
  } else {
    pulseGen.pulserHighFreqHandler();
  }
}

//
// PULSER_2 INTERRUPT HANDLER 
// 
#if defined(ARDUINO_ARCH_AVR)
  #if defined(__AVR_ATmega32U4__)	
ISR(TIMER3_COMPA_vect) 
  #else
ISR(TIMER2_COMPA_vect) 
  #endif
#elif defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
void ARDUINO_ISR_ATTR pulseGenISR2()
#else
void pulseGenISR2()
#endif
{
  pulseGen.pulser2Handler();
}

//
// PULSER_3 INTERRUPT HANDLER 
// 
#if defined(ARDUINO_ARCH_AVR)
ISR(TIMER0_COMPA_vect) 
#elif defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
void ARDUINO_ISR_ATTR pulseGenISR3()
#else
void pulseGenISR3()
#endif
{
  pulseGen.pulser3Handler();
}