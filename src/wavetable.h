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

#ifndef __WAVETABLE_H__
#define __WAVETABLE_H__

#include <inttypes.h>

// 8bit lookuptables for wave shapes for modulation pruporses
//const volatile uint8_t Ramp[]={0,5,10,16,21,27,32,37,43,48,54,59,65,70,75,81,86,92,97,103,108,113,119,124,130,135,141,146,151,157,162,168,173,179,184,189,195,200,206,211,217,222,227,233,238,244,249,255};
//const volatile uint8_t InvRamp[]={255,249,244,238,233,227,222,217,211,206,200,195,189,184,179,173,168,162,157,151,146,141,135,130,124,119,113,108,103,97,92,86,81,75,70,65,59,54,48,43,37,32,27,21,16,10,5,0};
//const volatile uint8_t Triangle[]={0,10,21,32,43,54,65,75,86,97,108,119,130,141,151,162,173,184,195,206,217,227,238,249,249,238,227,217,206,195,184,173,162,151,141,130,119,108,97,86,75,65,54,43,32,21,10,0};
//const volatile uint8_t Rect[]={255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//const volatile uint8_t InvRect[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255};
//const volatile uint8_t _sine[]={0,1,5,10,18,27,39,52,66,82,98,115,132,149,165,181,196,210,222,233,241,248,252,255,255,252,248,241,233,222,210,196,181,165,149,132,115,98,82,66,52,39,27,18,10,5,1,0};
//const volatile uint8_t InvSine[]={255,254,250,245,237,228,216,203,189,173,157,140,123,106,90,74,59,45,33,22,14,7,3,0,0,3,7,14,22,33,45,59,74,90,106,123,140,157,173,189,203,216,228,237,245,250,254,255};

#endif /* __WAVETABLE_H__ */


