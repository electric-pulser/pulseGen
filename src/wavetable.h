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
// 48bytes long
const uint8_t _WAVETABLE_SINE[]={0,1,5,10,18,27,39,52,66,82,98,115,132,149,165,181,196,210,222,233,241,248,252,255,255,252,248,241,233,222,210,196,181,165,149,132,115,98,82,66,52,39,27,18,10,5,1,0};
const uint8_t _WAVETABLE_TRIANGLE[]={0,10,21,32,43,54,65,75,86,97,108,119,130,141,151,162,173,184,195,206,217,227,238,249,249,238,227,217,206,195,184,173,162,151,141,130,119,108,97,86,75,65,54,43,32,21,10,0};
const uint8_t _WAVETABLE_RECTANGLE[]={255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
const uint8_t _WAVETABLE_RAMP[]={0,5,10,16,21,27,32,37,43,48,54,59,65,70,75,81,86,92,97,103,108,113,119,124,130,135,141,146,151,157,162,168,173,179,184,189,195,200,206,211,217,222,227,233,238,244,249,255};

#define WAVETABLE_SIZE  (sizeof(_WAVETABLE_SINE) / sizeof(_WAVETABLE_SINE[0]))

/* 256bits wavetable?
const int8_t SIN256_DATA []  =
        {
                -1, 3, 6, 9, 12, 15, 18, 21, 24, 28,
                31, 34, 37, 40, 43, 46, 48, 51, 54, 57, 60, 63, 65, 68, 71, 73, 76, 78, 81, 83,
                85, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106, 108, 109, 111, 112, 114, 115,
                117, 118, 119, 120, 121, 122, 123, 124, 124, 125, 126, 126, 127, 127, 127, 127,
                127, 127, 127, 127, 127, 127, 127, 126, 126, 125, 124, 124, 123, 122, 121, 120,
                119, 118, 117, 115, 114, 112, 111, 109, 108, 106, 104, 102, 100, 98, 96, 94, 92,
                90, 88, 85, 83, 81, 78, 76, 73, 71, 68, 65, 63, 60, 57, 54, 51, 48, 46, 43, 40,
                37, 34, 31, 28, 24, 21, 18, 15, 12, 9, 6, 3, 0, -4, -7, -10, -13, -16, -19, -22,
                -25, -29, -32, -35, -38, -41, -44, -47, -49, -52, -55, -58, -61, -64, -66, -69,
                -72, -74, -77, -79, -82, -84, -86, -89, -91, -93, -95, -97, -99, -101, -103,
                -105, -107, -109, -110, -112, -113, -115, -116, -118, -119, -120, -121, -122,
                -123, -124, -125, -125, -126, -127, -127, -128, -128, -128, -128, -128, -128,
                -128, -128, -128, -128, -128, -127, -127, -126, -125, -125, -124, -123, -122,
                -121, -120, -119, -118, -116, -115, -113, -112, -110, -109, -107, -105, -103,
                -101, -99, -97, -95, -93, -91, -89, -86, -84, -82, -79, -77, -74, -72, -69, -66,
                -64, -61, -58, -55, -52, -49, -47, -44, -41, -38, -35, -32, -29, -25, -22, -19,
                -16, -13, -10, -7, -4,
        };
*/
#endif /* __WAVETABLE_H__ */


