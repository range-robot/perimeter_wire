/*
  Audio Frequencimeter library for Arduino Zero.
  Copyright (c) 2015 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
  
  
  Thanks to Amanda Ghassaei
  for the frequency detection algorithm posted on:
  
	http://www.instructables.com/id/Arduino-Frequency-Detection/
	Sept 2012
*/


#include "Arduino.h"
#include "wiring_private.h"
#include "FixedPrecision.h"

#pragma once

typedef fixed_point<int64_t, 16, int64_t> mid_t;
typedef int ampl_t;

bool ADCisSyncing(void);
int ADCread();
    
class AudioFrequencyMeter {
  public:

    AudioFrequencyMeter();

    void begin(int pin, unsigned int sampleRate);
    void end(void);
    
    void setClippingPin(int pin);
    void checkClipping(void);

    void setTimerTolerance(int tolerance);
    void setSlopeTolerance(int tolerance);
 
    mid_t getMid(void);
    int getData(void);
    ampl_t getAmpl(void);
    int getSamples(void);
  private:
    void initializeVariables(void);
    void ADCconfigure();
    void ADCenable(void);
    void ADCdisable(void);
    void ADCsetMux(void);
    
    void tcConfigure(void);
    bool tcIsSyncing(void);
    void tcEnable(void);
    void tcDisable(void);
    void tcReset(void);

  private:
    int samplePin;           // Pin used to sample the signal
  public:
    unsigned int sampleRate; // ADC sample rate
};
