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

#include "AudioFrequencyMeter.h"
#include "IIRBiquad.h"

#define ARRAY_DEPTH             20
#define NOT_INITIALIZED         -1
#define ADCRESOLUTION           1024
#define BOTTOMPOINT             0
#define TOPPOINT                (ADCRESOLUTION -1)

static bool clipping;
static int clippingPin;

static int newData, prevData;                           // Variables to store ADC result
static mid_t mid = mid_t((TOPPOINT - BOTTOMPOINT) / 2.0);
static ampl_t ampl = 0;
static int samples = 0;


// mid filter C=0.01 (480Hz @ 48kHz)
// http://www.micromodeler.com/dsp/
// Make sure the gain is 1 (sum of coefficients == 1, modeller is not exact at that point)
static iir_biquad<mid_t> mid_filter(
  mid_t(0.939), // a1
  mid_t(0, 0),  // a2
  mid_t(0.9695),// b0
  mid_t(-0.9695),// b1
  mid_t(0, 0)); // b2

AudioFrequencyMeter::AudioFrequencyMeter() {
  initializeVariables();
}

void AudioFrequencyMeter::begin(int pin, unsigned int rate)
{
  samplePin = pin;                              // Store ADC channel to sample
  sampleRate = rate;                            // Store sample rate value
  analogRead(pin);                              // To start setting-up the ADC

  ADCdisable();
  ADCconfigure();
  ADCenable();
  tcConfigure();
  tcEnable();
}

void AudioFrequencyMeter::end()
{
  ADCdisable();
  tcDisable();
  tcReset();
}

void AudioFrequencyMeter::setClippingPin(int pin)
{
  clippingPin = pin;                              // Store the clipping pin value
  pinMode(clippingPin, OUTPUT);
}

void AudioFrequencyMeter::checkClipping()
{
  if (clipping) {
    digitalWrite(clippingPin, LOW);
    clipping = false;
  }
}

mid_t AudioFrequencyMeter::getMid()
{
  // atomic read (64 bit value)
  noInterrupts();
  mid_t val = mid;
  interrupts();
  return val;
}

ampl_t AudioFrequencyMeter::getAmpl()
{
  return ampl;
}

int AudioFrequencyMeter::getData()
{
  return newData;
}

int AudioFrequencyMeter::getSamples(void) {
  int s = samples;
  samples = 0;
  return s;
}
/*
   Private Utility Functions
*/

void AudioFrequencyMeter::initializeVariables()
{
  clipping = false;
  clippingPin = NOT_INITIALIZED;
  newData = 0;
  prevData = 0;
}

void AudioFrequencyMeter::ADCconfigure()
{
  ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
  while (ADCisSyncing())
    ;

  ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV16_Val;     // Divide Clock by 8 -> ~100kHz
  while (ADCisSyncing())
    ;


  while (ADCisSyncing())
    ;

  ADC->SAMPCTRL.reg = 0x1F;                                    // Set max Sampling Time Length
  while (ADCisSyncing())
    ;

  ADCsetMux();
}

bool ADCisSyncing()
{
  return (ADC->STATUS.bit.SYNCBUSY);
}

void AudioFrequencyMeter::ADCdisable()
{
  ADC->CTRLA.bit.ENABLE = 0x00;                               // Disable ADC
  while (ADCisSyncing())
    ;
}

void AudioFrequencyMeter::ADCenable()
{
  ADC->CTRLA.bit.ENABLE = 0x01;                               // Enable ADC
  while (ADCisSyncing())
    ;
}

void AudioFrequencyMeter::ADCsetMux()
{
  if ( samplePin < A0 )
  {
    samplePin += A0;
  }

  pinPeripheral(samplePin, g_APinDescription[samplePin].ulPinType);

  while (ADCisSyncing())
    ;
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[samplePin].ulADCChannelNumber; // Selection for the positive ADC input
}

void AudioFrequencyMeter::tcConfigure()
{
  // Enable GCLK for TCC2 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
  while (GCLK->STATUS.bit.SYNCBUSY);

  tcReset();

  // Set Timer counter Mode to 16 bits
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;

  // Set TC5 mode as match frequency
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;

  TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
  while (tcIsSyncing())
    ;

  sampleRate = SystemCoreClock / (TC5->COUNT16.CC[0].reg + 1);

  // Configure interrupt request
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0x00);
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable the TC5 interrupt request
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  while (tcIsSyncing())
    ;
}

bool AudioFrequencyMeter::tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void AudioFrequencyMeter::tcEnable()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (tcIsSyncing())
    ;
}

void AudioFrequencyMeter::tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing())
    ;
  while (TC5->COUNT16.CTRLA.bit.SWRST)
    ;
}

void AudioFrequencyMeter::tcDisable()
{
  // Disable TC5
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing())
    ;
}

int ADCread()
{
  int returnValue;

  while (ADCisSyncing())
    ;

  ADC->SWTRIG.bit.START = 1;

  while ( ADC->INTFLAG.bit.RESRDY == 0 );   					// Waiting for conversion to complete
  returnValue = ADC->RESULT.reg;            					// Store the value

  while (ADCisSyncing())
    ;

  ADC->SWTRIG.bit.START = 0;

  return returnValue;
} 

void TC5_Handler (void)
{
  samples++;

  prevData = newData;
  newData = ADCread();
  //newData = analogRead(A0);

  mid_filter.put(mid_t(newData, 0));
  mid = mid_filter.get();
  
  // rectifier
  static int rect = 0;
  int amplitude = abs((int)mid);

  rect = rect - 1;
  if (amplitude > rect)
    rect = amplitude;
  else if (rect < 0)
    rect = 0;

  ampl = rect;

  TC5->COUNT16.INTFLAG.bit.MC0 = 1;     // Clear interrupt
}
