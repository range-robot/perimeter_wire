/*
  Simple Frequency Meter for Arduino Zero

  Demonstrates how to sample an input signal and get back its frequency

  This example code is in the public domain

  http://arduino.cc/en/Tutorial/SimpleAudioFrequencyMeter

  created by Arturo Guadalupi <a.guadalupi@arduino.cc>
  10 Nov 2015
*/

#include "AudioFrequencyMeter.h"
static int time;

AudioFrequencyMeter meter;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("started");

  meter.begin(A0, 48000);             // Intialize A0 at sample rate of 45kHz

  time = micros();
}

void loop() {
  double dt = (micros() - time) / 1000000.0;
  time = micros();
  
  // put your main code here, to run repeatedly:
  /*
  Serial.print(time);
  Serial.print(" ");
  Serial.print(meter.getData());
  Serial.print(" ");
  */
  mid_t mid = meter.getMid();
  Serial.print((int)mid);
  Serial.print(" ");
  Serial.print((float)meter.getAmpl());
  Serial.print(" ");

  /*
  int samples = meter.getSamples();
  Serial.print(samples);
  Serial.print(" ");
  Serial.println(((double)samples) / dt);
  */
  Serial.println();
}
