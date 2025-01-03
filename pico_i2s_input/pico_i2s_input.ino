/*
  This example demonstrates I2S output with the optional MCLK  signal.
  Note: System clock sppeds used here may not be compatible with all other libraries
  requiring specific sys_clks, particularly Pico PIO USB.

  Original  17 November 2016
  by Sandeep Mistry
  modified for RP2040 by Earle F. Philhower, III <earlephilhower@yahoo.com>

    bool setBCLK(pin_size_t pin);
    - This assigns two adjacent pins - the pin after this one (one greater)
      is the WS (word select) signal, which toggles before the sample for
      each channel is sent

    bool setDATA(pin_size_t pin);
    - Sets the DOUT pin, can be any valid GPIO pin


    modified for MCLK by Richard Palmer June 2023
*/

/*
this tests i2s using pico2w and the teensy audio board rev D
*/

#include "control_sgtl5000.h"

#include <I2S.h>

#define FAST_MEM __not_in_flash("mydata")


// Create the I2S port using a PIO state machine
I2S FAST_MEM i2s(INPUT);

// GPIO pin numbers
// #define pDOUT 19
#define pDIN 19
#define pBCLK 20
#define pWS (pBCLK+1)
#define pMCLK 22  // optional MCLK pin

const int FAST_MEM frequency = 50; // frequency of square wave in Hz
const int FAST_MEM sampleRate = 48000; // minimum for many i2s DACs
const int FAST_MEM bitsPerSample = 32;
const int FAST_MEM amplitude = 1 << (bitsPerSample - 2); // amplitude of square wave = 1/2 of maximum

#define MCLK_MUL  256     // depends on audio hardware. Suits common hardware.

const int FAST_MEM halfWavelength = sampleRate / (2 * frequency); // half wavelength of square wave

int32_t FAST_MEM sample = amplitude; // current sample value
int FAST_MEM count = 0;

AudioControlSGTL5000 codecCtl;

// void __isr i2sOutputCallback() {
//   // Serial.println(i2s.availableForWrite());
//   //512 bytes to fill - where does this get set?
//   auto ts = micros();
//   for(size_t i=0;  i < 64; i++) {

//   // while(i2s.availableForWrite() >1) {
//     if (count % halfWavelength == 0) {
//       // invert the sample every half wavelength count multiple to generate square wave
//       sample = -1 * sample;
//     }

//     i2s.write32(sample, sample);

//     // increment the counter for the next sample
//     count++;
//   }
//   auto elapsed = micros() - ts;
//   constexpr double quantumLength = 1.0/((64.0/48000.0) * 1000000.0);
//   float dspload = elapsed * quantumLength;
//   Serial.println(dspload);
//   if (dspload > 0.9) {
//     Serial.println(">90% audio dsp capacity");
//   }
  
// }

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("I2S with MCLK - square wave");

  // set the system clock to a compatible value to the samplerate
  i2s.setSysClk(sampleRate); // best to do this before starting anything clock-dependent



  i2s.setBCLK(pBCLK);
  i2s.setDATA(pDIN);
  i2s.setMCLK(pMCLK); // must be run before setFrequency() and i2s.begin()
  i2s.setMCLKmult(MCLK_MUL); // also before i2s.begin()
  i2s.setBitsPerSample(bitsPerSample);


  i2s.setBuffers(2, 8); //what does this actually do? doesn't seem to affect buffer size of the callback?

  // i2s.onTransmit(i2sOutputCallback);


  // start I2S at the sample rate with 16-bits per sample
  if (!i2s.begin(sampleRate)) {
    Serial.println("Failed to initialize I2S!");
    while (100); // do nothing
  }


  codecCtl.enable();
  codecCtl.volume(0.5);
  codecCtl.inputSelect(AUDIO_INPUT_LINEIN);
  codecCtl.lineInLevel(15);

}

size_t counter=0;
void loop() {
    int32_t l, r;
    i2s.read32(&l, &r);
    // NOTE: Adafruit microphone word size needs to match the BPS above.
    // int32_t l, r;
    // i2s.read32(&l, &r);
    if (counter++ % 1000 == 0) {
      Serial.printf("%d\t%d\n", l, r);  
    }
}
