

/*
this tests i2s using pico2w and the teensy audio board rev D
*/

#include "control_sgtl5000.h"
#include "maximilian.h"

// extern "C" {
#include "i2scode.h"
// }

#define FAST_MEM __not_in_flash("mydata")

const int FAST_MEM frequency = 200; // frequency of square wave in Hz
const int FAST_MEM sampleRate = 48000; // minimum for many i2s DACs
const int FAST_MEM bitsPerSample = 32;
const int FAST_MEM amplitude = 1 << (bitsPerSample - 2); // amplitude of square wave = 1/2 of maximum

const int FAST_MEM halfWavelength = sampleRate / (2 * frequency); // half wavelength of square wave

int32_t FAST_MEM sample = amplitude; // current sample value
int FAST_MEM count = 0;

AudioControlSGTL5000 codecCtl;


static __attribute__((aligned(8))) pio_i2s i2s;

maxiOsc osc, osc2;

float f1=20, f2=2000;

static void process_audio(const int32_t* input, int32_t* output, size_t num_frames) {
    // Just copy the input to the output
    for (size_t i = 0; i < num_frames; i++) {
        // output[i] = input[i];

        // output[i*2] = input[i*2];
        // output[(i*2) + 1] = input[(i*2) + 1];

        // output[i*2] = 0;
        // output[(i*2) + 1] = 0;

        // if (count % halfWavelength == 0) {
        //   // invert the sample every half wavelength count multiple to generate square wave
        //   sample = -1 * sample;
        // }

        output[i*2] = osc.sinewave(f1) * sample;
        output[(i*2) + 1] = osc2.sinewave(f2) * sample;
        // count++;
        f1 *= 1.00001;
        f2 *= 1.00001;
        if (f1 > 15000) {
          f1 = 20.0;
        }
        if (f2 > 15000) {
          f2 = 20.0;
        }

    }
}

static void dma_i2s_in_handler(void) {
    /* We're double buffering using chained TCBs. By checking which buffer the
     * DMA is currently reading from, we can identify which buffer it has just
     * finished reading (the completion of which has triggered this interrupt).
     */
    if (*(int32_t**)dma_hw->ch[i2s.dma_ch_in_ctrl].read_addr == i2s.input_buffer) {
        // It is inputting to the second buffer so we can overwrite the first
        process_audio(i2s.input_buffer, i2s.output_buffer, AUDIO_BUFFER_FRAMES);
    } else {
        // It is currently inputting the first buffer, so we write to the second
        process_audio(&i2s.input_buffer[STEREO_BUFFER_SIZE], &i2s.output_buffer[STEREO_BUFFER_SIZE], AUDIO_BUFFER_FRAMES);
    }
    dma_hw->ints0 = 1u << i2s.dma_ch_in_data;  // clear the IRQ
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);  
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("I2S test");

  maxiSettings::setup(48000, 2, 64);


  set_sys_clock_khz(132000, true);
  // set_sys_clock_khz(129600, true);
  Serial.printf("System Clock: %lu\n", clock_get_hz(clk_sys));

  //init i2s
  /*
  typedef struct i2s_config {
    uint32_t fs;
    uint32_t sck_mult;
    uint8_t  bit_depth;
    uint8_t  sck_pin;
    uint8_t  dout_pin;
    uint8_t  din_pin;
    uint8_t  clock_pin_base;
    bool     sck_enable;
} i2s_config;
  const i2s_config i2s_config_default = {48000, 256, 32, 
  10, //sck
  6,  //dout
  7, //din
  8, clock base bclk (9: lrclock)
   
  true};
*/
  i2s_config picoI2SConfig = i2s_config_default;
  picoI2SConfig.fs = sampleRate;
  picoI2SConfig.bit_depth = bitsPerSample;
  picoI2SConfig.sck_mult=512;
  i2s_program_start_synched(pio0, &picoI2SConfig, dma_i2s_in_handler, &i2s);

  // init i2c
  codecCtl.enable();
  codecCtl.volume(1);
  codecCtl.inputSelect(AUDIO_INPUT_LINEIN);
  codecCtl.lineInLevel(0);
  codecCtl.audioProcessorDisable();

  

}

void loop() {
}
