// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

// ; I2S audio output block. Synchronous with clock and input.
// ; Must run at BCK * 2.
// ;
// ; This block also outputs the word clock (also called frame or LR clock) and
// ; the bit clock.
// ;
// ; Set register x to (bit depth - 2) (e.g. for 24 bit audio, set to 22).
// ; Note that if this is needed to be synchronous with the SCK module,
// ; it is not possible to run 24-bit frames with an SCK of 256x fs. You must either
// ; run SCK at 384x fs (if your codec permits this) or use 32-bit frames, which
// ; work fine with 24-bit codecs.


#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// -------------- //
// i2s_out_master //
// -------------- //

#define i2s_out_master_wrap_target 0
#define i2s_out_master_wrap 7

#define i2s_out_master_offset_entry_point 0u

static const uint16_t i2s_out_master_program_instructions[] = {
            //     .wrap_target
    0xe03e, //  0: set    x, 30           side 0     
    0x8880, //  1: pull   noblock         side 1     
    0x6001, //  2: out    pins, 1         side 0     
    0x0842, //  3: jmp    x--, 2          side 1     
    0xf03e, //  4: set    x, 30           side 2     
    0x9880, //  5: pull   noblock         side 3     
    0x7001, //  6: out    pins, 1         side 2     
    0x1846, //  7: jmp    x--, 6          side 3     
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program i2s_out_master_program = {
    .instructions = i2s_out_master_program_instructions,
    .length = 8,
    .origin = -1,
};

static inline pio_sm_config i2s_out_master_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + i2s_out_master_wrap_target, offset + i2s_out_master_wrap);
    sm_config_set_sideset(&c, 2, false, false);
    return c;
}
#endif
