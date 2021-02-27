/**
 * Rotary encoder polling code adapted from
 * https://www.best-microcontroller-projects.com/rotary-encoder.html
 */

#include <Arduino.h>

#if 0
//BUILD_NANO
typedef unsigned short uint16_t;
typedef char int8_t;
typedef unsigned char uint8_t;
#endif

static long counter = 0;
static uint16_t clock_pin;
static uint16_t data_pin;

/**
 * Configure the rotary encode clock and data pins.
 */
void rotary_encoder_setup(uint16_t clk, uint16_t data) {
    clock_pin = clk;
    data_pin = data;
    pinMode(clock_pin, INPUT_PULLUP);
    pinMode(data_pin, INPUT_PULLUP);
}

uint16_t rotary_encoder_counter() {
    return counter;
}

/**
 * Read the rotary encoder and return +/-1 or 0, depending on
 * its state. +/-1 indicate CW or CCW rotation of one stop, 0
 * is no rotation.
 * 
 * This is based on a table of valid state transitions for a
 * rotary encoder (given that it output grey codes). In the 
 * rot_enc_table, a 0 is an invalid state and a 1 is a valid 
 * state. By eliminating invalid states the code is able to
 * 'debounce' the encoder by detecting not only a valid end
 * position, but a valid rotation. NB: A rotary encoder goes
 * through 4 states for a single detent. 
 * 
 * prevNextCode stores the previous and current (next) states 
 * of the data and clock pins in the low 4 bits. The rot_enc_table
 * then returns if the set of previous and 'next' states are 
 * possible given that the encoder returns grey code.
 */
int8_t read_rotary() {
    static int8_t rot_enc_table[] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};
    static uint8_t prevNextCode = 0;
    static uint16_t store = 0;

    // prevNextCode stores the previous and current states of the
    // data and clock pins in the low 4 bits.
    prevNextCode <<= 2;
    if (digitalRead(data_pin))
        prevNextCode |= 0x02;
    if (digitalRead(clock_pin))
        prevNextCode |= 0x01;
    prevNextCode &= 0x0f;

    // If valid then store as 16 bit data.
    if (rot_enc_table[prevNextCode]) {
        store <<= 4;
        store |= prevNextCode;
        if ((store & 0xff) == 0x2b)
            return -1;
        if ((store & 0xff) == 0x17)
            return 1;
    }
    return 0;
}

/**
 * Return the state of the rotary encoder. Each call to poll()
 * reads one detent of the encoder.
 */
long rotary_encoder_poll() {
    long v = 0;
    if ((v = read_rotary())) {
        counter += v;
    }
    return counter;
}