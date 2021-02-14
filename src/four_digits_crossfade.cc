/*
  exixe modules:
  https://github.com/dekuNukem/exixe

  library docs:
  https://github.com/dekuNukem/exixe/tree/master/arduino_library

  Based on Demo 3: Loop digits with crossfade animation

  Four digit simple clock
*/

#include <Arduino.h>

#include <RTClibExtended.h>
#include <RotaryEncoder.h>
#include <Wire.h>

#include <Adafruit_BMP085.h>

#include "exixe.h"

#define GPIO04 4
#define GPIO05 5

#define MODE_SWITCH D8
#define ROTARY_CLK D3 // CLOCK
#define ROTARY_DAT D4 // DATA

#define digit_1_cs 1
#define digit_2_cs 3
#define digit_3_cs 10
#define digit_4_cs 16

#define BAUD_RATE 115200

#define INITIAL_FADE_TIME 250             // ms, time to fade in a digit when modes change
#define MEASUREMENT_UPDATE_INTERVAL 10000 // 10s

// Clock digits, left to right
exixe digit_1 = exixe(digit_1_cs);
exixe digit_2 = exixe(digit_2_cs);
exixe digit_3 = exixe(digit_3_cs);
exixe digit_4 = exixe(digit_4_cs);

#define NUM_TUBES 4
#define DIGIT_1 0
#define DIGIT_2 1
#define DIGIT_3 2
#define DIGIT_4 3

exixe *tubes[NUM_TUBES];
unsigned int values[NUM_TUBES];
unsigned int old_values[NUM_TUBES];

// brightness, led_brightness and color_index are adjusted by the
// rotary encoder and can be negative
int brightness = 127;        // tube brightness, 0 - 127
unsigned int fade_time = 15; // 30 frames = 1 second
int led_brightness = 64;     // 0 - 127

#define NUM_COLORS 3
unsigned int red[NUM_COLORS] = {127, 0, 0};
unsigned int green[NUM_COLORS] = {0, 127, 0};
unsigned int blue[NUM_COLORS] = {0, 0, 127};
int color_index = 2;

// Real time clock
RTC_DS3231 RTC; // we are using the DS3231 RTC

enum DisplayMode {
    hours_mins = 0,
    mins_secs = 1,
    temperature = 2,
    pressure = 3
};

enum ControlMode {
    info = 0,
    display_intensity = 1,
    color = 2,
    led_intensity = 3
};

volatile DisplayMode display_mode = hours_mins;

Adafruit_BMP085 bmp;

bool bmp_ok = false; // true after sucessful init

// Setup a RotaryEncoder with 4 steps per latch for the 2 signal input pins:
// RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
RotaryEncoder encoder(ROTARY_CLK, ROTARY_DAT, RotaryEncoder::LatchMode::TWO03);

// Current encoder position
long pos = 0;

volatile bool control_mode_change = false;
volatile unsigned long control_mode_switch_time = 0;
#define MODE_SWITCH_INTERVAL 100 // ms

volatile ControlMode control_mode = info;

ICACHE_RAM_ATTR void mode_switch() {
    cli();
    if (millis() > control_mode_switch_time + MODE_SWITCH_INTERVAL) {
        switch (control_mode) {
        case info:
            control_mode = display_intensity;
            break;
        case display_intensity:
            control_mode = color;
            break;
        case color:
            control_mode = led_intensity;
            break;
        case led_intensity:
            control_mode = info;
            break;
        default:
            break;
        }
        control_mode_switch_time = millis();
    }
    sei();
}

volatile unsigned long rotary_change_time = 0;
#define ROTARY_INTERVAL 3 // ms

ICACHE_RAM_ATTR void rotary_encoder() {
    cli();

    // only record one tick per iteration of the main loop
#if 1
    if (millis() > (rotary_change_time + ROTARY_INTERVAL)
        && encoder.getPosition() == pos) {
        encoder.tick();
        rotary_change_time = millis();
    }
#else
    if (encoder.getPosition() == pos)
        encoder.tick();
#endif

    sei();
}

/**
 * Copy the values[] array to the old_values[] array
 */
void save_values() {
    for (int i = 0; i < NUM_TUBES; ++i) {
        old_values[i] = values[i];
    }
}

/**
 * Set the values[] array.
 */
void set_values(unsigned int left_pair, unsigned int right_pair) {
    values[DIGIT_1] = left_pair / 10;
    values[DIGIT_2] = left_pair - values[DIGIT_1] * 10;
    values[DIGIT_3] = right_pair / 10;
    values[DIGIT_4] = right_pair - values[DIGIT_3] * 10;
}

/**
 * If old_values[] is not equal to values[], return the index of the leftmost
 * digit that differs (1, 2, 3, 4). If old_values[] == values[], return 0;
 */
unsigned int values_changed() {
    for (int i = 0; i < NUM_TUBES; ++i) {
        if (old_values[i] != values[i])
            return NUM_TUBES - i;
    }
    return 0;
}

/**
 * Given an array of the tubes and values, starting the left most digit and
 * going to the rightmost, use the crossfade to update the last 'count' tubes'
 * values.
 * 
 * Only call this function if at least one tube's value should change.
 * 
 * @param tubes An array of exixe pointers - digit_1 == [0], ...
 * @param values An array of digit values
 * @param count the number of tubes/digits to change (1-4), starting
 * with the fourth tube and working backward.
 */
void digit_crossfade(unsigned int count) {

    switch (count) {
    case 4:
        tubes[DIGIT_1]->crossfade_init(values[DIGIT_1], fade_time * 2, brightness, 0);
    case 3:
        tubes[DIGIT_2]->crossfade_init(values[DIGIT_2], fade_time * 2, brightness, 0);
    case 2:
        tubes[DIGIT_3]->crossfade_init(values[DIGIT_3], fade_time, brightness, 0);
    case 1:
        tubes[DIGIT_4]->crossfade_init(values[DIGIT_4], fade_time, brightness, 0);
        break;
    }

    if (display_mode == temperature || display_mode == pressure)
        digit_2.set_dots(0, brightness);
    else
        digit_2.set_dots(0, 0);

    bool done;
    do {
        done = true;
        switch (count) {
        case 4:
            done = (tubes[DIGIT_1]->crossfade_run() == EXIXE_ANIMATION_FINISHED) && done;
            //delay(5);
        case 3:
            done = (tubes[DIGIT_2]->crossfade_run() == EXIXE_ANIMATION_FINISHED) && done;
            //delay(5);
        case 2:
            done = (tubes[DIGIT_3]->crossfade_run() == EXIXE_ANIMATION_FINISHED) && done;
            //delay(5);
        case 1:
            done = (tubes[DIGIT_4]->crossfade_run() == EXIXE_ANIMATION_FINISHED) && done;
            //delay(5);
        }
    } while (!done);
}

void set_digit(int count, exixe *tube, bool fade) {
    if (fade) {
        unsigned int delta = ceil(INITIAL_FADE_TIME / brightness);
        for (int b = 0; b < brightness + 1; b++) {
            tube->show_digit(count, b, 0);
            delay(delta);
        }
    } else {
        tube->show_digit(count, brightness, 0);
    }
}

void display(bool fade = false) {
    set_digit(values[DIGIT_1], tubes[DIGIT_1], fade);
    set_digit(values[DIGIT_2], tubes[DIGIT_2], fade);

    if (display_mode == temperature || display_mode == pressure)
        tubes[DIGIT_2]->set_dots(0, brightness);
    else
        tubes[DIGIT_2]->set_dots(0, 0);

    set_digit(values[DIGIT_3], tubes[DIGIT_3], fade);
    set_digit(values[DIGIT_4], tubes[DIGIT_4], fade);
}

bool check_control_mode_switch() {
    if (control_mode_change) {
        switch (control_mode) {
        case info:
            control_mode = display_intensity;
            break;
        case display_intensity:
            control_mode = color;
            break;
        case color:
            control_mode = led_intensity;
            break;
        case led_intensity:
            control_mode = info;
            break;
        default:
            break;
        }
        control_mode_change = false;
        return true;
    }

    return false;
}

void display_mode_forward() {
    switch (display_mode) {
    case hours_mins:
        display_mode = mins_secs;
        break;
    case mins_secs:
        display_mode = temperature;
        break;
    case temperature:
        display_mode = pressure;
        break;
    case pressure:
        display_mode = hours_mins;
        break;
    default:
        break;
    }
}

void display_mode_backward() {
    switch (display_mode) {
    case hours_mins:
        display_mode = pressure;
        break;
    case mins_secs:
        display_mode = hours_mins;
        break;
    case temperature:
        display_mode = mins_secs;
        break;
    case pressure:
        display_mode = temperature;
        break;
    default:
        break;
    }
}

#define MAX_COLOR_VALUE 127

int red_value = MAX_COLOR_VALUE;
int green_value = 0;
int blue_value = 0;

enum ColorState {
    red_to_yellow = 0,
    yellow_to_green = 1,
    green_to_blue = 2,
    blue_to_red = 3
};

ColorState color_state = red_to_yellow;

void change_color(int steps) {
    switch (color_state) {
    case red_to_yellow:
        red_value = MAX_COLOR_VALUE;
        green_value += steps;
        if (green_value > MAX_COLOR_VALUE) {
            green_value = MAX_COLOR_VALUE;
            color_state = yellow_to_green;
        }
        break;

    case yellow_to_green:
        green_value = MAX_COLOR_VALUE;
        red_value -= steps;
        if (red_value <= 0) {
            red_value = 0;
            color_state = green_to_blue;
        }
        break;

    case green_to_blue:
        red_value = 0;
        green_value -= steps;
        if (green_value < 0)
            green_value = 0;
        blue_value += steps;
        if (blue_value > MAX_COLOR_VALUE) {
            blue_value = MAX_COLOR_VALUE;
            color_state = blue_to_red;
        }
        break;

    case blue_to_red:
        green_value = 0;
        blue_value -= steps;
        if (blue_value < 0)
            blue_value = 0;
        red_value += steps;
        if (red_value > MAX_COLOR_VALUE) {
            red_value = MAX_COLOR_VALUE;
            color_state = red_to_yellow;
        }
        break;
    }
    // 255, green 0 .. 255, blue 0; // red to orange to yellow
    // red 255 .. 0, green 255, blue 0; // yellow to green
    // red 0, green 255 .. 0, blue 0 ..255 // green to blue
    // red 0 .. 255, green 0, blue 255 .. 0 // blue to purple to red again
}

void show_color() {
    float scale_factor = (led_brightness / 127.0);
#if 0
    unsigned int r = red[color_index] * scale_factor;
    unsigned int g = green[color_index] * scale_factor;
    unsigned int b = blue[color_index] * scale_factor;
#else
    unsigned int r = red_value * scale_factor;
    unsigned int g = green_value * scale_factor;
    unsigned int b = blue_value * scale_factor;
#endif

    digit_1.set_led(r, g, b);
    digit_2.set_led(r, g, b);
    digit_3.set_led(r, g, b);
    digit_4.set_led(r, g, b);
}

/**
 * @return temperature in degrees Fahrenheit.
 */
float get_temp() {
    return bmp.readTemperature() * (9.0 / 5.0) + 32.0; // C --> F
}

/**
 * Butte is 1,682 m (5,518 ft)
 * One hPa is 0.0002953 inch Hg
 * @aparam alt Altitude in meters.
 * @return sea level pressure in inches of Hg
 */
float get_sea_level_pressure(float alt = 1682) {
    return bmp.readSealevelPressure(alt) * 0.0002953;
}

/**
 * @return sea level pressure in inches of Hg
 */
float get_pressure() {
    return bmp.readPressure() * 0.0002953;
}

void setup() {
    // MODE_SWITCH is D8 which must be low during boot and is pulled by the switch
    // But using FALLING seems more reliable
    pinMode(MODE_SWITCH, INPUT);
    attachInterrupt(digitalPinToInterrupt(MODE_SWITCH), mode_switch, FALLING);

    // Rotary encoder - INPUT_PULLUP set by the rotary encode ctor
    //pinMode(ROTARY_CLK, INPUT_PULLUP);
    //pinMode(ROTARY_DAT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), rotary_encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_DAT), rotary_encoder, CHANGE);

    pinMode(digit_1_cs, OUTPUT);
    pinMode(digit_2_cs, OUTPUT);
    pinMode(digit_3_cs, OUTPUT);
    pinMode(digit_4_cs, OUTPUT);

    // Setup the I2C bus for the DS3231 clock
    // On the NodeMCU, GPIOR04 is D2, GPIOR05 is D1
    int sda = GPIO04;
    int scl = GPIO05;
    Wire.begin(sda, scl);

    if (bmp.begin()) {
        bmp_ok = true;
    }

    RTC.begin(); // always returns true

#if ADJUST_TIME
    // Run this here, before serial configuration to shorten the delay
    // between the compiled-in times and the set operation.
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));

    DateTime adjusted_time(RTC.now() + TIME_OFFSET);
    RTC.adjust(adjusted_time);
#endif

    // This configures the SPI bus
    digit_1.spi_init();

    digit_1.clear();
    digit_2.clear();
    digit_3.clear();
    digit_4.clear();

    tubes[DIGIT_1] = &digit_1;
    tubes[DIGIT_2] = &digit_2;
    tubes[DIGIT_3] = &digit_3;
    tubes[DIGIT_4] = &digit_4;

    show_color();

    // Initial display; show hours & mins
    DateTime now(RTC.now());
    // record these values for use later to determine when dgits change
    //old_left_pair = now.hour();
    //old_right_pair = now.minute();
    // display(old_left_pair, old_right_pair, true /* fade */);

    set_values(now.hour(), now.minute());
    save_values();

    display(true);
}

unsigned long temperature_update_time = 0;
unsigned long pressure_update_time = 0;

void loop() {

    DateTime now(RTC.now());

    // Test the rotary encoder and update brightness
    long newPos = encoder.getPosition();

    switch (control_mode) {
    case info: {
        if (newPos != pos) {
            if (newPos > pos)
                display_mode_forward();
            else if (newPos < pos)
                display_mode_backward();

            pos = newPos;

            switch (display_mode) {
            case hours_mins:
                set_values(now.hour(), now.minute());
                break;

            case mins_secs:
                set_values(now.minute(), now.second());
                break;

            case temperature: {
                temperature_update_time = millis();
                float temperature = get_temp();
                unsigned int int_temp = trunc(temperature);
                set_values(int_temp, trunc((temperature - int_temp) * 100));
                break;
            }

            case pressure: {
                pressure_update_time = millis();
                float pressure = get_sea_level_pressure(); // get_pressure();
                unsigned int int_press = trunc(pressure);
                set_values(int_press, trunc((pressure - int_press) * 100));
                break;
            }

            default:
                set_values(now.hour(), now.minute());
                break;
            }

            display(true);
            save_values();
        }
        break;
    }

    case display_intensity: {
        if (newPos != pos) {
            brightness += 5 * (newPos - pos);
            if (brightness > 127)
                brightness = 127;
            else if (brightness < 0)
                brightness = 0;

            display(false);

            pos = newPos;
        }
        break;
    }

    case color: {
        if (newPos != pos) {
            change_color((newPos - pos) * 10);
#if 0
            color_index += (newPos - pos);

            if (color_index == NUM_COLORS)
                color_index = 0;
            else if (color_index < 0)
                color_index = NUM_COLORS - 1;
#endif
            show_color();

            pos = newPos;
        }
        break;
    }

    case led_intensity: {
        if (newPos != pos) {
            led_brightness += 5 * (newPos - pos);
            if (led_brightness > 127)
                led_brightness = 127;
            else if (led_brightness < 0)
                led_brightness = 0;

            show_color();

            pos = newPos;
        }
        break;
    }

    default:
        break;
    }

    unsigned int digit = 0;
    switch (display_mode) {
    case hours_mins:
        set_values(now.hour(), now.minute());

        if ((digit = values_changed())) {
            digit_crossfade(digit);
            save_values();
        }
        break;

    case mins_secs:
        set_values(now.minute(), now.second());
        if ((digit = values_changed())) {
            digit_crossfade(digit);
            save_values();
        }
        break;

    case temperature: {
        // less display flicker
        if (temperature_update_time + MEASUREMENT_UPDATE_INTERVAL > millis())
            break;
        temperature_update_time = millis();
        float temperature = get_temp();
        unsigned int int_temp = trunc(temperature);
        set_values(int_temp, trunc((temperature - int_temp) * 100));
        if ((digit = values_changed())) {
            digit_crossfade(digit);
            save_values();
        }
        break;
    }

    case pressure: {
        if (pressure_update_time + MEASUREMENT_UPDATE_INTERVAL > millis())
            break;
        pressure_update_time = millis();
        float pressure = get_sea_level_pressure(); // get_pressure();
        unsigned int int_press = trunc(pressure);
        set_values(int_press, trunc((pressure - int_press) * 100));
        if ((digit = values_changed())) {
            digit_crossfade(digit);
            save_values();
        }
        break;
    }
    default:
        break;
    }

    // noise on the SPI bus can hose the LEDs; refresh them.
    show_color();
}
