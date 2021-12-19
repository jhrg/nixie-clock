/*
  Four digit clock. Hour/Mins, Mins/Seconds, Temp, Sea-level Pressure
  
  exixe modules:
  https://github.com/dekuNukem/exixe

  library docs:
  https://github.com/dekuNukem/exixe/tree/master/arduino_library

  Based on Demo 3: Loop digits with crossfade animation

  jhrg
*/

#include <Arduino.h>

#include <EEPROM.h>

#include <PinChangeInterrupt.h>
#include <RTClibExtended.h>
#include <Rotary.h>
#include <Wire.h>

#include "Adafruit_BMP085.h"
#include "exixe.h"

// Override these in platformio.ini
#ifndef ROTARY_SWITCH_REVERSED
#define ROTARY_SWITCH_REVERSED 0
#endif

#ifndef ADJUST_TIME
#define ADJUST_TIME 0
#endif

#ifndef TIME_OFFSET
#define TIME_OFFSET 47
#endif

#ifndef RESET_PARAMS
#define RESET_PARAMS 0
#endif

// Define GPIO pins

#define TWI_SDA A5
#define TWI_SCL A4

#define MODE_SWITCH 2 // Promini INT0
#define ROTARY_CLK 5  // CLOCK
#define ROTARY_DAT 6  // DATA

// At some point, this could be made smart, but for now
// it is set low (enabled) at startup. 
#define HV_PS_ENABLE 4 // Turn on the HV (180V) power supply

#define digit_1_cs 7  // MSD hours
#define digit_2_cs 8  // hours
#define digit_3_cs 9  // MSD minutes
#define digit_4_cs 10 // minutes

// SPI bus is 10 SS, 11 MOSI, 12 MISO, 13 CLK

#define BAUD_RATE 9600

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

unsigned int left_dot[NUM_TUBES];
unsigned int right_dot[NUM_TUBES];
unsigned int values[NUM_TUBES];
unsigned int old_values[NUM_TUBES];

unsigned int fade_time = 7; // 30 frames = 1 second

// brightness, led_brightness and color_values are adjusted by the
// rotary encoder and can be negative. These values are saved in
// EEPROM so they carry over even when the clock is off.

#define SAVED_STATE_ADDRESS 0
#define MAX_COLOR_VALUE 127
#define MAX_BRIGHTNESS 127

struct SavedState {
    int brightness = MAX_BRIGHTNESS;         // tube brightness, 0 - 127
    int led_brightness = MAX_BRIGHTNESS / 2; // 0 - 127

    int red_value = MAX_COLOR_VALUE;
    int green_value = 0;
    int blue_value = 0;
} saved_state;

// Real time clock
RTC_DS3231 RTC; // we are using the DS3231 RTC

/**
 * @brief State of the display
 */
enum DisplayMode {
    off = 0,
    hours_mins = 1,
    mins_secs = 2,
    temperature = 3,
    pressure = 4,
    set_time = 5
};

volatile DisplayMode display_mode = hours_mins;

Adafruit_BMP085 bmp;

// Current encoder position
volatile long encoder_position = 0;
// Last encoder position the code used. This is updated once any
// change in the encoder is processesed. Not modified by an ISR.
long old_encoder_position = 0;

Rotary rotary_encoder = Rotary(ROTARY_CLK, ROTARY_DAT);

/** 
 * @brief State of the push button control
 */
enum ControlMode {
    info = 0,
    display_intensity = 1,
    color = 2,
    led_intensity = 3,
    set_hours = 4,
    set_minutes = 5
};

// volatile bool control_mode_change = false;
volatile unsigned long control_mode_switch_time = 0;
#define MODE_SWITCH_INTERVAL 100 // ms
#define LONG_MODE_SWITCH_PRESS 2000

volatile ControlMode control_mode = info;
volatile ControlMode prev_control_mode = info;

// Is the timed_mode_switch ISR triggered on the rising or falling
// edge of the interrupt?
volatile int control_mode_switch_duration = 0;

// The next two functions are ISRs that implement the mode-switch.
// Forward declaration
void timed_mode_switch_release();

/**
 * The ISR for the mode switch. Triggerred when the switch is pressed.
 * The mode switch GPIO is held LOW normally and a button press causes
 * the input to go high. The ISR is triggerred on the rising edge of
 * the interrupt. Capture the time and set the duration to zero. Then
 * register a second ISR for the button release, which will be triggerred
 * when the GPIO pin state drops back to the LOW level.
 * 
 * @note Interrupts are disabled in ISR functions and millis() does
 * not advance.
 */
void timed_mode_switch_push() {
    // Do not allow control mode switches when the display is off
    if (display_mode == off)
        return;

    cli();
    if (millis() > control_mode_switch_time + MODE_SWITCH_INTERVAL) {
        Serial.println("timed_mode_switch_push");
        // Triggered on th rising edge is the button press; start the timer
        control_mode_switch_time = millis();
        control_mode_switch_duration = 0;
        attachInterrupt(digitalPinToInterrupt(MODE_SWITCH), timed_mode_switch_release, FALLING);
    }
    sei();
}

/**
 * When the mode switch is released, this ISR is run.
 */
void timed_mode_switch_release() {
    // Do not allow control mode switches when the display is off
    if (display_mode == off)
        return;

    cli();
    if (millis() > control_mode_switch_time + MODE_SWITCH_INTERVAL) {
        Serial.println("timed_mode_switch_release");
        attachInterrupt(digitalPinToInterrupt(MODE_SWITCH), timed_mode_switch_push, RISING);
        control_mode_switch_duration = millis() - control_mode_switch_time;
        control_mode_switch_time = millis();
        prev_control_mode = control_mode; // Used to update the 'mode display'

        if (control_mode_switch_duration > LONG_MODE_SWITCH_PRESS) {
            Serial.println("long press");
            switch (control_mode) {
            case info:
            case display_intensity:
            case color:
            case led_intensity:
                control_mode = set_hours;
                break;

            case set_hours:
                control_mode = info;
                break;

            case set_minutes:
                control_mode = info;
                break;
            }
        } else {
            Serial.println("short press");
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
                // led_intensity is the end of the cycle of the four main modes
                control_mode = info;
                break;

            // If in set-time mode, a short press goes from hours to minutes
            case set_hours:
                control_mode = set_minutes;
                break;

            case set_minutes:
                control_mode = info;
                break;

            default:
                break;
            }
        }

        Serial.print("control_mode: ");
        Serial.println(control_mode);
    }
    sei();
}

/**
 * Copy the values[] array to the old_values[] array. Comparing a new time
 * with the data in old_values[] is how the digits are updated when the 
 * time changes and not otherwise.
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
 * most significant) digit that differs (1, 2, 3, 4). If old_values[] == values[], 
 * return 0
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
 * @param count the number of tubes/digits to change (1-4), starting
 * with the fourth tube (least significant) and working backward.
 */
void digit_crossfade(unsigned int count) {
    int fade = fade_time * count;
    switch (count) {
    case 4:
        tubes[DIGIT_1]->crossfade_init(values[DIGIT_1], fade, saved_state.brightness, 0);
    case 3:
        tubes[DIGIT_2]->crossfade_init(values[DIGIT_2], fade, saved_state.brightness, 0);
    case 2:
        tubes[DIGIT_3]->crossfade_init(values[DIGIT_3], fade, saved_state.brightness, 0);
    case 1:
        tubes[DIGIT_4]->crossfade_init(values[DIGIT_4], fade, saved_state.brightness, 0);
        break;
    }

    for (int i = 0; i < NUM_TUBES; ++i) {
        tubes[i]->set_dots(left_dot[i], right_dot[i]);
    }

    bool done;
    do {
        done = true;
        switch (count) {
        case 4:
            done = (tubes[DIGIT_1]->crossfade_run() == EXIXE_ANIMATION_FINISHED) && done;
        case 3:
            done = (tubes[DIGIT_2]->crossfade_run() == EXIXE_ANIMATION_FINISHED) && done;
        case 2:
            done = (tubes[DIGIT_3]->crossfade_run() == EXIXE_ANIMATION_FINISHED) && done;
        case 1:
            done = (tubes[DIGIT_4]->crossfade_run() == EXIXE_ANIMATION_FINISHED) && done;
        }
    } while (!done);
}

void set_digit(int count, exixe *tube, bool fade) {
    if (fade) {
        // max() keeps brightness from being == 0 
        unsigned int delta = ceil(INITIAL_FADE_TIME / max(saved_state.brightness, 1));
        for (int b = 0; b < saved_state.brightness + 1; b++) {
            tube->show_digit(count, b, 0);
            delay(delta);
        }
    } else {
        tube->show_digit(count, saved_state.brightness, 0);
    }
}

void display(bool fade = false) {
    for (int i = 0; i < NUM_TUBES; ++i) {
        set_digit(values[i], tubes[i], fade);
        tubes[i]->set_dots(left_dot[i], right_dot[i]);
    }
}

void display_mode_forward() {
    switch (display_mode) {
    case off:
        display_mode = hours_mins;
        break;
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
        display_mode = off;
        break;
    default:
        break;
    }

    Serial.print("Display moed: "); Serial.println(display_mode);
}

void display_mode_backward() {
    switch (display_mode) {
    case off:
        display_mode = pressure;
        break;
    case hours_mins:
        display_mode = off;
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

    Serial.print("Display mode: "); Serial.println(display_mode);
}

/**
 * Cycle through the colors of the rainbow. If steps is positive,
 * move from red toward blue to purple and back to red again. If
 * steps is negative, move in the opposite direction. The global 
 * variables red_, blue_ and green_value are modified.
 */
void change_color(int steps) {
    // Here are the component color values used:
    // 255, green 0 .. 255, blue 0; red to orange to yellow
    // red 255 .. 0, green 255, blue 0; yellow to green
    // red 0, green 255 .. 0, blue 0 ..255; green to blue
    // red 0 .. 255, green 0, blue 255 .. 0; blue to purple to red again

    enum ColorState {
        red_to_yellow = 0,
        yellow_to_green = 1,
        green_to_blue = 2,
        blue_to_red = 3
    };

    static ColorState color_state = red_to_yellow;

    switch (color_state) {
    case red_to_yellow:
        saved_state.red_value = MAX_COLOR_VALUE;
        saved_state.green_value += steps;
        if (saved_state.green_value > MAX_COLOR_VALUE) {
            saved_state.green_value = MAX_COLOR_VALUE;
            color_state = yellow_to_green;
        } else if (saved_state.green_value < 0) {
            saved_state.green_value = 0;
            color_state = blue_to_red;
        }
        break;

    case yellow_to_green:
        saved_state.green_value = MAX_COLOR_VALUE;
        saved_state.red_value -= steps;
        if (saved_state.red_value <= 0) {
            saved_state.red_value = 0;
            color_state = green_to_blue;
        } else if (saved_state.red_value > MAX_COLOR_VALUE) {
            saved_state.red_value = MAX_COLOR_VALUE;
            color_state = red_to_yellow;
        }
        break;

    case green_to_blue:
        saved_state.red_value = 0;
        saved_state.green_value -= steps;
        if (saved_state.green_value < 0)
            saved_state.green_value = 0;
        saved_state.blue_value += steps;
        if (saved_state.blue_value > MAX_COLOR_VALUE) {
            saved_state.blue_value = MAX_COLOR_VALUE;
            color_state = blue_to_red;
        } else if (saved_state.blue_value < 0) {
            saved_state.blue_value = 0;
            color_state = yellow_to_green;
        }
        break;

    case blue_to_red:
        saved_state.green_value = 0;
        saved_state.blue_value -= steps;
        if (saved_state.blue_value < 0)
            saved_state.blue_value = 0;
        saved_state.red_value += steps;
        if (saved_state.red_value > MAX_COLOR_VALUE) {
            saved_state.red_value = MAX_COLOR_VALUE;
            color_state = red_to_yellow;
        } else if (saved_state.red_value < 0) {
            saved_state.red_value = 0;
            color_state = green_to_blue;
        }
        break;
    }
}

void show_color() {
    float scale_factor = (saved_state.led_brightness / 127.0);
    unsigned int r = saved_state.red_value * scale_factor;
    unsigned int g = saved_state.green_value * scale_factor;
    unsigned int b = saved_state.blue_value * scale_factor;

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

#define hPa_per_inch_Hg 0.0002953
/**
 * Butte is 1,682 m (5,518 ft)
 * One hPa is 0.0002953 inch Hg
 * @aparam alt Altitude in meters.
 * @return sea level pressure in inches of Hg
 */
float get_sea_level_pressure(float alt = 1682) {
    return bmp.readSealevelPressure(alt) * hPa_per_inch_Hg;
}

/**
 * @return sea level pressure in inches of Hg
 */
float get_pressure() {
    return bmp.readPressure() * hPa_per_inch_Hg;
}

void zero_dots() {
    for (int i = 0; i < NUM_TUBES; ++i) {
        left_dot[i] = 0;
        right_dot[i] = 0;
    }
}

void decimal_point(bool on = true) {
    if (on) {
        left_dot[DIGIT_2] = 0;
        right_dot[DIGIT_2] = saved_state.brightness;
    } else {
        left_dot[DIGIT_2] = 0;
        right_dot[DIGIT_2] = 0;
    }
}

void display_dots() {
    for (int i = 0; i < NUM_TUBES; ++i) {
        tubes[i]->set_dots(left_dot[i], right_dot[i]);
    }
}

/**
 * @brief ISR for the rotary encoder. 
 */
void rotary_encoder_event() {
    unsigned char result = rotary_encoder.process();
    if (result == DIR_NONE) {
        // do nothing
    } 
#if ROTARY_SWITCH_REVERSED
    else if (result == DIR_CCW) {
        Serial.println("ClockWise");
        ++encoder_position;
    } 
    else if (result == DIR_CW) {
        Serial.println("CounterClockWise");
        --encoder_position;
    }   
#else
    else if (result == DIR_CW) {
        Serial.println("ClockWise");
        ++encoder_position;
    } 
    else if (result == DIR_CCW) {
        Serial.println("CounterClockWise");
        --encoder_position;
    }
#endif
}

bool bmp_ok = false;

void setup() {
    // Enable some minor, chatty, messages on the serial line.
    Serial.begin(BAUD_RATE);
    Serial.println("boot");

    // MODE_SWITCH is D8 which must be low during boot and is pulled by the switch
    // But using FALLING seems more reliable
    pinMode(MODE_SWITCH, INPUT);
    attachInterrupt(digitalPinToInterrupt(MODE_SWITCH), timed_mode_switch_push, RISING);

    // Set up the roatary encoder using Pin Change Interrupts
    pinMode(ROTARY_CLK, INPUT_PULLUP);
    pinMode(ROTARY_DAT, INPUT_PULLUP);

    attachPCINT(digitalPinToPCINT(ROTARY_CLK), rotary_encoder_event, CHANGE);
    attachPCINT(digitalPinToPCINT(ROTARY_DAT), rotary_encoder_event, CHANGE);

    pinMode(HV_PS_ENABLE, OUTPUT);
    digitalWrite(HV_PS_ENABLE, LOW);

    pinMode(digit_1_cs, OUTPUT);
    pinMode(digit_2_cs, OUTPUT);
    pinMode(digit_3_cs, OUTPUT);
    pinMode(digit_4_cs, OUTPUT);

    int bmp_tries = 0;
    while (bmp_tries < 10 && !(bmp_ok = bmp.begin())) {
        Serial.print("bmp init trial: ");
        Serial.println(bmp_tries);
        delay(100);
        bmp_tries++;
    }
    Serial.print("bmp init trial: ");
    Serial.println(bmp_tries);

    Serial.print("bmp temp: ");
    Serial.println(get_temp());
    Serial.print("bmp SLP: ");
    Serial.println(get_sea_level_pressure());

    RTC.begin(); // always returns true
    DateTime boot_time(RTC.now());
    Serial.print("RTC boot: ");
    Serial.println(boot_time.unixtime());

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

    // Set or clear RESET_PARAMS in the platformio.ini file. This ensures
    // that the EEPROM (flash for the ESP8266) has values that won't
    // freak out the MCU on boot because the memory has random values by 
    // default.
#if RESET_PARAMS
    EEPROM.put(SAVED_STATE_ADDRESS, saved_state);
#else
    EEPROM.get(SAVED_STATE_ADDRESS, saved_state);
#endif

    show_color();

    // Initial display; show hours & mins
    DateTime now(RTC.now());
    set_values(now.hour(), now.minute());
    save_values();

    zero_dots();

    display(true);

#if 0
    Serial.println("After display(true) call");
#endif

    control_mode = info;
}

unsigned long temperature_update_time = 0;
unsigned long pressure_update_time = 0;

void loop() {

    // If the control/configuration mode has changed, update the display.
    if (prev_control_mode != control_mode) {
 
        prev_control_mode = control_mode;

        switch (control_mode) {
        case info:
            // When we're here changing modes, save the state. Only copies
            // to EEPROM when commit is called when we transition to the info
            // state. 
            EEPROM.put(SAVED_STATE_ADDRESS, saved_state);
            zero_dots();
            break;

        case display_intensity:
            left_dot[DIGIT_1] = 0;
            right_dot[DIGIT_1] = MAX_BRIGHTNESS;
            break;

        case color:
            left_dot[DIGIT_1] = MAX_BRIGHTNESS;
            right_dot[DIGIT_1] = 0;
            break;

        case led_intensity:
            left_dot[DIGIT_1] = MAX_BRIGHTNESS;
            right_dot[DIGIT_1] = MAX_BRIGHTNESS;
            break;

        case set_hours:
            zero_dots();
            left_dot[DIGIT_1] = MAX_BRIGHTNESS;
            right_dot[DIGIT_2] = MAX_BRIGHTNESS;
            break;

        case set_minutes:
            zero_dots();
            left_dot[DIGIT_3] = MAX_BRIGHTNESS;
            right_dot[DIGIT_4] = MAX_BRIGHTNESS;
            break;

        default:
            zero_dots();
            break;
        }

        display_dots();
    }

    // Get the current time
    DateTime now(RTC.now());

    // Based on the control mode, look at the encoder position and
    // act accordingly. For the 'info' mode, display hour/min, min/sec,
    // et cetera. For other modes, change the configuration. If the
    // rotary encoder position changed, update the display if that
    // is needed.
    switch (control_mode) {
    case info: {
        if (encoder_position != old_encoder_position) {
            if (encoder_position > old_encoder_position)
                display_mode_forward();
            else if (encoder_position < old_encoder_position)
                display_mode_backward();

            old_encoder_position = encoder_position;

            switch (display_mode) {
            case off:
                set_values(now.hour(), now.minute());
                zero_dots();
                digitalWrite(HV_PS_ENABLE, HIGH);   // active low
                break;

            case hours_mins:
                set_values(now.hour(), now.minute());
                decimal_point(false);
                break;

            case mins_secs:
                set_values(now.minute(), now.second());
                decimal_point(false);
                break;

            case temperature: {
                temperature_update_time = millis();
                float temperature = get_temp();
                unsigned int int_temp = trunc(temperature);
                set_values(int_temp, trunc((temperature - int_temp) * 100));
                decimal_point();
                break;
            }

            case pressure: {
                pressure_update_time = millis();
                float pressure = get_sea_level_pressure(); // get_pressure();
                unsigned int int_press = trunc(pressure);
                set_values(int_press, trunc((pressure - int_press) * 100));
                decimal_point();
                break;
            }

            default:
                set_values(now.hour(), now.minute());
                zero_dots();
                break;
            }

            // If the display mode is not 'off', enable the HV for the tubes
            if (display_mode != off)
                digitalWrite(HV_PS_ENABLE, LOW);   // active low

            display(true);
            save_values();
        }
        break;
    }

    case display_intensity: {
        if (encoder_position != old_encoder_position) {
            saved_state.brightness += 5 * (encoder_position - old_encoder_position);
            if (saved_state.brightness > 127)
                saved_state.brightness = 127;
            else if (saved_state.brightness < 0)
                saved_state.brightness = 0;

            display(false);

            old_encoder_position = encoder_position;
        }
        break;
    }

    case color: {
        if (encoder_position != old_encoder_position) {
            change_color((encoder_position - old_encoder_position) * 10);
            show_color();

            old_encoder_position = encoder_position;
        }
        break;
    }

    case led_intensity: {
        if (encoder_position != old_encoder_position) {
            saved_state.led_brightness += 5 * (encoder_position - old_encoder_position);
            if (saved_state.led_brightness > 127)
                saved_state.led_brightness = 127;
            else if (saved_state.led_brightness < 0)
                saved_state.led_brightness = 0;

            show_color();

            old_encoder_position = encoder_position;
        }
        break;
    }

    case set_hours:
        if (encoder_position != old_encoder_position) {
            long offset = encoder_position - old_encoder_position;

            // set the clock.
            TimeSpan ts(0, offset, 0, 0); // Get a time span for offset hours
            now = now + ts;               // Add the hour_value timespan to the current time
            RTC.adjust(now);              // Update the clock

            // Update the display
            set_values(now.hour(), now.minute());
            display(true);
            save_values();
            old_encoder_position = encoder_position;
        }
        break;

    case set_minutes:
        if (encoder_position != old_encoder_position) {
            long offset = encoder_position - old_encoder_position;

            // set the clock.
            TimeSpan ts(0, 0, offset, 0); // Get a time span for offset minutes
            now = now + ts;               // Add the hour_value timespan to the current time
            TimeSpan ts2(0, 0, 0, now.second());
            now = now - ts2; // zero the seconds
            RTC.adjust(now); // Update the clock

            // Update the display
            set_values(now.hour(), now.minute());
            display(true);
            save_values();
            old_encoder_position = encoder_position;
        }
        break;

    default:
        break;
    }

    // Check the time, pressure or temp and update as needed.
    unsigned int digit = 0;
    switch (display_mode) {
    case hours_mins:
        set_values(now.hour(), now.minute());

        if ((digit = values_changed())) {
            Serial.print("Time: "); Serial.print(now.hour());Serial.print(":"); Serial.println(now.minute());
            digit_crossfade(digit);
            save_values();
        }
        break;

    case mins_secs:
        set_values(now.minute(), now.second());
        if ((digit = values_changed())) {
            Serial.print("Time: "); Serial.print(now.minute());Serial.print(":"); Serial.println(now.second());
            digit_crossfade(digit);
            save_values();
        }
        break;

    case temperature: {
        // reduce display flicker
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

    display(false);
    // noise on the SPI bus can hose the LEDs; refresh them.
    show_color();
}
