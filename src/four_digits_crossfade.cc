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
#include <Wire.h>

#include <Adafruit_BMP085.h>

#include "exixe.h"

#define GPIO04 4
#define GPIO05 5

#define MODE_SWITCH 3 // RX

#define digit_1_cs 15 // D8
#define digit_2_cs 2  // D4
#define digit_3_cs 0  // D3
#define digit_4_cs 16 // D0

// Number opf seconds it takes to compile, upload and run this program
// at 9600 baud. Used to adjust the clock
#define TIME_OFFSET 12

#define BAUD_RATE 115200

#define INITIAL_FADE_TIME 250 // ms, time to fade in a digit when modes change

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

unsigned int brightness = 64; // 0 - 127
unsigned int fade_time = 15;  // 30 frames = 1 second

// Real time clock
RTC_DS3231 RTC; // we are using the DS3231 RTC

enum Mode {
    hours_mins = 0,
    mins_secs = 1,
    temperature = 2,
    pressure = 3
};

Mode mode = hours_mins;

Adafruit_BMP085 bmp;

bool bmp_ok = false; // true after sucessful init

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

    if (mode == temperature || mode == pressure)
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
        for (unsigned int b = 0; b < brightness + 1; b++) {
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

    if (mode == temperature || mode == pressure)
        tubes[DIGIT_2]->set_dots(0, brightness);
    else
        tubes[DIGIT_2]->set_dots(0, 0);

    set_digit(values[DIGIT_3], tubes[DIGIT_3], fade);
    set_digit(values[DIGIT_4], tubes[DIGIT_4], fade);
}

/**
 * Return true if the mode changed and update the mode global.
 */
bool check_mode_switch() {
    if (digitalRead(MODE_SWITCH) == LOW) {
        delay(100);
        if (digitalRead(MODE_SWITCH) == LOW) {
            switch (mode) {
            case hours_mins:
                mode = mins_secs;
                break;
            case mins_secs:
                mode = temperature;
                break;
            case temperature:
                mode = pressure;
                break;
            case pressure:
                mode = hours_mins;
                break;
            default:
                mode = hours_mins;
                break;
            }

            return true;
        }
    }

    return false;
}

void show_color() {
    digit_1.set_led(0, 0, 32);
    digit_2.set_led(0, 0, 32);
    digit_3.set_led(0, 0, 32);
    digit_4.set_led(0, 0, 32);
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
    Serial.begin(BAUD_RATE);
    Serial.println("Boot");
    Serial.flush();

    pinMode(MODE_SWITCH, INPUT);

    // Setup the I2C bus for the DS3231 clock
    // On the NodeMCU, GPIOR04 is D2, GPIOR05 is D1
    int sda = GPIO04;
    int scl = GPIO05;
    Wire.begin(sda, scl);

    if (bmp.begin()) {
        bmp_ok = true;
    }

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

void loop() {
    DateTime now(RTC.now());

    // If we switched modes, update the display
    if (check_mode_switch()) {
        switch (mode) {
        case hours_mins:
            set_values(now.hour(), now.minute());
            break;

        case mins_secs:
            set_values(now.minute(), now.second());
            break;

        case temperature: {
            float temperature = get_temp();
            unsigned int int_temp = trunc(temperature);
            set_values(int_temp, trunc((temperature - int_temp) * 100));
            break;
        }

        case pressure: {
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

    } else {
        unsigned int digit = 0;
        switch (mode) {
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
    }

    // noise on the SPI bus can hose the LEDs; refresh them.
    show_color();

    delay(100); // 0.1s
}
