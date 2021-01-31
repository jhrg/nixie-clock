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

#define MODE_SWITCH 16

#define digit_1_cs 15 // D8
#define digit_2_cs 2  // D4
#define digit_3_cs 0  // D3
// #define mins_cs 16       // D0
#define digit_4_cs 10 // SD3 was: 3 // RX

// Number opf seconds it takes to compile, upload and run this program
// at 9600 baud. Used to adjust the clock
#define TIME_OFFSET 12

#define BAUD_RATE 115200

#define INITIAL_FADE_TIME 250 // ms, time to fade in a digit when modes change

// set to 1 for hours & mins (24h) or 0 for mins & secs
//#define HOURS_MINS 0

//unsigned char count = 0;

// Set in setup() and thereafter loop()
unsigned int old_left_pair;
unsigned int old_right_pair;

// Clock digits
exixe digit_1 = exixe(digit_1_cs);
exixe digit_2 = exixe(digit_2_cs);
exixe digit_3 = exixe(digit_3_cs);
exixe digit_4 = exixe(digit_4_cs);

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

void set_digit_crossfade(int count, exixe *tube) {
    /*
        1st arg: Digit to show, 0 to 9
        2nd arg: how many frames does crossfade last, 30 frames = 1 second
        3rd arg: digit brightness, 0 to 127
        4th arg: overdrive, 0 disable 1 enable

        This function sets up the crossfade animation
        call crossfade_run() to actually start it
    */
    tube->crossfade_init(count, 15, 127, 0);

    // crossfade_run() is non-blocking and returns right away
    // call it regularly(at least every 33ms) for a smooth animation
    // check its return value to see if the animation is finished
    while (tube->crossfade_run() == EXIXE_ANIMATION_IN_PROGRESS)
        ;
}

void two_digit_crossfade(int count_1, exixe *tube_1, int count_2, exixe *tube_2) {
    /*
  1st arg: Digit to show, 0 to 9
  2nd arg: how many frames does crossfade last, 30 frames = 1 second
  3rd arg: digit brightness, 0 to 127
  4th arg: overdrive, 0 disable 1 enable

  This function sets up the crossfade animation
  call crossfade_run() to actually start it
  */
    tube_1->crossfade_init(count_1, 15, 127, 0);
    tube_2->crossfade_init(count_2, 15, 127, 0);

    // crossfade_run() is non-blocking and returns right away
    // call it regularly(at least every 33ms) for a smooth animation
    // check its return value to see if the animation is finished
    while (tube_1->crossfade_run() == EXIXE_ANIMATION_IN_PROGRESS)
        tube_2->crossfade_run();

    while (tube_2->crossfade_run() == EXIXE_ANIMATION_IN_PROGRESS)
        ;
}

#if 1
#define NUM_TUBES 4
typedef exixe *Tubes[NUM_TUBES];

/**
 * Given an array of the tubes and values, starting the left most digit and
 * going to the rightmost, use the crossfade to update the last 'count' tubes'
 * values.
 * 
 * Only call this function if at least one tube's value should change.
 * 
 * @param tubes An array of exixe pointers - digit_1 == [0], ...
 * @param value An array of digit values
 * @param count the number of tubes/digits to change (1-4), starting
 * with the fourth tube and working backward.
 */
void digit_crossfade(Tubes tubes, int value[NUM_TUBES], int count) {

    int i = NUM_TUBES - 1;
    do {
        tubes[i]->crossfade_init(value[i], 15, 127, 0);
        --i;
    } while (i >= (NUM_TUBES - count));

    bool done;
    do {
        done = true;
        switch (count) {
        case 4:
            done = done && (tubes[0]->crossfade_run() == EXIXE_ANIMATION_FINISHED);
        case 3:
            done = done && (tubes[1]->crossfade_run() == EXIXE_ANIMATION_FINISHED);
        case 2:
            done = done && (tubes[2]->crossfade_run() == EXIXE_ANIMATION_FINISHED);
        case 1:
            done = done && (tubes[3]->crossfade_run() == EXIXE_ANIMATION_FINISHED);
            break;
        }
    } while (!done);
}
#endif

void set_digit(int count, exixe *tube, bool fade) {
    if (fade) {
        unsigned int delta = ceil(INITIAL_FADE_TIME / 127);
        for (int brightness = 0; brightness < 128; brightness++) {
            tube->show_digit(count, brightness, 0);
            delay(delta);
        }
    } else {
        tube->show_digit(count, 127, 0);
    }
}

void display(unsigned int left_pair, unsigned int right_pair, bool fade = false) {
    int left_pair_1 = left_pair / 10;
    set_digit(left_pair_1, &digit_1, fade);

    int left_pair_2 = left_pair - (left_pair_1 * 10);
    set_digit(left_pair_2, &digit_2, fade);

    if (mode == temperature)
        digit_2.set_dots(0, 127);
    else
        digit_2.set_dots(0, 0);

    int right_pair_1 = right_pair / 10;
    set_digit(right_pair_1, &digit_3, fade);

    int right_pair_2 = right_pair - (right_pair_1 * 10);
    set_digit(right_pair_2, &digit_4, fade);
}

void display_crossfade(unsigned int left_pair, unsigned int right_pair) {
    int left_pair_1 = left_pair / 10;
    // int old_ten_hour = left_pair / 10;
    set_digit(left_pair_1, &digit_1, false);

    int left_pair_2 = left_pair - (left_pair_1 * 10);
    set_digit(left_pair_2, &digit_2, false);

    if (mode == temperature)
        digit_2.set_dots(0, 127);
    else
        digit_2.set_dots(0, 0);

    int right_pair_1 = right_pair / 10;
    int old_right_pair_1 = old_right_pair / 10;

    int right_pair_2 = right_pair - (right_pair_1 * 10);

    if (right_pair_1 != old_right_pair_1) {
        two_digit_crossfade(right_pair_1, &digit_3, right_pair_2, &digit_4);
    } else {
        set_digit_crossfade(right_pair_2, &digit_4);
    }
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
    digit_1.set_led(0, 0, 64);
    digit_2.set_led(0, 0, 64);
    digit_3.set_led(0, 0, 64);
    digit_4.set_led(0, 0, 64);
}

/**
 * @return temperature in degrees Fahrenheit.
 */
float get_temp() {
    return bmp.readTemperature() * (9.0 / 5.0) + 32.0; // C --> F
}

/**
 * Butte is 1,691.64 m (5,550 ft)
 * One hPa is 0.0002953 inch Hg
 * @aparam alt Altitude in meters.
 * @return sea level pressure in inches of Hg
 */
float get_sea_level_pressure(float alt = 1691.64) {
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

    show_color();

    // Initial display; show hours & mins
    DateTime now(RTC.now());
    // record these values for use later to determine when dgits change
    old_left_pair = now.hour();
    old_right_pair = now.minute();
    display(old_left_pair, old_right_pair, true /* fade */);
}

void loop() {
    DateTime now(RTC.now());

    // If we switched modes, update the display
    if (check_mode_switch()) {
        unsigned int left_pair;
        unsigned int right_pair;
        switch (mode) {
        case hours_mins:
            left_pair = now.hour();
            right_pair = now.minute();
            break;

        case mins_secs:
            left_pair = now.minute();
            right_pair = now.second();
            break;

        case temperature: {
            float temperature = get_temp();
            left_pair = (unsigned int)trunc(temperature);
            right_pair = (unsigned int)trunc((temperature - left_pair) * 10);
            break;
        }

        case pressure: {
            float pressure = get_pressure();
            left_pair = (unsigned int)trunc(pressure);
            right_pair = (unsigned int)trunc((pressure - left_pair) * 10);
            break;
        }

        default:
            left_pair = now.hour();
            right_pair = now.minute();

            break;
        }

        display(left_pair, right_pair, true /* fade */);

        old_left_pair = left_pair;
        old_right_pair = right_pair;
    } else {
        switch (mode) {
        case hours_mins:
            if (old_left_pair != now.hour() || old_right_pair != now.minute()) {
                display_crossfade(now.hour(), now.minute());

                old_left_pair = now.hour();
                old_right_pair = now.minute();
            }
            break;

        case mins_secs:
            if (old_left_pair != now.minute() || old_right_pair != now.second()) {
                display_crossfade(now.minute(), now.second());

                old_left_pair = now.minute();
                old_right_pair = now.second();
            }
            break;

        case temperature: {
            float temperature = get_temp();
            unsigned int new_left_pair = (unsigned int)trunc(temperature);
            unsigned int new_right_pair = (unsigned int)trunc((temperature - new_left_pair) * 10);
            if (old_left_pair != new_left_pair || old_right_pair != new_right_pair) {
                display_crossfade(new_left_pair, new_right_pair);

                old_left_pair = new_left_pair;
                old_right_pair = new_right_pair;
            }
            break;
        }

        case pressure: {
            float temperature = get_pressure();
            unsigned int new_left_pair = (unsigned int)trunc(temperature);
            unsigned int new_right_pair = (unsigned int)trunc((temperature - new_left_pair) * 10);
            if (old_left_pair != new_left_pair || old_right_pair != new_right_pair) {
                display_crossfade(new_left_pair, new_right_pair);

                old_left_pair = new_left_pair;
                old_right_pair = new_right_pair;
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
