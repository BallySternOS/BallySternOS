/**************************************************************************
 *     This file is part of the Bally/Stern OS for Arduino Project.

    I, Dick Hamill, the author of this program disclaim all copyright
    in order to make this program freely available in perpetuity to
    anyone who would like to use it. Dick Hamill, 12/1/2020

    BallySternOS is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    BallySternOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    See <https://www.gnu.org/licenses/>.
 */

#ifndef BSOS_CONFIG_H

/***

	Use this file to set game-specific and hardware-specific parameters

***/

// Hardware Rev 1 generally uses an Arduino Nano & (option) 74125
// Hardware Rev 2 uses an Arduino Nano, a 74155, and a 74240
#define BALLY_STERN_OS_HARDWARE_REV   1


//#define BALLY_STERN_OS_USE_DIP_SWITCHES 
//#define BALLY_STERN_OS_USE_SQUAWK_AND_TALK
//#define BALLY_STERN_OS_USE_SB100
//#define BALLY_STERN_OS_USE_SB300
//#define USE_WAV_TRIGGER
#define USE_WAV_TRIGGER_1p3
//#define BALLY_STERN_OS_USE_AUX_LAMPS
//#define BALLY_STERN_OS_USE_7_DIGIT_DISPLAYS
//#define BALLY_STERN_OS_DIMMABLE_DISPLAYS
#define BALLY_STERN_OS_SOFTWARE_DISPLAY_INTERRUPT

// Fast boards might need a slower lamp strobe
//#define BSOS_SLOW_DOWN_LAMP_STROBE

#ifdef BALLY_STERN_OS_USE_7_DIGIT_DISPLAYS
#define BALLY_STERN_OS_MAX_DISPLAY_SCORE  9999999
#define BALLY_STERN_OS_NUM_DIGITS         7
#define BALLY_STERN_OS_ALL_DIGITS_MASK    0x7F
#else
#define BALLY_STERN_OS_MAX_DISPLAY_SCORE  999999
#define BALLY_STERN_OS_NUM_DIGITS         6
#define BALLY_STERN_OS_ALL_DIGITS_MASK    0x3F
#endif

#ifdef BALLY_STERN_OS_USE_AUX_LAMPS
#define BSOS_NUM_LAMP_BITS 22
#define BSOS_MAX_LAMPS     88
#else
#define BSOS_NUM_LAMP_BITS 15
#define BSOS_MAX_LAMPS     60
#endif 

#define CONTSOL_DISABLE_FLIPPERS      0x40
#define CONTSOL_DISABLE_COIN_LOCKOUT  0x20

// This define needs to be set for the number of loops 
// needed to get a delay of 80 us
// So, set it to (0.000080) / (1/Clock Frequency)
// Assuming Frequency = 500kHz,  40 = (0.000080) / (1/500000)
#define BSOS_NUM_SWITCH_LOOPS 80
// 60 us
// So, set this to (0.000060) / (1/Clock Frequency)
#define BSOS_NUM_LAMP_LOOPS   60


#define BSOS_CREDITS_EEPROM_BYTE          5
#define BSOS_HIGHSCORE_EEPROM_START_BYTE  1
#define BSOS_AWARD_SCORE_1_EEPROM_START_BYTE      10
#define BSOS_AWARD_SCORE_2_EEPROM_START_BYTE      14
#define BSOS_AWARD_SCORE_3_EEPROM_START_BYTE      18
#define BSOS_TOTAL_PLAYS_EEPROM_START_BYTE        26
#define BSOS_TOTAL_REPLAYS_EEPROM_START_BYTE      30
#define BSOS_TOTAL_HISCORE_BEATEN_START_BYTE      34
#define BSOS_CHUTE_2_COINS_START_BYTE             38
#define BSOS_CHUTE_1_COINS_START_BYTE             42
#define BSOS_CHUTE_3_COINS_START_BYTE             46

#define BSOS_CONFIG_H
#endif
