/**************************************************************************
 *     This file is part of the Bally/Stern OS for Arduino Project.

    I, Dick Hamill, the author of this program disclaim all copyright
    in order to make this program freely available in perpetuity to
    anyone who would like to use it. Dick Hamill, 6/1/2020

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

#ifndef BALLY_STERN_OS_H

#define BALLY_STERN_OS_MAJOR_VERSION  1
#define BALLY_STERN_OS_MINOR_VERSION  5

//#define BALLY_STERN_OS_USE_DIP_SWITCHES 
//#define BALLY_STERN_OS_USE_SQUAWK_AND_TALK
#define BALLY_STERN_OS_USE_SB100
//#define BALLY_STERN_OS_USE_AUX_LAMPS
//#define BALLY_STERN_OS_NO_INTERRUPT_OF_AUX_LAMPS

struct PlayfieldAndCabinetSwitch {
  byte switchNum;
  byte solenoid;
  byte solenoidHoldTime;
};


#ifdef BALLY_STERN_OS_USE_AUX_LAMPS
#define BSOS_NUM_LAMP_BITS 22
#define BSOS_MAX_LAMPS     88
#else
#define BSOS_NUM_LAMP_BITS 15
#define BSOS_MAX_LAMPS     60
#endif 

// Arduino wiring
// J5       DEFINITION   ARDUINO
// n/a      IRQ           2
// PIN 32 - NOT USED
// PIN 31 - GND           GND  
// PIN 30 - +5V           VIN
// PIN 29 - Not Used
// PIN 28 - Ext Mem
// PIN 27 - Theta2        4
// PIN 26 - VMA *         A5
// PIN 25 - /RESET
// PIN 24 - /HLT *        A6
// PIN 23 - R/W *         3
// PIN 22 - A0 *          A0
// PIN 21 - A1 *          A1
// PIN 20 - A2
// PIN 19 - A3 *          A2
// PIN 18 - A4 *          A3
// PIN 17 - A5
// PIN 16 - A6
// PIN 15 - A7 *          A4
// PIN 14 - A8
// PIN 13 - A9 *          GND
// PIN 12 - A10           GND
// PIN 11 - A11           GND
// PIN 10 - A12 *         GND
// PIN 9 - A13            
// PIN 8 - D0             5
// PIN 7 - D1             6
// PIN 6 - D2             7 
// PIN 5 - D3             8
// PIN 4 - D4             9
// PIN 3 - D5             10
// PIN 2 - D6             11
// PIN 1 - D7             12

// PIA @ U11 = A4, A7, !A9, !A12      
// PIA @ U10 = A3, A7, !A9, !A12


#define SW_SELF_TEST_SWITCH 0x7F
#define SOL_NONE 0x0F
#define SWITCH_STACK_EMPTY  0xFF
#define CONTSOL_DISABLE_FLIPPERS      0x40
#define CONTSOL_DISABLE_COIN_LOCKOUT  0x20

// This define needs to be set for the number of loops 
// needed to get a delay of 80 us
// So, set it to (0.000080) / (1/Clock Frequency)
// Assuming Frequency = 500kHz,  40 = (0.000080) / (1/500000)
#define BSOS_NUM_SWITCH_LOOPS 70
// 60 us
// So, set this to (0.000060) / (1/Clock Frequency)
#define BSOS_NUM_LAMP_LOOPS   30

// Fast boards might need a slower lamp strobe
#define BSOS_SLOW_DOWN_LAMP_STROBE  0

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


// Function Prototypes

//   Initialization
void BSOS_InitializeMPU(); // This function used to take clock speed as a parameter - now delays are in defines at the top of this file
void BSOS_SetupGameSwitches(int s_numSwitches, int s_numPrioritySwitches, PlayfieldAndCabinetSwitch *s_gameSwitchArray);
//void BSOS_SetupGameLights(int s_numLights, PlayfieldLight *s_gameLightArray);
byte BSOS_GetDipSwitches(byte index);

// EEProm Helper Functions
//unsigned long BSOS_ReadHighScoreFromEEProm();
//void BSOS_WriteHighScoreToEEProm(unsigned long score);
byte BSOS_ReadByteFromEEProm(unsigned short startByte);
void BSOS_WriteByteToEEProm(unsigned short startByte, byte value);
unsigned long BSOS_ReadULFromEEProm(unsigned short startByte, unsigned long defaultValue=0);
void BSOS_WriteULToEEProm(unsigned short startByte, unsigned long value);

//   Swtiches
byte BSOS_PullFirstFromSwitchStack();
boolean BSOS_ReadSingleSwitchState(byte switchNum);

//   Solenoids
void BSOS_PushToSolenoidStack(byte solenoidNumber, byte numPushes, boolean disableOverride = false);
void BSOS_SetCoinLockout(boolean lockoutOn = false, byte solbit = CONTSOL_DISABLE_COIN_LOCKOUT);
void BSOS_SetDisableFlippers(boolean disableFlippers = true, byte solbit = CONTSOL_DISABLE_FLIPPERS);
byte BSOS_ReadContinuousSolenoids();
void BSOS_DisableSolenoidStack();
void BSOS_EnableSolenoidStack();
boolean BSOS_PushToTimedSolenoidStack(byte solenoidNumber, byte numPushes, unsigned long whenToFire, boolean disableOverride = false);
void BSOS_UpdateTimedSolenoidStack(unsigned long curTime);

//   Displays
void BSOS_SetDisplay(int displayNumber, unsigned long value, boolean blankByMagnitude=false, byte minDigits=2);
void BSOS_SetDisplayBlank(int displayNumber, byte bitMask);
void BSOS_SetDisplayCredits(int value, boolean displayOn = true, boolean showBothDigits=true);
void BSOS_SetDisplayMatch(int value, boolean displayOn = true, boolean showBothDigits=true);
void BSOS_SetDisplayBallInPlay(int value, boolean displayOn = true, boolean showBothDigits=true);
void BSOS_SetDisplayFlash(int displayNumber, unsigned long value, unsigned long curTime, int period=500, byte minDigits=2);
void BSOS_SetDisplayFlashCredits(unsigned long curTime, int period=100);
void BSOS_CycleAllDisplays(unsigned long curTime, byte digitNum=0); // Self-test function
byte BSOS_GetDisplayBlank(int displayNumber);

//   Lamps
void BSOS_SetLampState(int lampNum, byte s_lampState, byte s_lampDim=0, int s_lampFlashPeriod=0);
void BSOS_ApplyFlashToLamps(unsigned long curTime);
void BSOS_FlashAllLamps(unsigned long curTime); // Self-test function
void BSOS_TurnOffAllLamps();
void BSOS_SetDimDivisor(byte level=1, byte divisor=2); // 2 means 50% duty cycle, 3 means 33%, 4 means 25%...

//   Sound
#ifdef BALLY_STERN_OS_USE_SQUAWK_AND_TALK
void BSOS_PlaySoundSquawkAndTalk(byte soundByte);
#endif
#ifdef BALLY_STERN_OS_USE_SB100
void BSOS_PlaySB100(byte soundByte);
#endif

//   General
byte BSOS_DataRead(int address);


#ifdef BALLY_STERN_CPP_FILE
  int NumGameSwitches = 0;
  int NumGamePrioritySwitches = 0;
//  int NumGameLights = 0;
  
//  PlayfieldLight *GameLights = NULL;
  PlayfieldAndCabinetSwitch *GameSwitches = NULL;
#endif


#define BALLY_STERN_OS_H
#endif
