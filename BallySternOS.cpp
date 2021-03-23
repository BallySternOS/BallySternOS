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
 
#include <Arduino.h>
#include <EEPROM.h>
//#define DEBUG_MESSAGES    1
#define BALLY_STERN_CPP_FILE
#include "BallySternOS.h"

#ifndef BALLY_STERN_OS_HARDWARE_REV
#define BALLY_STERN_OS_HARDWARE_REV 1
#endif

// To use this library, take the example_BSOS_Config.h, 
// edit it for your hardware and game parameters and put
// it in your game's code folder as BSOS_Config.h
// (so when you fetch new versions of the library, you won't 
// overwrite your config)
#include "BSOS_Config.h"

#ifdef BALLY_STERN_OS_USE_7_DIGIT_DISPLAYS
#define BSOS_NUM_DIGITS     7
#define BSOS_00_MASK        0x60
#else
#define BSOS_NUM_DIGITS     6
#define BSOS_00_MASK        0x30
#endif

// Global variables
volatile byte DisplayDigits[5][BSOS_NUM_DIGITS];
volatile byte DisplayDigitEnable[5];
#ifdef BALLY_STERN_OS_DIMMABLE_DISPLAYS
volatile boolean DisplayDim[5];
#endif
volatile boolean DisplayOffCycle = false;
volatile byte CurrentDisplayDigit=0;
volatile byte LampStates[BSOS_NUM_LAMP_BITS], LampDim0[BSOS_NUM_LAMP_BITS], LampDim1[BSOS_NUM_LAMP_BITS];
volatile byte LampFlashPeriod[BSOS_MAX_LAMPS];
byte DimDivisor1 = 2;
byte DimDivisor2 = 3;

volatile byte SwitchesMinus2[5];
volatile byte SwitchesMinus1[5];
volatile byte SwitchesNow[5];
#ifdef BALLY_STERN_OS_USE_DIP_SWITCHES
byte DipSwitches[4];
#endif


#define SOLENOID_STACK_SIZE 64
#define SOLENOID_STACK_EMPTY 0xFF
volatile byte SolenoidStackFirst;
volatile byte SolenoidStackLast;
volatile byte SolenoidStack[SOLENOID_STACK_SIZE];
boolean SolenoidStackEnabled = true;
volatile byte CurrentSolenoidByte = 0xFF;

#define TIMED_SOLENOID_STACK_SIZE 32
struct TimedSolenoidEntry {
  byte inUse;
  unsigned long pushTime;
  byte solenoidNumber;
  byte numPushes;
  byte disableOverride;
};
TimedSolenoidEntry TimedSolenoidStack[32];

#define SWITCH_STACK_SIZE   64
#define SWITCH_STACK_EMPTY  0xFF
volatile byte SwitchStackFirst;
volatile byte SwitchStackLast;
volatile byte SwitchStack[SWITCH_STACK_SIZE];

#if (BALLY_STERN_OS_HARDWARE_REV==1)
#define ADDRESS_U10_A           0x14
#define ADDRESS_U10_A_CONTROL   0x15
#define ADDRESS_U10_B           0x16
#define ADDRESS_U10_B_CONTROL   0x17
#define ADDRESS_U11_A           0x18
#define ADDRESS_U11_A_CONTROL   0x19
#define ADDRESS_U11_B           0x1A
#define ADDRESS_U11_B_CONTROL   0x1B
#define ADDRESS_SB100           0x10

#elif (BALLY_STERN_OS_HARDWARE_REV==2)
#define ADDRESS_U10_A           0x00
#define ADDRESS_U10_A_CONTROL   0x01
#define ADDRESS_U10_B           0x02
#define ADDRESS_U10_B_CONTROL   0x03
#define ADDRESS_U11_A           0x08
#define ADDRESS_U11_A_CONTROL   0x09
#define ADDRESS_U11_B           0x0A
#define ADDRESS_U11_B_CONTROL   0x0B
#define ADDRESS_SB100           0x10
#define ADDRESS_SB100_CHIMES    0x18
#define ADDRESS_SB300_SQUARE_WAVES  0x10
#define ADDRESS_SB300_ANALOG        0x18

#endif 

void BSOS_DataWrite(int address, byte data) {
  
  // Set data pins to output
  // Make pins 5-7 output (and pin 3 for R/W)
  DDRD = DDRD | 0xE8;
  // Make pins 8-12 output
  DDRB = DDRB | 0x1F;

  // Set R/W to LOW
  PORTD = (PORTD & 0xF7);

  // Put data on pins
  // Put lower three bits on 5-7
  PORTD = (PORTD&0x1F) | ((data&0x07)<<5);
  // Put upper five bits on 8-12
  PORTB = (PORTB&0xE0) | (data>>3);

  // Set up address lines
  PORTC = (PORTC & 0xE0) | address;

  // Wait for a falling edge of the clock
  while((PIND & 0x10));

  // Pulse VMA over one clock cycle
  // Set VMA ON
  PORTC = PORTC | 0x20;
  
  // Wait while clock is low
  while(!(PIND & 0x10));
  // Wait while clock is high
// Doesn't seem to help --  while((PIND & 0x10));

  // Set VMA OFF
  PORTC = PORTC & 0xDF;

  // Unset address lines
  PORTC = PORTC & 0xE0;
  
  // Set R/W back to HIGH
  PORTD = (PORTD | 0x08);

  // Set data pins to input
  // Make pins 5-7 input
  DDRD = DDRD & 0x1F;
  // Make pins 8-12 input
  DDRB = DDRB & 0xE0;
}



byte BSOS_DataRead(int address) {
  
  // Set data pins to input
  // Make pins 5-7 input
  DDRD = DDRD & 0x1F;
  // Make pins 8-12 input
  DDRB = DDRB & 0xE0;

  // Set R/W to HIGH
  DDRD = DDRD | 0x08;
  PORTD = (PORTD | 0x08);

  // Set up address lines
  PORTC = (PORTC & 0xE0) | address;

  // Wait for a falling edge of the clock
  while((PIND & 0x10));

  // Pulse VMA over one clock cycle
  // Set VMA ON
  PORTC = PORTC | 0x20;
  
  // Wait while clock is low
  while(!(PIND & 0x10));

  byte inputData = (PIND>>5) | (PINB<<3);

  // Set VMA OFF
  PORTC = PORTC & 0xDF;

  // Wait for a falling edge of the clock
// Doesn't seem to help  while((PIND & 0x10));

  // Set R/W to LOW
  PORTD = (PORTD & 0xF7);

  // Clear address lines
  PORTC = (PORTC & 0xE0);

  return inputData;
}


void WaitClockCycle(int numCycles=1) {
  for (int count=0; count<numCycles; count++) {
    // Wait while clock is low
    while(!(PIND & 0x10));
  
    // Wait for a falling edge of the clock
    while((PIND & 0x10));
  }
}


void TestLightOn() {
  BSOS_DataWrite(ADDRESS_U11_A_CONTROL, BSOS_DataRead(ADDRESS_U11_A_CONTROL) | 0x08);
}

void TestLightOff() {
  BSOS_DataWrite(ADDRESS_U11_A_CONTROL, BSOS_DataRead(ADDRESS_U11_A_CONTROL) & 0xF7);
}



void InitializeU10PIA() {
  // CA1 - Self Test Switch
  // CB1 - zero crossing detector
  // CA2 - NOR'd with display latch strobe
  // CB2 - lamp strobe 1
  // PA0-7 - output for switch bank, lamps, and BCD
  // PB0-7 - switch returns
  
  BSOS_DataWrite(ADDRESS_U10_A_CONTROL, 0x38);
  // Set up U10A as output
  BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
  // Set bit 3 to write data
  BSOS_DataWrite(ADDRESS_U10_A_CONTROL, BSOS_DataRead(ADDRESS_U10_A_CONTROL)|0x04);
  // Store F0 in U10A Output
  BSOS_DataWrite(ADDRESS_U10_A, 0xF0);
  
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x33);
  // Set up U10B as input
  BSOS_DataWrite(ADDRESS_U10_B, 0x00);
  // Set bit 3 so future reads will read data
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL)|0x04);

}

#ifdef BALLY_STERN_OS_USE_DIP_SWITCHES
void ReadDipSwitches() {
  byte backupU10A = BSOS_DataRead(ADDRESS_U10_A);
  byte backupU10BControl = BSOS_DataRead(ADDRESS_U10_B_CONTROL);

  // Turn on Switch strobe 5 & Read Switches
  BSOS_DataWrite(ADDRESS_U10_A, 0x20);
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, backupU10BControl & 0xF7);
  // Wait for switch capacitors to charge
  WaitClockCycle(BSOS_NUM_SWITCH_LOOPS);
  DipSwitches[0] = BSOS_DataRead(ADDRESS_U10_B);
 
  // Turn on Switch strobe 6 & Read Switches
  BSOS_DataWrite(ADDRESS_U10_A, 0x40);
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, backupU10BControl & 0xF7);
  // Wait for switch capacitors to charge
  WaitClockCycle(BSOS_NUM_SWITCH_LOOPS);
  DipSwitches[1] = BSOS_DataRead(ADDRESS_U10_B);

  // Turn on Switch strobe 7 & Read Switches
  BSOS_DataWrite(ADDRESS_U10_A, 0x80);
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, backupU10BControl & 0xF7);
  // Wait for switch capacitors to charge
  WaitClockCycle(BSOS_NUM_SWITCH_LOOPS);
  DipSwitches[2] = BSOS_DataRead(ADDRESS_U10_B);

  // Turn on U10 CB2 (strobe 8) and read switches
  BSOS_DataWrite(ADDRESS_U10_A, 0x00);
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, backupU10BControl | 0x08);
  // Wait for switch capacitors to charge
  WaitClockCycle(BSOS_NUM_SWITCH_LOOPS);
  DipSwitches[3] = BSOS_DataRead(ADDRESS_U10_B);

  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, backupU10BControl);
  BSOS_DataWrite(ADDRESS_U10_A, backupU10A);
}
#endif

void InitializeU11PIA() {
  // CA1 - Display interrupt generator
  // CB1 - test connector pin 32
  // CA2 - lamp strobe 2
  // CB2 - solenoid bank select
  // PA0-7 - display digit enable
  // PB0-7 - solenoid data

  BSOS_DataWrite(ADDRESS_U11_A_CONTROL, 0x31);
  // Set up U11A as output
  BSOS_DataWrite(ADDRESS_U11_A, 0xFF);
  // Set bit 3 to write data
  BSOS_DataWrite(ADDRESS_U11_A_CONTROL, BSOS_DataRead(ADDRESS_U11_A_CONTROL)|0x04);
  // Store 00 in U11A Output
  BSOS_DataWrite(ADDRESS_U11_A, 0x00);
  
  BSOS_DataWrite(ADDRESS_U11_B_CONTROL, 0x30);
  // Set up U11B as output
  BSOS_DataWrite(ADDRESS_U11_B, 0xFF);
  // Set bit 3 so future reads will read data
  BSOS_DataWrite(ADDRESS_U11_B_CONTROL, BSOS_DataRead(ADDRESS_U11_B_CONTROL)|0x04);
  // Store 9F in U11B Output
  BSOS_DataWrite(ADDRESS_U11_B, 0x9F);
  CurrentSolenoidByte = 0x9F;
  
}


int SpaceLeftOnSwitchStack() {
  if (SwitchStackFirst>=SWITCH_STACK_SIZE || SwitchStackLast>=SWITCH_STACK_SIZE) return 0;
  if (SwitchStackLast>=SwitchStackFirst) return ((SWITCH_STACK_SIZE-1) - (SwitchStackLast-SwitchStackFirst));
  return (SwitchStackFirst - SwitchStackLast) - 1;
}

void PushToSwitchStack(byte switchNumber) {
  if ((switchNumber>39 && switchNumber!=SW_SELF_TEST_SWITCH)) return;

  // If the switch stack last index is out of range, then it's an error - return
  if (SpaceLeftOnSwitchStack()==0) return;

  // Self test is a special case - there's no good way to debounce it
  // so if it's already first on the stack, ignore it
  if (switchNumber==SW_SELF_TEST_SWITCH) {
    if (SwitchStackLast!=SwitchStackFirst && SwitchStack[SwitchStackFirst]==SW_SELF_TEST_SWITCH) return;
  }

  SwitchStack[SwitchStackLast] = switchNumber;
  
  SwitchStackLast += 1;
  if (SwitchStackLast==SWITCH_STACK_SIZE) {
    // If the end index is off the end, then wrap
    SwitchStackLast = 0;
  }
}


byte BSOS_PullFirstFromSwitchStack() {
  // If first and last are equal, there's nothing on the stack
  if (SwitchStackFirst==SwitchStackLast) return SWITCH_STACK_EMPTY;

  byte retVal = SwitchStack[SwitchStackFirst];

  SwitchStackFirst += 1;
  if (SwitchStackFirst>=SWITCH_STACK_SIZE) SwitchStackFirst = 0;

  return retVal;
}


boolean BSOS_ReadSingleSwitchState(byte switchNum) {
  if (switchNum>39) return false;

  int switchByte = switchNum/8;
  int switchBit = switchNum%8;
  if ( ((SwitchesNow[switchByte])>>switchBit) & 0x01 ) return true;
  else return false;
}


int SpaceLeftOnSolenoidStack() {
  if (SolenoidStackFirst>=SOLENOID_STACK_SIZE || SolenoidStackLast>=SOLENOID_STACK_SIZE) return 0;
  if (SolenoidStackLast>=SolenoidStackFirst) return ((SOLENOID_STACK_SIZE-1) - (SolenoidStackLast-SolenoidStackFirst));
  return (SolenoidStackFirst - SolenoidStackLast) - 1;
}


void BSOS_PushToSolenoidStack(byte solenoidNumber, byte numPushes, boolean disableOverride) {
  if (solenoidNumber>14) return;

  // if the solenoid stack is disabled and this isn't an override push, then return
  if (!disableOverride && !SolenoidStackEnabled) return;

  // If the solenoid stack last index is out of range, then it's an error - return
  if (SpaceLeftOnSolenoidStack()==0) return;

  for (int count=0; count<numPushes; count++) {
    SolenoidStack[SolenoidStackLast] = solenoidNumber;
    
    SolenoidStackLast += 1;
    if (SolenoidStackLast==SOLENOID_STACK_SIZE) {
      // If the end index is off the end, then wrap
      SolenoidStackLast = 0;
    }
    // If the stack is now full, return
    if (SpaceLeftOnSolenoidStack()==0) return;
  }
}

void PushToFrontOfSolenoidStack(byte solenoidNumber, byte numPushes) {
  // If the stack is full, return
  if (SpaceLeftOnSolenoidStack()==0  || !SolenoidStackEnabled) return;

  for (int count=0; count<numPushes; count++) {
    if (SolenoidStackFirst==0) SolenoidStackFirst = SOLENOID_STACK_SIZE-1;
    else SolenoidStackFirst -= 1;
    SolenoidStack[SolenoidStackFirst] = solenoidNumber;
    if (SpaceLeftOnSolenoidStack()==0) return;
  }
  
}

byte PullFirstFromSolenoidStack() {
  // If first and last are equal, there's nothing on the stack
  if (SolenoidStackFirst==SolenoidStackLast) return SOLENOID_STACK_EMPTY;
  
  byte retVal = SolenoidStack[SolenoidStackFirst];

  SolenoidStackFirst += 1;
  if (SolenoidStackFirst>=SOLENOID_STACK_SIZE) SolenoidStackFirst = 0;

  return retVal;
}


boolean BSOS_PushToTimedSolenoidStack(byte solenoidNumber, byte numPushes, unsigned long whenToFire, boolean disableOverride) {
  for (int count=0; count<TIMED_SOLENOID_STACK_SIZE; count++) {
    if (!TimedSolenoidStack[count].inUse) {
      TimedSolenoidStack[count].inUse = true;
      TimedSolenoidStack[count].pushTime = whenToFire;
      TimedSolenoidStack[count].disableOverride = disableOverride;
      TimedSolenoidStack[count].solenoidNumber = solenoidNumber;
      TimedSolenoidStack[count].numPushes = numPushes;
      return true;
    }
  }
  return false;
}


void BSOS_UpdateTimedSolenoidStack(unsigned long curTime) {
  for (int count=0; count<TIMED_SOLENOID_STACK_SIZE; count++) {
    if (TimedSolenoidStack[count].inUse && TimedSolenoidStack[count].pushTime<curTime) {
      BSOS_PushToSolenoidStack(TimedSolenoidStack[count].solenoidNumber, TimedSolenoidStack[count].numPushes, TimedSolenoidStack[count].disableOverride);
      TimedSolenoidStack[count].inUse = false;
    }
  }
}



volatile int numberOfU10Interrupts = 0;
volatile int numberOfU11Interrupts = 0;
volatile byte InsideZeroCrossingInterrupt = 0;


void InterruptService2() {
  byte u10AControl = BSOS_DataRead(ADDRESS_U10_A_CONTROL);
  if (u10AControl & 0x80) {
    // self test switch
    if (BSOS_DataRead(ADDRESS_U10_A_CONTROL) & 0x80) PushToSwitchStack(SW_SELF_TEST_SWITCH);
    BSOS_DataRead(ADDRESS_U10_A);
  }

  // If we get a weird interupt from U11B, clear it
  byte u11BControl = BSOS_DataRead(ADDRESS_U11_B_CONTROL);
  if (u11BControl & 0x80) {
    BSOS_DataRead(ADDRESS_U11_B);    
  }

  byte u11AControl = BSOS_DataRead(ADDRESS_U11_A_CONTROL);
  byte u10BControl = BSOS_DataRead(ADDRESS_U10_B_CONTROL);

  // If the interrupt bit on the display interrupt is on, do the display refresh
  if (u11AControl & 0x80) {
    // Backup U10A
    byte backupU10A = BSOS_DataRead(ADDRESS_U10_A);
    
    // Disable lamp decoders & strobe latch
    BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) | 0x08);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) & 0xF7);
#ifdef BALLY_STERN_OS_USE_AUX_LAMPS
    // Also park the aux lamp board 
    BSOS_DataWrite(ADDRESS_U11_A_CONTROL, BSOS_DataRead(ADDRESS_U11_A_CONTROL) | 0x08);
    BSOS_DataWrite(ADDRESS_U11_A_CONTROL, BSOS_DataRead(ADDRESS_U11_A_CONTROL) & 0xF7);    
#endif

    // Blank Displays
    BSOS_DataWrite(ADDRESS_U10_A_CONTROL, BSOS_DataRead(ADDRESS_U10_A_CONTROL) & 0xF7);
    BSOS_DataWrite(ADDRESS_U11_A, (BSOS_DataRead(ADDRESS_U11_A) & 0x03) | 0x01);
    BSOS_DataWrite(ADDRESS_U10_A, 0x0F);

    // Write current display digits to 5 displays
    for (int displayCount=0; displayCount<5; displayCount++) {

      if (CurrentDisplayDigit<BSOS_NUM_DIGITS) {
        // The BCD for this digit is in b4-b7, and the display latch strobes are in b0-b3 (and U11A:b0)
        byte displayDataByte = ((DisplayDigits[displayCount][CurrentDisplayDigit])<<4) | 0x0F;
        byte displayEnable = ((DisplayDigitEnable[displayCount])>>CurrentDisplayDigit)&0x01;
  
        // if this digit shouldn't be displayed, then set data lines to 0xFX so digit will be blank
        if (!displayEnable) displayDataByte = 0xFF;
#ifdef BALLY_STERN_OS_DIMMABLE_DISPLAYS        
        if (DisplayDim[displayCount] && DisplayOffCycle) displayDataByte = 0xFF;
#endif        
  
        // Set low the appropriate latch strobe bit
        if (displayCount<4) {
          displayDataByte &= ~(0x01<<displayCount);
        }
        // Write out the digit & strobe (if it's 0-3)
        BSOS_DataWrite(ADDRESS_U10_A, displayDataByte);
        if (displayCount==4) {            
          // Strobe #5 latch on U11A:b0
          BSOS_DataWrite(ADDRESS_U11_A, BSOS_DataRead(ADDRESS_U11_A) & 0xFE);
        }

        // Need to delay a little to make sure the strobe is low for long enough
        WaitClockCycle(4);

        // Put the latch strobe bits back high
        if (displayCount<4) {
          displayDataByte |= 0x0F;
          BSOS_DataWrite(ADDRESS_U10_A, displayDataByte);
        } else {
          BSOS_DataWrite(ADDRESS_U11_A, BSOS_DataRead(ADDRESS_U11_A) | 0x01);
          
          // Set proper display digit enable
#ifdef BALLY_STERN_OS_USE_7_DIGIT_DISPLAYS          
          byte displayDigitsMask = (0x02<<CurrentDisplayDigit) | 0x01;
#else
          byte displayDigitsMask = (0x04<<CurrentDisplayDigit) | 0x01;
#endif          
          BSOS_DataWrite(ADDRESS_U11_A, displayDigitsMask);
        }
      }
    }

    // Stop Blanking (current digits are all latched and ready)
    BSOS_DataWrite(ADDRESS_U10_A_CONTROL, BSOS_DataRead(ADDRESS_U10_A_CONTROL) | 0x08);

    // Restore 10A from backup
    BSOS_DataWrite(ADDRESS_U10_A, backupU10A);    

    CurrentDisplayDigit = CurrentDisplayDigit + 1;
    if (CurrentDisplayDigit>=BSOS_NUM_DIGITS) {
      CurrentDisplayDigit = 0;
      DisplayOffCycle ^= true;
    }
    numberOfU11Interrupts+=1;
  }

  // If the IRQ bit of U10BControl is set, do the Zero-crossing interrupt handler
  if ((u10BControl & 0x80) && (InsideZeroCrossingInterrupt==0)) {
    InsideZeroCrossingInterrupt = InsideZeroCrossingInterrupt + 1;

    byte u10BControlLatest = BSOS_DataRead(ADDRESS_U10_B_CONTROL);

    // Backup contents of U10A
    byte backup10A = BSOS_DataRead(ADDRESS_U10_A);

    // Latch 0xFF separately without interrupt clear
    BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) | 0x08);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) & 0xF7);
    // Read U10B to clear interrupt
    BSOS_DataRead(ADDRESS_U10_B);

    // Turn off U10BControl interrupts
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x30);

    // Copy old switch values
    byte switchCount;
    byte startingClosures;
    byte validClosures;
    for (switchCount=0; switchCount<5; switchCount++) {
      SwitchesMinus2[switchCount] = SwitchesMinus1[switchCount];
      SwitchesMinus1[switchCount] = SwitchesNow[switchCount];

      // Enable playfield strobe
      BSOS_DataWrite(ADDRESS_U10_A, 0x01<<switchCount);
      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x34);

      // Delay for switch capacitors to charge
      WaitClockCycle(BSOS_NUM_SWITCH_LOOPS);
      
      // Read the switches
      SwitchesNow[switchCount] = BSOS_DataRead(ADDRESS_U10_B);

      //Unset the strobe
      BSOS_DataWrite(ADDRESS_U10_A, 0x00);

      // Some switches need to trigger immediate closures (bumpers & slings)
      startingClosures = (SwitchesNow[switchCount]) & (~SwitchesMinus1[switchCount]);
      boolean immediateSolenoidFired = false;
      // If one of the switches is starting to close (off, on)
      if (startingClosures) {
        // Loop on bits of switch byte
        for (byte bitCount=0; bitCount<8 && immediateSolenoidFired==false; bitCount++) {
          // If this switch bit is closed
          if (startingClosures&0x01) {
            byte startingSwitchNum = switchCount*8 + bitCount;
            // Loop on immediate switch data
            for (int immediateSwitchCount=0; immediateSwitchCount<NumGamePrioritySwitches && immediateSolenoidFired==false; immediateSwitchCount++) {
              // If this switch requires immediate action
              if (GameSwitches && startingSwitchNum==GameSwitches[immediateSwitchCount].switchNum) {
                // Start firing this solenoid (just one until the closure is validate
                PushToFrontOfSolenoidStack(GameSwitches[immediateSwitchCount].solenoid, 1);
                immediateSolenoidFired = true;
              }
            }
          }
          startingClosures = startingClosures>>1;
        }
      }

      immediateSolenoidFired = false;
      validClosures = (SwitchesNow[switchCount] & SwitchesMinus1[switchCount]) & ~SwitchesMinus2[switchCount];
      // If there is a valid switch closure (off, on, on)
      if (validClosures) {
        // Loop on bits of switch byte
        for (byte bitCount=0; bitCount<8; bitCount++) {
          // If this switch bit is closed
          if (validClosures&0x01) {
            byte validSwitchNum = switchCount*8 + bitCount;
            // Loop through all switches and see what's triggered
            for (int validSwitchCount=0; validSwitchCount<NumGameSwitches; validSwitchCount++) {

              // If we've found a valid closed switch
              if (GameSwitches && GameSwitches[validSwitchCount].switchNum==validSwitchNum) {

                // If we're supposed to trigger a solenoid, then do it
                if (GameSwitches[validSwitchCount].solenoid!=SOL_NONE) {
                  if (validSwitchCount<NumGamePrioritySwitches && immediateSolenoidFired==false) {
                    PushToFrontOfSolenoidStack(GameSwitches[validSwitchCount].solenoid, GameSwitches[validSwitchCount].solenoidHoldTime);
                  } else {
                    BSOS_PushToSolenoidStack(GameSwitches[validSwitchCount].solenoid, GameSwitches[validSwitchCount].solenoidHoldTime);
                  }
                } // End if this is a real solenoid
              } // End if this is a switch in the switch table
            } // End loop on switches in switch table
            // Push this switch to the game rules stack
            PushToSwitchStack(validSwitchNum);
          }
          validClosures = validClosures>>1;
        }        
      }

      // There are no port reads or writes for the rest of the loop, 
      // so we can allow the display interrupt to fire
      interrupts();
      
      // Wait so total delay will allow lamp SCRs to get to the proper voltage
      WaitClockCycle(BSOS_NUM_LAMP_LOOPS);
      
      noInterrupts();
    }
    BSOS_DataWrite(ADDRESS_U10_A, backup10A);

    // If we need to turn off momentary solenoids, do it first
    byte momentarySolenoidAtStart = PullFirstFromSolenoidStack();
    if (momentarySolenoidAtStart!=SOLENOID_STACK_EMPTY) {
      CurrentSolenoidByte = (CurrentSolenoidByte&0xF0) | momentarySolenoidAtStart;
      BSOS_DataWrite(ADDRESS_U11_B, CurrentSolenoidByte);
    } else {
      CurrentSolenoidByte = (CurrentSolenoidByte&0xF0) | SOL_NONE;
      BSOS_DataWrite(ADDRESS_U11_B, CurrentSolenoidByte);
    }

#ifndef BALLY_STERN_OS_USE_AUX_LAMPS
    for (int lampBitCount = 0; lampBitCount<BSOS_NUM_LAMP_BITS; lampBitCount++) {
      byte lampData = 0xF0 + lampBitCount;

      interrupts();
      BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
      noInterrupts();
      
      // Latch address & strobe
      BSOS_DataWrite(ADDRESS_U10_A, lampData);
#ifdef BSOS_SLOW_DOWN_LAMP_STROBE      
      WaitClockCycle();
#endif      

      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x38);
#ifdef BSOS_SLOW_DOWN_LAMP_STROBE      
      WaitClockCycle();
#endif      

      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x30);
#ifdef BSOS_SLOW_DOWN_LAMP_STROBE      
      WaitClockCycle();
#endif      

      // Use the inhibit lines to set the actual data to the lamp SCRs 
      // (here, we don't care about the lower nibble because the address was already latched)
      byte lampOutput = LampStates[lampBitCount];
      // Every other time through the cycle, we OR in the dim variable
      // in order to dim those lights
      if (numberOfU10Interrupts%DimDivisor1) lampOutput |= LampDim0[lampBitCount];
      if (numberOfU10Interrupts%DimDivisor2) lampOutput |= LampDim1[lampBitCount];

      BSOS_DataWrite(ADDRESS_U10_A, lampOutput);
#ifdef BSOS_SLOW_DOWN_LAMP_STROBE      
      WaitClockCycle();
#endif      
    }

    // Latch 0xFF separately without interrupt clear
    BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) | 0x08);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) & 0xF7);

#else 

    for (int lampBitCount=0; lampBitCount<15; lampBitCount++) {
      byte lampData = 0xF0 + lampBitCount;

      interrupts();
      BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
      noInterrupts();
      
      // Latch address & strobe
      BSOS_DataWrite(ADDRESS_U10_A, lampData);
#ifdef BSOS_SLOW_DOWN_LAMP_STROBE      
      WaitClockCycle();
#endif      

      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x38);
#ifdef BSOS_SLOW_DOWN_LAMP_STROBE      
      WaitClockCycle();
#endif      

      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x30);
#ifdef BSOS_SLOW_DOWN_LAMP_STROBE      
      WaitClockCycle();
#endif      

      // Use the inhibit lines to set the actual data to the lamp SCRs 
      // (here, we don't care about the lower nibble because the address was already latched)
      byte lampOutput = LampStates[lampBitCount];
      // Every other time through the cycle, we OR in the dim variable
      // in order to dim those lights
      if (numberOfU10Interrupts%DimDivisor1) lampOutput |= LampDim0[lampBitCount];
      if (numberOfU10Interrupts%DimDivisor2) lampOutput |= LampDim1[lampBitCount];

      BSOS_DataWrite(ADDRESS_U10_A, lampOutput);
#ifdef BSOS_SLOW_DOWN_LAMP_STROBE      
      WaitClockCycle();
#endif      

    }
    // Latch 0xFF separately without interrupt clear
    // to park 0xFF in main lamp board
    BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) | 0x08);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) & 0xF7);

    for (int lampBitCount=15; lampBitCount<22; lampBitCount++) {
      byte lampOutput = (LampStates[lampBitCount]&0xF0) | (lampBitCount-15);
      // Every other time through the cycle, we OR in the dim variable
      // in order to dim those lights
      if (numberOfU10Interrupts%DimDivisor1) lampOutput |= LampDim0[lampBitCount];
      if (numberOfU10Interrupts%DimDivisor2) lampOutput |= LampDim1[lampBitCount];

      interrupts();
      BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
      noInterrupts();

      BSOS_DataWrite(ADDRESS_U10_A, lampOutput | 0xF0);
      BSOS_DataWrite(ADDRESS_U11_A_CONTROL, BSOS_DataRead(ADDRESS_U11_A_CONTROL) | 0x08);
      BSOS_DataWrite(ADDRESS_U11_A_CONTROL, BSOS_DataRead(ADDRESS_U11_A_CONTROL) & 0xF7);    
      BSOS_DataWrite(ADDRESS_U10_A, lampOutput);
    }
    
    BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
    BSOS_DataWrite(ADDRESS_U11_A_CONTROL, BSOS_DataRead(ADDRESS_U11_A_CONTROL) | 0x08);
    BSOS_DataWrite(ADDRESS_U11_A_CONTROL, BSOS_DataRead(ADDRESS_U11_A_CONTROL) & 0xF7);

#endif 

    interrupts();
    noInterrupts();

    InsideZeroCrossingInterrupt = 0;
    BSOS_DataWrite(ADDRESS_U10_A, backup10A);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, u10BControlLatest);

    // Read U10B to clear interrupt
    BSOS_DataRead(ADDRESS_U10_B);
    numberOfU10Interrupts+=1;
  }
}


#if defined (BALLY_STERN_OS_SOFTWARE_DISPLAY_INTERRUPT)

ISR(TIMER1_COMPA_vect) {    //This is the interrupt request
  // Backup U10A
  byte backupU10A = BSOS_DataRead(ADDRESS_U10_A);
  
  // Disable lamp decoders & strobe latch
  BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) | 0x08);
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) & 0xF7);
#ifdef BALLY_STERN_OS_USE_AUX_LAMPS
  // Also park the aux lamp board 
  BSOS_DataWrite(ADDRESS_U11_A_CONTROL, BSOS_DataRead(ADDRESS_U11_A_CONTROL) | 0x08);
  BSOS_DataWrite(ADDRESS_U11_A_CONTROL, BSOS_DataRead(ADDRESS_U11_A_CONTROL) & 0xF7);    
#endif

  // Blank Displays
  BSOS_DataWrite(ADDRESS_U10_A_CONTROL, BSOS_DataRead(ADDRESS_U10_A_CONTROL) & 0xF7);
  BSOS_DataWrite(ADDRESS_U11_A, (BSOS_DataRead(ADDRESS_U11_A) & 0x03) | 0x01);
  BSOS_DataWrite(ADDRESS_U10_A, 0x0F);

  // Write current display digits to 5 displays
  for (int displayCount=0; displayCount<5; displayCount++) {

    if (CurrentDisplayDigit<BSOS_NUM_DIGITS) {
      // The BCD for this digit is in b4-b7, and the display latch strobes are in b0-b3 (and U11A:b0)
      byte displayDataByte = ((DisplayDigits[displayCount][CurrentDisplayDigit])<<4) | 0x0F;
      byte displayEnable = ((DisplayDigitEnable[displayCount])>>CurrentDisplayDigit)&0x01;

      // if this digit shouldn't be displayed, then set data lines to 0xFX so digit will be blank
      if (!displayEnable) displayDataByte = 0xFF;
#ifdef BALLY_STERN_OS_DIMMABLE_DISPLAYS        
      if (DisplayDim[displayCount] && DisplayOffCycle) displayDataByte = 0xFF;
#endif        

      // Set low the appropriate latch strobe bit
      if (displayCount<4) {
        displayDataByte &= ~(0x01<<displayCount);
      }
      // Write out the digit & strobe (if it's 0-3)
      BSOS_DataWrite(ADDRESS_U10_A, displayDataByte);
      if (displayCount==4) {            
        // Strobe #5 latch on U11A:b0
        BSOS_DataWrite(ADDRESS_U11_A, BSOS_DataRead(ADDRESS_U11_A) & 0xFE);
      }

      // Need to delay a little to make sure the strobe is low for long enough
      WaitClockCycle(4);

      // Put the latch strobe bits back high
      if (displayCount<4) {
        displayDataByte |= 0x0F;
        BSOS_DataWrite(ADDRESS_U10_A, displayDataByte);
      } else {
        BSOS_DataWrite(ADDRESS_U11_A, BSOS_DataRead(ADDRESS_U11_A) | 0x01);
        
        // Set proper display digit enable
#ifdef BALLY_STERN_OS_USE_7_DIGIT_DISPLAYS          
        byte displayDigitsMask = (0x02<<CurrentDisplayDigit) | 0x01;
#else
        byte displayDigitsMask = (0x04<<CurrentDisplayDigit) | 0x01;
#endif          
        BSOS_DataWrite(ADDRESS_U11_A, displayDigitsMask);
      }
    }
  }

  // Stop Blanking (current digits are all latched and ready)
  BSOS_DataWrite(ADDRESS_U10_A_CONTROL, BSOS_DataRead(ADDRESS_U10_A_CONTROL) | 0x08);

  // Restore 10A from backup
  BSOS_DataWrite(ADDRESS_U10_A, backupU10A);    

  CurrentDisplayDigit = CurrentDisplayDigit + 1;
  if (CurrentDisplayDigit>=BSOS_NUM_DIGITS) {
    CurrentDisplayDigit = 0;
    DisplayOffCycle ^= true;
  }
}

void InterruptService3() {
  byte u10AControl = BSOS_DataRead(ADDRESS_U10_A_CONTROL);
  if (u10AControl & 0x80) {
    // self test switch
    if (BSOS_DataRead(ADDRESS_U10_A_CONTROL) & 0x80) PushToSwitchStack(SW_SELF_TEST_SWITCH);
    BSOS_DataRead(ADDRESS_U10_A);
  }

  // If we get a weird interupt from U11B, clear it
  byte u11BControl = BSOS_DataRead(ADDRESS_U11_B_CONTROL);
  if (u11BControl & 0x80) {
    BSOS_DataRead(ADDRESS_U11_B);    
  }

  byte u11AControl = BSOS_DataRead(ADDRESS_U11_A_CONTROL);
  byte u10BControl = BSOS_DataRead(ADDRESS_U10_B_CONTROL);

  // If the interrupt bit on the display interrupt is on, do the display refresh
  if (u11AControl & 0x80) {
    BSOS_DataRead(ADDRESS_U11_A);
    numberOfU11Interrupts+=1;
  }

  // If the IRQ bit of U10BControl is set, do the Zero-crossing interrupt handler
  if ((u10BControl & 0x80) && (InsideZeroCrossingInterrupt==0)) {
    InsideZeroCrossingInterrupt = InsideZeroCrossingInterrupt + 1;

    byte u10BControlLatest = BSOS_DataRead(ADDRESS_U10_B_CONTROL);

    // Backup contents of U10A
    byte backup10A = BSOS_DataRead(ADDRESS_U10_A);

    // Latch 0xFF separately without interrupt clear
    BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) | 0x08);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) & 0xF7);
    // Read U10B to clear interrupt
    BSOS_DataRead(ADDRESS_U10_B);

    // Turn off U10BControl interrupts
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x30);

    // Copy old switch values
    byte switchCount;
    byte startingClosures;
    byte validClosures;
    for (switchCount=0; switchCount<5; switchCount++) {
      SwitchesMinus2[switchCount] = SwitchesMinus1[switchCount];
      SwitchesMinus1[switchCount] = SwitchesNow[switchCount];

      // Enable playfield strobe
      BSOS_DataWrite(ADDRESS_U10_A, 0x01<<switchCount);
      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x34);

      // Delay for switch capacitors to charge
      WaitClockCycle(BSOS_NUM_SWITCH_LOOPS);
      
      // Read the switches
      SwitchesNow[switchCount] = BSOS_DataRead(ADDRESS_U10_B);

      //Unset the strobe
      BSOS_DataWrite(ADDRESS_U10_A, 0x00);

      // Some switches need to trigger immediate closures (bumpers & slings)
      startingClosures = (SwitchesNow[switchCount]) & (~SwitchesMinus1[switchCount]);
      boolean immediateSolenoidFired = false;
      // If one of the switches is starting to close (off, on)
      if (startingClosures) {
        // Loop on bits of switch byte
        for (byte bitCount=0; bitCount<8 && immediateSolenoidFired==false; bitCount++) {
          // If this switch bit is closed
          if (startingClosures&0x01) {
            byte startingSwitchNum = switchCount*8 + bitCount;
            // Loop on immediate switch data
            for (int immediateSwitchCount=0; immediateSwitchCount<NumGamePrioritySwitches && immediateSolenoidFired==false; immediateSwitchCount++) {
              // If this switch requires immediate action
              if (GameSwitches && startingSwitchNum==GameSwitches[immediateSwitchCount].switchNum) {
                // Start firing this solenoid (just one until the closure is validate
                PushToFrontOfSolenoidStack(GameSwitches[immediateSwitchCount].solenoid, 1);
                immediateSolenoidFired = true;
              }
            }
          }
          startingClosures = startingClosures>>1;
        }
      }

      immediateSolenoidFired = false;
      validClosures = (SwitchesNow[switchCount] & SwitchesMinus1[switchCount]) & ~SwitchesMinus2[switchCount];
      // If there is a valid switch closure (off, on, on)
      if (validClosures) {
        // Loop on bits of switch byte
        for (byte bitCount=0; bitCount<8; bitCount++) {
          // If this switch bit is closed
          if (validClosures&0x01) {
            byte validSwitchNum = switchCount*8 + bitCount;
            // Loop through all switches and see what's triggered
            for (int validSwitchCount=0; validSwitchCount<NumGameSwitches; validSwitchCount++) {

              // If we've found a valid closed switch
              if (GameSwitches && GameSwitches[validSwitchCount].switchNum==validSwitchNum) {

                // If we're supposed to trigger a solenoid, then do it
                if (GameSwitches[validSwitchCount].solenoid!=SOL_NONE) {
                  if (validSwitchCount<NumGamePrioritySwitches && immediateSolenoidFired==false) {
                    PushToFrontOfSolenoidStack(GameSwitches[validSwitchCount].solenoid, GameSwitches[validSwitchCount].solenoidHoldTime);
                  } else {
                    BSOS_PushToSolenoidStack(GameSwitches[validSwitchCount].solenoid, GameSwitches[validSwitchCount].solenoidHoldTime);
                  }
                } // End if this is a real solenoid
              } // End if this is a switch in the switch table
            } // End loop on switches in switch table
            // Push this switch to the game rules stack
            PushToSwitchStack(validSwitchNum);
          }
          validClosures = validClosures>>1;
        }        
      }

      // There are no port reads or writes for the rest of the loop, 
      // so we can allow the display interrupt to fire
      interrupts();
      
      // Wait so total delay will allow lamp SCRs to get to the proper voltage
      WaitClockCycle(BSOS_NUM_LAMP_LOOPS);
      
      noInterrupts();
    }
    BSOS_DataWrite(ADDRESS_U10_A, backup10A);

    // If we need to turn off momentary solenoids, do it first
    byte momentarySolenoidAtStart = PullFirstFromSolenoidStack();
    if (momentarySolenoidAtStart!=SOLENOID_STACK_EMPTY) {
      CurrentSolenoidByte = (CurrentSolenoidByte&0xF0) | momentarySolenoidAtStart;
      BSOS_DataWrite(ADDRESS_U11_B, CurrentSolenoidByte);
    } else {
      CurrentSolenoidByte = (CurrentSolenoidByte&0xF0) | SOL_NONE;
      BSOS_DataWrite(ADDRESS_U11_B, CurrentSolenoidByte);
    }

#ifndef BALLY_STERN_OS_USE_AUX_LAMPS
    for (int lampBitCount = 0; lampBitCount<BSOS_NUM_LAMP_BITS; lampBitCount++) {
      byte lampData = 0xF0 + lampBitCount;

      interrupts();
      BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
      noInterrupts();
      
      // Latch address & strobe
      BSOS_DataWrite(ADDRESS_U10_A, lampData);
#ifdef BSOS_SLOW_DOWN_LAMP_STROBE      
      WaitClockCycle();
#endif      

      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x38);
#ifdef BSOS_SLOW_DOWN_LAMP_STROBE      
      WaitClockCycle();
#endif      

      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x30);
#ifdef BSOS_SLOW_DOWN_LAMP_STROBE      
      WaitClockCycle();
#endif      

      // Use the inhibit lines to set the actual data to the lamp SCRs 
      // (here, we don't care about the lower nibble because the address was already latched)
      byte lampOutput = LampStates[lampBitCount];
      // Every other time through the cycle, we OR in the dim variable
      // in order to dim those lights
      if (numberOfU10Interrupts%DimDivisor1) lampOutput |= LampDim0[lampBitCount];
      if (numberOfU10Interrupts%DimDivisor2) lampOutput |= LampDim1[lampBitCount];

      BSOS_DataWrite(ADDRESS_U10_A, lampOutput);
#ifdef BSOS_SLOW_DOWN_LAMP_STROBE      
      WaitClockCycle();
#endif      
    }

    // Latch 0xFF separately without interrupt clear
    BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) | 0x08);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) & 0xF7);

#else 

    for (int lampBitCount=0; lampBitCount<15; lampBitCount++) {
      byte lampData = 0xF0 + lampBitCount;

      interrupts();
      BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
      noInterrupts();
      
      // Latch address & strobe
      BSOS_DataWrite(ADDRESS_U10_A, lampData);
#ifdef BSOS_SLOW_DOWN_LAMP_STROBE      
      WaitClockCycle();
#endif      

      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x38);
#ifdef BSOS_SLOW_DOWN_LAMP_STROBE      
      WaitClockCycle();
#endif      

      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x30);
#ifdef BSOS_SLOW_DOWN_LAMP_STROBE      
      WaitClockCycle();
#endif      

      // Use the inhibit lines to set the actual data to the lamp SCRs 
      // (here, we don't care about the lower nibble because the address was already latched)
      byte lampOutput = LampStates[lampBitCount];
      // Every other time through the cycle, we OR in the dim variable
      // in order to dim those lights
      if (numberOfU10Interrupts%DimDivisor1) lampOutput |= LampDim0[lampBitCount];
      if (numberOfU10Interrupts%DimDivisor2) lampOutput |= LampDim1[lampBitCount];

      BSOS_DataWrite(ADDRESS_U10_A, lampOutput);
#ifdef BSOS_SLOW_DOWN_LAMP_STROBE      
      WaitClockCycle();
#endif      

    }
    // Latch 0xFF separately without interrupt clear
    // to park 0xFF in main lamp board
    BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) | 0x08);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) & 0xF7);

    for (int lampBitCount=15; lampBitCount<22; lampBitCount++) {
      byte lampOutput = (LampStates[lampBitCount]&0xF0) | (lampBitCount-15);
      // Every other time through the cycle, we OR in the dim variable
      // in order to dim those lights
      if (numberOfU10Interrupts%DimDivisor1) lampOutput |= LampDim0[lampBitCount];
      if (numberOfU10Interrupts%DimDivisor2) lampOutput |= LampDim1[lampBitCount];

      interrupts();
      BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
      noInterrupts();

      BSOS_DataWrite(ADDRESS_U10_A, lampOutput | 0xF0);
      BSOS_DataWrite(ADDRESS_U11_A_CONTROL, BSOS_DataRead(ADDRESS_U11_A_CONTROL) | 0x08);
      BSOS_DataWrite(ADDRESS_U11_A_CONTROL, BSOS_DataRead(ADDRESS_U11_A_CONTROL) & 0xF7);    
      BSOS_DataWrite(ADDRESS_U10_A, lampOutput);
    }
    
    BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
    BSOS_DataWrite(ADDRESS_U11_A_CONTROL, BSOS_DataRead(ADDRESS_U11_A_CONTROL) | 0x08);
    BSOS_DataWrite(ADDRESS_U11_A_CONTROL, BSOS_DataRead(ADDRESS_U11_A_CONTROL) & 0xF7);

#endif 

    interrupts();
    noInterrupts();

    InsideZeroCrossingInterrupt = 0;
    BSOS_DataWrite(ADDRESS_U10_A, backup10A);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, u10BControlLatest);

    // Read U10B to clear interrupt
    BSOS_DataRead(ADDRESS_U10_B);
    numberOfU10Interrupts+=1;
  }
}



#endif 




void BSOS_SetDisplay(int displayNumber, unsigned long value, boolean blankByMagnitude, byte minDigits) {
  if (displayNumber<0 || displayNumber>4) return;

  byte blank = 0x00;

  for (int count=0; count<BSOS_NUM_DIGITS; count++) {
    blank = blank * 2;
    if (value!=0 || count<minDigits) blank |= 1;
    DisplayDigits[displayNumber][(BSOS_NUM_DIGITS-1)-count] = value%10;
    value /= 10;    
  }

  if (blankByMagnitude) DisplayDigitEnable[displayNumber] = blank;
//  else DisplayDigitEnable[displayNumber] = 0x3F;
}

void BSOS_SetDisplayBlank(int displayNumber, byte bitMask) {
  if (displayNumber<0 || displayNumber>4) return;
  
  DisplayDigitEnable[displayNumber] = bitMask;
}

// This is confusing -
// Digit mask is like this
//   bit=   b7 b6 b5 b4 b3 b2 b1 b0
//   digit=  x  x  6  5  4  3  2  1
//   (with digit 6 being the least-significant, 1's digit
//  
// so, looking at it from left to right on the display
//   digit=  1  2  3  4  5  6
//   bit=   b0 b1 b2 b3 b4 b5

/*
void BSOS_SetDisplayBlankByMagnitude(int displayNumber, unsigned long value, byte minDigits) {
  if (displayNumber<0 || displayNumber>4) return;

  DisplayDigitEnable[displayNumber] = 0x20;
  if (value>9 || minDigits>1) DisplayDigitEnable[displayNumber] |= 0x10;
  if (value>99 || minDigits>2) DisplayDigitEnable[displayNumber] |= 0x08;
  if (value>999 || minDigits>3) DisplayDigitEnable[displayNumber] |= 0x04;
  if (value>9999 || minDigits>4) DisplayDigitEnable[displayNumber] |= 0x02;
  if (value>99999 || minDigits>5) DisplayDigitEnable[displayNumber] |= 0x01;
}
*/

byte BSOS_GetDisplayBlank(int displayNumber) {
  if (displayNumber<0 || displayNumber>4) return 0;
  return DisplayDigitEnable[displayNumber];
}

#if defined(BALLY_STERN_OS_SOFTWARE_DISPLAY_INTERRUPT) && defined(BALLY_STERN_OS_ADJUSTABLE_DISPLAY_INTERRUPT)
void BSOS_SetDisplayRefreshConstant(int intervalConstant) {
  cli();
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for selected increment
  OCR1A = intervalConstant;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();
}
#endif


/*
void BSOS_SetDisplayBlankForCreditMatch(boolean creditsOn, boolean matchOn) {
  DisplayDigitEnable[4] = 0;
  if (creditsOn) DisplayDigitEnable[4] |= 0x03;
  if (matchOn) DisplayDigitEnable[4] |= 0x18;
}
*/

void BSOS_SetDisplayFlash(int displayNumber, unsigned long value, unsigned long curTime, int period, byte minDigits) {
  // A period of zero toggles display every other time
  if (period) {
    if ((curTime/period)%2) {
      BSOS_SetDisplay(displayNumber, value, true, minDigits);
    } else {
      BSOS_SetDisplayBlank(displayNumber, 0);
    }
  }
  
}


void BSOS_SetDisplayFlashCredits(unsigned long curTime, int period) {
  if (period) {
    if ((curTime/period)%2) {
      DisplayDigitEnable[4] |= 0x06;
    } else {
      DisplayDigitEnable[4] &= 0x39;
    }
  }
}


void BSOS_SetDisplayCredits(int value, boolean displayOn, boolean showBothDigits) {
#ifdef BALLY_STERN_OS_USE_6_DIGIT_CREDIT_DISPLAY_WITH_7_DIGIT_DISPLAYS
  DisplayDigits[4][2] = (value%100) / 10;
  DisplayDigits[4][3] = (value%10);
  byte enableMask = DisplayDigitEnable[4] & BALLY_STERN_OS_MASK_SHIFT_1;
#else
  DisplayDigits[4][1] = (value%100) / 10;
  DisplayDigits[4][2] = (value%10);
  byte enableMask = DisplayDigitEnable[4] & BALLY_STERN_OS_MASK_SHIFT_1;
#endif 

  if (displayOn) {
#ifdef BALLY_STERN_OS_USE_6_DIGIT_CREDIT_DISPLAY_WITH_7_DIGIT_DISPLAYS
  if (value>9 || showBothDigits) enableMask |= BALLY_STERN_OS_MASK_SHIFT_2;
#else
  if (value>9 || showBothDigits) enableMask |= BALLY_STERN_OS_MASK_SHIFT_2; 
#endif
    else enableMask |= 0x04;
  }

  DisplayDigitEnable[4] = enableMask;
}

void BSOS_SetDisplayMatch(int value, boolean displayOn, boolean showBothDigits) {
  BSOS_SetDisplayBallInPlay(value, displayOn, showBothDigits);
}

void BSOS_SetDisplayBallInPlay(int value, boolean displayOn, boolean showBothDigits) {
#ifdef BALLY_STERN_OS_USE_6_DIGIT_CREDIT_DISPLAY_WITH_7_DIGIT_DISPLAYS
  DisplayDigits[4][5] = (value%100) / 10;
  DisplayDigits[4][6] = (value%10); 
  byte enableMask = DisplayDigitEnable[4] & BALLY_STERN_OS_MASK_SHIFT_2;
#else
  DisplayDigits[4][4] = (value%100) / 10;
  DisplayDigits[4][5] = (value%10); 
  byte enableMask = DisplayDigitEnable[4] & BALLY_STERN_OS_MASK_SHIFT_2;
#endif

  if (displayOn) {
#ifdef BALLY_STERN_OS_USE_6_DIGIT_CREDIT_DISPLAY_WITH_7_DIGIT_DISPLAYS
  if (value>9 || showBothDigits) enableMask |= BALLY_STERN_OS_MASK_SHIFT_1;
#else
  if (value>9 || showBothDigits) enableMask |= BALLY_STERN_OS_MASK_SHIFT_1;
#endif
      else enableMask |= 0x20;
  }

  DisplayDigitEnable[4] = enableMask;
}


/*
void BSOS_SetDisplayBIPBlank(byte digitsOn) {
  if (digitsOn==0) DisplayDigitEnable[4] &= 0x0F;
  else if (digitsOn==1) DisplayDigitEnable[4] = (DisplayDigitEnable[4] & 0x0F)|0x20;
  else if (digitsOn==2) DisplayDigitEnable[4] = (DisplayDigitEnable[4] & 0x0F)|0x30;  
}
*/

void BSOS_SetDimDivisor(byte level, byte divisor) {
  if (level==1) DimDivisor1 = divisor;
  if (level==2) DimDivisor2 = divisor;
}

void BSOS_SetLampState(int lampNum, byte s_lampState, byte s_lampDim, int s_lampFlashPeriod) {
  if (lampNum>=BSOS_MAX_LAMPS || lampNum<0) return;
  
  if (s_lampState) {
    int adjustedLampFlash = s_lampFlashPeriod/50;
    
    if (s_lampFlashPeriod!=0 && adjustedLampFlash==0) adjustedLampFlash = 1;
    if (adjustedLampFlash>250) adjustedLampFlash = 250;
    
    // Only turn on the lamp if there's no flash, because if there's a flash
    // then the lamp will be turned on by the ApplyFlashToLamps function
    if (s_lampFlashPeriod==0) LampStates[lampNum/4] &= ~(0x10<<(lampNum%4));
    LampFlashPeriod[lampNum] = adjustedLampFlash;
  } else {
    LampStates[lampNum/4] |= (0x10<<(lampNum%4));
    LampFlashPeriod[lampNum] = 0;
  }

  if (s_lampDim & 0x01) {    
    LampDim0[lampNum/4] |= (0x10<<(lampNum%4));
  } else {
    LampDim0[lampNum/4] &= ~(0x10<<(lampNum%4));
  }

  if (s_lampDim & 0x02) {    
    LampDim1[lampNum/4] |= (0x10<<(lampNum%4));
  } else {
    LampDim1[lampNum/4] &= ~(0x10<<(lampNum%4));
  }

}


void BSOS_ApplyFlashToLamps(unsigned long curTime) {
  for (int count=0; count<BSOS_MAX_LAMPS; count++) {
    if ( LampFlashPeriod[count]!=0 ) {
      unsigned long adjustedLampFlash = (unsigned long)LampFlashPeriod[count] * (unsigned long)50;
      if ((curTime/adjustedLampFlash)%2) {
        LampStates[count/4] &= ~(0x10<<(count%4));
      } else {
        LampStates[count/4] |= (0x10<<(count%4));
      }
    } // end if this light should flash
  } // end loop on lights
}


void BSOS_FlashAllLamps(unsigned long curTime) {
  for (int count=0; count<BSOS_MAX_LAMPS; count++) {
    BSOS_SetLampState(count, 1, 0, 500);  
  }

  BSOS_ApplyFlashToLamps(curTime);
}

void BSOS_TurnOffAllLamps() {
  for (int count=0; count<BSOS_MAX_LAMPS; count++) {
    BSOS_SetLampState(count, 0, 0, 0);  
  }
}


void BSOS_InitializeMPU() {
  // Wait for board to boot
  delay(100);

  // Start out with everything tri-state, in case the original
  // CPU is running
  // Set data pins to input
  // Make pins 2-7 input
  DDRD = DDRD & 0x03;
  // Make pins 8-13 input
  DDRB = DDRB & 0xC0;
  // Set up the address lines A0-A5 as input (for now)
  DDRC = DDRC & 0xC0;

  unsigned long startTime = millis();
  boolean sawHigh = false;
  boolean sawLow = false;
  // for three seconds, look for activity on the VMA line (A5)
  // If we see anything, then the MPU is active so we shouldn't run
  while ((millis()-startTime)<2000) {
    if (digitalRead(A5)) sawHigh = true;
    else sawLow = true;
  }
  // If we saw both a high and low signal, then someone is toggling the 
  // VMA line, so we should hang here forever (until reset)
  if (sawHigh && sawLow) {
    while (1);
  }
  
  // Arduino A0 = MPU A0
  // Arduino A1 = MPU A1
  // Arduino A2 = MPU A3
  // Arduino A3 = MPU A4
  // Arduino A4 = MPU A7
  // Arduino A5 = MPU VMA
  // Set up the address lines A0-A7 as output
  DDRC = DDRC | 0x3F;

#if (BALLY_STERN_OS_HARDWARE_REV==1)
  // Set up D13 as address line A5 (and set it low)
  DDRB = DDRB | 0x20;
  PORTB = PORTB & 0xDF;
#elif (BALLY_STERN_OS_HARDWARE_REV==2) 
  // Set up D13 as address line A7 (and set it high)
  DDRB = DDRB | 0x20;
  PORTB = PORTB | 0x20;
#endif 

  // Set up A6 as output
  pinMode(A6, OUTPUT); // /HLT

  // Arduino 2 = /IRQ (input)
  // Arduino 3 = R/W (output)
  // Arduino 4 = Clk (input)
  // Arduino 5 = D0
  // Arduino 6 = D1
  // Arduino 7 = D3
  // Set up control lines & data lines
  DDRD = DDRD & 0xEB;
  DDRD = DDRD | 0xE8;

  digitalWrite(3, HIGH);  // Set R/W line high (Read)
  digitalWrite(A5, LOW);  // Set VMA line LOW
  digitalWrite(A6, HIGH); // Set

  pinMode(2, INPUT);

  // Prep the address bus (all lines zero)
  BSOS_DataRead(0);
  // Set up the PIAs
  InitializeU10PIA();
  InitializeU11PIA();

  // Read values from MPU dip switches
#ifdef BALLY_STERN_OS_USE_DIP_SWITCHES  
  ReadDipSwitches();
#endif 
  
  // Reset address bus
  BSOS_DataRead(0);

  // Reset solenoid stack
  SolenoidStackFirst = 0;
  SolenoidStackLast = 0;

  // Reset switch stack
  SwitchStackFirst = 0;
  SwitchStackLast = 0;

  CurrentDisplayDigit = 0; 

  // Set default values for the displays
  for (int displayCount=0; displayCount<5; displayCount++) {
    for (int digitCount=0; digitCount<6; digitCount++) {
      DisplayDigits[displayCount][digitCount] = 0;
    }
    DisplayDigitEnable[displayCount] = 0x03;
#ifdef BALLY_STERN_OS_DIMMABLE_DISPLAYS    
    DisplayDim[displayCount] = false;
#endif
  }

  // Turn off all lamp states
  for (int lampNibbleCounter=0; lampNibbleCounter<BSOS_NUM_LAMP_BITS; lampNibbleCounter++) {
    LampStates[lampNibbleCounter] = 0xFF;
    LampDim0[lampNibbleCounter] = 0x00;
    LampDim1[lampNibbleCounter] = 0x00;
  }

  for (int lampFlashCount=0; lampFlashCount<BSOS_MAX_LAMPS; lampFlashCount++) {
    LampFlashPeriod[lampFlashCount] = 0;
  }

  // Reset all the switch values 
  // (set them as closed so that if they're stuck they don't register as new events)
  byte switchCount;
  for (switchCount=0; switchCount<5; switchCount++) {
    SwitchesMinus2[switchCount] = 0xFF;
    SwitchesMinus1[switchCount] = 0xFF;
    SwitchesNow[switchCount] = 0xFF;
  }
  
  // Hook up the interrupt
#if not defined (BALLY_STERN_OS_SOFTWARE_DISPLAY_INTERRUPT)
  attachInterrupt(digitalPinToInterrupt(2), InterruptService2, LOW);
#else
/*
  cli();
  TCCR2A|=(1<<WGM21);     //Set the CTC mode
  OCR2A=0xBA;            //Set the value for 3ms
  TIMSK2|=(1<<OCIE2A);   //Set the interrupt request
  TCCR2B|=(1<<CS22);     //Set the prescale 1/64 clock
  sei();                 //Enable interrupt
*/  

  cli();
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for selected increment
  OCR1A = BALLY_STERN_OS_SOFTWARE_DISPLAY_INTERRUPT_INTERVAL;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();
  
  attachInterrupt(digitalPinToInterrupt(2), InterruptService3, LOW);
#endif  
  BSOS_DataRead(0);  // Reset address bus

  // Cleary all possible interrupts by reading the registers
  BSOS_DataRead(ADDRESS_U11_A);
  BSOS_DataRead(ADDRESS_U11_B);
  BSOS_DataRead(ADDRESS_U10_A);
  BSOS_DataRead(ADDRESS_U10_B);
  BSOS_DataRead(0);  // Reset address bus

}

byte BSOS_GetDipSwitches(byte index) {
#ifdef BALLY_STERN_OS_USE_DIP_SWITCHES
  if (index>3) return 0x00;
  return DipSwitches[index];
#else
  return 0x00 & index;
#endif
}


void BSOS_SetupGameSwitches(int s_numSwitches, int s_numPrioritySwitches, PlayfieldAndCabinetSwitch *s_gameSwitchArray) {
  NumGameSwitches = s_numSwitches;
  NumGamePrioritySwitches = s_numPrioritySwitches;
  GameSwitches = s_gameSwitchArray;
}


/*
void BSOS_SetupGameLights(int s_numLights, PlayfieldLight *s_gameLightArray) {
  NumGameLights = s_numLights;
  GameLights = s_gameLightArray;
}
*/
/*
void BSOS_SetContinuousSolenoids(byte continuousSolenoidMask = CONTSOL_DISABLE_FLIPPERS | CONTSOL_DISABLE_COIN_LOCKOUT) {
  CurrentSolenoidByte = (CurrentSolenoidByte&0x0F) | continuousSolenoidMask;
  BSOS_DataWrite(ADDRESS_U11_B, CurrentSolenoidByte);
}
*/


void BSOS_SetCoinLockout(boolean lockoutOn, byte solbit) {
  if (lockoutOn) {
    CurrentSolenoidByte = CurrentSolenoidByte & ~solbit;
  } else {
    CurrentSolenoidByte = CurrentSolenoidByte | solbit;
  }
  BSOS_DataWrite(ADDRESS_U11_B, CurrentSolenoidByte);
}


void BSOS_SetDisableFlippers(boolean disableFlippers, byte solbit) {
  if (disableFlippers) {
    CurrentSolenoidByte = CurrentSolenoidByte | solbit;
  } else {
    CurrentSolenoidByte = CurrentSolenoidByte & ~solbit;
  }
  
  BSOS_DataWrite(ADDRESS_U11_B, CurrentSolenoidByte);
}


byte BSOS_ReadContinuousSolenoids() {
  return BSOS_DataRead(ADDRESS_U11_B);
}


void BSOS_DisableSolenoidStack() {
  SolenoidStackEnabled = false;
}


void BSOS_EnableSolenoidStack() {
  SolenoidStackEnabled = true;
}



void BSOS_CycleAllDisplays(unsigned long curTime, byte digitNum) {
  int displayDigit = (curTime/250)%10;
  unsigned long value;
#ifdef BALLY_STERN_OS_USE_7_DIGIT_DISPLAYS
  value = displayDigit*1111111;
#else  
  value = displayDigit*111111;
#endif

  byte displayNumToShow = 0;
  byte displayBlank = BALLY_STERN_OS_ALL_DIGITS_MASK;

  if (digitNum!=0) {
    displayNumToShow = (digitNum-1)/6;
#ifdef BALLY_STERN_OS_USE_7_DIGIT_DISPLAYS
    displayBlank = (0x40)>>((digitNum-1)%7);
#else    
    displayBlank = (0x20)>>((digitNum-1)%6);
#endif    
  }

  for (int count=0; count<5; count++) {
    if (digitNum) {
      BSOS_SetDisplay(count, value);
      if (count==displayNumToShow) BSOS_SetDisplayBlank(count, displayBlank);
      else BSOS_SetDisplayBlank(count, 0);
    } else {
      BSOS_SetDisplay(count, value, true);
    }
  }
}

#ifdef BALLY_STERN_OS_USE_SQUAWK_AND_TALK

void BSOS_PlaySoundSquawkAndTalk(byte soundByte) {

  byte oldSolenoidControlByte, soundControlByte;

  // mask further zero-crossing interrupts during this 
  InsideZeroCrossingInterrupt += 1;

  // Get the current value of U11:PortB
  oldSolenoidControlByte = BSOS_DataRead(ADDRESS_U11_B);
  soundControlByte = oldSolenoidControlByte; 
  
  // Mask off momentary solenoids
  soundControlByte &= 0xF0;
  // Add in lower nibble
  soundControlByte |= (soundByte&0x0F);

  // Put 1s on momentary solenoid lines
  BSOS_DataWrite(ADDRESS_U11_B, oldSolenoidControlByte | 0x0F);

  // Strobe sound latch
  BSOS_DataWrite(ADDRESS_U11_B_CONTROL, BSOS_DataRead(ADDRESS_U11_B_CONTROL)|0x08);
  
  // put the new byte on U11:PortB
  BSOS_DataWrite(ADDRESS_U11_B, soundControlByte);
  
  // wait 142 microseconds
  delayMicroseconds(142);

  // remove lower nibble
  soundControlByte &= 0xF0;
  // Put upper nibble on lines
  soundControlByte |= (soundByte/16);
  // put the new byte on U11:PortB
  BSOS_DataWrite(ADDRESS_U11_B, soundControlByte);

  // wait 78 microseconds
  delayMicroseconds(78);
  BSOS_DataWrite(ADDRESS_U11_B, oldSolenoidControlByte);

  // Turn off sound latch
  BSOS_DataWrite(ADDRESS_U11_B_CONTROL, BSOS_DataRead(ADDRESS_U11_B_CONTROL)&0xF7);

  InsideZeroCrossingInterrupt -= 1;
}
#endif

// With hardware rev 1, this function relies on D13 being connected to A5 because it writes to address 0xA0
// A0  - A0   0
// A1  - A1   0   
// A2  - n/c  0
// A3  - A2   0
// A4  - A3   0
// A5  - D13  1
// A6  - n/c  0
// A7  - A4   1
// A8  - n/c  0
// A9  - GND  0
// A10 - n/c  0
// A11 - n/c  0
// A12 - GND  0
// A13 - n/c  0
#ifdef BALLY_STERN_OS_USE_SB100
void BSOS_PlaySB100(byte soundByte) {

#if (BALLY_STERN_OS_HARDWARE_REV==1)
  PORTB = PORTB | 0x20;
#endif 

  BSOS_DataWrite(ADDRESS_SB100, soundByte);

#if (BALLY_STERN_OS_HARDWARE_REV==1)
  PORTB = PORTB & 0xDF;
#endif 
  
}

#if (BALLY_STERN_OS_HARDWARE_REV==2)
void BSOS_PlaySB100Chime(byte soundByte) {

  BSOS_DataWrite(ADDRESS_SB100_CHIMES, soundByte);

}
#endif 
#endif


#if (BALLY_STERN_OS_HARDWARE_REV==2 && defined(BALLY_STERN_OS_USE_SB300))

void BSOS_PlaySB300SquareWave(byte soundRegister, byte soundByte) {
  BSOS_DataWrite(ADDRESS_SB300_SQUARE_WAVES+soundRegister, soundByte);
}

void BSOS_PlaySB300Analog(byte soundRegister, byte soundByte) {
  BSOS_DataWrite(ADDRESS_SB300_ANALOG+soundRegister, soundByte);
}

#endif 

// EEProm Helper functions

void BSOS_WriteByteToEEProm(unsigned short startByte, byte value) {
  EEPROM.write(startByte, value);
}

byte BSOS_ReadByteFromEEProm(unsigned short startByte) {
  byte value = EEPROM.read(startByte);

  // If this value is unset, set it
  if (value==0xFF) {
    value = 0;
    BSOS_WriteByteToEEProm(startByte, value);
  }
  return value;
}



unsigned long BSOS_ReadULFromEEProm(unsigned short startByte, unsigned long defaultValue) {
  unsigned long value;

  value = (((unsigned long)EEPROM.read(startByte+3))<<24) | 
          ((unsigned long)(EEPROM.read(startByte+2))<<16) | 
          ((unsigned long)(EEPROM.read(startByte+1))<<8) | 
          ((unsigned long)(EEPROM.read(startByte)));

  if (value==0xFFFFFFFF) {
    value = defaultValue; 
    BSOS_WriteULToEEProm(startByte, value);
  }
  return value;
}


void BSOS_WriteULToEEProm(unsigned short startByte, unsigned long value) {
  EEPROM.write(startByte+3, (byte)(value>>24));
  EEPROM.write(startByte+2, (byte)((value>>16) & 0x000000FF));
  EEPROM.write(startByte+1, (byte)((value>>8) & 0x000000FF));
  EEPROM.write(startByte, (byte)(value & 0x000000FF));
}
