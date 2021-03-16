int SwitchChargeDelay     = 150;
int SwitchDischargeDelay  = 100;

//#define BALLY_STERN_OS_USE_AUX_LAMPS

#ifdef BALLY_STERN_OS_USE_AUX_LAMPS
#define BSOS_NUM_LAMP_BITS 22
#define BSOS_MAX_LAMPS     88
#else
#define BSOS_NUM_LAMP_BITS 15
#define BSOS_MAX_LAMPS     60
#endif 

#define SOL_NONE 0x0F
#define BSOS_SLOW_DOWN_LAMP_STROBE  0

#define ADDRESS_U10_A           0x14
#define ADDRESS_U10_A_CONTROL   0x15
#define ADDRESS_U10_B           0x16
#define ADDRESS_U10_B_CONTROL   0x17
#define ADDRESS_U11_A           0x18
#define ADDRESS_U11_A_CONTROL   0x19
#define ADDRESS_U11_B           0x1A
#define ADDRESS_U11_B_CONTROL   0x1B



#define ADDRESS_U10_A           0x14
#define ADDRESS_U10_A_CONTROL   0x15
#define ADDRESS_U10_B           0x16
#define ADDRESS_U10_B_CONTROL   0x17
#define ADDRESS_U11_A           0x18
#define ADDRESS_U11_A_CONTROL   0x19
#define ADDRESS_U11_B           0x1A
#define ADDRESS_U11_B_CONTROL   0x1B
#define ADDRESS_SB100           0x10

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


byte DipSwitches[4];

void WaitOneClockCycle() {
  // Wait while clock is low
  while(!(PIND & 0x10));

  // Wait for a falling edge of the clock
  while((PIND & 0x10));
}

void WaitClockCycles(unsigned long numCycles) {
  for (unsigned long count=0; count<numCycles; count++) {
    // Wait while clock is low
    while(!(PIND & 0x10));
  
    // Wait for a falling edge of the clock
    while((PIND & 0x10));
  }
}



void ReadDipSwitches() {
  /*byte backupU10A = */BSOS_DataRead(ADDRESS_U10_A);
  byte backupU10BControl = BSOS_DataRead(ADDRESS_U10_B_CONTROL);
  delay(1);

  // Turn on Switch strobe 5 & Read Switches
  BSOS_DataWrite(ADDRESS_U10_A, 0x20);
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, backupU10BControl & 0xF7);
  // Wait for switch capacitors to charge
//  for (int count=0; count<SwitchChargeDelay; count++) WaitOneClockCycle();
  delayMicroseconds(SwitchChargeDelay);
  DipSwitches[0] = BSOS_DataRead(ADDRESS_U10_B);
  delay(1);

  // Turn on Switch strobe 6 & Read Switches
  BSOS_DataWrite(ADDRESS_U10_A, 0x40);
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, backupU10BControl & 0xF7);
  // Wait for switch capacitors to charge
//  for (int count=0; count<SwitchChargeDelay; count++) WaitOneClockCycle();
  delayMicroseconds(SwitchChargeDelay);
  DipSwitches[1] = BSOS_DataRead(ADDRESS_U10_B);
  delay(1);

  // Turn on Switch strobe 7 & Read Switches
  BSOS_DataWrite(ADDRESS_U10_A, 0x80);
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, backupU10BControl & 0xF7);
  // Wait for switch capacitors to charge
//  for (int count=0; count<SwitchChargeDelay; count++) WaitOneClockCycle();
  delayMicroseconds(SwitchChargeDelay);
  DipSwitches[2] = BSOS_DataRead(ADDRESS_U10_B);
  delay(1);

  // Turn on U10 CB2 (strobe 8) and read switches
  BSOS_DataWrite(ADDRESS_U10_A, 0x00);
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, backupU10BControl | 0x08);
  // Wait for switch capacitors to charge
//  for (int count=0; count<SwitchChargeDelay; count++) WaitOneClockCycle();
  delayMicroseconds(SwitchChargeDelay);
  DipSwitches[3] = BSOS_DataRead(ADDRESS_U10_B);
  delay(1);

//  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, backupU10BControl);
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x37);
  BSOS_DataWrite(ADDRESS_U10_A, 0x00);
  delay(1);

}


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
//  CurrentSolenoidByte = 0x9F;
  
}


volatile unsigned long numberOfU11Interrupts = 0;
volatile unsigned long numberOfU10Interrupts = 0;
volatile byte InsideZeroCrossingInterrupt = 0;
unsigned long TestStartTime = 0;
unsigned long TestEndTime = 0;


void TestFrequenciesISR() {

  byte u10AControl = BSOS_DataRead(ADDRESS_U10_A_CONTROL);
  if (u10AControl & 0x80) {
    // self test switch
    BSOS_DataRead(ADDRESS_U10_A);
    BSOS_DataRead(ADDRESS_U10_A);
  }

  byte u11AControl = BSOS_DataRead(ADDRESS_U11_A_CONTROL);
  byte u10BControl = BSOS_DataRead(ADDRESS_U10_B_CONTROL);

  // If the interrupt bit on the display interrupt is on, do the display refresh
  if (u11AControl & 0x80) {
    BSOS_DataRead(ADDRESS_U11_A);
    BSOS_DataRead(ADDRESS_U11_A);
    numberOfU11Interrupts += 1;
  }

  // If the IRQ bit of U10BControl is set, do the Zero-crossing interrupt handler
  if (u10BControl & 0x80) {
    BSOS_DataRead(ADDRESS_U10_B);
    BSOS_DataRead(ADDRESS_U10_B);
    numberOfU10Interrupts += 1;
  }
}

void StartFrequencyTest() {

  numberOfU10Interrupts = 0;
  numberOfU11Interrupts = 0;
  // Hook up the interrupt
  attachInterrupt(digitalPinToInterrupt(2), TestFrequenciesISR, LOW);
}

void FinishFrequencyTest() {

  // Detach the interrupt
  detachInterrupt(digitalPinToInterrupt(2));

  unsigned long totalMillis = millis() - TestStartTime;

  char buf[128];
  char floatStr[10];
  sprintf(buf, ".. In 5 seconds, saw %lu U10B interrupts and %lu U11A interrupts\n", numberOfU10Interrupts, numberOfU11Interrupts);
  Serial.write(buf);

  dtostrf(((float)numberOfU10Interrupts*1000.0f)/((float)totalMillis), 4, 2, floatStr);
  sprintf(buf, ".. Zero-crossing approx. %s times a second\n", floatStr);
  Serial.write(buf);
  dtostrf(((float)numberOfU11Interrupts*1000.0f)/((float)totalMillis), 4, 2, floatStr);
  sprintf(buf, ".. Display interrupt approx. %s times a second\n", floatStr);
  Serial.write(buf);

}



// Global variables
volatile byte SwitchesMinus2[5];
volatile byte SwitchesMinus1[5];
volatile byte SwitchesNow[5];

#define SWITCH_STACK_SIZE   64
#define SWITCH_STACK_EMPTY  0xFF
#define SW_SELF_TEST_SWITCH 0x7F
#define NOISY_SWITCH_OFFSET 50
volatile byte SwitchStackFirst;
volatile byte SwitchStackLast;
volatile byte SwitchStack[SWITCH_STACK_SIZE];

volatile byte DisplayDigits[5][6];
volatile byte DisplayDigitEnable[5];
volatile byte CurrentDisplayDigit=0;

volatile byte LampStates[BSOS_NUM_LAMP_BITS], LampDim0[BSOS_NUM_LAMP_BITS], LampDim1[BSOS_NUM_LAMP_BITS];
volatile byte LampFlashPeriod[BSOS_MAX_LAMPS];
byte DimDivisor1 = 2;
byte DimDivisor2 = 3;

int NumGameSwitches = 0;
int NumGamePrioritySwitches = 0;
struct PlayfieldAndCabinetSwitch {
  byte switchNum;
  byte solenoid;
  byte solenoidHoldTime;
};
PlayfieldAndCabinetSwitch *GameSwitches = NULL;

#define SOLENOID_STACK_SIZE 64
#define SOLENOID_STACK_EMPTY 0xFF
volatile byte SolenoidStackFirst;
volatile byte SolenoidStackLast;
volatile byte SolenoidStack[SOLENOID_STACK_SIZE];
boolean SolenoidStackEnabled = true;
volatile byte CurrentSolenoidByte = 0xFF;


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


void SwitchReadingISR() {
  
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
    
    // Disable lamp decoders & strobe latch
    BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) | 0x08);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, BSOS_DataRead(ADDRESS_U10_B_CONTROL) & 0xF7);

    // Blank Displays
    BSOS_DataWrite(ADDRESS_U10_A_CONTROL, BSOS_DataRead(ADDRESS_U10_A_CONTROL) & 0xF7);
    BSOS_DataWrite(ADDRESS_U11_A, (BSOS_DataRead(ADDRESS_U11_A) & 0x03) | 0x01);
    BSOS_DataWrite(ADDRESS_U10_A, 0x0F);

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

//    int waitCount = 0;
    
    // Copy old switch values
    byte switchCount;
    byte validClosures, noise;
    for (switchCount=0; switchCount<5; switchCount++) {
      SwitchesMinus2[switchCount] = SwitchesMinus1[switchCount];
      SwitchesMinus1[switchCount] = SwitchesNow[switchCount];

      // Enable playfield strobe
      BSOS_DataWrite(ADDRESS_U10_A, 0x01<<switchCount);
      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x34);

      // Delay for switch capacitors to charge
//      for (waitCount=0; waitCount<SwitchChargeDelay; waitCount++) WaitOneClockCycle();      
      delayMicroseconds(SwitchChargeDelay);
      
      // Read the switches
      SwitchesNow[switchCount] = BSOS_DataRead(ADDRESS_U10_B);

      //Unset the strobe
      BSOS_DataWrite(ADDRESS_U10_A, 0x00);

      validClosures = (SwitchesNow[switchCount] & SwitchesMinus1[switchCount]) & ~SwitchesMinus2[switchCount];
      // If there is a valid switch closure (off, on, on)
      if (validClosures) {
        // Loop on bits of switch byte
        for (byte bitCount=0; bitCount<8; bitCount++) {
          // If this switch bit is closed
          if (validClosures&0x01) {
            byte validSwitchNum = switchCount*8 + bitCount;
            PushToSwitchStack(validSwitchNum);
          }
          validClosures = validClosures>>1;
        }        
      }

      noise = ((~SwitchesNow[switchCount]) & SwitchesMinus1[switchCount]) & ~SwitchesMinus2[switchCount];
      if (noise) {
        // Loop on bits of switch byte
        for (byte bitCount=0; bitCount<8; bitCount++) {
          // If this switch bit is closed
          if (noise&0x01) {
            byte noisySwitchNum = (switchCount*8 + bitCount) + NOISY_SWITCH_OFFSET;
            PushToSwitchStack(noisySwitchNum);
          }
          noise = noise>>1;
        }        
      }

      // There are no port reads or writes for the rest of the loop, 
      // so we can allow the display interrupt to fire
      interrupts();
      
      // Wait so total delay will allow lamp SCRs to get to the proper voltage
//      for (waitCount=0; waitCount<SwitchDischargeDelay; waitCount++) WaitOneClockCycle();
      delayMicroseconds(SwitchDischargeDelay);
      noInterrupts();
    }
    BSOS_DataWrite(ADDRESS_U10_A, backup10A);

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



void SwitchAndDisplayISR() {
  
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

      if (CurrentDisplayDigit<6) {
        // The BCD for this digit is in b4-b7, and the display latch strobes are in b0-b3 (and U11A:b0)
        byte displayDataByte = ((DisplayDigits[displayCount][CurrentDisplayDigit])<<4) | 0x0F;
        byte displayEnable = ((DisplayDigitEnable[displayCount])>>CurrentDisplayDigit)&0x01;
  
        // if this digit shouldn't be displayed, then set data lines to 0xFX so digit will be blank
        if (!displayEnable) displayDataByte = 0xFF;
  
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
//        WaitOneClockCycle();
//        WaitOneClockCycle();
//        WaitOneClockCycle();
//        WaitOneClockCycle();
        delayMicroseconds(5);

        // Put the latch strobe bits back high
        if (displayCount<4) {
          displayDataByte |= 0x0F;
          BSOS_DataWrite(ADDRESS_U10_A, displayDataByte);
        } else {
          BSOS_DataWrite(ADDRESS_U11_A, BSOS_DataRead(ADDRESS_U11_A) | 0x01);
          
          // Set proper display digit enable
          byte displayDigitsMask = (0x04<<CurrentDisplayDigit) | 0x01;
          BSOS_DataWrite(ADDRESS_U11_A, displayDigitsMask);
        }
      }
    }

    // Stop Blanking (current digits are all latched and ready)
    BSOS_DataWrite(ADDRESS_U10_A_CONTROL, BSOS_DataRead(ADDRESS_U10_A_CONTROL) | 0x08);

    // Restore 10A from backup
    BSOS_DataWrite(ADDRESS_U10_A, backupU10A);    

    CurrentDisplayDigit = CurrentDisplayDigit + 1;
    if (CurrentDisplayDigit>5) {
      CurrentDisplayDigit = 0;
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

//    int waitCount = 0;
    
    // Copy old switch values
    byte switchCount;
    byte validClosures, noise;
    for (switchCount=0; switchCount<5; switchCount++) {
      SwitchesMinus2[switchCount] = SwitchesMinus1[switchCount];
      SwitchesMinus1[switchCount] = SwitchesNow[switchCount];

      // Enable playfield strobe
      BSOS_DataWrite(ADDRESS_U10_A, 0x01<<switchCount);
      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x34);

      // Delay for switch capacitors to charge
//      for (waitCount=0; waitCount<SwitchChargeDelay; waitCount++) WaitOneClockCycle();      
      delayMicroseconds(SwitchChargeDelay);
      
      // Read the switches
      SwitchesNow[switchCount] = BSOS_DataRead(ADDRESS_U10_B);

      //Unset the strobe
      BSOS_DataWrite(ADDRESS_U10_A, 0x00);

      validClosures = (SwitchesNow[switchCount] & SwitchesMinus1[switchCount]) & ~SwitchesMinus2[switchCount];
      // If there is a valid switch closure (off, on, on)
      if (validClosures) {
        // Loop on bits of switch byte
        for (byte bitCount=0; bitCount<8; bitCount++) {
          // If this switch bit is closed
          if (validClosures&0x01) {
            byte validSwitchNum = switchCount*8 + bitCount;
            PushToSwitchStack(validSwitchNum);
          }
          validClosures = validClosures>>1;
        }        
      }

      noise = ((~SwitchesNow[switchCount]) & SwitchesMinus1[switchCount]) & ~SwitchesMinus2[switchCount];
      if (noise) {
        // Loop on bits of switch byte
        for (byte bitCount=0; bitCount<8; bitCount++) {
          // If this switch bit is closed
          if (noise&0x01) {
            byte noisySwitchNum = (switchCount*8 + bitCount) + NOISY_SWITCH_OFFSET;
            PushToSwitchStack(noisySwitchNum);
          }
          noise = noise>>1;
        }        
      }

      // There are no port reads or writes for the rest of the loop, 
      // so we can allow the display interrupt to fire
      interrupts();
      
      // Wait so total delay will allow lamp SCRs to get to the proper voltage
//      for (waitCount=0; waitCount<SwitchDischargeDelay; waitCount++) WaitOneClockCycle();
      delayMicroseconds(SwitchDischargeDelay);
      noInterrupts();
    }
    BSOS_DataWrite(ADDRESS_U10_A, backup10A);

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



void SwitchDisplayLampISR() {
  
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

      if (CurrentDisplayDigit<6) {
        // The BCD for this digit is in b4-b7, and the display latch strobes are in b0-b3 (and U11A:b0)
        byte displayDataByte = ((DisplayDigits[displayCount][CurrentDisplayDigit])<<4) | 0x0F;
        byte displayEnable = ((DisplayDigitEnable[displayCount])>>CurrentDisplayDigit)&0x01;
  
        // if this digit shouldn't be displayed, then set data lines to 0xFX so digit will be blank
        if (!displayEnable) displayDataByte = 0xFF;
  
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
//        WaitOneClockCycle();
//        WaitOneClockCycle();
//        WaitOneClockCycle();
//        WaitOneClockCycle();
        delayMicroseconds(5);

        // Put the latch strobe bits back high
        if (displayCount<4) {
          displayDataByte |= 0x0F;
          BSOS_DataWrite(ADDRESS_U10_A, displayDataByte);
        } else {
          BSOS_DataWrite(ADDRESS_U11_A, BSOS_DataRead(ADDRESS_U11_A) | 0x01);
          
          // Set proper display digit enable
          byte displayDigitsMask = (0x04<<CurrentDisplayDigit) | 0x01;
          BSOS_DataWrite(ADDRESS_U11_A, displayDigitsMask);
        }
      }
    }

    // Stop Blanking (current digits are all latched and ready)
    BSOS_DataWrite(ADDRESS_U10_A_CONTROL, BSOS_DataRead(ADDRESS_U10_A_CONTROL) | 0x08);

    // Restore 10A from backup
    BSOS_DataWrite(ADDRESS_U10_A, backupU10A);    

    CurrentDisplayDigit = CurrentDisplayDigit + 1;
    if (CurrentDisplayDigit>5) {
      CurrentDisplayDigit = 0;
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

//    int waitCount = 0;
    
    // Copy old switch values
    byte switchCount;
    byte validClosures;
    byte noise;
    for (switchCount=0; switchCount<5; switchCount++) {
      SwitchesMinus2[switchCount] = SwitchesMinus1[switchCount];
      SwitchesMinus1[switchCount] = SwitchesNow[switchCount];

      // Enable playfield strobe
      BSOS_DataWrite(ADDRESS_U10_A, 0x01<<switchCount);
      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x34);

      // Delay for switch capacitors to charge
//      for (waitCount=0; waitCount<SwitchChargeDelay; waitCount++) WaitOneClockCycle();      
      delayMicroseconds(SwitchChargeDelay);
      
      // Read the switches
      SwitchesNow[switchCount] = BSOS_DataRead(ADDRESS_U10_B);

      //Unset the strobe
      BSOS_DataWrite(ADDRESS_U10_A, 0x00);

      validClosures = (SwitchesNow[switchCount] & SwitchesMinus1[switchCount]) & ~SwitchesMinus2[switchCount];
      // If there is a valid switch closure (off, on, on)
      if (validClosures) {
        // Loop on bits of switch byte
        for (byte bitCount=0; bitCount<8; bitCount++) {
          // If this switch bit is closed
          if (validClosures&0x01) {
            byte validSwitchNum = switchCount*8 + bitCount;
            PushToSwitchStack(validSwitchNum);
          }
          validClosures = validClosures>>1;
        }        
      }

      noise = ((~SwitchesNow[switchCount]) & SwitchesMinus1[switchCount]) & ~SwitchesMinus2[switchCount];
      if (noise) {
        // Loop on bits of switch byte
        for (byte bitCount=0; bitCount<8; bitCount++) {
          // If this switch bit is closed
          if (noise&0x01) {
            byte noisySwitchNum = (switchCount*8 + bitCount) + NOISY_SWITCH_OFFSET;
            PushToSwitchStack(noisySwitchNum);
          }
          noise = noise>>1;
        }        
      }

      // There are no port reads or writes for the rest of the loop, 
      // so we can allow the display interrupt to fire
      interrupts();
      
      // Wait so total delay will allow lamp SCRs to get to the proper voltage
//      for (waitCount=0; waitCount<SwitchDischargeDelay; waitCount++) WaitOneClockCycle();
      delayMicroseconds(SwitchDischargeDelay);
      noInterrupts();
    }
    BSOS_DataWrite(ADDRESS_U10_A, backup10A);

#ifndef BALLY_STERN_OS_USE_AUX_LAMPS
    for (int lampBitCount = 0; lampBitCount<BSOS_NUM_LAMP_BITS; lampBitCount++) {
      byte lampData = 0xF0 + lampBitCount;

      interrupts();
      BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
      noInterrupts();
      
      // Latch address & strobe
      BSOS_DataWrite(ADDRESS_U10_A, lampData);

      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x38);

      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x30);

      // Use the inhibit lines to set the actual data to the lamp SCRs 
      // (here, we don't care about the lower nibble because the address was already latched)
      byte lampOutput = LampStates[lampBitCount];
      // Every other time through the cycle, we OR in the dim variable
      // in order to dim those lights
      if (numberOfU10Interrupts%DimDivisor1) lampOutput |= LampDim0[lampBitCount];
      if (numberOfU10Interrupts%DimDivisor2) lampOutput |= LampDim1[lampBitCount];

      BSOS_DataWrite(ADDRESS_U10_A, lampOutput);
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

      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x38);

      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x30);

      // Use the inhibit lines to set the actual data to the lamp SCRs 
      // (here, we don't care about the lower nibble because the address was already latched)
      byte lampOutput = LampStates[lampBitCount];
      // Every other time through the cycle, we OR in the dim variable
      // in order to dim those lights
      if (numberOfU10Interrupts%DimDivisor1) lampOutput |= LampDim0[lampBitCount];
      if (numberOfU10Interrupts%DimDivisor2) lampOutput |= LampDim1[lampBitCount];

      BSOS_DataWrite(ADDRESS_U10_A, lampOutput);

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




int SpaceLeftOnSolenoidStack() {
  if (SolenoidStackFirst>=SOLENOID_STACK_SIZE || SolenoidStackLast>=SOLENOID_STACK_SIZE) return 0;
  if (SolenoidStackLast>=SolenoidStackFirst) return ((SOLENOID_STACK_SIZE-1) - (SolenoidStackLast-SolenoidStackFirst));
  return (SolenoidStackFirst - SolenoidStackLast) - 1;
}


void BSOS_PushToSolenoidStack(byte solenoidNumber, byte numPushes, boolean disableOverride = false){
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

// I think this should go before 10A is blasted with FF above
    // Backup U10A
//    byte backupU10A = BSOS_DataRead(ADDRESS_U10_A);

    // Blank Displays
    BSOS_DataWrite(ADDRESS_U10_A_CONTROL, BSOS_DataRead(ADDRESS_U10_A_CONTROL) & 0xF7);
    BSOS_DataWrite(ADDRESS_U11_A, (BSOS_DataRead(ADDRESS_U11_A) & 0x03) | 0x01);
    BSOS_DataWrite(ADDRESS_U10_A, 0x0F);

    // Write current display digits to 5 displays
    for (int displayCount=0; displayCount<5; displayCount++) {

      if (CurrentDisplayDigit<7) {
        // The BCD for this digit is in b4-b7, and the display latch strobes are in b0-b3 (and U11A:b0)
        byte displayDataByte = ((DisplayDigits[displayCount][CurrentDisplayDigit])<<4) | 0x0F;
        byte displayEnable = ((DisplayDigitEnable[displayCount])>>CurrentDisplayDigit)&0x01;
  
        // if this digit shouldn't be displayed, then set data lines to 0xFX so digit will be blank
        if (!displayEnable) displayDataByte = 0xFF;
//        if (DisplayDim[displayCount] && DisplayOffCycle) displayDataByte = 0xFF;
  
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
//        WaitOneClockCycle();
//        WaitOneClockCycle();
//        WaitOneClockCycle();
//        WaitOneClockCycle();
        delayMicroseconds(5);

        // Put the latch strobe bits back high
        if (displayCount<4) {
          displayDataByte |= 0x0F;
          BSOS_DataWrite(ADDRESS_U10_A, displayDataByte);
        } else {
          BSOS_DataWrite(ADDRESS_U11_A, BSOS_DataRead(ADDRESS_U11_A) | 0x01);
          
          // Set proper display digit enable
          byte displayDigitsMask = (0x02<<CurrentDisplayDigit) | 0x01;
          BSOS_DataWrite(ADDRESS_U11_A, displayDigitsMask);
        }
      }
    }

    // Stop Blanking (current digits are all latched and ready)
    BSOS_DataWrite(ADDRESS_U10_A_CONTROL, BSOS_DataRead(ADDRESS_U10_A_CONTROL) | 0x08);

    // Restore 10A from backup
    BSOS_DataWrite(ADDRESS_U10_A, backupU10A);

    CurrentDisplayDigit = CurrentDisplayDigit + 1;
    if (CurrentDisplayDigit>6) {
      CurrentDisplayDigit = 0;
//      DisplayOffCycle ^= true;
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

//    int waitCount = 0;
    
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
//      for (waitCount=0; waitCount<SwitchChargeDelay; waitCount++) WaitOneClockCycle();      
      delayMicroseconds(SwitchChargeDelay);
      
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
//      for (waitCount=0; waitCount<SwitchDischargeDelay; waitCount++) WaitOneClockCycle();
      delayMicroseconds(SwitchDischargeDelay);
      
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
//      if (BSOS_SLOW_DOWN_LAMP_STROBE) WaitOneClockCycle();
      if (BSOS_SLOW_DOWN_LAMP_STROBE) delayMicroseconds(1);

      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x38);
//      if (BSOS_SLOW_DOWN_LAMP_STROBE) WaitOneClockCycle();
      if (BSOS_SLOW_DOWN_LAMP_STROBE) delayMicroseconds(1);

      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x30);
//      if (BSOS_SLOW_DOWN_LAMP_STROBE) WaitOneClockCycle();
      if (BSOS_SLOW_DOWN_LAMP_STROBE) delayMicroseconds(1);

      // Use the inhibit lines to set the actual data to the lamp SCRs 
      // (here, we don't care about the lower nibble because the address was already latched)
      byte lampOutput = LampStates[lampBitCount];
      // Every other time through the cycle, we OR in the dim variable
      // in order to dim those lights
      if (numberOfU10Interrupts%DimDivisor1) lampOutput |= LampDim0[lampBitCount];
      if (numberOfU10Interrupts%DimDivisor2) lampOutput |= LampDim1[lampBitCount];

      BSOS_DataWrite(ADDRESS_U10_A, lampOutput);
//      if (BSOS_SLOW_DOWN_LAMP_STROBE) WaitOneClockCycle();
      if (BSOS_SLOW_DOWN_LAMP_STROBE) delayMicroseconds(1);
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
//      if (BSOS_SLOW_DOWN_LAMP_STROBE) WaitOneClockCycle();
      if (BSOS_SLOW_DOWN_LAMP_STROBE) delayMicroseconds(1);

      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x38);
//      if (BSOS_SLOW_DOWN_LAMP_STROBE) WaitOneClockCycle();
      if (BSOS_SLOW_DOWN_LAMP_STROBE) delayMicroseconds(1);

      BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x30);
//      if (BSOS_SLOW_DOWN_LAMP_STROBE) WaitOneClockCycle();
      if (BSOS_SLOW_DOWN_LAMP_STROBE) delayMicroseconds(1);

      // Use the inhibit lines to set the actual data to the lamp SCRs 
      // (here, we don't care about the lower nibble because the address was already latched)
      byte lampOutput = LampStates[lampBitCount];
      // Every other time through the cycle, we OR in the dim variable
      // in order to dim those lights
      if (numberOfU10Interrupts%DimDivisor1) lampOutput |= LampDim0[lampBitCount];
      if (numberOfU10Interrupts%DimDivisor2) lampOutput |= LampDim1[lampBitCount];

      BSOS_DataWrite(ADDRESS_U10_A, lampOutput);
//      if (BSOS_SLOW_DOWN_LAMP_STROBE) WaitOneClockCycle();
      if (BSOS_SLOW_DOWN_LAMP_STROBE) delayMicroseconds(1);

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
    BSOS_DataWrite(ADDRESS_U10_A, 0x0F);

#endif 

    interrupts();
    noInterrupts();

    // If we need to start any solenoids, do them now
    // (we know we need to start if we weren't already firing any solenoids
    // and there's currently something on the stack)
    if (0 && momentarySolenoidAtStart==SOLENOID_STACK_EMPTY) {
      byte startingMomentarySolenoid = PullFirstFromSolenoidStack();
      if (startingMomentarySolenoid!=SOL_NONE) {
        CurrentSolenoidByte = (CurrentSolenoidByte&0xF0) | startingMomentarySolenoid;
        BSOS_DataWrite(ADDRESS_U11_B, CurrentSolenoidByte);
      }
    }

    InsideZeroCrossingInterrupt = 0;
    BSOS_DataWrite(ADDRESS_U10_A, backup10A);
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, u10BControlLatest);

    // Read U10B to clear interrupt
    BSOS_DataRead(ADDRESS_U10_B);
    numberOfU10Interrupts+=1;
  }
}




void StartSwitchTest(bool testWithDisplays, bool testWithLamps, bool fullTest) {
  // Reset switch stack
  SwitchStackFirst = 0;
  SwitchStackLast = 0;

  SwitchChargeDelay     = 120;
  SwitchDischargeDelay  = 70;

  for (int count=0; count<5; count++) {
    SwitchesMinus2[count] = 0xFF;
    SwitchesMinus1[count] = 0xFF;
    SwitchesNow[count] = 0xFF;
  }

  CurrentDisplayDigit=0;
  for (int displayCount=0; displayCount<5; displayCount++) {
    for (int digitCount=0; digitCount<6; digitCount++) {
      DisplayDigits[displayCount][digitCount] = digitCount + 1;
    }
    DisplayDigitEnable[displayCount] = 0x3F;
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

  numberOfU10Interrupts = 0;
  numberOfU11Interrupts = 0;
  // Hook up the interrupt
  if (!fullTest) {
    if (!testWithDisplays && !testWithLamps) attachInterrupt(digitalPinToInterrupt(2), SwitchReadingISR, LOW);
    else if (testWithDisplays && !testWithLamps) attachInterrupt(digitalPinToInterrupt(2), SwitchAndDisplayISR, LOW);
    else attachInterrupt(digitalPinToInterrupt(2), SwitchDisplayLampISR, LOW);
  } else {
    attachInterrupt(digitalPinToInterrupt(2), InterruptService2, LOW);
  }
}

void FinishSwitchTest() {
  
  // Detach the interrupt
  detachInterrupt(digitalPinToInterrupt(2));
  
}



/*******
 * This program does the following diagnostics:
 * 1) Checks to make sure the M6800 is not running
 * 2) Initializes the PIA chips
 * 3) Reads DIP switches and reports value through serial port
 * 4) Attaches interrupt and counts frequency of display & zero-crossing interrupt
 * 5) Attaches interrupt to read switches
 */


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



void CheckForM6800() {
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
    
}

void setup() {
  Serial.begin(115200);
  Serial.write("Start of program\n");

  CheckForM6800();

  Serial.write("Didn't see M6800 running ... continuing\n");

  // Set up the address lines A0-A7 as output
  DDRC = DDRC | 0x3F;

  // Set up control lines & data lines
  DDRD = DDRD & 0xEB;
  DDRD = DDRD | 0xE8;

  InitializeU10PIA();
  InitializeU11PIA();
}


byte TestNumber = 0;
byte LastClockState = 0;
unsigned long NumberOfRisingClocks = 0;
unsigned long LastTimingChange;


void loop() {
  char buf[128];

  switch (TestNumber) {
    case 0:
      ReadDipSwitches();
      for (byte count=0; count<4; count++) {
        sprintf(buf, ".. DIP bank %d = 0x%02X\n", count, DipSwitches[count]);
        Serial.write(buf);
      }
      Serial.write("Done Reading DIP switches\n");
      TestNumber += 1;
      TestStartTime = 0;
    break;
    case 1:
      if (TestStartTime==0) {
        Serial.write("Starting frequency test - this will take 5 seconds\n");
        StartFrequencyTest();
        TestStartTime = millis();
      } else if ( (millis()-TestStartTime)>5000) {
        FinishFrequencyTest();
        Serial.write("Done with frequency test\n");
        TestNumber += 1;
        TestStartTime = 0;
      }
    break;
    case 2:
      if (TestStartTime==0) {
        Serial.write("Starting clock test - this will take about 2 seconds\n");
        NumberOfRisingClocks = 0;
        LastClockState = 0;
        TestStartTime = millis();
//        for (unsigned long count=0; count<1000000; count++) {
//          WaitOneClockCycle();
//        }
        WaitClockCycles(1000000);
        TestEndTime = millis();
      } else {
        sprintf(buf, ".. Clock frequency approximately %d kHz\n", (int)(1000000/(TestEndTime-TestStartTime)));
        Serial.write(buf);
        Serial.write("Done with clock test\n");
        TestNumber += 1;
        TestStartTime = 0;
      }      
    break;
    case 3:
      if (TestStartTime==0) {
        Serial.write("Starting switch test - reporting switch closures for 30 seconds\n");
        StartSwitchTest(false, false, false);
        LastTimingChange = millis();
        TestStartTime = millis();
      } else if ( (millis()-TestStartTime)>30000 ) {
        FinishSwitchTest();
        Serial.write("Done with switch test\n");
        TestNumber += 1;
        TestStartTime = 0;
      } else if ( (millis()-LastTimingChange)>5000 ) {
        LastTimingChange = millis();
        SwitchChargeDelay -= 10;
        SwitchDischargeDelay -= 10;
        if (SwitchChargeDelay<10) SwitchChargeDelay = 10;
        if (SwitchDischargeDelay<10) SwitchDischargeDelay = 10;
        sprintf(buf, ".. Switch Charge Delay = %d, Switch Discharge Delay = %d\n", SwitchChargeDelay, SwitchDischargeDelay);        
        Serial.write(buf);
      } else {
        byte switchHit = BSOS_PullFirstFromSwitchStack();
        while (switchHit!=SWITCH_STACK_EMPTY) {
          if (switchHit<NOISY_SWITCH_OFFSET) {
            sprintf(buf, ".. Switch Hit = 0x%02X\n", switchHit);
          } else {
            sprintf(buf, ".. Noise on switch = 0x%02X\n", (switchHit-NOISY_SWITCH_OFFSET));
          }
          Serial.write(buf);
          switchHit = BSOS_PullFirstFromSwitchStack();
        }
      }
    break;
    case 4:
      if (TestStartTime==0) {
        Serial.write("Starting switch test with Displays (30 seconds)\n");
        StartSwitchTest(true, false, false);
        LastTimingChange = millis();
        TestStartTime = millis();
      } else if ( (millis()-TestStartTime)>30000 ) {
        FinishSwitchTest();
        Serial.write("Done with switch/display test\n");
        TestNumber += 1;
        TestStartTime = 0;
      } else if ( (millis()-LastTimingChange)>5000 ) {
        LastTimingChange = millis();
        SwitchChargeDelay -= 10;
        SwitchDischargeDelay -= 10;
        if (SwitchChargeDelay<10) SwitchChargeDelay = 10;
        if (SwitchDischargeDelay<10) SwitchDischargeDelay = 10;
        sprintf(buf, ".. Switch Charge Delay = %d, Switch Discharge Delay = %d\n", SwitchChargeDelay, SwitchDischargeDelay);        
        Serial.write(buf);
      } else {
        byte switchHit = BSOS_PullFirstFromSwitchStack();
        while (switchHit!=SWITCH_STACK_EMPTY) {
          if (switchHit<NOISY_SWITCH_OFFSET) {
            sprintf(buf, ".. Switch Hit = 0x%02X\n", switchHit);
          } else {
            sprintf(buf, ".. Noise on switch = 0x%02X\n", (switchHit-NOISY_SWITCH_OFFSET));
          }
          Serial.write(buf);
          switchHit = BSOS_PullFirstFromSwitchStack();
        }
      }    
    break;
    case 5:
      if (TestStartTime==0) {
        Serial.write("Starting switch test with Displays & Lamps (60 seconds)\n");
        StartSwitchTest(true, true, false);
        LastTimingChange = millis();
        TestStartTime = millis();
      } else if ( (millis()-TestStartTime)>60000 ) {
        FinishSwitchTest();
        Serial.write("Done with switch/display/lamp test\n");
        TestNumber += 1;
        TestStartTime = 0;
      } else if ( (millis()-LastTimingChange)>5000 ) {
        LastTimingChange = millis();
        SwitchChargeDelay -= 10;
        SwitchDischargeDelay -= 10;
        if (SwitchChargeDelay<10) SwitchChargeDelay = 10;
        if (SwitchDischargeDelay<10) SwitchDischargeDelay = 10;
        sprintf(buf, ".. Switch Charge Delay = %d, Switch Discharge Delay = %d\n", SwitchChargeDelay, SwitchDischargeDelay);        
        Serial.write(buf);
      } else {
        byte switchHit = BSOS_PullFirstFromSwitchStack();
        while (switchHit!=SWITCH_STACK_EMPTY) {
          if (switchHit<NOISY_SWITCH_OFFSET) {
            sprintf(buf, ".. Switch Hit = 0x%02X\n", switchHit);
          } else {
            sprintf(buf, ".. Noise on switch = 0x%02X\n", (switchHit-NOISY_SWITCH_OFFSET));
          }
          Serial.write(buf);
          switchHit = BSOS_PullFirstFromSwitchStack();
        }

        byte lampOnNum = (millis()/100)%BSOS_MAX_LAMPS;
        for (int count=0; count<BSOS_MAX_LAMPS; count++) {
          BSOS_SetLampState(count, (count==lampOnNum)||count<10, 0, 0);
        }
      }    
      
    break;
    case 6:
      if (TestStartTime==0) {
        Serial.write("Starting switch test with full ISR (60 seconds)\n");
        StartSwitchTest(true, true, true);
        LastTimingChange = millis();
        TestStartTime = millis();
      } else if ( (millis()-TestStartTime)>60000 ) {
        FinishSwitchTest();
        Serial.write("Done with switch/display/lamp/solenoid test\n");
        TestNumber += 1;
        TestStartTime = 0;
      } else if ( (millis()-LastTimingChange)>5000 ) {
        LastTimingChange = millis();
        SwitchChargeDelay -= 10;
        SwitchDischargeDelay -= 10;
        if (SwitchChargeDelay<10) SwitchChargeDelay = 10;
        if (SwitchDischargeDelay<10) SwitchDischargeDelay = 10;
        sprintf(buf, ".. Switch Charge Delay = %d, Switch Discharge Delay = %d\n", SwitchChargeDelay, SwitchDischargeDelay);        
        Serial.write(buf);
      } else {
        byte switchHit = BSOS_PullFirstFromSwitchStack();
        while (switchHit!=SWITCH_STACK_EMPTY) {
          if (switchHit<NOISY_SWITCH_OFFSET) {
            sprintf(buf, ".. Switch Hit = 0x%02X\n", switchHit);
          } else {
            sprintf(buf, ".. Noise on switch = 0x%02X\n", (switchHit-NOISY_SWITCH_OFFSET));
          }
          Serial.write(buf);
          switchHit = BSOS_PullFirstFromSwitchStack();
        }

        byte lampOnNum = (millis()/100)%BSOS_MAX_LAMPS;
        for (int count=0; count<BSOS_MAX_LAMPS; count++) {
          BSOS_SetLampState(count, (count==lampOnNum)||count<10, 0, 0);
        }
      }    
      
    break;
  }

  BSOS_DataRead(0);
}
