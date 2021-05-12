#define BSOS_SWITCH_DELAY_IN_MICROSECONDS         250
#define BSOS_TIMING_LOOP_PADDING_IN_MICROSECONDS  50
#define BALLY_STERN_OS_HARDWARE_REV   1


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

  byte newPortB, newPortD;
  newPortB = (PORTB&0xE0) | (data>>3);
  newPortD = (PORTD&0x1F) | ((data&0x07)<<5);
  newPortD = newPortD & 0xF7;

  // Wait while clock is low
//  while(!(PIND & 0x10));

  // Wait while clock is high
  while((PIND & 0x10));

  // Set VMA OFF
//  PORTC = PORTC & 0xDF;

  // Set up address lines
  PORTC = (PORTC & 0xC0) | address;

  // Set data pins to output
  // Make pins 5-7 output (and pin 3 for R/W)
  DDRD = DDRD | 0xE8;
  // Make pins 8-12 output
  DDRB = DDRB | 0x1F;

  // Wait while clock is low
  while(!(PIND & 0x10));

  // Wait while clock is high
  while((PIND & 0x10));

  // Set R/W to LOW
  PORTD = (PORTD & 0xF7);
  
  // Set VMA ON
  PORTC = PORTC | 0x20;

  // Put data on pins
  // Put lower three bits on 5-7
  PORTD = newPortD;
  // Put upper five bits on 8-12
  PORTB = newPortB;

  // Wait while clock is low
  while(!(PIND & 0x10));

  // Wait while clock is high
  while((PIND & 0x10));

  // Wait while clock is low
//  while(!(PIND & 0x10));

  // Set R/W back to HIGH
  PORTD = (PORTD | 0x08);

  // Set data pins to input
  // Make pins 5-7 input
  DDRD = DDRD & 0x1F;
  // Make pins 8-12 input
  DDRB = DDRB & 0xE0;
//  PORTB |= 0x1F;
//  PORTD |= 0xE0;
  
  // Turn off address lines so we don't clear interrupts
  PORTC = (PORTC & 0xE0);
#if (BALLY_STERN_OS_HARDWARE_REV==1)
  // Leave A7 on
  PORTC |= 0x10;
#endif
}


byte BSOS_DataRead(int address) {

  // Wait while clock is low
  while(!(PIND & 0x10));

  // Wait while the clock is high
  while((PIND & 0x10));

  // Set up address lines
  PORTC = (PORTC & 0xE0) | address;

  // Wait while clock is low
  while(!(PIND & 0x10));

  // Wait while the clock is high
  while((PIND & 0x10));
  
  // Wait while clock is low
  while(!(PIND & 0x10));

  byte inputData = (PIND>>5) | (PINB<<3);

  // Turn off address lines so we don't clear interrupts
  PORTC = (PORTC & 0xE0);
#if (BALLY_STERN_OS_HARDWARE_REV==1)
  // Leave A7 on
  PORTC |= 0x10;
#endif

  return inputData;
}




boolean TestControlRegister(int controlRegisterAddress, byte value) {
  BSOS_DataWrite(controlRegisterAddress, value);
  byte returnVal = BSOS_DataRead(controlRegisterAddress);
  char buf[100];
  if ( (returnVal&0x3F)!=value ) {
    // report failure
    sprintf(buf, "    FAIL: Wrote 0x%02X, got back 0x%02X (masked to 0x%02X)\n", value, returnVal, (returnVal&0x3F));
    Serial.write(buf);
    return false;
  }
  return true;
}

boolean TestPort(int portAddress) {
  char buf[100];
  boolean testPassed = true;
  for (int count=0; count<256; count++) {
    BSOS_DataWrite(portAddress, count & 0xFF);
    delayMicroseconds(40); // This is a loop of 3 in the original code
    byte returnValue = BSOS_DataRead(portAddress);
    if (returnValue!=count) {
      sprintf(buf, "    FAIL: Wrote 0x%02X, got back 0x%02X\n", count, returnValue);
      Serial.write(buf);
      testPassed = false;
    }
  }
  return testPassed;
}

boolean TestControlRegisterAndPort(int controlRegisterAddress, int portAddress, const char *portName) {
  boolean passed = true;
  Serial.write("Testing port ");
  Serial.write(portName);
  Serial.write("\n");
  Serial.write("  Control Register & DDR\n");
  passed &= TestControlRegister(controlRegisterAddress, 0x31);
  passed &= TestControlRegister(controlRegisterAddress, 0x39);
  passed &= TestPort(portAddress);
  Serial.write("  Control Register & Port\n");
  passed &= TestControlRegister(controlRegisterAddress, 0x35);
  passed &= TestControlRegister(controlRegisterAddress, 0x3D);
  passed &= TestPort(portAddress);
  if (passed) {
    Serial.write("Port ");
    Serial.write(portName);
    Serial.write(" PASSED\n");
  }

  return passed;
}


boolean TestPIAChips() {

  boolean testPassed = true;

  testPassed &= TestControlRegisterAndPort(ADDRESS_U10_A_CONTROL, ADDRESS_U10_A, "U10A");
  testPassed &= TestControlRegisterAndPort(ADDRESS_U10_B_CONTROL, ADDRESS_U10_B, "U10B");
  testPassed &= TestControlRegisterAndPort(ADDRESS_U11_A_CONTROL, ADDRESS_U11_A, "U11A");
  testPassed &= TestControlRegisterAndPort(ADDRESS_U11_B_CONTROL, ADDRESS_U11_B, "U11B");

  if (!testPassed) return true;
  return false;
/*
  boolean anyRegisterFailed = false;
  boolean chipPassed = true;
  byte testVal; 

  Serial.write("Attempting to read & write from U10\n");
  Serial.write("  Testing U10A Control Register\n");

  for (int count=0; count<256; count++) {
    BSOS_DataWrite(ADDRESS_U10_A_CONTROL, count);
    testVal = (BSOS_DataRead(ADDRESS_U10_A_CONTROL))&0x3F;
    char buf[128];
    if (testVal!=(count&0x3F)) {
      sprintf(buf, "    FAIL: Wrote 0x%02X, got back 0x%02X\n", count, testVal);
      chipPassed = false;
      anyRegisterFailed = true;
      Serial.write(buf);
    }
    delay(1);
  }

  if (!chipPassed) Serial.write("    The U10A control register failed\n");
  else Serial.write("    The U10A control register passed\n");

  chipPassed = true;
  Serial.write("  Testing U10B Control Register\n");
  for (int count=0; count<256; count++) {
    BSOS_DataWrite(ADDRESS_U10_B_CONTROL, count);
    testVal = (BSOS_DataRead(ADDRESS_U10_B_CONTROL))&0x3F;
    char buf[128];
    if (testVal!=(count&0x3F)) {
      sprintf(buf, "    FAIL: Wrote 0x%02X, got back 0x%02X\n", count, testVal);
      chipPassed = false;
      anyRegisterFailed = true;
      Serial.write(buf);
    }
    delay(1);
  }

  if (!chipPassed) Serial.write("    The U10B control register failed\n");
  else Serial.write("    The U10B control register passed\n");

  
  Serial.write("  Testing U11A Control Register\n");
  for (int count=0; count<256; count++) {
    BSOS_DataWrite(ADDRESS_U11_A_CONTROL, count);
    testVal = (BSOS_DataRead(ADDRESS_U11_A_CONTROL))&0x3F;
    char buf[128];
    if (testVal!=(count&0x3F)) {
      sprintf(buf, "    FAIL: Wrote 0x%02X, got back 0x%02X\n", count, testVal);
      chipPassed = false;
      anyRegisterFailed = true;
      Serial.write(buf);
    }
    delay(1);
  }

  if (!chipPassed) Serial.write("    The U11A control register failed\n");
  else Serial.write("    The U11A control register passed\n");

  chipPassed = true;
  Serial.write("  Testing U11B Control Register\n");
  for (int count=0; count<256; count++) {
    BSOS_DataWrite(ADDRESS_U11_B_CONTROL, count);
    testVal = (BSOS_DataRead(ADDRESS_U11_B_CONTROL))&0x3F;
    char buf[128];
    if (testVal!=(count&0x3F)) {
      sprintf(buf, "    FAIL: Wrote 0x%02X, got back 0x%02X\n", count, testVal);
      chipPassed = false;
      anyRegisterFailed = true;
      Serial.write(buf);
    }
    delay(1);
  }

  if (!chipPassed) Serial.write("    The U11B control register failed\n");
  else Serial.write("    The U11B control register passed\n");

  return anyRegisterFailed;
*/
}



void InitializeU10PIA() {
  // CA1 - Self Test Switch
  // CB1 - zero crossing detector
  // CA2 - NOR'd with display latch strobe
  // CB2 - lamp strobe 1
  // PA0-7 - output for switch bank, lamps, and BCD
  // PB0-7 - switch returns
  byte newCRVal;

  BSOS_DataWrite(ADDRESS_U10_A_CONTROL, 0x38);
  // Set up U10A as output
  BSOS_DataWrite(ADDRESS_U10_A, 0xFF);
  // Set bit 3 to write data
  newCRVal = BSOS_DataRead(ADDRESS_U10_A_CONTROL)|0x04;
  BSOS_DataWrite(ADDRESS_U10_A_CONTROL, newCRVal);
  // Store F0 in U10A Output
  BSOS_DataWrite(ADDRESS_U10_A, 0xF0);
  
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x33);
  // Set up U10B as input
  BSOS_DataWrite(ADDRESS_U10_B, 0x00);
  // Set bit 3 so future reads will read data
  newCRVal = BSOS_DataRead(ADDRESS_U10_B_CONTROL)|0x04;
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, newCRVal);


}


byte DipSwitches[4];

void WaitClockCycles(int numCycles) {
  for (int count=0; count<numCycles; count++) {
    // Wait while clock is low
    while(!(PIND & 0x10));
  
    // Wait for a falling edge of the clock
    while((PIND & 0x10));
  }
}


void ReadDipSwitches() {
  byte backupU10A = BSOS_DataRead(ADDRESS_U10_A);
  byte backupU10BControl = BSOS_DataRead(ADDRESS_U10_B_CONTROL);
  BSOS_DataWrite(ADDRESS_U10_A_CONTROL, 0x3C);
  BSOS_DataWrite(ADDRESS_U10_A, 0x00);
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x33);
  BSOS_DataWrite(ADDRESS_U10_B, 0x00);
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, 0x37);
  delay(1);

  // Turn on Switch strobe 5 & Read Switches
  BSOS_DataWrite(ADDRESS_U10_A, 0x20);
  // Wait for switch capacitors to charge
  delayMicroseconds(BSOS_SWITCH_DELAY_IN_MICROSECONDS);
  DipSwitches[0] = BSOS_DataRead(ADDRESS_U10_B);
  BSOS_DataWrite(ADDRESS_U10_A, 0x00);
  delayMicroseconds(BSOS_TIMING_LOOP_PADDING_IN_MICROSECONDS);

  // Turn on Switch strobe 6 & Read Switches
  BSOS_DataWrite(ADDRESS_U10_A, 0x40);
  // Wait for switch capacitors to charge
  delayMicroseconds(BSOS_SWITCH_DELAY_IN_MICROSECONDS);
  DipSwitches[1] = BSOS_DataRead(ADDRESS_U10_B);
  BSOS_DataWrite(ADDRESS_U10_A, 0x00);
  delayMicroseconds(BSOS_TIMING_LOOP_PADDING_IN_MICROSECONDS);

  // Read from a phony address to wake up the monitor
  BSOS_DataRead(ADDRESS_U10_B_CONTROL|ADDRESS_U11_B_CONTROL);

  // Turn on Switch strobe 7 & Read Switches
  BSOS_DataWrite(ADDRESS_U10_A, 0x80);
  // Wait for switch capacitors to charge
  delayMicroseconds(BSOS_SWITCH_DELAY_IN_MICROSECONDS);
  DipSwitches[2] = BSOS_DataRead(ADDRESS_U10_B);
  BSOS_DataWrite(ADDRESS_U10_A, 0x00);
  delayMicroseconds(BSOS_TIMING_LOOP_PADDING_IN_MICROSECONDS);

  // Turn on U10 CB2 (strobe 8) and read switches
  BSOS_DataWrite(ADDRESS_U10_A, 0x00);
  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, backupU10BControl | 0x08);
  // Wait for switch capacitors to charge
  delayMicroseconds(BSOS_SWITCH_DELAY_IN_MICROSECONDS);
  DipSwitches[3] = BSOS_DataRead(ADDRESS_U10_B);

  BSOS_DataWrite(ADDRESS_U10_B_CONTROL, backupU10BControl);
  BSOS_DataWrite(ADDRESS_U10_A, backupU10A);
}


void InitializeU11PIA() {
  // CA1 - Display interrupt generator
  // CB1 - test connector pin 32
  // CA2 - lamp strobe 2
  // CB2 - solenoid bank select
  // PA0-7 - display digit enable
  // PB0-7 - solenoid data
  byte newCRVal;

  BSOS_DataWrite(ADDRESS_U11_A_CONTROL, 0x31);
  // Set up U11A as output
  BSOS_DataWrite(ADDRESS_U11_A, 0xFF);
  // Set bit 3 to write data
  newCRVal = BSOS_DataRead(ADDRESS_U11_A_CONTROL)|0x04;
  BSOS_DataWrite(ADDRESS_U11_A_CONTROL, newCRVal);
  // Store 00 in U11A Output
  BSOS_DataWrite(ADDRESS_U11_A, 0x00);
  
  BSOS_DataWrite(ADDRESS_U11_B_CONTROL, 0x30);
  // Set up U11B as output
  BSOS_DataWrite(ADDRESS_U11_B, 0xFF);
  // Set bit 3 so future reads will read data
  newCRVal = BSOS_DataRead(ADDRESS_U11_B_CONTROL)|0x04;
  BSOS_DataWrite(ADDRESS_U11_B_CONTROL, newCRVal);
  // Store 9F in U11B Output
  BSOS_DataWrite(ADDRESS_U11_B, 0x9F);
//  CurrentSolenoidByte = 0x9F;

}

boolean SetupPIAChips() {
  boolean failed = false;
  
  InitializeU10PIA();
  InitializeU11PIA();
  ReadDipSwitches();

  Serial.write("Testing the clock cycle (if this hangs here, the conncection to clock is faulty)\n");
  unsigned long startTime = micros();
  WaitClockCycles(10000);
  unsigned long totalTime = micros() - startTime;
  char buf[128];
  sprintf(buf, "It took %lu microseconds for 10000 clock cycles\n", totalTime);
  Serial.write(buf);
  unsigned long frequency = 10000000 / totalTime;
  sprintf(buf, "Clock frequency (very) approximately = %lu kHz\n", frequency);
  Serial.write(buf);

  for (int count=0; count<4; count++) {
    char buf[128];
    sprintf(buf, "  DIP Bank %d = 0x%X\n", count, DipSwitches[count]);
    Serial.write(buf);
  }

  if (frequency<400 || frequency>2000) {
    Serial.write("  Clock frequency out of range\n");
    failed = true;
  }

  return failed;
}


volatile unsigned long numU11AInterrupts = 0;
volatile unsigned long numU10BInterrupts = 0;
void InterruptServiceRoutine() {
  byte u10AControl = BSOS_DataRead(ADDRESS_U10_A_CONTROL);
  if (u10AControl & 0x80) {
    // self test switch
    BSOS_DataRead(ADDRESS_U10_A);
  }

  byte u11AControl = BSOS_DataRead(ADDRESS_U11_A_CONTROL);
  byte u10BControl = BSOS_DataRead(ADDRESS_U10_B_CONTROL);

  // If the interrupt bit on the display interrupt is on, do the display refresh
  if (u11AControl & 0x80) {
    BSOS_DataRead(ADDRESS_U11_A);
    numU11AInterrupts += 1;
  }

  // If the IRQ bit of U10BControl is set, do the Zero-crossing interrupt handler
  if (u10BControl & 0x80) {
    BSOS_DataRead(ADDRESS_U10_B);
    numU10BInterrupts += 1;
  }
}


boolean TestInterrupts() {
  boolean failed = false;
  Serial.write("Starting interrupt tests - this is going to take 5 seconds to test\n");

  unsigned long startTime = millis();

  numU10BInterrupts = 0;
  numU11AInterrupts = 0;
  // Hook up the interrupt
  attachInterrupt(digitalPinToInterrupt(2), InterruptServiceRoutine, LOW);

  // Wait for 5 seconds
  while ((millis()-startTime)<5000) {
  }

  // Hook up the interrupt
  detachInterrupt(digitalPinToInterrupt(2));

  char buf[128];
  char floatStr[10];
  sprintf(buf, "  In 5 seconds, saw %lu U10B interrupts and %lu U11A interrupts\n", numU10BInterrupts, numU11AInterrupts);
  Serial.write(buf);

  dtostrf(((float)numU10BInterrupts)/5.0f, 4, 2, floatStr);
  sprintf(buf, "  Zero-crossing approx. %s times a second\n", floatStr);
  Serial.write(buf);
  dtostrf(((float)numU11AInterrupts)/5.0f, 4, 2, floatStr);
  sprintf(buf, "  Display interrupt approx. %s times a second\n", floatStr);
  Serial.write(buf);

  Serial.write("  Interrupt tests done - if frequencies aren't approx 120 & 320, there's a problem with the interrupt line.\n");

  if (numU10BInterrupts==0 || numU11AInterrupts==0) failed = true;

  return failed;
}


void setup() {
  Serial.begin(115200);
  Serial.write("Start of program\n");

  // Start out with everything tri-state, in case the original
  // CPU is running
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  pinMode(13, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  Serial.write("Monitoring VMA signal, looking for presence of M6800 processor\n");

  unsigned long startTime = millis();
  boolean sawHigh = false;
  boolean sawLow = false;
  // for three seconds, look for activity on the VMA line (A5)
  // If we see anything, then the MPU is active so we shouldn't run
  while ((millis()-startTime)<3000) {
    if (digitalRead(A5)) sawHigh = true;
    else sawLow = true;
  }
  // If we saw both a high and low signal, then someone is toggling the 
  // VMA line, so we should hang here forever (until reset)
  if (sawHigh && sawLow) {
    Serial.write("Saw presence of M6800 processor -- program will halt\n");
    while (1);
  }

  Serial.write("Saw no sign of M6800 -- program will continue\n");

  // Arduino A0 = MPU A0
  // Arduino A1 = MPU A1
  // Arduino A2 = MPU A3
  // Arduino A3 = MPU A4
  // Arduino A4 = MPU A7
  // Arduino A5 = MPU VMA
  // Set up the address lines A0-A7 as output
  DDRC = DDRC | 0x3F;

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
  pinMode(A5, OUTPUT);
  digitalWrite(A5, LOW);  // Set VMA line LOW
  digitalWrite(A6, HIGH); // Set

  pinMode(2, INPUT);
  pinMode(13, INPUT_PULLUP);

}


void TestLightOn() {
  byte newCRVal = BSOS_DataRead(ADDRESS_U11_A_CONTROL) | 0x08;
  BSOS_DataWrite(ADDRESS_U11_A_CONTROL, newCRVal);
}

void TestLightOff() {
  byte newCRVal = BSOS_DataRead(ADDRESS_U11_A_CONTROL) & 0xF7;
  BSOS_DataWrite(ADDRESS_U11_A_CONTROL, newCRVal);
}



void loop() {
  if (TestPIAChips()) {
    Serial.write("FAILED\n");
  }
  if (SetupPIAChips()) {
    Serial.write("FAILED\n");
  }
  if (TestInterrupts()) {
    Serial.write("FAILED\n");
  }
  InitializeU10PIA();
  InitializeU11PIA();

  Serial.write("END OF TESTS\n");  
  for (byte count=0; count<2; count++) {
    delay(500);
    TestLightOn();
    Serial.write("ON\n");  
    delay(500);
    TestLightOff();
    Serial.write("OFF\n");  
  }
  Serial.write("\n\nGoing to try again...\n");
}
