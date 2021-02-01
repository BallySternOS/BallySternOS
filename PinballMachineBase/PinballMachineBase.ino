#include "BallySternOS.h"
#include "PinballMachineBase.h"
#include "SelfTestAndAudit.h"
#include "BSOS_Config.h"

#define DEBUG_MESSAGES  1

// MachineState
//  0 - Attract Mode
//  negative - self-test modes
//  positive - game play
int MachineState = 0;
boolean MachineStateChanged = true;
#define MACHINE_STATE_ATTRACT         0
#define MACHINE_STATE_INIT_GAMEPLAY   1
#define MACHINE_STATE_INIT_NEW_BALL   2
#define MACHINE_STATE_UNVALIDATED     3
#define MACHINE_STATE_NORMAL_GAMEPLAY 4
#define MACHINE_STATE_COUNTDOWN_BONUS 90
#define MACHINE_STATE_MATCH_MODE      95
#define MACHINE_STATE_BALL_OVER       100
#define MACHINE_STATE_GAME_OVER       110


#define TIME_TO_WAIT_FOR_BALL         100


// Game/machine global variables
unsigned long HighScore = 0;
int Credits = 0;
int MaximumCredits = 20;
boolean FreePlayMode = false;

byte CurrentPlayer = 0;
byte CurrentBallInPlay = 1;
byte CurrentNumPlayers = 0;
unsigned long CurrentScores[4];
boolean SamePlayerShootsAgain = false;

unsigned long CurrentTime = 0;
unsigned long BallTimeInTrough = 0;
unsigned long BallFirstSwitchHitTime = 0;

boolean BallSaveUsed = false;
byte BallSaveNumSeconds = 0;
byte BallsPerGame = 3;


void setup() {
  if (DEBUG_MESSAGES) {
    Serial.begin(115200);
  }

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
  while ((millis()-startTime)<1200) {
    if (digitalRead(A5)) sawHigh = true;
    else sawLow = true;
  }
  // If we saw both a high and low signal, then someone is toggling the 
  // VMA line, so we should hang here forever (until reset)
  if (sawHigh && sawLow) {
    while (1);
  }
    
  // Tell the OS about game-specific lights and switches
  BSOS_SetupGameSwitches(NUM_SWITCHES_WITH_TRIGGERS, NUM_PRIORITY_SWITCHES_WITH_TRIGGERS, TriggeredSwitches);

  if (DEBUG_MESSAGES) {
    Serial.write("Attempting to initialize the MPU\n");
  }
 
  // Set up the chips and interrupts
  BSOS_InitializeMPU();
  BSOS_DisableSolenoidStack();
  BSOS_SetDisableFlippers(true);

  byte dipBank = BSOS_GetDipSwitches(0);

  // Use dip switches to set up game variables
  if (DEBUG_MESSAGES) {
    char buf[32];
    sprintf(buf, "DipBank 0 = 0x%02X\n", dipBank);
    Serial.write(buf);
  }

  HighScore = BSOS_ReadULFromEEProm(BSOS_HIGHSCORE_EEPROM_START_BYTE, 10000);
  Credits = BSOS_ReadByteFromEEProm(BSOS_CREDITS_EEPROM_BYTE);
  if (Credits>MaximumCredits) Credits = MaximumCredits;

  BallsPerGame = 3;

  if (DEBUG_MESSAGES) {
    Serial.write("Done with setup\n");
  }

}


void SetPlayerLamps(byte numPlayers, int flashPeriod=0) {
  BSOS_SetLampState(PLAYER_1_UP, (numPlayers==1)?1:0, 0, flashPeriod);
  BSOS_SetLampState(PLAYER_2_UP, (numPlayers==2)?1:0, 0, flashPeriod);
  BSOS_SetLampState(PLAYER_3_UP, (numPlayers==3)?1:0, 0, flashPeriod);
  BSOS_SetLampState(PLAYER_4_UP, (numPlayers==4)?1:0, 0, flashPeriod);
}



void AddCredit() {
  if (Credits<MaximumCredits) {
    Credits++;
    BSOS_WriteByteToEEProm(BSOS_CREDITS_EEPROM_BYTE, Credits);
//    PlaySoundEffect(SOUND_EFFECT_ADD_CREDIT);
  } else {
  }

  BSOS_SetCoinLockout((Credits<MaximumCredits)?false:true);

}


boolean AddPlayer() {

  if (Credits<1 && !FreePlayMode) return false;
  if (CurrentNumPlayers>=4) return false;

  CurrentNumPlayers += 1;
  BSOS_SetDisplay(CurrentNumPlayers-1, 0);
  BSOS_SetDisplayBlank(CurrentNumPlayers-1, 0x30);

  if (!FreePlayMode) {
    Credits -= 1;
    BSOS_WriteByteToEEProm(BSOS_CREDITS_EEPROM_BYTE, Credits);
  }

  BSOS_SetDisplayCredits(Credits);

  return true;
}


int InitNewBall(bool curStateChanged, byte playerNum, int ballNum) {  

  if (curStateChanged) {
    BallFirstSwitchHitTime = 0;
    SamePlayerShootsAgain = false;

    BSOS_SetDisableFlippers(false);
    BSOS_EnableSolenoidStack(); 
    BSOS_SetDisplayCredits(Credits, true);
    SetPlayerLamps(playerNum+1, 500);
    
    if (BSOS_ReadSingleSwitchState(SW_OUTHOLE)) {
      BSOS_PushToTimedSolenoidStack(SOL_OUTHOLE, 4, CurrentTime + 100);
    }

    for (int count=0; count<CurrentNumPlayers; count++) {
      BSOS_SetDisplay(count, CurrentScores[count], true, 2);
    }

    BSOS_SetDisplayBallInPlay(ballNum);
    BSOS_SetLampState(BALL_IN_PLAY, 1);
    BSOS_SetLampState(TILT, 0);

    if (BallSaveNumSeconds>0) {
      BSOS_SetLampState(SAME_PLAYER, 1, 0, 500);
//      BSOS_SetLampState(HEAD_SAME_PLAYER, 1, 0, 500);
    }
  }
  
  // We should only consider the ball initialized when 
  // the ball is no longer triggering the SW_OUTHOLE
  if (BSOS_ReadSingleSwitchState(SW_OUTHOLE)) {
    return MACHINE_STATE_INIT_NEW_BALL;
  } else {
    return MACHINE_STATE_NORMAL_GAMEPLAY;
  }
  
}



int RunSelfTest(int curState, boolean curStateChanged) {
  int returnState = curState;
  CurrentNumPlayers = 0;

  // Any state that's greater than CHUTE_3 is handled by the Base Self-test code
  // Any that's less, is machine specific, so we handle it here.
  if (curState>=MACHINE_STATE_TEST_CHUTE_3_COINS) {
    returnState = RunBaseSelfTest(returnState, curStateChanged, CurrentTime, SW_CREDIT_RESET);  
  } else {
    returnState = MACHINE_STATE_ATTRACT;
  }

  return returnState;
}



byte AttractLastHeadMode = 255;
byte AttractLastPlayfieldMode = 255;

int RunAttractMode(int curState, boolean curStateChanged) {

  int returnState = curState;

  // If this is the first time in the attract mode loop
  if (curStateChanged) {
    BSOS_DisableSolenoidStack();
    BSOS_TurnOffAllLamps();
    BSOS_SetDisableFlippers(true);
    if (DEBUG_MESSAGES) {
      Serial.write("Entering Attract Mode\n\r");
    }
    for (int count=0; count<4; count++) {
      BSOS_SetDisplayBlank(count, 0x00);     
    }
    BSOS_SetDisplayCredits(Credits);
    BSOS_SetDisplayBallInPlay(0);
    AttractLastHeadMode = 255;
    AttractLastPlayfieldMode = 255;
  }

  // Alternate displays between high score and blank
  if ((CurrentTime/6000)%2==0) {

    if (AttractLastHeadMode!=1) {
      BSOS_SetLampState(HIGH_SCORE, 1, 0, 250);
      BSOS_SetLampState(GAME_OVER, 0);
      SetPlayerLamps(0);
  
      for (int count=0; count<4; count++) {
        BSOS_SetDisplay(count, HighScore, true, 2);
      }
      BSOS_SetDisplayCredits(Credits, true);
      BSOS_SetDisplayBallInPlay(0, true);
    }
    AttractLastHeadMode = 1;
    
  } else {
    if (AttractLastHeadMode!=2) {
      BSOS_SetLampState(HIGH_SCORE, 0);
      BSOS_SetLampState(GAME_OVER, 1);
      BSOS_SetDisplayCredits(Credits, true);
      BSOS_SetDisplayBallInPlay(0, true);
      for (int count=0; count<4; count++) {
        if (CurrentNumPlayers>0) {
          if (count<CurrentNumPlayers) {
            BSOS_SetDisplay(count, CurrentScores[count], true, 2); 
          } else {
            BSOS_SetDisplayBlank(count, 0x00);
            BSOS_SetDisplay(count, 0);          
          }          
        } else {
          BSOS_SetDisplayBlank(count, 0x30);
          BSOS_SetDisplay(count, 0);          
        }
      }
    }
    SetPlayerLamps(((CurrentTime/250)%4) + 1);
    AttractLastHeadMode = 2;
  }

  if ((CurrentTime/10000)%3==0) {  
    if (AttractLastPlayfieldMode!=1) {
      BSOS_TurnOffAllLamps();
    }
    
    AttractLastPlayfieldMode = 1;
  } else {
    if (AttractLastPlayfieldMode!=2) {
      BSOS_TurnOffAllLamps();
    }

    AttractLastPlayfieldMode = 2;
  }

  byte switchHit;
  while ( (switchHit=BSOS_PullFirstFromSwitchStack())!=SWITCH_STACK_EMPTY ) {
    if (switchHit==SW_CREDIT_RESET) {
      if (AddPlayer()) returnState = MACHINE_STATE_INIT_GAMEPLAY;
    } else if (switchHit==SW_COIN_1 || switchHit==SW_COIN_2 || switchHit==SW_COIN_3) {
      AddCredit();
      BSOS_SetDisplayCredits(Credits, true);
    } else if (switchHit==SW_SELF_TEST_SWITCH && (CurrentTime-GetLastSelfTestChangedTime())>500) {
      returnState = MACHINE_STATE_TEST_LIGHTS;
      SetLastSelfTestChangedTime(CurrentTime);
    } else {
#ifdef DEBUG_MESSAGES
      char buf[128];
      sprintf(buf, "Switch 0x%02X\n", switchHit);
      Serial.write(buf);
#endif      
    }
  }

  return returnState;
}



boolean PlayerUpLightBlinking = false;

int NormalGamePlay() {
  int returnState = MACHINE_STATE_NORMAL_GAMEPLAY;


  // If the playfield hasn't been validated yet, flash score and player up num
  if (BallFirstSwitchHitTime==0) {
    BSOS_SetDisplayFlash(CurrentPlayer, CurrentScores[CurrentPlayer], CurrentTime, 500, 2);
    if (!PlayerUpLightBlinking) {
      SetPlayerLamps((CurrentPlayer+1), 500);
      PlayerUpLightBlinking = true;
    }
  } else {
    if (PlayerUpLightBlinking) {
      SetPlayerLamps((CurrentPlayer+1));
      PlayerUpLightBlinking = false;
    }
  }

  
  // Check to see if ball is in the outhole
  if (BSOS_ReadSingleSwitchState(SW_OUTHOLE)) {
    if (BallTimeInTrough==0) {
      BallTimeInTrough = CurrentTime;
    } else {
      // Make sure the ball stays on the sensor for at least 
      // 0.5 seconds to be sure that it's not bouncing
      if ((CurrentTime-BallTimeInTrough)>500) {

        if (BallFirstSwitchHitTime==0) BallFirstSwitchHitTime = CurrentTime;
        
        // if we haven't used the ball save, and we're under the time limit, then save the ball
        if (  !BallSaveUsed && 
              ((CurrentTime-BallFirstSwitchHitTime)/1000)<((unsigned long)BallSaveNumSeconds) ) {
        
          BSOS_PushToTimedSolenoidStack(SOL_OUTHOLE, 4, CurrentTime + 100);
          if (BallFirstSwitchHitTime>0) {
            BallSaveUsed = true;
            BSOS_SetLampState(SAME_PLAYER, 0);
//            BSOS_SetLampState(HEAD_SAME_PLAYER, 0);
          }
          BallTimeInTrough = CurrentTime;

          returnState = MACHINE_STATE_NORMAL_GAMEPLAY;          
        } else {
          returnState = MACHINE_STATE_COUNTDOWN_BONUS;
        }
      }
    }
  } else {
    BallTimeInTrough = 0;
  }

  return returnState;
}


unsigned long InitGameStartTime = 0;

int InitGamePlay(boolean curStateChanged) {
  int returnState = MACHINE_STATE_INIT_GAMEPLAY;

  if (curStateChanged) {
    InitGameStartTime = CurrentTime;
    BSOS_SetCoinLockout((Credits>=MaximumCredits)?true:false);
    BSOS_SetDisableFlippers(true);
    BSOS_DisableSolenoidStack();
    BSOS_TurnOffAllLamps();
    BSOS_SetDisplayBallInPlay(1);

    // Set up general game variables
    CurrentPlayer = 0;
    CurrentBallInPlay = 1;
    for (int count=0; count<4; count++) CurrentScores[count] = 0;

    // if the ball is in the outhole, then we can move on
    if (BSOS_ReadSingleSwitchState(SW_OUTHOLE)) {
      if (DEBUG_MESSAGES) {
        Serial.write("Ball is in trough - starting new ball\n");
      }
      BSOS_EnableSolenoidStack();
      BSOS_SetDisableFlippers(false);
      returnState = MACHINE_STATE_INIT_NEW_BALL;
    } else {

      if (DEBUG_MESSAGES) {
        Serial.write("Ball is not in trough - firing stuff and giving it a chance to come back\n");
      }
      
      // Otherwise, let's see if it's in a spot where it could get trapped,
      // for instance, a saucer (if the game has one)
//      BSOS_PushToSolenoidStack(SOL_SAUCER, 5, true);

      // And then set a base time for when we can continue
      InitGameStartTime = CurrentTime;
    }

    for (int count=0; count<4; count++) {
      BSOS_SetDisplay(count, 0);
      BSOS_SetDisplayBlank(count, 0x00);
    }
  }

  // Wait for TIME_TO_WAIT_FOR_BALL seconds, or until the ball appears
  // The reason to bail out after TIME_TO_WAIT_FOR_BALL is just
  // in case the ball is already in the shooter lane.
  if ((CurrentTime-InitGameStartTime)>TIME_TO_WAIT_FOR_BALL || BSOS_ReadSingleSwitchState(SW_OUTHOLE)) {
    BSOS_EnableSolenoidStack();
    BSOS_SetDisableFlippers(false);
    returnState = MACHINE_STATE_INIT_NEW_BALL;
  }
  
  return returnState;  
}

int RunGamePlayMode(int curState, boolean curStateChanged) {
  int returnState = curState;
  unsigned long scoreAtTop = CurrentScores[CurrentPlayer];
  
  // Very first time into gameplay loop
  if (curState==MACHINE_STATE_INIT_GAMEPLAY) {
    returnState = InitGamePlay(curStateChanged);    
  } else if (curState==MACHINE_STATE_INIT_NEW_BALL) {
    returnState = InitNewBall(curStateChanged, CurrentPlayer, CurrentBallInPlay);
  } else if (curState==MACHINE_STATE_NORMAL_GAMEPLAY) {
    returnState = NormalGamePlay();
  } else if (curState==MACHINE_STATE_COUNTDOWN_BONUS) {
//    returnState = CountdownBonus(curStateChanged);
    returnState = MACHINE_STATE_BALL_OVER;
  } else if (curState==MACHINE_STATE_BALL_OVER) {    
    if (SamePlayerShootsAgain) {
      returnState = MACHINE_STATE_INIT_NEW_BALL;
    } else {
      CurrentPlayer+=1;
      if (CurrentPlayer>=CurrentNumPlayers) {
        CurrentPlayer = 0;
        CurrentBallInPlay+=1;
      }
        
      if (CurrentBallInPlay>BallsPerGame) {
//        CheckHighScores();
//        PlaySoundEffect(SOUND_EFFECT_GAME_OVER);
        SetPlayerLamps(0);
        for (int count=0; count<CurrentNumPlayers; count++) {
          BSOS_SetDisplay(count, CurrentScores[count], true, 2);
        }

        returnState = MACHINE_STATE_MATCH_MODE;
      }
      else returnState = MACHINE_STATE_INIT_NEW_BALL;
    }    
  } else if (curState==MACHINE_STATE_MATCH_MODE) {
    returnState = MACHINE_STATE_GAME_OVER;
  } else {
    returnState = MACHINE_STATE_ATTRACT;
  }

  byte switchHit;
  while ( (switchHit=BSOS_PullFirstFromSwitchStack())!=SWITCH_STACK_EMPTY ) {

    switch (switchHit) {
      case SW_SELF_TEST_SWITCH:
        returnState = MACHINE_STATE_TEST_LIGHTS;
        SetLastSelfTestChangedTime(CurrentTime);
        break; 
      case SW_COIN_1:
      case SW_COIN_2:
      case SW_COIN_3:
        AddCredit();
        BSOS_SetDisplayCredits(Credits, true);
        break;
      case SW_CREDIT_RESET:
        if (CurrentBallInPlay<2) {
          // If we haven't finished the first ball, we can add players
          AddPlayer();
        } else {
          // If the first ball is over, pressing start again resets the game
          returnState = MACHINE_STATE_INIT_GAMEPLAY;
        }
        if (DEBUG_MESSAGES) {
          Serial.write("Start game button pressed\n\r");
        }
        break;        
    }
  }

  if (scoreAtTop!=CurrentScores[CurrentPlayer]) {
    Serial.write("Score changed\n");
  }
  return returnState;
}


void loop() {
  // This line has to be in the main loop
  BSOS_DataRead(0);

  CurrentTime = millis();
  int newMachineState = MachineState;

  // Machine state is self-test/attract/game play
  if (MachineState<0) {
    newMachineState = RunSelfTest(MachineState, MachineStateChanged);
  } else if (MachineState==MACHINE_STATE_ATTRACT) {
    newMachineState = RunAttractMode(MachineState, MachineStateChanged);
  } else {
    newMachineState = RunGamePlayMode(MachineState, MachineStateChanged);
  }

  if (newMachineState!=MachineState) {
    MachineState = newMachineState;
    MachineStateChanged = true;
  } else {
    MachineStateChanged = false;
  }

  BSOS_ApplyFlashToLamps(CurrentTime);
  BSOS_UpdateTimedSolenoidStack(CurrentTime);
}
