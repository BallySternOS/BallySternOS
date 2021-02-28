/**************************************************************************
       This file is part of Meteor2021

    I, Dick Hamill, the author of this program disclaim all copyright
    in order to make this program freely available in perpetuity to
    anyone who would like to use it. Dick Hamill, 12/1/2020

    Meteor2021 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Meteor2021 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    See <https://www.gnu.org/licenses/>.
*/

// Ideas
// 1) clutch shot
// 3) make launch harder (remove dead bumper option & clear option)
// 5) wow & special conditions
// 6) Meteor cleared but not reset (bonsux=7?)


#include "BSOS_Config.h"
#include "BallySternOS.h"
#include "Meteor.h"
#include "SelfTestAndAudit.h"
#include <EEPROM.h>

// Wav Trigger defines have been moved to BSOS_Config.h

//#define USE_SCORE_OVERRIDES

#if defined(USE_WAV_TRIGGER) || defined(USE_WAV_TRIGGER_1p3)
#include "SendOnlyWavTrigger.h"
SendOnlyWavTrigger wTrig;             // Our WAV Trigger object
#endif

#define METEOR2021_MAJOR_VERSION  2021
#define METEOR2021_MINOR_VERSION  1
#define DEBUG_MESSAGES  0


/*********************************************************************

    Game specific code

*********************************************************************/

// MachineState
//  0 - Attract Mode
//  negative - self-test modes
//  positive - game play
char MachineState = 0;
boolean MachineStateChanged = true;
#define MACHINE_STATE_ATTRACT         0
#define MACHINE_STATE_INIT_GAMEPLAY   1
#define MACHINE_STATE_INIT_NEW_BALL   2
#define MACHINE_STATE_NORMAL_GAMEPLAY 4
#define MACHINE_STATE_COUNTDOWN_BONUS 99
#define MACHINE_STATE_BALL_OVER       100
#define MACHINE_STATE_MATCH_MODE      110

#define MACHINE_STATE_ADJUST_FREEPLAY           -17
#define MACHINE_STATE_ADJUST_BALL_SAVE          -18
#define MACHINE_STATE_ADJUST_MUSIC_LEVEL        -19
#define MACHINE_STATE_ADJUST_TOURNAMENT_SCORING -20
#define MACHINE_STATE_ADJUST_TILT_WARNING       -21
#define MACHINE_STATE_ADJUST_AWARD_OVERRIDE     -22
#define MACHINE_STATE_ADJUST_BALLS_OVERRIDE     -23
#define MACHINE_STATE_ADJUST_SCROLLING_SCORES   -24
#define MACHINE_STATE_ADJUST_EXTRA_BALL_AWARD   -25
#define MACHINE_STATE_ADJUST_SPECIAL_AWARD      -26
#define MACHINE_STATE_ADJUST_DIM_LEVEL          -27
#define MACHINE_STATE_ADJUST_DONE               -28

#define GAME_MODE_SKILL_SHOT                    0
#define GAME_MODE_UNSTRUCTURED_PLAY             2
#define GAME_MODE_METEOR_STORM_START            3
#define GAME_MODE_METEOR_STORM                  4
#define GAME_MODE_METEOR_STORM_FINISHED         5
#define GAME_MODE_ORPHEUS_START                 6
#define GAME_MODE_ORPHEUS                       7
#define GAME_MODE_ORPHEUS_FRAGMENTED            8
#define GAME_MODE_ORPHEUS_FINISH                9
#define GAME_MODE_ORPHEUS_FAIL                  10

#define EEPROM_BALL_SAVE_BYTE           100
#define EEPROM_FREE_PLAY_BYTE           101
#define EEPROM_MUSIC_LEVEL_BYTE         102
#define EEPROM_SKILL_SHOT_BYTE          103
#define EEPROM_TILT_WARNING_BYTE        104
#define EEPROM_AWARD_OVERRIDE_BYTE      105
#define EEPROM_BALLS_OVERRIDE_BYTE      106
#define EEPROM_TOURNAMENT_SCORING_BYTE  107
#define EEPROM_SCROLLING_SCORES_BYTE    110
#define EEPROM_DIM_LEVEL_BYTE           113
#define EEPROM_EXTRA_BALL_SCORE_BYTE    140
#define EEPROM_SPECIAL_SCORE_BYTE       144

#define SOUND_EFFECT_NONE                     0
#define SOUND_EFFECT_SPINNER                  1
#define SOUND_EFFECT_UNLIT_ROLLOVER           2
#define SOUND_EFFECT_LEFT_INLANE              3
#define SOUND_EFFECT_LEFT_OUTLANE             4
#define SOUND_EFFECT_RIGHT_INLANE             5
#define SOUND_EFFECT_RIGHT_OUTLANE            6
#define SOUND_EFFECT_METEOR_DROP_1            7
#define SOUND_EFFECT_METEOR_DROP_2            8
#define SOUND_EFFECT_METEOR_DROP_3            9
#define SOUND_EFFECT_METEOR_DROP_4            10
#define SOUND_EFFECT_METEOR_DROP_5            11
#define SOUND_EFFECT_METEOR_DROP_6            12
#define SOUND_EFFECT_ADD_CREDIT               13
#define SOUND_EFFECT_THUMPER_BUMPER_HIT       14
#define SOUND_EFFECT_SHOOT_AGAIN              15
#define SOUND_EFFECT_MISSION_FAILED           16
#define SOUND_EFFECT_ROCKET_1_UNLIT           17
#define SOUND_EFFECT_FRAGMENT_ELIMINATED      18
#define SOUND_EFFECT_JACKPOT                  19
#define SOUND_EFFECT_ADD_PLAYER_1             20
#define SOUND_EFFECT_ADD_PLAYER_2             SOUND_EFFECT_ADD_PLAYER_1+1
#define SOUND_EFFECT_ADD_PLAYER_3             SOUND_EFFECT_ADD_PLAYER_1+2
#define SOUND_EFFECT_ADD_PLAYER_4             SOUND_EFFECT_ADD_PLAYER_1+3
#define SOUND_EFFECT_PLAYER_1_UP              24
#define SOUND_EFFECT_PLAYER_2_UP              SOUND_EFFECT_PLAYER_1_UP+1
#define SOUND_EFFECT_PLAYER_3_UP              SOUND_EFFECT_PLAYER_1_UP+2
#define SOUND_EFFECT_PLAYER_4_UP              SOUND_EFFECT_PLAYER_1_UP+3
#define SOUND_EFFECT_NICE_SHOT                28
#define SOUND_EFFECT_WIZARD_SCORING           29
#define SOUND_EFFECT_BALL_OVER                30
#define SOUND_EFFECT_GAME_OVER                31
#define SOUND_EFFECT_BONUS_COUNT              32
#define SOUND_EFFECT_2X_BONUS_COUNT           33
#define SOUND_EFFECT_3X_BONUS_COUNT           34
#define SOUND_EFFECT_4X_BONUS_COUNT           35
#define SOUND_EFFECT_5X_BONUS_COUNT           36
#define SOUND_EFFECT_6X_BONUS_COUNT           37
#define SOUND_EFFECT_7X_BONUS_COUNT           38
#define SOUND_EFFECT_EXTRA_BALL               40
#define SOUND_EFFECT_TILT_WARNING             41
#define SOUND_EFFECT_MATCH_SPIN               42
#define SOUND_EFFECT_1_TARGET_HIT             44
#define SOUND_EFFECT_2_TARGET_HIT             45
#define SOUND_EFFECT_3_TARGET_HIT             46
#define SOUND_EFFECT_WOW_HIT                  47
#define SOUND_EFFECT_SPECIAL_HIT              48
#define SOUND_EFFECT_STANDUP_UNLIT            49
#define SOUND_EFFECT_SKILL_SHOT               50
#define SOUND_EFFECT_SUPER_SKILL_SHOT         51
#define SOUND_EFFECT_ROCKET_3_UNLIT           53
#define SOUND_EFFECT_SLING1                   54
#define SOUND_EFFECT_SLING2                   55
#define SOUND_EFFECT_SLING3                   56
#define SOUND_EFFECT_METEOR_STORM_START       60
#define SOUND_EFFECT_METEOR_STORM_SECTOR_1    61
#define SOUND_EFFECT_METEOR_STORM_SECTOR_2    62
#define SOUND_EFFECT_METEOR_STORM_SECTOR_1_2  63
#define SOUND_EFFECT_METEOR_STORM_SECTOR_3    64
#define SOUND_EFFECT_METEOR_STORM_SECTOR_3_1  65
#define SOUND_EFFECT_METEOR_STORM_SECTOR_3_2  66
#define SOUND_EFFECT_METEOR_STORM_SECTOR_3_2_1  67
#define SOUND_EFFECT_METEOR_IMPACT            68
#define SOUND_EFFECT_METEOR_HIT               69
#define SOUND_EFFECT_ROCKET_1_ARMED           70
#define SOUND_EFFECT_ROCKET_2_ARMED           71
#define SOUND_EFFECT_ROCKET_3_ARMED           72
#define SOUND_EFFECT_ROCKET_LAUNCHED          73
#define SOUND_EFFECT_ROCKET_1_MISSED          74
#define SOUND_EFFECT_ROCKET_2_MISSED          75
#define SOUND_EFFECT_ROCKET_3_MISSED          76
#define SOUND_EFFECT_TILT                     77
#define SOUND_EFFECT_1_METEOR_REMAINING       78
#define SOUND_EFFECT_2_METEORS_REMAINING      79
#define SOUND_EFFECT_3_METEORS_REMAINING      80
#define SOUND_EFFECT_ORPHEUS_START            81
#define SOUND_EFFECT_ORPHEUS_FRAGMENTED       82
#define SOUND_EFFECT_ORPHEUS_HIT_1            83
#define SOUND_EFFECT_ORPHEUS_HIT_2            84
#define SOUND_EFFECT_METEOR_STORM_START_2     85
#define SOUND_EFFECT_METEOR_STORM_START_3     86
#define SOUND_EFFECT_METEOR_STORM_START_4     87
#define SOUND_EFFECT_GAME_START_INTRO         88
#define SOUND_EFFECT_METEOR_INTRO             89
#define SOUND_EFFECT_BACKGROUND_1             90
#define SOUND_EFFECT_BACKGROUND_2             91
#define SOUND_EFFECT_BACKGROUND_3             92
#define SOUND_EFFECT_BACKGROUND_4             93
#define SOUND_EFFECT_BACKGROUND_5             94
#define SOUND_EFFECT_BACKGROUND_6             95
#define SOUND_EFFECT_BACKGROUND_WIZ           96
#define SOUND_EFFECT_BACKGROUND_ORPHEUS       97
#define SOUND_EFFECT_ORPHEUS_FINISH           98


#define SKILL_SHOT_DURATION 15
#define MAX_DISPLAY_BONUS     7
#define TILT_WARNING_DEBOUNCE_TIME      1000
#define BONUS_UNDERLIGHTS_OFF   0
#define BONUS_UNDERLIGHTS_DIM   1
#define BONUS_UNDERLIGHTS_FULL  2

#define METEOR_HIT_REWARD               8000
#define MODE_QUALIFY_TIME               45000
#define SPINNER_ANIMATION_DURATION      3000
#define DROP_SWITCH_BLACKOUT_PERIOD     1000
#define ROCKET_ANIMATION_DURATION       2000
#define METEOR_STORM_START_DURATION     6500
#define METEOR_STORM_TIME_TO_ARM        15000
#define ORPHEUS_START_DURATION          8000
#define ORPHEUS_DURATION                60000
#define ORPHEUS_FRAGMENTED_DURATION     30000
#define ORPHEUS_FAIL_DURATION           5000
#define ORPHEUS_FINISH_DURATION_SHORT   10000  
#define ORPHEUS_FINISH_DURATION_LONG    20000  
#define METEOR_HIT_SHORT_DISPLAY        3000
#define METEOR_HIT_LONG_DISPLAY         6000

#define SPOT_METEOR_TARGET_LIGHT_1            0x01
#define SPOT_METEOR_TARGET_LIGHT_2            0x02
#define SPOT_METEOR_TARGET_LIGHT_3            0x04

#define ROCKET_1_COLLECT_LIGHT                0x01
#define ROCKET_3_COLLECT_LIGHT                0x02

#define METEOR_STORM_SECTOR_1           0x01
#define METEOR_STORM_SECTOR_2           0x02
#define METEOR_STORM_SECTOR_3           0x04


// These defines dictate the ways rockets can be launched
#define LAUNCH_ALL_ROCKETS_WITH_STANDUP_UNTIL_LEVEL   1
#define LAUNCH_ROCKETS_WITH_BANK_CLEAR_UNTIL_LEVEL    2
#define LAUNCH_ROCKETS_WITH_DEAD_BUMPER_UNTIL_LEVEL   4


/*********************************************************************

    Machine state and options

*********************************************************************/
unsigned long HighScore = 0;
unsigned long AwardScores[3];
byte Credits = 0;
boolean FreePlayMode = false;
byte MusicLevel = 3;
byte BallSaveNumSeconds = 0;
unsigned long ExtraBallValue = 0;
unsigned long SpecialValue = 0;
unsigned long CurrentTime = 0;
byte MaximumCredits = 40;
byte BallsPerGame = 3;
byte DimLevel = 2;
byte ScoreAwardReplay = 0;
boolean HighScoreReplay = true;
boolean MatchFeature = true;
byte SpecialLightAward = 0;
boolean TournamentScoring = false;
boolean ResetScoresToClearVersion = false;
boolean ScrollingScores = true;



/*********************************************************************

    Game State

*********************************************************************/
byte CurrentPlayer = 0;
byte CurrentBallInPlay = 1;
byte CurrentNumPlayers = 0;
byte Bonus[3];
byte BonusX[4];
byte CurrentBonusX;
byte GameMode = GAME_MODE_SKILL_SHOT;
byte MaxTiltWarnings = 2;
byte NumTiltWarnings = 0;

boolean SamePlayerShootsAgain = false;
boolean BallSaveUsed = false;
boolean CurrentlyShowingBallSave = false;
boolean ExtraBallCollected = false;
boolean SpecialCollected = false;
boolean ShowingModeStats = false;

unsigned long CurrentScores[4];
unsigned long CurrentScoreOfCurrentPlayer = 0;
unsigned long BallFirstSwitchHitTime = 0;
unsigned long BallTimeInTrough = 0;
unsigned long GameModeStartTime = 0;
unsigned long GameModeEndTime = 0;
unsigned long LastTiltWarningTime = 0;


int LastReportedValue = 0;

byte SkillShotWowLit = 0;
byte StandupAward = 0; 
byte StandupHits = 0;
byte SpinnerValue = 0;
byte SpinnerHitPhase = 11;
byte MeteorStatus[4];
byte CurrentMeteorTargetsValid = 0;
byte Current3BankTargetsHit[3]; 
byte MeteorSolenoid[6] = {13, 11, 10, 7, 4, 3};
byte BonusAnimationShown[3];
byte SpotMeteorTargetByte = 0x00;
byte CollectRocketsByte = 0x00;
byte CollectRocketCurrentLevel = 0;
byte MeteorStormSectors = 0;
byte MeteorStormDescent[3] = {0, 0, 0};
byte MeteorStormDescentSpeed[3] = {0, 0, 0};
byte RocketsArmed = 0;
byte NumberOfMeteorsHit[4];
byte OrpheusPhase;
byte OrpheusRound[4];
byte OrpheusScoring = false;
byte TargetWowPhase = 0;
byte NumberOfSpins[4];
byte OrpheusCompletedFlag;

boolean SkillShotHit;
boolean JackpotLit = false;
boolean SpotMeteorReverse = false;
boolean RocketsFiredFromBank = false;
boolean MeteorHitStarted = false;

unsigned long LastTimeSpinnerHit = 0;
unsigned long BonusAnimationStart[3];
unsigned long BonusXAnimationStart = 0;
unsigned long Rocket1BumperAnimationTime = 0;
unsigned long Rocket3BumperAnimationTime = 0;
unsigned long StandupResetTime = 0;
unsigned long CollectRocketAnimationStart = 0;
unsigned long LastTimeRocketBonusPlayed = 0; 
unsigned long MeteorStormStart = 0;
unsigned long RocketLaunchedTime[3] = {0, 0, 0};
unsigned long LastMeteorCheckTime = 0;
unsigned long MeteorHitStartTime = 0;
unsigned long MeteorHitEndTime = 0;
unsigned long LastOrpheusHitTime = 0;



void ReadStoredParameters() {
  HighScore = BSOS_ReadULFromEEProm(BSOS_HIGHSCORE_EEPROM_START_BYTE, 10000);
  Credits = BSOS_ReadByteFromEEProm(BSOS_CREDITS_EEPROM_BYTE);
  if (Credits > MaximumCredits) Credits = MaximumCredits;

  ReadSetting(EEPROM_FREE_PLAY_BYTE, 0);
  FreePlayMode = (EEPROM.read(EEPROM_FREE_PLAY_BYTE)) ? true : false;

  BallSaveNumSeconds = ReadSetting(EEPROM_BALL_SAVE_BYTE, 15);
  if (BallSaveNumSeconds > 20) BallSaveNumSeconds = 20;

  MusicLevel = ReadSetting(EEPROM_MUSIC_LEVEL_BYTE, 3);
  if (MusicLevel > 3) MusicLevel = 3;

  TournamentScoring = (ReadSetting(EEPROM_TOURNAMENT_SCORING_BYTE, 0)) ? true : false;

  MaxTiltWarnings = ReadSetting(EEPROM_TILT_WARNING_BYTE, 2);
  if (MaxTiltWarnings > 2) MaxTiltWarnings = 2;

  byte awardOverride = ReadSetting(EEPROM_AWARD_OVERRIDE_BYTE, 99);
  if (awardOverride != 99) {
    ScoreAwardReplay = awardOverride;
  }

  byte ballsOverride = ReadSetting(EEPROM_BALLS_OVERRIDE_BYTE, 99);
  if (ballsOverride == 3 || ballsOverride == 5) {
    BallsPerGame = ballsOverride;
  } else {
    if (ballsOverride != 99) EEPROM.write(EEPROM_BALLS_OVERRIDE_BYTE, 99);
  }

  ScrollingScores = (ReadSetting(EEPROM_SCROLLING_SCORES_BYTE, 1)) ? true : false;

  ExtraBallValue = BSOS_ReadULFromEEProm(EEPROM_EXTRA_BALL_SCORE_BYTE);
  if (ExtraBallValue % 1000 || ExtraBallValue > 100000) ExtraBallValue = 20000;

  SpecialValue = BSOS_ReadULFromEEProm(EEPROM_SPECIAL_SCORE_BYTE);
  if (SpecialValue % 1000 || SpecialValue > 100000) SpecialValue = 40000;

  DimLevel = ReadSetting(EEPROM_DIM_LEVEL_BYTE, 2);
  if (DimLevel < 2 || DimLevel > 3) DimLevel = 2;
  BSOS_SetDimDivisor(1, DimLevel);

  AwardScores[0] = BSOS_ReadULFromEEProm(BSOS_AWARD_SCORE_1_EEPROM_START_BYTE);
  AwardScores[1] = BSOS_ReadULFromEEProm(BSOS_AWARD_SCORE_2_EEPROM_START_BYTE);
  AwardScores[2] = BSOS_ReadULFromEEProm(BSOS_AWARD_SCORE_3_EEPROM_START_BYTE);

}


void setup() {
  if (DEBUG_MESSAGES) {
    Serial.begin(57600);
  }

  // Tell the OS about game-specific lights and switches
  BSOS_SetupGameSwitches(NUM_SWITCHES_WITH_TRIGGERS, NUM_PRIORITY_SWITCHES_WITH_TRIGGERS, TriggeredSwitches);

  // Set up the chips and interrupts
  BSOS_InitializeMPU();
  BSOS_DisableSolenoidStack();
  BSOS_SetDisableFlippers(true);

  // Read parameters from EEProm
  ReadStoredParameters();
  BSOS_SetCoinLockout((Credits >= MaximumCredits) ? true : false);

  CurrentScores[0] = METEOR2021_MAJOR_VERSION;
//  CurrentScoreOfCurrentPlayer = METEOR2021_MAJOR_VERSION;
  CurrentScores[1] = METEOR2021_MINOR_VERSION;
  CurrentScores[2] = BALLY_STERN_OS_MAJOR_VERSION;
  CurrentScores[3] = BALLY_STERN_OS_MINOR_VERSION;
  ResetScoresToClearVersion = true;

#if defined(USE_WAV_TRIGGER) || defined(USE_WAV_TRIGGER_1p3)
  // WAV Trigger startup at 57600
  wTrig.start();
  wTrig.stopAllTracks();
//  wTrig.masterGain(0);
//  wTrig.setAmpPwr(false);
  delay(10);

  // Send a stop-all command and reset the sample-rate offset, in case we have
  //  reset while the WAV Trigger was already playing.
//  wTrig.samplerateOffset(0);
#endif

  StopAudio();
  CurrentTime = millis();
  PlaySoundEffect(SOUND_EFFECT_METEOR_INTRO);
}

byte ReadSetting(byte setting, byte defaultValue) {
  byte value = EEPROM.read(setting);
  if (value == 0xFF) {
    EEPROM.write(setting, defaultValue);
    return defaultValue;
  }
  return value;
}


void StartMeteorStorm() {
  MeteorStormSectors = CurrentTime & 0x07;
  if (MeteorStormSectors==0) MeteorStormSectors = 1;
  PlaySoundEffect(SOUND_EFFECT_METEOR_STORM_SECTOR_1 + (MeteorStormSectors-1));

  for (int count=0; count<3; count++) {
    MeteorStormDescent[count] = (MeteorStormSectors & (1<<count))?7:0;
    MeteorStormDescentSpeed[count] = 4 + ((CurrentTime>>count)&0x01F);
    RocketLaunchedTime[count] = 0;
  }
  MeteorStormStart = CurrentTime;          
}


void SetMeteorDropTargets(byte dropsDown, unsigned long timeOffset = 0, boolean resetBank = true) {
  if (resetBank) BSOS_PushToTimedSolenoidStack(SOL_RESET_METEOR_BANK, 10, CurrentTime+timeOffset);
  LastMeteorCheckTime = CurrentTime + timeOffset + 1000;
  unsigned long targetDropTime = CurrentTime + timeOffset + 250;
  for (int count=0; count<6; count++) {
    if ((dropsDown & (1<<count))) {
      // This target should be down        
      BSOS_PushToTimedSolenoidStack(MeteorSolenoid[count], 6, targetDropTime);
      targetDropTime += 150;
    }
  }

  CurrentMeteorTargetsValid = dropsDown;
}


byte CheckSequentialSwitches(byte startingSwitch, byte numSwitches) {
  byte returnSwitches = 0; 
  for (byte count=0; count<numSwitches; count++) {
    returnSwitches |= (BSOS_ReadSingleSwitchState(startingSwitch+count)<<count);
  }
  return returnSwitches;
}


////////////////////////////////////////////////////////////////////////////
//
//  Lamp Management functions
//
////////////////////////////////////////////////////////////////////////////

byte BonusStartingLamps[3] = {LEFT_ROCKET_1K, MIDDLE_ROCKET_1K, RIGHT_ROCKET_1K};

void ShowBonus(byte *bonusArray, unsigned long *bonusOverride) {
    
  for (byte count=0; count<7; count++) {
    for (int i=0; i<3; i++) {
      if (bonusOverride[i]==0) BSOS_SetLampState(BonusStartingLamps[i]-count, (count+1)==bonusArray[i]);
    }
  }
  
}


void ShowMeteorLamps() {
  byte currentSwitches = CheckSequentialSwitches(SW_DROP_TARGET_R, 6);
  
  if (GameMode==GAME_MODE_SKILL_SHOT) {
    for (int count=METEOR_DROP_R; count<=METEOR_DROP_M; count++) {
      byte alternatingPhase = ((CurrentTime/100)+count%2)%4;
      BSOS_SetLampState(count, alternatingPhase, alternatingPhase%2);
    }

    byte wowPhase = 5 - ((CurrentTime/1000)%6);
    SkillShotWowLit = wowPhase + SW_DROP_TARGET_R;
    for (int count=METEOR_WOW_R; count<=METEOR_WOW_M; count++) {
      BSOS_SetLampState(count, (count-METEOR_WOW_R)==wowPhase);
    }
  } else if (GameMode==GAME_MODE_ORPHEUS_START) {
    byte lampPhase = ((CurrentTime-GameModeStartTime)/125)%10;
    for (int count=0; count<6; count++) {
      BSOS_SetLampState(METEOR_DROP_R+count, (count==lampPhase)||(count==(10-lampPhase)));
      BSOS_SetLampState(METEOR_WOW_M-count, (count==lampPhase)||(count==(10-lampPhase)));
    }
  } else if (GameMode==GAME_MODE_ORPHEUS) {
    byte wowPhase = 5;
    byte numBits = 6-CountBits(CurrentMeteorTargetsValid);
    byte bitPos;
    byte posBasedOnTime = ((CurrentTime/1500)%numBits)+1;
    for (bitPos = 0; bitPos<6 && posBasedOnTime; bitPos++) {
      if ((CurrentMeteorTargetsValid & (0x20>>bitPos))==0x00) posBasedOnTime -= 1;
    }
    wowPhase = 5-(bitPos-1);
    SkillShotWowLit = wowPhase + SW_DROP_TARGET_R;
    for (int count=0; count<6; count++) {
      BSOS_SetLampState(METEOR_DROP_R+count, (currentSwitches & (1<<count))?0:1, 0, 200);
      BSOS_SetLampState(METEOR_WOW_R+count, (count==wowPhase));
    }
  } else {
    for (int count=0; count<6; count++) {
      BSOS_SetLampState(METEOR_DROP_R+count, currentSwitches&(1<<count)?0:1);
      BSOS_SetLampState(METEOR_WOW_R+count, 0);
    }
  }
}



void ShowStandupTargetLamps() {
  if (GameMode==GAME_MODE_SKILL_SHOT) {
  } else if (GameMode==GAME_MODE_METEOR_STORM) {
    byte lampPhase = ((CurrentTime-GameModeStartTime)/100)%3;

    BSOS_SetLampState(STANDUP_TARGET_WOW, RocketsArmed && lampPhase==1);
    BSOS_SetLampState(STANDUP_TARGET_SPECIAL, RocketsArmed && lampPhase==2);

  } else {
    
    boolean wowOn = (StandupAward==2);
    boolean specialOn = (StandupAward==3);
    boolean flashing = ((StandupResetTime-CurrentTime)<500);
    
    BSOS_SetLampState(STANDUP_TARGET_WOW, wowOn, 0, flashing?100:0);
    BSOS_SetLampState(STANDUP_TARGET_SPECIAL, specialOn, 0, flashing?100:0);
  }
  
}


byte ShowBonusAnimation(byte topLamp, unsigned long animationStartTime, byte animationShown, byte animationCap) {
  byte bonusElapsed = ((CurrentTime-animationStartTime)/100) + 1;

  if (animationShown==0) {
    for (int count=0; count<7; count++) {
      BSOS_SetLampState(topLamp+count, 0);      
    }

    // First time in loop, we can play initial sound
    //PlaySoundEffect();
  }

  if (bonusElapsed>=(animationCap*2)) {
    return 0xFF;
  }

  if (bonusElapsed>animationShown) {
    if (bonusElapsed<=animationCap) {
      CurrentScoreOfCurrentPlayer += 1000;
      for (int count=0; count<7; count++) {
        BSOS_SetLampState(topLamp+count, ((7-count)<=bonusElapsed));      
      }
    } else {
      for (int count=0; count<7; count++) {
        BSOS_SetLampState(topLamp+count, (((7-count)<=animationCap) && (7-count)>(bonusElapsed-animationCap)));      
      }
    }
    byte soundNum = SOUND_EFFECT_3_TARGET_HIT;
    if (topLamp==LEFT_ROCKET_7K) soundNum = SOUND_EFFECT_1_TARGET_HIT;
    else if (topLamp==MIDDLE_ROCKET_7K) soundNum = SOUND_EFFECT_2_TARGET_HIT;
    if ((CurrentTime-LastTimeRocketBonusPlayed)>300) {        
      PlaySoundEffect(soundNum);
      LastTimeRocketBonusPlayed = CurrentTime;
    }
  }

  if (animationCap==1) return 0xFF;
  return bonusElapsed;
}




void ShowLampAnimation(byte animationNum, unsigned long divisor, unsigned long baseTime, byte subOffset, boolean dim, boolean reverse=false, byte keepLampOn=99) {
  byte currentStep = (baseTime/divisor)%LAMP_ANIMATION_STEPS;
  if (reverse) currentStep = (LAMP_ANIMATION_STEPS-1) - currentStep;

  byte lampNum = 0;
  for (int byteNum=0; byteNum<8; byteNum++) {
    for (byte bitNum=0; bitNum<8; bitNum++) {

      // if there's a subOffset, turn off lights at that offset
      if (subOffset) {
        byte lampOff;
        lampOff = LampAnimations[animationNum][(currentStep+subOffset)%LAMP_ANIMATION_STEPS][byteNum] & (1<<bitNum);
        if (lampOff && lampNum!=keepLampOn) BSOS_SetLampState(lampNum, 0);
      }
      
      byte lampOn = LampAnimations[animationNum][currentStep][byteNum] & (1<<bitNum);
      if (lampOn) BSOS_SetLampState(lampNum, 1, dim);
      
      lampNum += 1;
    }
#if not defined (BALLY_STERN_OS_SOFTWARE_DISPLAY_INTERRUPT)    
    if (byteNum%2) BSOS_DataRead(0);
#endif
  }  
}


byte MeteorGoals[7] = {6, 13, 21, 30, 40, 55, 72};

byte GetNextMeteorGoal() {
  if (OrpheusRound[CurrentPlayer]<7) return MeteorGoals[OrpheusRound[CurrentPlayer]];
  return 100;
}


void ShowBonusLamps(boolean actuallyShowLamps = true) {


  if (CollectRocketAnimationStart!=0) {
    byte totalBonus = Bonus[0]+Bonus[1]+Bonus[2];
    
    if (CollectRocketCurrentLevel<totalBonus) {
      int collectionCount = (byte)((CurrentTime-CollectRocketAnimationStart)/110);
      if (collectionCount>CollectRocketCurrentLevel) {
    
        for (int count=0; count<7; count++) {
          BSOS_SetLampState(LEFT_ROCKET_1K-count, (count<Bonus[0]) && (count<collectionCount), 0, 20);            
          BSOS_SetLampState(MIDDLE_ROCKET_1K-count, (count<Bonus[1]) && (count<(collectionCount-(int)Bonus[0])), 0, 20);            
          BSOS_SetLampState(RIGHT_ROCKET_1K-count, (count<Bonus[2]) && (count<(collectionCount-(int)Bonus[0]-(int)Bonus[1])), 0, 20);          
        }
        
        if ((CollectRocketCurrentLevel%3)==0) PlaySoundEffect(SOUND_EFFECT_1_TARGET_HIT);
        CurrentScoreOfCurrentPlayer += 1000;
        CollectRocketCurrentLevel = collectionCount;
      }
    } else {
      CollectRocketAnimationStart = 0;
    }
  } else if (GameMode==GAME_MODE_UNSTRUCTURED_PLAY) {
    ShowBonus(Bonus, BonusAnimationStart);

    for (int i=0; i<3; i++) {
      if (BonusAnimationStart[i]) {
        BonusAnimationShown[i] = ShowBonusAnimation(BonusStartingLamps[i]-6, BonusAnimationStart[i], BonusAnimationShown[i], Bonus[i]);
        if (BonusAnimationShown[i]==0xFF) {
          BonusAnimationStart[i] = 0;
          BonusAnimationShown[i] = 0;
        }
      }
    }
 
  } else if (GameMode==GAME_MODE_METEOR_STORM_START) {
    byte scannerPhase = ((CurrentTime-GameModeStartTime)/700)%4;
    byte scannerPhase2 = (((CurrentTime-GameModeStartTime)/50)%13) + 1;
    if (scannerPhase2>7) scannerPhase2 = 14-scannerPhase2;
    for (int count=0; count<7; count++) {
      BSOS_SetLampState(LEFT_ROCKET_7K+count, (scannerPhase==0)||((count+1)==scannerPhase2));
      BSOS_SetLampState(MIDDLE_ROCKET_7K+count, (scannerPhase%2)||((count+1)==scannerPhase2));
      BSOS_SetLampState(RIGHT_ROCKET_7K+count, (scannerPhase==2)||((count+1)==scannerPhase2));
    }
  } else if (GameMode==GAME_MODE_METEOR_STORM) {
    for (int count=0; count<3; count++) {
      byte rocketLamp = LEFT_ROCKET_1K;
      if (count==1) rocketLamp = MIDDLE_ROCKET_1K;
      else if (count==2) rocketLamp = RIGHT_ROCKET_1K;
      for (int lampNum=0; lampNum<7; lampNum++) {
        boolean lampOn = false;
        int flashOn = 0;
        byte rocketHeight = 0; 
        if (RocketLaunchedTime[count]!=0) {
          rocketHeight = ((CurrentTime-RocketLaunchedTime[count])/300) + 1;
          if (rocketHeight==8) {
            RocketLaunchedTime[count] = 0;
            PlaySoundEffect(SOUND_EFFECT_ROCKET_1_MISSED+count);
          }
        }
        
        if (rocketHeight==(lampNum+1)) {
          if (rocketHeight==MeteorStormDescent[count]) {
            RocketLaunchedTime[count] = 0;
            StopAudio();
            PlaySoundEffect(SOUND_EFFECT_METEOR_HIT);
            ResumeBackgroundSong();
            lampOn = true;            
            flashOn = 50;
            if (RocketsFiredFromBank) CurrentScoreOfCurrentPlayer += ((unsigned long)rocketHeight * (METEOR_HIT_REWARD/2));
            else CurrentScoreOfCurrentPlayer += ((unsigned long)rocketHeight * (METEOR_HIT_REWARD));
            MeteorStormSectors &= ~(1<<count);
            MeteorStormDescent[count] = 0;
            NumberOfMeteorsHit[CurrentPlayer] += 1;
            byte nextMeteorGoal = GetNextMeteorGoal();

            int meteorsRemaining = (int)nextMeteorGoal - (int)NumberOfMeteorsHit[CurrentPlayer];

            MeteorHitEndTime = CurrentTime + METEOR_HIT_SHORT_DISPLAY;
            if (meteorsRemaining<4 && meteorsRemaining>0) {
              MeteorHitEndTime = CurrentTime + METEOR_HIT_LONG_DISPLAY;
              PlaySoundEffect(SOUND_EFFECT_1_METEOR_REMAINING + (meteorsRemaining-1));
            } 

            MeteorHitStartTime = CurrentTime;
            MeteorHitStarted = true;

          } else {
            lampOn = true;
          }          
        } else if (lampNum==0 && (RocketsArmed & (1<<count))) {
          lampOn = true;
          flashOn = 500;
        } else {
          lampOn = (((lampNum+1)==MeteorStormDescent[count]));
          flashOn = 100;
        }
        if (actuallyShowLamps) BSOS_SetLampState(rocketLamp-lampNum, lampOn, 0, flashOn);
      }
    }
  }
}

byte RocketDropLamps[] = {BOTTOM_LEFT_BANK_WOW, BOTTOM_LEFT_BANK_SPECIAL, TOP_LEFT_BANK_WOW, TOP_LEFT_DROP_TARGET_SPECIAL, RIGHT_BANK_WOW, RIGHT_BANK_SPECIAL};

void ShowRocketDropLamps() {
  if (GameMode==GAME_MODE_METEOR_STORM) {
    byte lampPhase = ((CurrentTime-GameModeStartTime)/200)%2;
    byte signalRocket = MeteorStormSectors & (~RocketsArmed);

    for (int count=0; count<3; count++) {
      
      byte signalForThisOne = signalRocket & (1<<count);
      BSOS_SetLampState(RocketDropLamps[count*2], lampPhase && signalForThisOne);
      BSOS_SetLampState(RocketDropLamps[count*2+1], (lampPhase==0) && signalForThisOne);
      
    }
  } else {
    for (int count=0; count<6; count++) {
      if (TargetWowPhase) BSOS_SetLampState(RocketDropLamps[count], count==((TargetWowPhase-1)*2), 0, 500);
      else BSOS_SetLampState(RocketDropLamps[count], 0);
    }
  }
}

byte BonusXLamps[] = {BONUS_1X, BONUS_2X, BONUS_4X};

void ShowBonusXLamps() {
  if (GameMode==GAME_MODE_SKILL_SHOT) {
    
  } else {
    if (BonusXAnimationStart==0) {
      for (int count=0; count<3; count++) {
        BSOS_SetLampState(BonusXLamps[count], CurrentBonusX & (1<<count));
      }
    } else {
      byte lampPhase = ((CurrentTime-BonusXAnimationStart)/100)%4;
      byte lampOn = (lampPhase?0xFF:0) & CurrentBonusX;
      for (int count=0; count<3; count++) {
        BSOS_SetLampState(BonusXLamps[count], lampOn & (1<<count), lampPhase%2);
      }
        
      if ((CurrentTime-BonusXAnimationStart)>5000) BonusXAnimationStart = 0;
    }
  }
}

void ShowSpinnerLamps() {
  byte currentSwitches = CheckSequentialSwitches(SW_DROP_TARGET_R, 6);
  if (GameMode==GAME_MODE_SKILL_SHOT) {
  } else {
    if (LastTimeSpinnerHit!=0 && (CurrentTime-LastTimeSpinnerHit)<SPINNER_ANIMATION_DURATION) {
      for (int count=0; count<6; count++) {
        boolean lampDim = !(currentSwitches & (1<<(5-count)));
        BSOS_SetLampState(SPINNER_1-count, (count>=SpinnerHitPhase) || (SpinnerHitPhase>6 && count<(SpinnerHitPhase-5)), lampDim?2:0);
      }
    } else {
      for (int count=0; count<6; count++) {
        BSOS_SetLampState(SPINNER_6+count, currentSwitches & (1<<count));
      }
      LastTimeSpinnerHit = 0;
      SpinnerHitPhase = 11;
    }
  }
}



void ShowShootAgainLamp() {

  if (!BallSaveUsed && BallSaveNumSeconds>0 && (CurrentTime-BallFirstSwitchHitTime)<((unsigned long)(BallSaveNumSeconds-1)*1000)) {
    unsigned long msRemaining = ((unsigned long)(BallSaveNumSeconds-1)*1000)-(CurrentTime-BallFirstSwitchHitTime);
    BSOS_SetLampState(SHOOT_AGAIN, 1, 0, (msRemaining<1000)?100:500);
  } else {
    BSOS_SetLampState(SHOOT_AGAIN, SamePlayerShootsAgain);
  }
}

byte RolloverLamps[] = {LEFT_ROLLOVER_AND_RIGHT_INLANE, RIGHT_ROLLOVER_AND_LEFT_RETURN, MIDDLE_ROLLOVER_AND_STANDUP_SPOT};

void ShowSpotMeteorLamps() {
  if (GameMode==GAME_MODE_SKILL_SHOT) {
    byte lampPhase = ((CurrentTime-GameModeStartTime)/300)%4;
    for (byte count=0; count<3; count++) {
      BSOS_SetLampState(RolloverLamps[count], lampPhase, lampPhase%2);
    }
    
  } else {
    BSOS_SetLampState(LEFT_ROLLOVER_AND_RIGHT_INLANE, SpotMeteorTargetByte&SPOT_METEOR_TARGET_LIGHT_1);
    BSOS_SetLampState(RIGHT_ROLLOVER_AND_LEFT_RETURN, SpotMeteorTargetByte&SPOT_METEOR_TARGET_LIGHT_3);
    BSOS_SetLampState(MIDDLE_ROLLOVER_AND_STANDUP_SPOT, StandupAward==1 && (SpotMeteorTargetByte&SPOT_METEOR_TARGET_LIGHT_2));    
  }
}


void ShowRocketBumperLamps() {
  if (GameMode==GAME_MODE_METEOR_STORM && OrpheusRound[CurrentPlayer]<LAUNCH_ROCKETS_WITH_DEAD_BUMPER_UNTIL_LEVEL) {
    BSOS_SetLampState(RIGHT_OUT_LANE_AND_1_BUMPER, MeteorStormSectors&METEOR_STORM_SECTOR_1, 1, 500);
    BSOS_SetLampState(LEFT_OUT_LANE_AND_3_BUMPER, MeteorStormSectors&METEOR_STORM_SECTOR_3, 1, 500);
  } else {
    boolean flash1 = (Rocket1BumperAnimationTime!=0 && ((CurrentTime-Rocket1BumperAnimationTime)<ROCKET_ANIMATION_DURATION));
    boolean flash3 = (Rocket3BumperAnimationTime!=0 && ((CurrentTime-Rocket3BumperAnimationTime)<ROCKET_ANIMATION_DURATION));
    BSOS_SetLampState(RIGHT_OUT_LANE_AND_1_BUMPER, (CollectRocketsByte&ROCKET_1_COLLECT_LIGHT)||flash1, 0, flash1?100:0);
    BSOS_SetLampState(LEFT_OUT_LANE_AND_3_BUMPER, (CollectRocketsByte&ROCKET_3_COLLECT_LIGHT)||flash3, 0, flash3?100:0);
  }
}


////////////////////////////////////////////////////////////////////////////
//
//  Display Management functions
//
////////////////////////////////////////////////////////////////////////////
unsigned long LastTimeScoreChanged = 0;
unsigned long LastTimeOverrideAnimated = 0;
unsigned long LastFlashOrDash = 0;
#ifdef USE_SCORE_OVERRIDES
unsigned long ScoreOverrideValue[4]= {0, 0, 0, 0};
byte ScoreOverrideStatus = 0;
#endif
byte LastScrollPhase = 0;

byte MagnitudeOfScore(unsigned long score) {
  if (score == 0) return 0;

  byte retval = 0;
  while (score > 0) {
    score = score / 10;
    retval += 1;
  }
  return retval;
}

#ifdef USE_SCORE_OVERRIDES
void OverrideScoreDisplay(byte displayNum, unsigned long value, boolean animate) {
  if (displayNum>3) return;
  ScoreOverrideStatus |= (0x10<<displayNum);
  if (animate) ScoreOverrideStatus |= (0x01<<displayNum);
  else ScoreOverrideStatus &= ~(0x01<<displayNum);
  ScoreOverrideValue[displayNum] = value;
}
#endif

byte GetDisplayMask(byte numDigits) {
  byte displayMask = 0;
  for (byte digitCount=0; digitCount<numDigits; digitCount++) {
    displayMask |= (0x20>>digitCount);
  }  
  return displayMask;
}


void ShowPlayerScores(byte displayToUpdate, boolean flashCurrent, boolean dashCurrent, unsigned long allScoresShowValue=0) {

#ifdef USE_SCORE_OVERRIDES      
  if (displayToUpdate==0xFF) ScoreOverrideStatus = 0;
#endif  

  byte displayMask = 0x3F;
  unsigned long displayScore = 0;
  unsigned long overrideAnimationSeed = CurrentTime/250;
  byte scrollPhaseChanged = false;

  byte scrollPhase = ((CurrentTime-LastTimeScoreChanged)/250)%16;
  if (scrollPhase!=LastScrollPhase) {
    LastScrollPhase = scrollPhase;
    scrollPhaseChanged = true;
  }

  boolean updateLastTimeAnimated = false;

  for (byte scoreCount=0; scoreCount<4; scoreCount++) {

#ifdef USE_SCORE_OVERRIDES      
    // If this display is currently being overriden, then we should update it
    if (allScoresShowValue==0 && (ScoreOverrideStatus & (0x10<<scoreCount))) {
      displayScore = ScoreOverrideValue[scoreCount];
      byte numDigits = MagnitudeOfScore(displayScore);
      if (numDigits==0) numDigits = 1;
      if (numDigits<(BALLY_STERN_OS_NUM_DIGITS-1) && (ScoreOverrideStatus & (0x01<<scoreCount))) {
        if (overrideAnimationSeed!=LastTimeOverrideAnimated) {
          updateLastTimeAnimated = true;
          byte shiftDigits = (overrideAnimationSeed)%(((BALLY_STERN_OS_NUM_DIGITS+1)-numDigits)+((BALLY_STERN_OS_NUM_DIGITS-1)-numDigits));
          if (shiftDigits>=((BALLY_STERN_OS_NUM_DIGITS+1)-numDigits)) shiftDigits = (BALLY_STERN_OS_NUM_DIGITS-numDigits)*2-shiftDigits;
          byte digitCount;
          displayMask = GetDisplayMask(numDigits);
          for (digitCount=0; digitCount<shiftDigits; digitCount++) {
            displayScore *= 10;
            displayMask = displayMask>>1;
          }
          BSOS_SetDisplayBlank(scoreCount, 0x00);
          BSOS_SetDisplay(scoreCount, displayScore, false);
          BSOS_SetDisplayBlank(scoreCount, displayMask);
        }
      } else {
        BSOS_SetDisplay(scoreCount, displayScore, true);
      }
      
    } else {
#endif      
      // No override, update scores designated by displayToUpdate
      //CurrentScores[CurrentPlayer] = CurrentScoreOfCurrentPlayer;
      if (allScoresShowValue==0) displayScore = (scoreCount==CurrentPlayer && MachineState>=MACHINE_STATE_INIT_GAMEPLAY)?CurrentScoreOfCurrentPlayer:CurrentScores[scoreCount];
      else displayScore = allScoresShowValue;

      // If we're updating all displays, or the one currently matching the loop, or if we have to scroll
      if (displayToUpdate==0xFF || displayToUpdate==scoreCount || displayScore>BALLY_STERN_OS_MAX_DISPLAY_SCORE) {

        // Don't show this score if it's not a current player score (even if it's scrollable)
        if (displayToUpdate==0xFF && (scoreCount>=CurrentNumPlayers&&CurrentNumPlayers!=0) && allScoresShowValue==0) {
          BSOS_SetDisplayBlank(scoreCount, 0x00);
          continue;
        }

        if (displayScore>BALLY_STERN_OS_MAX_DISPLAY_SCORE) {
          // Score needs to be scrolled
          if ((CurrentTime-LastTimeScoreChanged)<4000) {
            BSOS_SetDisplay(scoreCount, displayScore%(BALLY_STERN_OS_MAX_DISPLAY_SCORE+1), false);  
            BSOS_SetDisplayBlank(scoreCount, BALLY_STERN_OS_ALL_DIGITS_MASK);
          } else {

            // Scores are scrolled 10 digits and then we wait for 6
            if (scrollPhase<11 && scrollPhaseChanged) {
              byte numDigits = MagnitudeOfScore(displayScore);
              
              // Figure out top part of score
              unsigned long tempScore = displayScore;
              if (scrollPhase<BALLY_STERN_OS_NUM_DIGITS) {
                displayMask = BALLY_STERN_OS_ALL_DIGITS_MASK;
                for (byte scrollCount=0; scrollCount<scrollPhase; scrollCount++) {
                  displayScore = (displayScore % (BALLY_STERN_OS_MAX_DISPLAY_SCORE+1)) * 10;
                  displayMask = displayMask >> 1;
                }
              } else {
                displayScore = 0; 
                displayMask = 0x00;
              }

              // Add in lower part of score
              if ((numDigits+scrollPhase)>10) {
                byte numDigitsNeeded = (numDigits+scrollPhase)-10;
                for (byte scrollCount=0; scrollCount<(numDigits-numDigitsNeeded); scrollCount++) {
                  tempScore /= 10;
                }
                displayMask |= GetDisplayMask(MagnitudeOfScore(tempScore));
                displayScore += tempScore;
              }
              BSOS_SetDisplayBlank(scoreCount, displayMask);
              BSOS_SetDisplay(scoreCount, displayScore);
            }
          }          
        } else {
          if (flashCurrent) {
            unsigned long flashSeed = CurrentTime/250;
            if (flashSeed != LastFlashOrDash) {
              LastFlashOrDash = flashSeed;
              if (((CurrentTime/250)%2)==0) BSOS_SetDisplayBlank(scoreCount, 0x00);
              else BSOS_SetDisplay(scoreCount, displayScore, true, 2);
            }
          } else if (dashCurrent) {
            unsigned long dashSeed = CurrentTime/50;
            if (dashSeed != LastFlashOrDash) {
              LastFlashOrDash = dashSeed;
              byte dashPhase = (CurrentTime/60)%36;
              byte numDigits = MagnitudeOfScore(displayScore);
              if (dashPhase<12) { 
                displayMask = GetDisplayMask((numDigits==0)?2:numDigits);
                if (dashPhase<7) {
                  for (byte maskCount=0; maskCount<dashPhase; maskCount++) {
                    displayMask &= ~(0x01<<maskCount);
                  }
                } else {
                  for (byte maskCount=12; maskCount>dashPhase; maskCount--) {
                    displayMask &= ~(0x20>>(maskCount-dashPhase-1));
                  }
                }
                BSOS_SetDisplay(scoreCount, displayScore);
                BSOS_SetDisplayBlank(scoreCount, displayMask);
              } else {
                BSOS_SetDisplay(scoreCount, displayScore, true, 2);
              }
            }
          } else {
            BSOS_SetDisplay(scoreCount, displayScore, true, 2);          
          }
        }
      } // End if this display should be updated
#ifdef USE_SCORE_OVERRIDES            
    } // End on non-overridden
#endif    
  } // End loop on scores

  if (updateLastTimeAnimated) {
    LastTimeOverrideAnimated = overrideAnimationSeed;
  }

}




////////////////////////////////////////////////////////////////////////////
//
//  Machine State Helper functions
//
////////////////////////////////////////////////////////////////////////////
boolean AddPlayer(boolean resetNumPlayers = false) {

  if (Credits < 1 && !FreePlayMode) return false;
  if (CurrentNumPlayers >= 4) return false;

  if (resetNumPlayers) CurrentNumPlayers = 0;
  CurrentNumPlayers += 1;
  BSOS_SetDisplay(CurrentNumPlayers - 1, 0);
  BSOS_SetDisplayBlank(CurrentNumPlayers - 1, 0x30);

  if (!FreePlayMode) {
    Credits -= 1;
    BSOS_WriteByteToEEProm(BSOS_CREDITS_EEPROM_BYTE, Credits);
    BSOS_SetDisplayCredits(Credits);
    BSOS_SetCoinLockout(false);
  }
  PlaySoundEffect(SOUND_EFFECT_ADD_PLAYER_1 + (CurrentNumPlayers - 1));

  BSOS_WriteULToEEProm(BSOS_TOTAL_PLAYS_EEPROM_START_BYTE, BSOS_ReadULFromEEProm(BSOS_TOTAL_PLAYS_EEPROM_START_BYTE) + 1);

  return true;
}

void AddCoinToAudit(byte switchHit) {

  unsigned short coinAuditStartByte = 0;

  switch (switchHit) {
    case SW_COIN_3: coinAuditStartByte = BSOS_CHUTE_3_COINS_START_BYTE; break;
    case SW_COIN_2: coinAuditStartByte = BSOS_CHUTE_2_COINS_START_BYTE; break;
    case SW_COIN_1: coinAuditStartByte = BSOS_CHUTE_1_COINS_START_BYTE; break;
  }

  if (coinAuditStartByte) {
    BSOS_WriteULToEEProm(coinAuditStartByte, BSOS_ReadULFromEEProm(coinAuditStartByte) + 1);
  }

}


void AddCredit(boolean playSound = false, byte numToAdd = 1) {
  if (Credits < MaximumCredits) {
    Credits += numToAdd;
    if (Credits > MaximumCredits) Credits = MaximumCredits;
    BSOS_WriteByteToEEProm(BSOS_CREDITS_EEPROM_BYTE, Credits);
    if (playSound) PlaySoundEffect(SOUND_EFFECT_ADD_CREDIT);
    BSOS_SetDisplayCredits(Credits);
    BSOS_SetCoinLockout(false);
  } else {
    BSOS_SetDisplayCredits(Credits);
    BSOS_SetCoinLockout(true);
  }

}

void AddSpecialCredit() {
  AddCredit(false, 1);
  BSOS_PushToTimedSolenoidStack(SOL_KNOCKER, 3, CurrentTime, true);
  BSOS_WriteULToEEProm(BSOS_TOTAL_REPLAYS_EEPROM_START_BYTE, BSOS_ReadULFromEEProm(BSOS_TOTAL_REPLAYS_EEPROM_START_BYTE) + 1);  
}

void AwardSpecial() {
  if (SpecialCollected) return;
  SpecialCollected = true;
  if (TournamentScoring) {
    CurrentScoreOfCurrentPlayer += SpecialValue;
  } else {
    AddSpecialCredit();
  }
}

void AwardExtraBall() {
  if (ExtraBallCollected) return;
  ExtraBallCollected = true;
  if (TournamentScoring) {
    CurrentScoreOfCurrentPlayer += ExtraBallValue;
  } else {
    SamePlayerShootsAgain = true;
    BSOS_SetLampState(SHOOT_AGAIN, SamePlayerShootsAgain);
    StopAudio();
    PlaySoundEffect(SOUND_EFFECT_EXTRA_BALL);
    ResumeBackgroundSong();
  }
}

#define ADJ_TYPE_LIST                 1
#define ADJ_TYPE_MIN_MAX              2
#define ADJ_TYPE_MIN_MAX_DEFAULT      3
#define ADJ_TYPE_SCORE                4
#define ADJ_TYPE_SCORE_WITH_DEFAULT   5
#define ADJ_TYPE_SCORE_NO_DEFAULT     6
byte AdjustmentType = 0;
byte NumAdjustmentValues = 0;
byte AdjustmentValues[8];
unsigned long AdjustmentScore;
byte *CurrentAdjustmentByte = NULL;
unsigned long *CurrentAdjustmentUL = NULL;
byte CurrentAdjustmentStorageByte = 0;
byte TempValue = 0;


int RunSelfTest(int curState, boolean curStateChanged) {
  int returnState = curState;
  CurrentNumPlayers = 0;

  if (curStateChanged) {
    // Send a stop-all command and reset the sample-rate offset, in case we have
    //  reset while the WAV Trigger was already playing.
    StopAudio();
  }

  // Any state that's greater than CHUTE_3 is handled by the Base Self-test code
  // Any that's less, is machine specific, so we handle it here.
  if (curState >= MACHINE_STATE_TEST_CHUTE_3_COINS) {
    returnState = RunBaseSelfTest(returnState, curStateChanged, CurrentTime, SW_CREDIT_RESET, SW_SLAM);
  } else {
    byte curSwitch = BSOS_PullFirstFromSwitchStack();

    if (curSwitch == SW_SELF_TEST_SWITCH && (CurrentTime - GetLastSelfTestChangedTime()) > 250) {
      SetLastSelfTestChangedTime(CurrentTime);
      returnState -= 1;
    }

    if (curSwitch == SW_SLAM) {
      returnState = MACHINE_STATE_ATTRACT;
    }

    if (curStateChanged) {
      for (int count = 0; count < 4; count++) {
        BSOS_SetDisplay(count, 0);
        BSOS_SetDisplayBlank(count, 0x00);
      }
      BSOS_SetDisplayCredits(MACHINE_STATE_TEST_SOUNDS - curState);
      BSOS_SetDisplayBallInPlay(0, false);
      CurrentAdjustmentByte = NULL;
      CurrentAdjustmentUL = NULL;
      CurrentAdjustmentStorageByte = 0;

      AdjustmentType = ADJ_TYPE_MIN_MAX;
      AdjustmentValues[0] = 0;
      AdjustmentValues[1] = 1;
      TempValue = 0;

      switch (curState) {
        case MACHINE_STATE_ADJUST_FREEPLAY:
          CurrentAdjustmentByte = (byte *)&FreePlayMode;
          CurrentAdjustmentStorageByte = EEPROM_FREE_PLAY_BYTE;
          break;
        case MACHINE_STATE_ADJUST_BALL_SAVE:
          AdjustmentType = ADJ_TYPE_LIST;
          NumAdjustmentValues = 5;
          AdjustmentValues[1] = 5;
          AdjustmentValues[2] = 10;
          AdjustmentValues[3] = 15;
          AdjustmentValues[4] = 20;
          CurrentAdjustmentByte = &BallSaveNumSeconds;
          CurrentAdjustmentStorageByte = EEPROM_BALL_SAVE_BYTE;
          break;
        case MACHINE_STATE_ADJUST_MUSIC_LEVEL:
          AdjustmentType = ADJ_TYPE_MIN_MAX_DEFAULT;
          AdjustmentValues[1] = 3;
          CurrentAdjustmentByte = &MusicLevel;
          CurrentAdjustmentStorageByte = EEPROM_MUSIC_LEVEL_BYTE;
          break;
        case MACHINE_STATE_ADJUST_TOURNAMENT_SCORING:
          CurrentAdjustmentByte = (byte *)&TournamentScoring;
          CurrentAdjustmentStorageByte = EEPROM_TOURNAMENT_SCORING_BYTE;
          break;
        case MACHINE_STATE_ADJUST_TILT_WARNING:
          AdjustmentValues[1] = 2;
          CurrentAdjustmentByte = &MaxTiltWarnings;
          CurrentAdjustmentStorageByte = EEPROM_TILT_WARNING_BYTE;
          break;
        case MACHINE_STATE_ADJUST_AWARD_OVERRIDE:
          AdjustmentType = ADJ_TYPE_MIN_MAX_DEFAULT;
          AdjustmentValues[1] = 7;
          CurrentAdjustmentByte = &ScoreAwardReplay;
          CurrentAdjustmentStorageByte = EEPROM_AWARD_OVERRIDE_BYTE;
          break;
        case MACHINE_STATE_ADJUST_BALLS_OVERRIDE:
          AdjustmentType = ADJ_TYPE_LIST;
          NumAdjustmentValues = 3;
          AdjustmentValues[0] = 3;
          AdjustmentValues[1] = 5;
          AdjustmentValues[2] = 99;
          CurrentAdjustmentByte = &BallsPerGame;
          CurrentAdjustmentStorageByte = EEPROM_BALLS_OVERRIDE_BYTE;
          break;
        case MACHINE_STATE_ADJUST_SCROLLING_SCORES:
          CurrentAdjustmentByte = (byte *)&ScrollingScores;
          CurrentAdjustmentStorageByte = EEPROM_SCROLLING_SCORES_BYTE;
          break;

        case MACHINE_STATE_ADJUST_EXTRA_BALL_AWARD:
          AdjustmentType = ADJ_TYPE_SCORE_WITH_DEFAULT;
          CurrentAdjustmentUL = &ExtraBallValue;
          CurrentAdjustmentStorageByte = EEPROM_EXTRA_BALL_SCORE_BYTE;
          break;

        case MACHINE_STATE_ADJUST_SPECIAL_AWARD:
          AdjustmentType = ADJ_TYPE_SCORE_WITH_DEFAULT;
          CurrentAdjustmentUL = &SpecialValue;
          CurrentAdjustmentStorageByte = EEPROM_SPECIAL_SCORE_BYTE;
          break;

        case MACHINE_STATE_ADJUST_DIM_LEVEL:
          AdjustmentType = ADJ_TYPE_LIST;
          NumAdjustmentValues = 2;
          AdjustmentValues[0] = 2;
          AdjustmentValues[1] = 3;
          CurrentAdjustmentByte = &DimLevel;
          CurrentAdjustmentStorageByte = EEPROM_DIM_LEVEL_BYTE;
          for (int count = 0; count < 7; count++) BSOS_SetLampState(MIDDLE_ROCKET_7K + count, 1, 1);
          break;

        case MACHINE_STATE_ADJUST_DONE:
          returnState = MACHINE_STATE_ATTRACT;
          break;
      }

    }

    // Change value, if the switch is hit
    if (curSwitch == SW_CREDIT_RESET) {

      if (CurrentAdjustmentByte && (AdjustmentType == ADJ_TYPE_MIN_MAX || AdjustmentType == ADJ_TYPE_MIN_MAX_DEFAULT)) {
        byte curVal = *CurrentAdjustmentByte;
        curVal += 1;
        if (curVal > AdjustmentValues[1]) {
          if (AdjustmentType == ADJ_TYPE_MIN_MAX) curVal = AdjustmentValues[0];
          else {
            if (curVal > 99) curVal = AdjustmentValues[0];
            else curVal = 99;
          }
        }
        *CurrentAdjustmentByte = curVal;
        if (CurrentAdjustmentStorageByte) EEPROM.write(CurrentAdjustmentStorageByte, curVal);
      } else if (CurrentAdjustmentByte && AdjustmentType == ADJ_TYPE_LIST) {
        byte valCount = 0;
        byte curVal = *CurrentAdjustmentByte;
        byte newIndex = 0;
        for (valCount = 0; valCount < (NumAdjustmentValues - 1); valCount++) {
          if (curVal == AdjustmentValues[valCount]) newIndex = valCount + 1;
        }
        *CurrentAdjustmentByte = AdjustmentValues[newIndex];
        if (CurrentAdjustmentStorageByte) EEPROM.write(CurrentAdjustmentStorageByte, AdjustmentValues[newIndex]);
      } else if (CurrentAdjustmentUL && (AdjustmentType == ADJ_TYPE_SCORE_WITH_DEFAULT || AdjustmentType == ADJ_TYPE_SCORE_NO_DEFAULT)) {
        unsigned long curVal = *CurrentAdjustmentUL;
        curVal += 5000;
        if (curVal > 100000) curVal = 0;
        if (AdjustmentType == ADJ_TYPE_SCORE_NO_DEFAULT && curVal == 0) curVal = 5000;
        *CurrentAdjustmentUL = curVal;
        if (CurrentAdjustmentStorageByte) BSOS_WriteULToEEProm(CurrentAdjustmentStorageByte, curVal);
      }

      if (curState == MACHINE_STATE_ADJUST_DIM_LEVEL) {
        BSOS_SetDimDivisor(1, DimLevel);
      }
    }

    // Show current value
    if (CurrentAdjustmentByte != NULL) {
      BSOS_SetDisplay(0, (unsigned long)(*CurrentAdjustmentByte), true);
    } else if (CurrentAdjustmentUL != NULL) {
      BSOS_SetDisplay(0, (*CurrentAdjustmentUL), true);
    }

  }

  if (curState == MACHINE_STATE_ADJUST_DIM_LEVEL) {
    for (int count = 0; count < 7; count++) BSOS_SetLampState(MIDDLE_ROCKET_7K + count, 1, (CurrentTime / 1000) % 2);
  }

  if (returnState == MACHINE_STATE_ATTRACT) {
    // If any variables have been set to non-override (99), return
    // them to dip switch settings
    // Balls Per Game, Player Loses On Ties, Novelty Scoring, Award Score
//    DecodeDIPSwitchParameters();
    ReadStoredParameters();
  }

  return returnState;
}




////////////////////////////////////////////////////////////////////////////
//
//  Audio Output functions
//
////////////////////////////////////////////////////////////////////////////

#if defined(USE_WAV_TRIGGER) || defined(USE_WAV_TRIGGER_1p3)
byte CurrentBackgroundSong = SOUND_EFFECT_NONE;
#endif

void StopAudio() {
#if defined(USE_WAV_TRIGGER) || defined(USE_WAV_TRIGGER_1p3)
  wTrig.stopAllTracks();
#endif
}

void ResumeBackgroundSong() {
  byte curSong = CurrentBackgroundSong;
  CurrentBackgroundSong = SOUND_EFFECT_NONE;
  PlayBackgroundSong(curSong);
}

void PlayBackgroundSong(byte songNum) {

#if defined(USE_WAV_TRIGGER) || defined(USE_WAV_TRIGGER_1p3)
  if (MusicLevel > 1) {
    if (CurrentBackgroundSong != songNum) {
      if (CurrentBackgroundSong != SOUND_EFFECT_NONE) wTrig.trackStop(CurrentBackgroundSong);
      if (songNum != SOUND_EFFECT_NONE) {
#ifdef USE_WAV_TRIGGER_1p3
        wTrig.trackPlayPoly(songNum, true);
#else
        wTrig.trackPlayPoly(songNum);
#endif
        wTrig.trackLoop(songNum, true);
        wTrig.trackGain(songNum, -4);
      }
      CurrentBackgroundSong = songNum;
    }
  }
#else
  byte test = songNum;
  songNum = test;
#endif

}



unsigned long NextSoundEffectTime = 0;

void PlaySoundEffect(byte soundEffectNum) {

  if (MusicLevel == 0) return;

#if defined(USE_WAV_TRIGGER) || defined(USE_WAV_TRIGGER_1p3)

#ifndef USE_WAV_TRIGGER_1p3
  if (  soundEffectNum == SOUND_EFFECT_THUMPER_BUMPER_HIT || 
        SOUND_EFFECT_SPINNER ) wTrig.trackStop(soundEffectNum);
#endif
  wTrig.trackPlayPoly(soundEffectNum);
//  char buf[128];
//  sprintf(buf, "s=%d\n", soundEffectNum);
//  Serial.write(buf);
#endif

}

inline void StopSoundEffect(byte soundEffectNum) {
#if defined(USE_WAV_TRIGGER) || defined(USE_WAV_TRIGGER_1p3)
  wTrig.trackStop(soundEffectNum);
#endif  
}

////////////////////////////////////////////////////////////////////////////
//
//  Attract Mode
//
////////////////////////////////////////////////////////////////////////////

unsigned long AttractLastLadderTime = 0;
byte AttractLastLadderBonus = 0;
unsigned long AttractLastStarTime = 0;
byte AttractLastHeadMode = 255;
byte AttractLastPlayfieldMode = 255;
byte InAttractMode = false;
byte FlashyLights[] = {LEFT_ROLLOVER_AND_RIGHT_INLANE, RIGHT_ROLLOVER_AND_LEFT_RETURN, MIDDLE_ROLLOVER_AND_STANDUP_SPOT, LEFT_OUT_LANE_AND_3_BUMPER, RIGHT_OUT_LANE_AND_1_BUMPER};


int RunAttractMode(int curState, boolean curStateChanged) {

  int returnState = curState;

  if (curStateChanged) {
    BSOS_DisableSolenoidStack();
    BSOS_TurnOffAllLamps();
    BSOS_SetDisableFlippers(true);
    if (DEBUG_MESSAGES) {
      Serial.write("Entering Attract Mode\n\r");
    }

    AttractLastHeadMode = 0;
    AttractLastPlayfieldMode = 0;
  }

  // Alternate displays between high score and blank
  if (CurrentTime<16000) {
    if (AttractLastHeadMode!=1) {
      ShowPlayerScores(0xFF, false, false);
      BSOS_SetDisplayCredits(Credits, true);
      BSOS_SetDisplayBallInPlay(0, true);
    }    
  } else if ((CurrentTime / 8000) % 2 == 0) {

    if (AttractLastHeadMode != 2) {
      BSOS_SetLampState(HIGH_SCORE_TO_DATE, 1, 0, 250);
      BSOS_SetLampState(GAME_OVER, 0);
      LastTimeScoreChanged = CurrentTime;
    }
    AttractLastHeadMode = 2;
    ShowPlayerScores(0xFF, false, false, HighScore);
  } else {
    if (AttractLastHeadMode != 3) {
      if (CurrentTime<32000) {
        for (int count = 0; count < 4; count++) {
          CurrentScores[count] = 0;
        }
        CurrentNumPlayers = 0;
      }
      BSOS_SetLampState(HIGH_SCORE_TO_DATE, 0);
      BSOS_SetLampState(GAME_OVER, 1);
//      BSOS_SetDisplayCredits(Credits, true);
//      BSOS_SetDisplayBallInPlay(0, true);
      LastTimeScoreChanged = CurrentTime;
    }
    ShowPlayerScores(0xFF, false, false);
    
    AttractLastHeadMode = 3;
  }

  byte attractPlayfieldPhase = ((CurrentTime/5000)%5); 

  if (attractPlayfieldPhase!=AttractLastPlayfieldMode) {
    BSOS_TurnOffAllLamps();
    AttractLastPlayfieldMode = attractPlayfieldPhase;
    if (attractPlayfieldPhase==2) GameMode = GAME_MODE_SKILL_SHOT;
    else GameMode = GAME_MODE_UNSTRUCTURED_PLAY;
    AttractLastLadderBonus = 1;
    AttractLastLadderTime = CurrentTime;
    if (attractPlayfieldPhase==2) {
      for (int count=0; count<5; count++) {
        BSOS_SetLampState(FlashyLights[count], 1, count%2, 220+(count*100));
      }
    }
  }

  if (attractPlayfieldPhase < 2) {
    ShowLampAnimation(1, 40, CurrentTime, 14, false, false);
  } else if (attractPlayfieldPhase==3) {
    ShowLampAnimation(0, 40, CurrentTime, 11, false, false, 12);
  } else if (attractPlayfieldPhase==2) {
    if ((CurrentTime-AttractLastLadderTime)>200) {
      Bonus[0] = AttractLastLadderBonus;
      Bonus[1] = 7-AttractLastLadderBonus;
      Bonus[2] = AttractLastLadderBonus;
      ShowBonus(Bonus, BonusAnimationStart);
      AttractLastLadderBonus += 1;
      if (AttractLastLadderBonus>7) AttractLastLadderBonus = 0;
      AttractLastLadderTime = CurrentTime;      
    }
#if not defined (BALLY_STERN_OS_SOFTWARE_DISPLAY_INTERRUPT)    
    BSOS_DataRead(0);
#endif    
    ShowMeteorLamps();
  } else {
    ShowLampAnimation(2, 40, CurrentTime, 14, false, false);
  }

  byte switchHit;
  while ( (switchHit = BSOS_PullFirstFromSwitchStack()) != SWITCH_STACK_EMPTY ) {
    if (switchHit == SW_CREDIT_RESET) {
      if (AddPlayer(true)) returnState = MACHINE_STATE_INIT_GAMEPLAY;
    }
    if (switchHit == SW_COIN_1 || switchHit == SW_COIN_2 || switchHit == SW_COIN_3) {
      AddCoinToAudit(switchHit);
      AddCredit(true, 1);
    }
    if (switchHit == SW_SELF_TEST_SWITCH && (CurrentTime - GetLastSelfTestChangedTime()) > 250) {
      returnState = MACHINE_STATE_TEST_LIGHTS;
      SetLastSelfTestChangedTime(CurrentTime);
    }
  }

  return returnState;
}





////////////////////////////////////////////////////////////////////////////
//
//  Game Play functions
//
////////////////////////////////////////////////////////////////////////////
byte CountBits(byte byteToBeCounted) {
  byte numBits = 0;

  for (byte count=0; count<8; count++) {
    numBits += (byteToBeCounted&0x01);
    byteToBeCounted = byteToBeCounted>>1;
  }

  return numBits;
}


void SetBonusX(byte newBonusX) {
  CurrentBonusX = newBonusX;
  if (CurrentBonusX>7) CurrentBonusX = 7;

  
  if (CurrentBonusX<3) {
    SpotMeteorTargetByte = 0x77;
    CollectRocketsByte = 0;
  } else if (CurrentBonusX==3) {
    SpotMeteorTargetByte = 0x33;
    CollectRocketsByte = 0x33;
  } else if (CurrentBonusX==4) {
    SpotMeteorTargetByte = 0x11;
    CollectRocketsByte = 0x77;
  } else {
    SpotMeteorTargetByte = 0x00;
    CollectRocketsByte = 0xFF;
  }

  
}


boolean HandleSpotMeteorSwitch(byte switchHit, boolean overrideSpot=false) {
  
  byte lampMask;
  if (switchHit==SW_LEFT_IN_LANE || switchHit==SW_UPPER_ROLLOVER) lampMask = SPOT_METEOR_TARGET_LIGHT_3;
  else if (switchHit==SW_MIDDLE_ROLLOVER || switchHit==SW_STAND_UP) lampMask = SPOT_METEOR_TARGET_LIGHT_2;
  else lampMask = SPOT_METEOR_TARGET_LIGHT_1;

  if (lampMask==SPOT_METEOR_TARGET_LIGHT_2 && StandupAward!=1) return false;

  if ((SpotMeteorTargetByte & lampMask) || overrideSpot) {
    byte meteorTarget = 0;
    byte currentSwitches = CheckSequentialSwitches(SW_DROP_TARGET_R, 6);
    if (SpotMeteorReverse) {
      for (int count=0; count<6; count++) {
        if (!(currentSwitches & (1<<count))) {
          meteorTarget = count;
          break;
        }
      }
    } else {
      for (int count=5; count>=0; count--) {
        if (!(currentSwitches & (1<<count))) {
          meteorTarget = count;
          break;
        }
      }
    }
    SpotMeteorReverse = !SpotMeteorReverse;
    BSOS_PushToSolenoidStack(MeteorSolenoid[meteorTarget], 6);
    SpotMeteorTargetByte &= ~(lampMask);

    return true;
  }

  return false;
}

boolean HandleStandupHit() {
  if (StandupAward==1) {
    // Hit with nothing lit
    CurrentScoreOfCurrentPlayer += 500;
    PlaySoundEffect(SOUND_EFFECT_STANDUP_UNLIT);    
  } else if (StandupAward==2) {
    AwardExtraBall();
  } else if (StandupAward==3) {
    AwardSpecial();
  }
  StandupAward = 1;
  return true;
}


void HandleMeteorDropTargetHit(byte switchHit) {

  byte targetBit = 1<<(switchHit-SW_DROP_TARGET_R);

  // Check to see if this drop target is not already down
  if ((CurrentMeteorTargetsValid&targetBit)==0) {

    if (switchHit==SW_DROP_TARGET_M || switchHit==SW_DROP_TARGET_R) {
      CurrentScoreOfCurrentPlayer += 6000;
    } else {
      CurrentScoreOfCurrentPlayer += 2000;
    }

    byte currentSwitches = CheckSequentialSwitches(SW_DROP_TARGET_R, 6);
    
    if (GameMode!=GAME_MODE_ORPHEUS && GameMode!=GAME_MODE_ORPHEUS_FRAGMENTED) {
      if (currentSwitches==0x3F) {
        BSOS_PushToTimedSolenoidStack(SOL_RESET_METEOR_BANK, 15, CurrentTime+500);
        SetBonusX(CurrentBonusX+1);
          if (GameMode!=GAME_MODE_METEOR_STORM_START && GameMode!=GAME_MODE_METEOR_STORM) {
            GameMode = GAME_MODE_METEOR_STORM_START;
            GameModeStartTime = 0;
          }
        BonusXAnimationStart = CurrentTime;
        PlaySoundEffect(SOUND_EFFECT_METEOR_DROP_6);
      } else {
        PlaySoundEffect(SOUND_EFFECT_METEOR_DROP_1 + (CountBits(currentSwitches)-1));
      }
    } else {
      CurrentMeteorTargetsValid |= targetBit;
      // Special rules for Orpheus
      if (switchHit==SkillShotWowLit) {
        CurrentScoreOfCurrentPlayer += 50000;
        PlaySoundEffect(SOUND_EFFECT_ORPHEUS_HIT_1 + CurrentTime%2);
        if (currentSwitches==0x3F) {
          if (GameMode==GAME_MODE_ORPHEUS) OrpheusCompletedFlag |= (1<<CurrentPlayer);
          GameMode = GAME_MODE_ORPHEUS_FINISH;
          GameModeStartTime = 0; 
          GameModeEndTime = CurrentTime + ORPHEUS_FINISH_DURATION_LONG;
        }
        LastOrpheusHitTime = CurrentTime;
      } else {
        CurrentScoreOfCurrentPlayer += 2000;
        SkillShotWowLit = 0;
        // if they missed the Orpheus chunk, put them into fragmented mode
        if (GameMode!=GAME_MODE_ORPHEUS_FRAGMENTED) {
          GameMode = GAME_MODE_ORPHEUS_FRAGMENTED;
          GameModeStartTime = 0;
        } else {
          
        }
        if (currentSwitches==0x3F) {
          GameModeStartTime = 0;
          GameMode = GAME_MODE_ORPHEUS_FINISH;
          GameModeEndTime = CurrentTime + ORPHEUS_FINISH_DURATION_SHORT;
        }
        LastOrpheusHitTime = CurrentTime;
      }      
    }
    
  }  
}


byte ResetSolenoid[] = {SOL_RESET_1_BANK, SOL_RESET_2_BANK, SOL_RESET_3_BANK};
byte StartingBankSwitch[] = {SW_LOWER_LEFT_3_TOP, SW_TOP_LEFT_3_TOP, SW_RIGHT_3_TOP};

void Handle3BankHit(byte switchNum, byte bankNum) {

  // Only score if this switch hasn't been hit yet
  byte switchMask = (1<<(switchNum-StartingBankSwitch[bankNum]));
  if (!(Current3BankTargetsHit[bankNum]&switchMask)) {
    if (TargetWowPhase==(bankNum+1) && switchMask==0x04) {
      AwardExtraBall();
      TargetWowPhase = 0;
    }
    if (GameMode==GAME_MODE_METEOR_STORM) {
      if (!(RocketsArmed&(1<<(bankNum)))) PlaySoundEffect(SOUND_EFFECT_ROCKET_1_ARMED + (bankNum));
    }
    RocketsArmed |= 1<<(bankNum);
    
    Current3BankTargetsHit[bankNum] |= switchMask;
    Bonus[bankNum] += 1;
    if (Bonus[bankNum]>MAX_DISPLAY_BONUS) Bonus[bankNum] = MAX_DISPLAY_BONUS;
    BonusAnimationStart[bankNum] = CurrentTime;
    BonusAnimationShown[bankNum] = 0;
  }

  if (CheckSequentialSwitches(StartingBankSwitch[bankNum], 3)==0x07) {
    BSOS_PushToTimedSolenoidStack(ResetSolenoid[bankNum], 12, 250);
    Current3BankTargetsHit[bankNum] = 0;
    if (GameMode==GAME_MODE_METEOR_STORM && OrpheusRound[CurrentPlayer]<LAUNCH_ROCKETS_WITH_BANK_CLEAR_UNTIL_LEVEL) {  
      RocketLaunchedTime[bankNum] = CurrentTime;
      PlaySoundEffect(SOUND_EFFECT_ROCKET_LAUNCHED);
      RocketsArmed &= ~(1<<(bankNum));
      RocketsFiredFromBank = true;
    }
  }

}


int InitGamePlay() {

  if (DEBUG_MESSAGES) {
    Serial.write("Starting game\n\r");
  }

  // The start button has been hit only once to get
  // us into this mode, so we assume a 1-player game
  // at the moment
  BSOS_EnableSolenoidStack();
  BSOS_SetCoinLockout((Credits >= MaximumCredits) ? true : false);
  BSOS_TurnOffAllLamps();

  // Turn back on all lamps that are needed

  // When we go back to attract mode, there will be no need to reset scores
  ResetScoresToClearVersion = false;

  // Reset displays & game state variables
  for (int count = 0; count < 4; count++) {
//    BSOS_SetDisplay(count, 0);
//    if (count == 0) BSOS_SetDisplayBlank(count, 0x30);
//    else BSOS_SetDisplayBlank(count, 0x00);

    // Initialize game-specific variables
    BonusX[count] = 1; 
    MeteorStatus[count] = 0;
    NumberOfMeteorsHit[count] = 0;
    OrpheusRound[count] = 0;
    NumberOfSpins[count] = 0;    
  }
  memset(CurrentScores, 0, 4*sizeof(unsigned long));
  OrpheusCompletedFlag = 0;
  
  SamePlayerShootsAgain = false;
  CurrentBallInPlay = 1;
  CurrentNumPlayers = 1;
  CurrentPlayer = 0;
  ShowPlayerScores(0xFF, false, false);

  return MACHINE_STATE_INIT_NEW_BALL;
}


int InitNewBall(bool curStateChanged, byte playerNum, int ballNum) {

  // If we're coming into this mode for the first time
  // then we have to do everything to set up the new ball
  if (curStateChanged) {
    BSOS_TurnOffAllLamps();
    SamePlayerShootsAgain = false;
    BallFirstSwitchHitTime = 0;

    BSOS_SetDisableFlippers(false);
    BSOS_EnableSolenoidStack();
//    BSOS_SetDisplayCredits(Credits, true);
    if (CurrentNumPlayers>1 && (ballNum!=1 || playerNum!=0)) PlaySoundEffect(SOUND_EFFECT_PLAYER_1_UP+playerNum);

    BSOS_SetDisplayBallInPlay(ballNum);
//    BSOS_SetLampState(TILT, 0);

    if (BallSaveNumSeconds > 0) {
      BSOS_SetLampState(SHOOT_AGAIN, 1, 0, 500);
    }

    BallSaveUsed = false;
    BallTimeInTrough = 0;
    NumTiltWarnings = 0;
    LastTiltWarningTime = 0;

    // Initialize game-specific start-of-ball lights & variables
    GameModeStartTime = 0;
    GameMode = GAME_MODE_SKILL_SHOT;
    SkillShotHit = false;
    StandupAward = 1;
    StandupHits = 0;
    StandupResetTime = 0;
    ExtraBallCollected = false;
    SpecialCollected = false;
    SpinnerValue = 0;
//    for (int i=0; i<3; i++) {
//      Bonus[i] = 0;
//      BonusAnimationStart[i] = 0;
//    }
    memset(Bonus, 0, 3*sizeof(byte));
    memset(BonusAnimationStart, 0, 3*sizeof(unsigned long));
    SpotMeteorReverse = false;
    SetBonusX(CurrentBonusX);

    // Start appropriate mode music
    if (BSOS_ReadSingleSwitchState(SW_OUTHOLE)) {
      BSOS_PushToTimedSolenoidStack(SOL_OUTHOLE, 4, CurrentTime + 600);
    }

    for (byte count=0; count<3; count++) {
      BSOS_PushToTimedSolenoidStack(ResetSolenoid[count], 12, CurrentTime + 100*((unsigned long)count));
      Current3BankTargetsHit[count] = 0;
    }
    
    Rocket1BumperAnimationTime = 0;
    Rocket3BumperAnimationTime = 0;
    CollectRocketAnimationStart = 0;
    CollectRocketCurrentLevel = 0;
    MeteorHitStarted = false;
    MeteorHitStartTime = 0;
    MeteorStormSectors = 0;
    OrpheusPhase = 0;
    OrpheusScoring = false;

    // Reset drop targets for skill shot
    SetMeteorDropTargets(0, 450);

    CurrentScoreOfCurrentPlayer = CurrentScores[CurrentPlayer];
    CurrentBonusX =  BonusX[CurrentPlayer];
  }

  // We should only consider the ball initialized when
  // the ball is no longer triggering the SW_OUTHOLE
  if (BSOS_ReadSingleSwitchState(SW_OUTHOLE)) {
    return MACHINE_STATE_INIT_NEW_BALL;
  } else {
    return MACHINE_STATE_NORMAL_GAMEPLAY;
  }

}


void PlayBackgroundSongBasedOnBall(byte ballNum) {
  if (ballNum==1) {
    PlayBackgroundSong(SOUND_EFFECT_BACKGROUND_1);
  } else if (ballNum==BallsPerGame) {
    PlayBackgroundSong(SOUND_EFFECT_BACKGROUND_6);
  } else {
    PlayBackgroundSong(SOUND_EFFECT_BACKGROUND_2 + CurrentTime%4);
  }
}


void AdvanceMeteors() {

  for (int count=0; count<3; count++) {
    if (MeteorStormSectors & (1<<count)) {
      byte newDescent = 7 - ((CurrentTime-MeteorStormStart)/((unsigned long)MeteorStormDescentSpeed[count]*500));
      MeteorStormDescent[count] = newDescent;
      if (newDescent==0) {
        MeteorStormSectors &= ~(1<<count);
        PlaySoundEffect(SOUND_EFFECT_METEOR_IMPACT);
      }
    }
  }
  
}


byte OrpheusMeteorMask[] = {0x2D, 0x17, 0x15, 0x12, 0x07, 0x05, 0x02};
byte GetOrpheusMeteorMask() {
  byte index = OrpheusRound[CurrentPlayer];
  if (index>7) index = 7;
  return OrpheusMeteorMask[index];
}

byte OrpheusFragmentMask[] = {0x2D, 0x15, 0x12, 0x15, 0x12, 0x00, 0x02};
byte GetOrpheusFragmentMask() {
  byte index = OrpheusRound[CurrentPlayer];
  if (index>7) index = 7;
  return OrpheusFragmentMask[index];
}


// This function manages all timers, flags, and lights
int ManageGameMode() {
  int returnState = MACHINE_STATE_NORMAL_GAMEPLAY;

//  byte currentStep = 0;
  boolean specialAnimationRunning = false;
  byte bitMask;

  switch ( (GameMode) ) {
    case GAME_MODE_SKILL_SHOT:
      if (GameModeStartTime==0) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = 0;
        if (CurrentBallInPlay==1) {
          PlayBackgroundSong(SOUND_EFFECT_GAME_START_INTRO);
        } else {
          PlayBackgroundSongBasedOnBall(CurrentBallInPlay);
        }
      }
      if (BallFirstSwitchHitTime!=0) {
        // Reset Meteor Drop Targets for player
        SetMeteorDropTargets(MeteorStatus[CurrentPlayer], 450);
            
        // Something has been hit, so we shouldn't be in skill shot anymore
        GameMode = GAME_MODE_UNSTRUCTURED_PLAY;
        GameModeStartTime = 0;
      }
    break;    
    
    case GAME_MODE_UNSTRUCTURED_PLAY:
      // If this is the first time in this mode
      if (GameModeStartTime==0) {
        GameModeStartTime = CurrentTime;
        if (CurrentBallInPlay==1) PlayBackgroundSongBasedOnBall(CurrentBallInPlay);
      }

      if (SkillShotHit && (CurrentTime-GameModeStartTime)<1500) {
        specialAnimationRunning = true;
        ShowLampAnimation(2, 40, (CurrentTime-GameModeStartTime), 14, false, true);
      }

      if ((CurrentTime-LastTimeSpinnerHit)>5000) {
        BSOS_SetDisplayCredits(Credits);
      }

      if (ExtraBallCollected==false && Bonus[0]>0 && Bonus[0]<7 && Bonus[0]==Bonus[1] && Bonus[0]==Bonus[2]) {
        if (TargetWowPhase==0) TargetWowPhase = 1;
      } else {
        TargetWowPhase = 0;        
      }

      if (CurrentTime > StandupResetTime) {
        StandupResetTime = 0;
        StandupAward = 1;
      }
      
    break;
    
    case GAME_MODE_METEOR_STORM_START:
      if (GameModeStartTime==0) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + METEOR_STORM_START_DURATION;
        if (NumberOfMeteorsHit[CurrentPlayer]==0) PlaySoundEffect(SOUND_EFFECT_METEOR_STORM_START);
        else PlaySoundEffect(SOUND_EFFECT_METEOR_STORM_START_2 + CurrentTime%3);
        MeteorStormSectors = 0;
        RocketsArmed = 0;     
      }
      if (CurrentTime>GameModeEndTime) {
        GameMode = GAME_MODE_METEOR_STORM;
        GameModeStartTime = 0;
      }
    break;
    
    case GAME_MODE_METEOR_STORM:
      if (GameModeStartTime==0) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + METEOR_STORM_TIME_TO_ARM;
        StartMeteorStorm();
      }

      AdvanceMeteors();

      if (MeteorHitStartTime!=0) {
        if (MeteorHitStarted) {
          MeteorHitStarted = false;
          BSOS_TurnOffAllLamps();
        }

        specialAnimationRunning = true;
        ShowBonusLamps(false);
        ShowLampAnimation(1, 40, (CurrentTime-MeteorHitStartTime), 14, false, false);

        if (CurrentTime>MeteorHitEndTime) {
          BSOS_TurnOffAllLamps();
          MeteorHitStartTime = 0;
          // If this is the last meteor, start a new one
          if (MeteorStormSectors==0) {
            StartMeteorStorm();
          }
        }
      } else if (MeteorStormSectors==0) {
        GameModeStartTime = 0;
        GameMode = GAME_MODE_UNSTRUCTURED_PLAY;
      } else if (NumberOfMeteorsHit[CurrentPlayer]>=GetNextMeteorGoal()) {
        GameModeStartTime = 0;
        GameMode = GAME_MODE_ORPHEUS_START;
        //NumberOfMeteorsHit[CurrentPlayer] = 0;
      }
    break;
    
    case GAME_MODE_ORPHEUS_START:
      if (GameModeStartTime==0) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + ORPHEUS_START_DURATION;
        BSOS_TurnOffAllLamps();
        StopAudio();
        PlaySoundEffect(SOUND_EFFECT_ORPHEUS_START);
        SetMeteorDropTargets(0x3F, 0, false);
        OrpheusPhase = 0;
      }

      specialAnimationRunning = true;
      ShowLampAnimation(0, 40, (CurrentTime-GameModeStartTime), 11, false, false, 12);
      if (CurrentTime>GameModeEndTime) {
        GameModeStartTime = 0;
        GameMode = GAME_MODE_ORPHEUS;
      }
    break;
    
    case GAME_MODE_ORPHEUS:
      if (GameModeStartTime==0) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + ORPHEUS_DURATION;
        SetMeteorDropTargets(GetOrpheusMeteorMask());
        BSOS_TurnOffAllLamps();
        PlayBackgroundSong(SOUND_EFFECT_BACKGROUND_ORPHEUS);
        LastOrpheusHitTime = CurrentTime;
        OrpheusPhase = 0;
      }

      if ((CurrentTime-LastOrpheusHitTime)>10000) {
        OrpheusPhase = (OrpheusPhase+1)%6;
        bitMask = GetOrpheusMeteorMask();
        for (byte count=0; count<OrpheusPhase; count++) {
          bitMask = (bitMask>>1)|(bitMask<<5);
        }
        LastOrpheusHitTime = CurrentTime;
        SetMeteorDropTargets(bitMask);
      }
      
      if (CurrentTime>GameModeEndTime) {
        GameModeStartTime = 0;
        GameMode = GAME_MODE_ORPHEUS_FAIL;
      }
    break;
    
    case GAME_MODE_ORPHEUS_FRAGMENTED:
      if (GameModeStartTime==0) {
        GameModeStartTime = CurrentTime;
        PlaySoundEffect(SOUND_EFFECT_ORPHEUS_FRAGMENTED);
        GameModeEndTime = CurrentTime + ORPHEUS_FRAGMENTED_DURATION;
        SetMeteorDropTargets(0x33);
        OrpheusPhase = 0;
        LastOrpheusHitTime = CurrentTime;
      }

      if ((CurrentTime-LastOrpheusHitTime)>5000) {
        OrpheusPhase = (OrpheusPhase+1)%6;
        bitMask = GetOrpheusFragmentMask();
        for (byte count=0; count<OrpheusPhase; count++) {
          bitMask = (bitMask>>1) | (bitMask<<5);
        }
        SetMeteorDropTargets(bitMask);
        LastOrpheusHitTime = CurrentTime;
      }
      
      if (CurrentTime>GameModeEndTime) {
        GameModeStartTime = 0;
        GameMode = GAME_MODE_ORPHEUS_FAIL;
      }
    break;

    case GAME_MODE_ORPHEUS_FAIL:
      if (GameModeStartTime==0) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + ORPHEUS_FAIL_DURATION;
        PlaySoundEffect(SOUND_EFFECT_MISSION_FAILED);
        BSOS_TurnOffAllLamps();
      }
      specialAnimationRunning = true;

      if (CurrentTime>GameModeEndTime) {
        GameModeStartTime = 0;
        GameMode = GAME_MODE_UNSTRUCTURED_PLAY;
        SetMeteorDropTargets(0);
        PlayBackgroundSongBasedOnBall(CurrentBallInPlay);
        OrpheusRound[CurrentPlayer] += 1;
      }      
    break;
    
    case GAME_MODE_ORPHEUS_FINISH:
      if (GameModeStartTime==0) {
        BSOS_TurnOffAllLamps();
        StopAudio();
        PlaySoundEffect(SOUND_EFFECT_ORPHEUS_FINISH);
        SetMeteorDropTargets(0x3F, 0, false);
        GameModeStartTime = CurrentTime;
        OrpheusScoring = true;
        // Game mode end time is controlled by caller
      }
      
      specialAnimationRunning = true;
      ShowLampAnimation(2, 20, (CurrentTime-GameModeStartTime), 11, false, false);

      if (CurrentTime>GameModeEndTime) {
        GameModeStartTime = 0;
        GameMode = GAME_MODE_UNSTRUCTURED_PLAY;
//        StopSoundEffect(SOUND_EFFECT_ORPHEUS_FINISH);
        StopAudio();
        SetMeteorDropTargets(0);
        PlayBackgroundSongBasedOnBall(CurrentBallInPlay);
        OrpheusRound[CurrentPlayer] += 1;
        OrpheusScoring = false;
      }
    break;

  }

  if ( GameModeStartTime!=0 && !specialAnimationRunning && NumTiltWarnings<=MaxTiltWarnings ) {
    ShowMeteorLamps();
    ShowStandupTargetLamps();
    ShowBonusLamps();
    ShowBonusXLamps();
#if not defined (BALLY_STERN_OS_SOFTWARE_DISPLAY_INTERRUPT)    
    BSOS_DataRead(0);
#endif    
    ShowSpinnerLamps();
    ShowShootAgainLamp();
    ShowSpotMeteorLamps();
    ShowRocketBumperLamps();
    ShowRocketDropLamps();
  }
  ShowPlayerScores(CurrentPlayer, (BallFirstSwitchHitTime==0)?true:false, (BallFirstSwitchHitTime>0 && ((CurrentTime-LastTimeScoreChanged)>2000))?true:false);

  // Check to see if ball is in the outhole
  if (BSOS_ReadSingleSwitchState(SW_OUTHOLE)) {
    if (BallTimeInTrough == 0) {
      BallTimeInTrough = CurrentTime;
    } else {
      // Make sure the ball stays on the sensor for at least
      // 0.5 seconds to be sure that it's not bouncing
      if ((CurrentTime - BallTimeInTrough) > 500) {

        if (BallFirstSwitchHitTime == 0 && NumTiltWarnings <= MaxTiltWarnings) {
          // Nothing hit yet, so return the ball to the player
          BSOS_PushToTimedSolenoidStack(SOL_OUTHOLE, 4, CurrentTime);
          BallTimeInTrough = 0;
          returnState = MACHINE_STATE_NORMAL_GAMEPLAY;
        } else {
          // if we haven't used the ball save, and we're under the time limit, then save the ball
          if (!BallSaveUsed && ((CurrentTime - BallFirstSwitchHitTime)) < ((unsigned long)BallSaveNumSeconds*1000)) {
            BSOS_PushToTimedSolenoidStack(SOL_OUTHOLE, 4, CurrentTime + 100);
            BallSaveUsed = true;
            PlaySoundEffect(SOUND_EFFECT_SHOOT_AGAIN);
            BSOS_SetLampState(SHOOT_AGAIN, 0);
            BallTimeInTrough = CurrentTime;
            returnState = MACHINE_STATE_NORMAL_GAMEPLAY;
          } else {
            ShowPlayerScores(0xFF, false, false);
            PlayBackgroundSong(SOUND_EFFECT_NONE);
            StopAudio();

            if (GameMode>=GAME_MODE_ORPHEUS_START) {
              OrpheusRound[CurrentPlayer] += 1;
            }
            
            if (CurrentBallInPlay<BallsPerGame) PlaySoundEffect(SOUND_EFFECT_BALL_OVER);
            // Turn off bonus animations
            memset(BonusAnimationStart, 0, 3*sizeof(unsigned long));
            returnState = MACHINE_STATE_COUNTDOWN_BONUS;
          }
        }
      }
    }
  } else {
    BallTimeInTrough = 0;
  }

  return returnState;
}



unsigned long CountdownStartTime = 0;
unsigned long LastCountdownReportTime = 0;
unsigned long BonusCountDownEndTime = 0;

int CountdownBonus(boolean curStateChanged) {

  // If this is the first time through the countdown loop
  if (curStateChanged) {
//    BSOS_SetLampState(BALL_IN_PLAY, 1, 0, 250);

    CountdownStartTime = CurrentTime;
    ShowBonus(Bonus, BonusAnimationStart);

    LastCountdownReportTime = CountdownStartTime;
    BonusCountDownEndTime = 0xFFFFFFFF;
  }

  if ((CurrentTime - LastCountdownReportTime) > 200) {

    if (Bonus[0] || Bonus[1] || Bonus[2]) {

      // Only give sound & score if this isn't a tilt
      if (NumTiltWarnings <= MaxTiltWarnings) {
        PlaySoundEffect(SOUND_EFFECT_BONUS_COUNT + (CurrentBonusX-1));
        CurrentScoreOfCurrentPlayer += 1000*((unsigned long)CurrentBonusX);
      }

      for (int i=0; i<3; i++) {
        if (Bonus[i]) {
          Bonus[i] -= 1;
          break;
        }
      }
      
      ShowBonus(Bonus, BonusAnimationStart);
    } else if (BonusCountDownEndTime == 0xFFFFFFFF) {
      BonusCountDownEndTime = CurrentTime + 1000;
    }
    LastCountdownReportTime = CurrentTime;
  }

  if (CurrentTime > BonusCountDownEndTime) {

    // Reset any lights & variables of goals that weren't completed
    BonusCountDownEndTime = 0xFFFFFFFF;
    return MACHINE_STATE_BALL_OVER;
  }

  return MACHINE_STATE_COUNTDOWN_BONUS;
}



void CheckHighScores() {
  unsigned long highestScore = 0;
  int highScorePlayerNum = 0;
  for (int count = 0; count < CurrentNumPlayers; count++) {
    if (CurrentScores[count] > highestScore) highestScore = CurrentScores[count];
    highScorePlayerNum = count;
  }

  if (highestScore > HighScore) {
    HighScore = highestScore;
    if (HighScoreReplay) {
      AddCredit(false, 3);
      BSOS_WriteULToEEProm(BSOS_TOTAL_REPLAYS_EEPROM_START_BYTE, BSOS_ReadULFromEEProm(BSOS_TOTAL_REPLAYS_EEPROM_START_BYTE) + 3);
    }
    BSOS_WriteULToEEProm(BSOS_HIGHSCORE_EEPROM_START_BYTE, highestScore);
    BSOS_WriteULToEEProm(BSOS_TOTAL_HISCORE_BEATEN_START_BYTE, BSOS_ReadULFromEEProm(BSOS_TOTAL_HISCORE_BEATEN_START_BYTE) + 1);

    for (int count = 0; count < 4; count++) {
      if (count == highScorePlayerNum) {
        BSOS_SetDisplay(count, CurrentScores[count], true, 2);
      } else {
        BSOS_SetDisplayBlank(count, 0x00);
      }
    }

    BSOS_PushToTimedSolenoidStack(SOL_KNOCKER, 3, CurrentTime, true);
    BSOS_PushToTimedSolenoidStack(SOL_KNOCKER, 3, CurrentTime + 300, true);
    BSOS_PushToTimedSolenoidStack(SOL_KNOCKER, 3, CurrentTime + 600, true);
  }
}






unsigned long MatchSequenceStartTime = 0;
unsigned long MatchDelay = 150;
byte MatchDigit = 0;
byte NumMatchSpins = 0;
byte ScoreMatches = 0;

int ShowMatchSequence(boolean curStateChanged) {
  if (!MatchFeature) return MACHINE_STATE_ATTRACT;

  if (curStateChanged) {
    MatchSequenceStartTime = CurrentTime;
    MatchDelay = 1500;
    MatchDigit = random(0, 10);
    NumMatchSpins = 0;
    BSOS_SetLampState(MATCH, 1, 0);
    BSOS_SetDisableFlippers();
    ScoreMatches = 0;
//    BSOS_SetLampState(BALL_IN_PLAY, 0);
  }

  if (NumMatchSpins < 40) {
    if (CurrentTime > (MatchSequenceStartTime + MatchDelay)) {
      MatchDigit += 1;
      if (MatchDigit > 9) MatchDigit = 0;
      //PlaySoundEffect(10+(MatchDigit%2));
      PlaySoundEffect(SOUND_EFFECT_MATCH_SPIN);
      BSOS_SetDisplayBallInPlay((int)MatchDigit * 10);
      MatchDelay += 50 + 4 * NumMatchSpins;
      NumMatchSpins += 1;
      BSOS_SetLampState(MATCH, NumMatchSpins % 2, 0);

      if (NumMatchSpins == 40) {
        BSOS_SetLampState(MATCH, 0);
        MatchDelay = CurrentTime - MatchSequenceStartTime;
      }
    }
  }

  if (NumMatchSpins >= 40 && NumMatchSpins <= 43) {
    if (CurrentTime > (MatchSequenceStartTime + MatchDelay)) {
      if ( (CurrentNumPlayers > (NumMatchSpins - 40)) && ((CurrentScores[NumMatchSpins - 40] / 10) % 10) == MatchDigit) {
        ScoreMatches |= (1 << (NumMatchSpins - 40));
        AddSpecialCredit();
        MatchDelay += 1000;
        NumMatchSpins += 1;
        BSOS_SetLampState(MATCH, 1);
      } else {
        NumMatchSpins += 1;
      }
      if (NumMatchSpins == 44) {
        MatchDelay += 5000;
      }
    }
  }

  if (NumMatchSpins > 43) {
    if (CurrentTime > (MatchSequenceStartTime + MatchDelay)) {
      return MACHINE_STATE_ATTRACT;
    }
  }

  for (int count = 0; count < 4; count++) {
    if ((ScoreMatches >> count) & 0x01) {
      // If this score matches, we're going to flash the last two digits
      if ( (CurrentTime / 200) % 2 ) {
        BSOS_SetDisplayBlank(count, BSOS_GetDisplayBlank(count) & 0x0F);
      } else {
        BSOS_SetDisplayBlank(count, BSOS_GetDisplayBlank(count) | 0x30);
      }
    }
  }

  return MACHINE_STATE_MATCH_MODE;
}



int RunGamePlayMode(int curState, boolean curStateChanged) {
  int returnState = curState;
  byte totalBonusAtTop = Bonus[0] + Bonus[1] + Bonus[2];
  //byte bonusAtTop2 = Bonus2;
  //byte bonusAtTop3 = Bonus3;
  unsigned long scoreAtTop = CurrentScoreOfCurrentPlayer;

  // Very first time into gameplay loop
  if (curState == MACHINE_STATE_INIT_GAMEPLAY) {
    returnState = InitGamePlay();
  } else if (curState == MACHINE_STATE_INIT_NEW_BALL) {
    returnState = InitNewBall(curStateChanged, CurrentPlayer, CurrentBallInPlay);
  } else if (curState == MACHINE_STATE_NORMAL_GAMEPLAY) {
    returnState = ManageGameMode();
  } else if (curState == MACHINE_STATE_COUNTDOWN_BONUS) {
    returnState = CountdownBonus(curStateChanged);
    ShowPlayerScores(0xFF, false, false);
  } else if (curState == MACHINE_STATE_BALL_OVER) {
    MeteorStatus[CurrentPlayer] = CheckSequentialSwitches(SW_DROP_TARGET_R, 6);
    if (GameMode>=GAME_MODE_ORPHEUS_START) MeteorStatus[CurrentPlayer] = 0;
    CurrentScores[CurrentPlayer] = CurrentScoreOfCurrentPlayer;
    BonusX[CurrentPlayer] = CurrentBonusX;
    
    if (SamePlayerShootsAgain) {
      PlaySoundEffect(SOUND_EFFECT_SHOOT_AGAIN);
      returnState = MACHINE_STATE_INIT_NEW_BALL;
    } else {
      
      CurrentPlayer += 1;
      if (CurrentPlayer >= CurrentNumPlayers) {
        CurrentPlayer = 0;
        CurrentBallInPlay += 1;
      }
      CurrentScoreOfCurrentPlayer = CurrentScores[CurrentPlayer];
      //CurrentPlayerMeteorStatus = MeteorStatus[CurrentPlayer];
      CurrentBonusX = BonusX[CurrentPlayer];
      scoreAtTop = CurrentScoreOfCurrentPlayer;
      
      if (CurrentBallInPlay > BallsPerGame) {
        CheckHighScores();
        PlaySoundEffect(SOUND_EFFECT_GAME_OVER);
        for (int count = 0; count < CurrentNumPlayers; count++) {
          BSOS_SetDisplay(count, CurrentScores[count], true, 2);
        }

        returnState = MACHINE_STATE_MATCH_MODE;
      }
      else returnState = MACHINE_STATE_INIT_NEW_BALL;
    }
  } else if (curState == MACHINE_STATE_MATCH_MODE) {
    returnState = ShowMatchSequence(curStateChanged);
  }

  byte switchHit;
  boolean shiftLamps = false;
  byte currentSwitches = CheckSequentialSwitches(SW_DROP_TARGET_R, 6);

  if (NumTiltWarnings <= MaxTiltWarnings) {
    while ( (switchHit = BSOS_PullFirstFromSwitchStack()) != SWITCH_STACK_EMPTY ) {

      if (DEBUG_MESSAGES) {
        char buf[128];
        sprintf(buf, "Switch Hit = %d\n", switchHit);
        Serial.write(buf);
      }
      unsigned long scoreAddition = 0;
        
      if (OrpheusScoring) {
        if (switchHit!=SW_SLAM && switchHit!=SW_TILT) {
          PlaySoundEffect(SOUND_EFFECT_WIZARD_SCORING);
          CurrentScoreOfCurrentPlayer += 25000;
        }
      }
      
      switch (switchHit) {
        case SW_SLAM:
          //          BSOS_DisableSolenoidStack();
          //          BSOS_SetDisableFlippers(true);
          //          BSOS_TurnOffAllLamps();
          //          BSOS_SetLampState(GAME_OVER, 1);
          //          delay(1000);
          //          return MACHINE_STATE_ATTRACT;
          break;
        case SW_TILT:
          // This should be debounced
          if ((CurrentTime - LastTiltWarningTime) > TILT_WARNING_DEBOUNCE_TIME) {
            LastTiltWarningTime = CurrentTime;
            NumTiltWarnings += 1;
            if (NumTiltWarnings > MaxTiltWarnings) {
              BSOS_DisableSolenoidStack();
              BSOS_SetDisableFlippers(true);
              BSOS_TurnOffAllLamps();
              StopAudio();
              PlaySoundEffect(SOUND_EFFECT_TILT);
              BSOS_SetLampState(TILT, 1);
            }
            PlaySoundEffect(SOUND_EFFECT_TILT_WARNING);
          }
          break;
        case SW_SELF_TEST_SWITCH:
          returnState = MACHINE_STATE_TEST_LIGHTS;
          SetLastSelfTestChangedTime(CurrentTime);
          break;
        case SW_STAND_UP:
          if (GameMode==GAME_MODE_SKILL_SHOT) {
          } else if (GameMode==GAME_MODE_METEOR_STORM && RocketsArmed) {  
            for (int count=0; count<3; count++) {
              if (RocketsArmed&(1<<count)) {
                RocketLaunchedTime[count] = CurrentTime;
                PlaySoundEffect(SOUND_EFFECT_ROCKET_LAUNCHED);
                RocketsArmed &= ~(1<<count);
                RocketsFiredFromBank = false;
                // If we're at a level where we only launch one rocket, then break
                if (OrpheusRound[CurrentPlayer]>=LAUNCH_ALL_ROCKETS_WITH_STANDUP_UNTIL_LEVEL) break;
              }
            }
          } else {
            if (!HandleSpotMeteorSwitch(switchHit)) {
              HandleStandupHit();
            }
            StandupHits += 1;
            StandupResetTime = CurrentTime + ((unsigned long)StandupHits)*100 + 100;
            if (!ExtraBallCollected) StandupAward = 2;
            else if (!SpecialCollected) StandupAward = 3;
          }
          if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
          break;
        case SW_LEFT_IN_LANE:
        case SW_RIGHT_IN_LANE:
        case SW_UPPER_ROLLOVER:
        case SW_MIDDLE_ROLLOVER:
        case SW_LOWER_ROLLOVER:
          if (GameMode==GAME_MODE_SKILL_SHOT) {
            PlaySoundEffect(SOUND_EFFECT_SKILL_SHOT);
            CurrentScoreOfCurrentPlayer += 25000;
            HandleSpotMeteorSwitch(switchHit, true);
            SkillShotHit = true;
          } else {
            if (!HandleSpotMeteorSwitch(switchHit)) {
              if (switchHit==SW_LEFT_IN_LANE) {
                PlaySoundEffect(SOUND_EFFECT_LEFT_INLANE);
              } else if (switchHit==SW_RIGHT_IN_LANE) {
                PlaySoundEffect(SOUND_EFFECT_RIGHT_INLANE);
              } else {
                PlaySoundEffect(SOUND_EFFECT_UNLIT_ROLLOVER);
              }
              
              CurrentScoreOfCurrentPlayer += 500;
            }
          }
          
          if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
          break;
        case SW_RIGHT_OUT_LANE:
          PlaySoundEffect(SOUND_EFFECT_RIGHT_OUTLANE);
          if (CollectRocketsByte&ROCKET_1_COLLECT_LIGHT) {
            CollectRocketAnimationStart = CurrentTime;
            CollectRocketCurrentLevel = 0;
            CollectRocketsByte &= ~ROCKET_1_COLLECT_LIGHT;
          } else {
            CurrentScoreOfCurrentPlayer += 1000;
          }
          if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
          break;
        case SW_LEFT_OUT_LANE:
          PlaySoundEffect(SOUND_EFFECT_LEFT_OUTLANE);
          if (CollectRocketsByte&ROCKET_3_COLLECT_LIGHT) {
            CollectRocketAnimationStart = CurrentTime;
            CollectRocketCurrentLevel = 0;
            CollectRocketsByte &= ~ROCKET_3_COLLECT_LIGHT;
          } else {
            CurrentScoreOfCurrentPlayer += 1000;
          }
          if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
          break;
        case SW_SPINNER:
          for (int count=0; count<6; count++) {
            if (currentSwitches&(1<<count)) {
              scoreAddition += 200;
              if (count==0 || count==5) scoreAddition += 600;
            }
          }
          CurrentScoreOfCurrentPlayer += (200 + scoreAddition);
          PlaySoundEffect(SOUND_EFFECT_SPINNER);
          SpinnerHitPhase = (SpinnerHitPhase+1)%12;
          LastTimeSpinnerHit = CurrentTime;
          shiftLamps = true;
          if (GameMode==GAME_MODE_UNSTRUCTURED_PLAY) {
            NumberOfSpins[CurrentPlayer] += 1;
            if (NumberOfSpins[CurrentPlayer]>99) NumberOfSpins[CurrentPlayer] = 99;
            BSOS_SetDisplayCredits(99-NumberOfSpins[CurrentPlayer]);
          }
          if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
          break;
        case SW_DROP_TARGET_M:
        case SW_DROP_TARGET_E_1:
        case SW_DROP_TARGET_T:
        case SW_DROP_TARGET_E_2:
        case SW_DROP_TARGET_O:
        case SW_DROP_TARGET_R:
          if ((CurrentTime-GameModeStartTime)>DROP_SWITCH_BLACKOUT_PERIOD || GameMode!=GAME_MODE_SKILL_SHOT) {
            if (GameMode!=GAME_MODE_SKILL_SHOT) {
              HandleMeteorDropTargetHit(switchHit);
            } else {
              if (switchHit==SkillShotWowLit) {
                PlaySoundEffect(SOUND_EFFECT_SUPER_SKILL_SHOT);
                CurrentScoreOfCurrentPlayer += 100000;
                SkillShotHit = true;
              } else {
                PlaySoundEffect(SOUND_EFFECT_SKILL_SHOT);
                CurrentScoreOfCurrentPlayer += 25000;
                SkillShotHit = true;
              }
            }
            if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
          }
          break;
        case SW_RIGHT_3_TOP:
        case SW_RIGHT_3_LOWER:
        case SW_RIGHT_3_MID:
          if ((CurrentTime-GameModeStartTime)>DROP_SWITCH_BLACKOUT_PERIOD || GameMode!=GAME_MODE_SKILL_SHOT) {
            Handle3BankHit(switchHit, 2);
            if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
          }
        break;
        case SW_TOP_LEFT_3_TOP:
        case SW_TOP_LEFT_3_LOWER:
        case SW_TOP_LEFT_3_MID:
          if ((CurrentTime-GameModeStartTime)>DROP_SWITCH_BLACKOUT_PERIOD || GameMode!=GAME_MODE_SKILL_SHOT) {
            Handle3BankHit(switchHit, 1);
            if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
          }
        break;
        case SW_LOWER_LEFT_3_TOP:
        case SW_LOWER_LEFT_3_LOWER:
        case SW_LOWER_LEFT_3_MID:
          if ((CurrentTime-GameModeStartTime)>DROP_SWITCH_BLACKOUT_PERIOD || GameMode!=GAME_MODE_SKILL_SHOT) {
            Handle3BankHit(switchHit, 0);
            if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
          }
        break;
        case SW_THUMPER_BUMPER:
          if (TargetWowPhase) {
            if (TargetWowPhase==3) TargetWowPhase = 1;
            else TargetWowPhase += 1;
          }
          CurrentScoreOfCurrentPlayer += (unsigned long)1000;
          PlaySoundEffect(SOUND_EFFECT_THUMPER_BUMPER_HIT);
          shiftLamps = true;
          if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
          break;
        case SW_BUMPER_UPPER_RIGHT:
          if (GameMode==GAME_MODE_METEOR_STORM && (MeteorStormSectors & METEOR_STORM_SECTOR_3) && OrpheusRound[CurrentPlayer]<LAUNCH_ROCKETS_WITH_DEAD_BUMPER_UNTIL_LEVEL) {
            RocketLaunchedTime[2] = CurrentTime;
            PlaySoundEffect(SOUND_EFFECT_ROCKET_LAUNCHED);
            RocketsArmed &= ~(METEOR_STORM_SECTOR_3);
          } else {
            if (CollectRocketsByte&ROCKET_3_COLLECT_LIGHT) {
              CollectRocketsByte &= ~ROCKET_3_COLLECT_LIGHT;
              Bonus[2] += 1;
              if (Bonus[2]>MAX_DISPLAY_BONUS) Bonus[2] = MAX_DISPLAY_BONUS;
              BonusAnimationStart[2] = CurrentTime;
              BonusAnimationShown[2] = 0;
            } else {
              PlaySoundEffect(SOUND_EFFECT_ROCKET_3_UNLIT);
            }
            Rocket3BumperAnimationTime = CurrentTime;
            if (Bonus[2]) CurrentScoreOfCurrentPlayer += (unsigned long)Bonus[2]*1000;
            else CurrentScoreOfCurrentPlayer += 500;
          }
          if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
          break;
        case SW_BUMPER_UPPER_LEFT:
          if (GameMode==GAME_MODE_METEOR_STORM && (MeteorStormSectors & METEOR_STORM_SECTOR_1) && OrpheusRound[CurrentPlayer]<LAUNCH_ROCKETS_WITH_DEAD_BUMPER_UNTIL_LEVEL) {
            RocketLaunchedTime[0] = CurrentTime;
            PlaySoundEffect(SOUND_EFFECT_ROCKET_LAUNCHED);
            RocketsArmed &= ~(METEOR_STORM_SECTOR_1);
          } else {
            if (CollectRocketsByte&ROCKET_1_COLLECT_LIGHT) {
              CollectRocketsByte &= ~ROCKET_1_COLLECT_LIGHT;
              //AddToBonus(1, 0, 0);
              Bonus[0] += 1;
              if (Bonus[0]>MAX_DISPLAY_BONUS) Bonus[0] = MAX_DISPLAY_BONUS;
              BonusAnimationStart[0] = CurrentTime;
              BonusAnimationShown[0] = 0;
            } else {
              PlaySoundEffect(SOUND_EFFECT_ROCKET_1_UNLIT);
            }
            Rocket1BumperAnimationTime = CurrentTime;
            if (Bonus[0]) CurrentScoreOfCurrentPlayer += (unsigned long)Bonus[0]*1000;
            else CurrentScoreOfCurrentPlayer += 500;
          }
          if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
          break;
        case SW_LEFT_SLINGSHOT:
        case SW_RIGHT_SLINGSHOT:
          if (TargetWowPhase) {
            if (TargetWowPhase==3) TargetWowPhase = 1;
            else TargetWowPhase += 1;
          }
          CurrentScoreOfCurrentPlayer += 10;
          PlaySoundEffect(SOUND_EFFECT_SLING1+(CurrentTime%3));
          shiftLamps = true;
          if (currentSwitches==0x3F) SetMeteorDropTargets(0, 450);
          if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
          break;
        case SW_COIN_1:
        case SW_COIN_2:
        case SW_COIN_3:
          AddCoinToAudit(switchHit);
          AddCredit(true, 1);
          break;
        case SW_CREDIT_RESET:
          if (CurrentBallInPlay < 2) {
            // If we haven't finished the first ball, we can add players
            AddPlayer();
          } else {
            // If the first ball is over, pressing start again resets the game
            if (Credits >= 1 || FreePlayMode) {
              if (!FreePlayMode) {
                Credits -= 1;
                BSOS_WriteByteToEEProm(BSOS_CREDITS_EEPROM_BYTE, Credits);
                BSOS_SetDisplayCredits(Credits);
              }
              returnState = MACHINE_STATE_INIT_GAMEPLAY;
            }
          }
          if (DEBUG_MESSAGES) {
            Serial.write("Start game button pressed\n\r");
          }
          break;
      }
    }
  } else {
    // We're tilted, so just wait for outhole
    while ( (switchHit = BSOS_PullFirstFromSwitchStack()) != SWITCH_STACK_EMPTY ) {
      switch (switchHit) {
        case SW_SELF_TEST_SWITCH:
          returnState = MACHINE_STATE_TEST_LIGHTS;
          SetLastSelfTestChangedTime(CurrentTime);
          break;
        case SW_COIN_1:
        case SW_COIN_2:
        case SW_COIN_3:
          AddCoinToAudit(switchHit);
          AddCredit(true, 1);
          break;
      }
    }
  }

  if (shiftLamps) {
    SpotMeteorTargetByte = (SpotMeteorTargetByte>>1)|(SpotMeteorTargetByte<<7);
    CollectRocketsByte = (CollectRocketsByte>>1)|(CollectRocketsByte<<7);
  }

  byte totalBonus = Bonus[0] + Bonus[1] + Bonus[2];
  if (totalBonus!=totalBonusAtTop) {
    ShowBonus(Bonus, BonusAnimationStart);
  }

  if (!ScrollingScores && CurrentScoreOfCurrentPlayer > BALLY_STERN_OS_MAX_DISPLAY_SCORE) {
    CurrentScoreOfCurrentPlayer -= BALLY_STERN_OS_MAX_DISPLAY_SCORE;
  }

  if (scoreAtTop != CurrentScoreOfCurrentPlayer) {
    LastTimeScoreChanged = CurrentTime;
    if (!TournamentScoring) {
      for (int awardCount = 0; awardCount < 3; awardCount++) {
        if (AwardScores[awardCount] != 0 && scoreAtTop < AwardScores[awardCount] && CurrentScoreOfCurrentPlayer >= AwardScores[awardCount]) {
          // Player has just passed an award score, so we need to award it
          if (((ScoreAwardReplay >> awardCount) & 0x01)) {
            AddSpecialCredit();
          } else if (!ExtraBallCollected) {
            AwardExtraBall();
          }
        }
      }
    }
  
  }

  return returnState;
}


void loop() {

//#if not defined (BALLY_STERN_OS_SOFTWARE_DISPLAY_INTERRUPT)    
  BSOS_DataRead(0);
//#endif  
  CurrentTime = millis();
  int newMachineState = MachineState;

  if (MachineState < 0) {
    newMachineState = RunSelfTest(MachineState, MachineStateChanged);
  } else if (MachineState == MACHINE_STATE_ATTRACT) {
    newMachineState = RunAttractMode(MachineState, MachineStateChanged);
  } else {
    newMachineState = RunGamePlayMode(MachineState, MachineStateChanged);
  }

  if (newMachineState != MachineState) {
    MachineState = newMachineState;
    MachineStateChanged = true;
  } else {
    MachineStateChanged = false;
  }

  BSOS_ApplyFlashToLamps(CurrentTime);
  BSOS_UpdateTimedSolenoidStack(CurrentTime);

}
