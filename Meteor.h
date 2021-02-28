/**************************************************************************
 *     This file is part of Meteor2021.

    I, Dick Hamill, the author of this program disclaim all copyright
    in order to make this program freely available in perpetuity to
    anyone who would like to use it. Dick Hamill, 6/1/2020

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


// Lamp Numbers (defines for lamps)
#define BOTTOM_LEFT_BANK_SPECIAL  0
#define LEFT_ROCKET_7K            1
#define LEFT_ROCKET_6K            2
#define LEFT_ROCKET_5K            3
#define LEFT_ROCKET_4K            4
#define LEFT_ROCKET_3K            5
#define LEFT_ROCKET_2K            6
#define LEFT_ROCKET_1K            7
#define TOP_LEFT_DROP_TARGET_SPECIAL  8  
#define MIDDLE_ROCKET_7K          9
#define MIDDLE_ROCKET_6K          10
#define MIDDLE_ROCKET_5K          11
#define MIDDLE_ROCKET_4K          12
#define MIDDLE_ROCKET_3K          13
#define MIDDLE_ROCKET_2K          14
#define MIDDLE_ROCKET_1K          15
#define MIDDLE_ROLLOVER_AND_STANDUP_SPOT  16
#define RIGHT_ROCKET_7K           17
#define RIGHT_ROCKET_6K           18
#define RIGHT_ROCKET_5K           19
#define RIGHT_ROCKET_4K           20
#define RIGHT_ROCKET_3K           21
#define RIGHT_ROCKET_2K           22
#define RIGHT_ROCKET_1K           23
#define STANDUP_TARGET_SPECIAL    24
#define RIGHT_BANK_SPECIAL        25
#define METEOR_WOW_R              26
#define METEOR_WOW_O              27
#define METEOR_WOW_E_2            28
#define METEOR_WOW_T              29
#define METEOR_WOW_E_1            30
#define METEOR_WOW_M              31
#define LEFT_ROLLOVER_AND_RIGHT_INLANE  32             
#define RIGHT_ROLLOVER_AND_LEFT_RETURN    33
#define METEOR_DROP_R             34 
#define METEOR_DROP_O             35
#define METEOR_DROP_E_2           36
#define METEOR_DROP_T             37
#define METEOR_DROP_E_1           38
#define METEOR_DROP_M             39
#define SHOOT_AGAIN                       40
#define RIGHT_OUT_LANE_AND_1_BUMPER       41            
#define SPINNER_6                         42
#define SPINNER_5                         43
#define SPINNER_4                         44
#define SPINNER_3                         45
#define SPINNER_2                         46
#define SPINNER_1                         47
#define HIGH_SCORE_TO_DATE                48       
#define LEFT_OUT_LANE_AND_3_BUMPER        49             
#define GAME_OVER                     50
#define TILT                          51
#define STANDUP_TARGET_WOW            52
#define RIGHT_BANK_WOW                53
#define TOP_LEFT_BANK_WOW             54
#define BOTTOM_LEFT_BANK_WOW          55
#define BONUS_1X                  56
#define BONUS_2X                  57
#define BONUS_4X                  58
#define MATCH                     59

/*
All lamps

    {321,870},
    {176,784},
    {292,788},
    {292,813},
    {292,839},
    {292,865},
    {292,892},
    {292,918},
    {292,943},
    {191,550},
    {323,788},
    {323,813},
    {323,839},
    {323,865},
    {323,892},
    {323,918},
    {323,943},
    {264,343},
    {351,788},
    {351,813},
    {351,839},
    {351,865},
    {351,892},
    {351,918},
    {351,943},
    {457,689},
    {556,784},
    {508,404},
    {479,426},
    {449,449},
    {416,469},
    {388,489},
    {355,514},
    {225,384},
    {302,302},
    {481,363},
    {451,386},
    {420,408},
    {390,428},
    {359,451},
    {327,473},
    {369,1166},
    {119,380},
    {595,528},
    {583,561},
    {571,591},
    {559,624},
    {544,654},
    {530,687},
    {374,48},
    {512,311},
    {402,48},
    {388,60},
    {455,731},
    {567,829},
    {182,589},
    {164,831},
    {467,987},
    {467,943},
    {465,898},
    {412,67}



*/ 


#define LAMP_ANIMATION_STEPS  16
byte LampAnimations[3][LAMP_ANIMATION_STEPS][8] = {
  {{0x00, 0x10, 0x30, 0x00, 0x00, 0x00, 0x00, 0x04},
  {0x00, 0x30, 0x70, 0x00, 0x00, 0x00, 0x00, 0x03},
  {0x00, 0x30, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x01},
  {0x00, 0xF0, 0xE0, 0x00, 0x00, 0x01, 0x00, 0x00},
  {0xE0, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0xE0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x70, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x30, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x18, 0x10, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00},
  {0x1D, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x0E, 0x19, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00},
  {0x0E, 0x1F, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00},
  {0x00, 0x1E, 0x0E, 0xFC, 0xFC, 0x00, 0x00, 0x00},
  {0x00, 0x1C, 0x0E, 0x05, 0x00, 0x7C, 0x10, 0x00},
  {0x00, 0x18, 0x1C, 0x00, 0x00, 0xF0, 0x10, 0x00},
  {0x00, 0x10, 0x18, 0x02, 0x00, 0x00, 0x20, 0x00}},
  {{0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x10, 0x38, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x38, 0x7C, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x7C, 0x6C, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0xEE, 0xC6, 0xEE, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0xC6, 0x82, 0xC6, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x82, 0x00, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x06},
  {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x07},
  {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x10, 0x01},
  {0x00, 0x00, 0x00, 0x03, 0x00, 0x80, 0x20, 0x00},
  {0x00, 0x00, 0x00, 0x02, 0x00, 0xC1, 0x60, 0x00},
  {0x00, 0x01, 0x00, 0x80, 0x00, 0x61, 0x40, 0x00},
  {0x00, 0x01, 0x00, 0xE0, 0x80, 0x38, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x70, 0xE0, 0x0C, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x1C, 0x3C, 0x04, 0x00, 0x00}},
  {{0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x03},
  {0x60, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x04},
  {0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0xA0, 0x00},
  {0x07, 0x06, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00},
  {0x00, 0x00, 0x00, 0x01, 0x00, 0x80, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x40, 0x00},
  {0x00, 0x01, 0x00, 0x80, 0x00, 0x0C, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x60, 0x80, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x18, 0x70, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x04, 0x0C, 0x00, 0x00, 0x00}}
};


// Defines for switches
#define SW_STAND_UP       3
#define SW_SPINNER        4
#define SW_CREDIT_RESET   5
#define SW_TILT           6
#define SW_SLAM           7
#define SW_RIGHT_3_TOP    8
#define SW_RIGHT_3_LOWER  9
#define SW_RIGHT_3_MID    10
#define SW_BUMPER_UPPER_RIGHT   11
#define SW_BUMPER_UPPER_LEFT    12
#define SW_THUMPER_BUMPER       13
#define SW_RIGHT_SLINGSHOT      14
#define SW_LEFT_SLINGSHOT       15
#define SW_TOP_LEFT_3_TOP       18
#define SW_TOP_LEFT_3_LOWER     19
#define SW_TOP_LEFT_3_MID       20
#define SW_LOWER_LEFT_3_TOP     21
#define SW_LOWER_LEFT_3_LOWER   22
#define SW_LOWER_LEFT_3_MID     23
#define SW_DROP_TARGET_R        26
#define SW_DROP_TARGET_O        27
#define SW_DROP_TARGET_E_2      28
#define SW_DROP_TARGET_T        29
#define SW_DROP_TARGET_E_1      30
#define SW_DROP_TARGET_M        31
#define SW_OUTHOLE              32
#define SW_RIGHT_OUT_LANE       33
#define SW_LEFT_OUT_LANE        34
#define SW_RIGHT_IN_LANE        35
#define SW_LEFT_IN_LANE         36
#define SW_UPPER_ROLLOVER       37
#define SW_MIDDLE_ROLLOVER      38
#define SW_LOWER_ROLLOVER       39

#define SW_COIN_1         1
#define SW_COIN_2         0
#define SW_COIN_3         2


// Defines for solenoids
#define SOL_RESET_1_BANK      6
#define SOL_RESET_2_BANK      2
#define SOL_RESET_3_BANK      8
#define SOL_RESET_METEOR_BANK 12
#define SOL_DROP_M            3
#define SOL_DROP_E_1          4
#define SOL_DROP_T            7
#define SOL_DROP_E_2          10
#define SOL_DROP_O            11
#define SOL_DROP_R            13
#define SOL_POP_BUMPER        1
#define SOL_KNOCKER           5
#define SOL_LEFT_SLING        0
#define SOL_RIGHT_SLING       9
#define SOL_OUTHOLE           14

// SWITCHES_WITH_TRIGGERS are for switches that will automatically
// activate a solenoid (like in the case of a chime that rings on a rollover)
// but SWITCHES_WITH_TRIGGERS are fully debounced before being activated
#define NUM_SWITCHES_WITH_TRIGGERS         3

// PRIORITY_SWITCHES_WITH_TRIGGERS are switches that trigger immediately
// (like for pop bumpers or slings) - they are not debounced completely
#define NUM_PRIORITY_SWITCHES_WITH_TRIGGERS 3

// Define automatic solenoid triggers (switch, solenoid, number of 1/120ths of a second to fire)
struct PlayfieldAndCabinetSwitch TriggeredSwitches[] = {
  { SW_THUMPER_BUMPER, SOL_POP_BUMPER, 4 },
  { SW_LEFT_SLINGSHOT, SOL_LEFT_SLING, 4 },
  { SW_RIGHT_SLINGSHOT, SOL_RIGHT_SLING, 4 },
};
