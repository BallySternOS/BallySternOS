#include "SendOnlyWavTrigger.h"

void SendOnlyWavTrigger::start(void) {
//  uint8_t txbuf[5];
	WTSerial.begin(57600);
}


// **************************************************************
/*
void SendOnlyWavTrigger::masterGain(int gain) {

uint8_t txbuf[7];
unsigned short vol;

	txbuf[0] = SOM1;
	txbuf[1] = SOM2;
	txbuf[2] = 0x07;
	txbuf[3] = CMD_MASTER_VOLUME;
	vol = (unsigned short)gain;
	txbuf[4] = (uint8_t)vol;
	txbuf[5] = (uint8_t)(vol >> 8);
	txbuf[6] = EOM;
	WTSerial.write(txbuf, 7);
}
*/
// **************************************************************
/*
void SendOnlyWavTrigger::setAmpPwr(bool enable) {

uint8_t txbuf[6];

    txbuf[0] = SOM1;
    txbuf[1] = SOM2;
    txbuf[2] = 0x06;
    txbuf[3] = CMD_AMP_POWER;
    txbuf[4] = enable;
    txbuf[5] = EOM;
    WTSerial.write(txbuf, 6);
}
*/

// **************************************************************
/*
void SendOnlyWavTrigger::setReporting(bool enable) {

uint8_t txbuf[6];

	txbuf[0] = SOM1;
	txbuf[1] = SOM2;
	txbuf[2] = 0x06;
	txbuf[3] = CMD_SET_REPORTING;
	txbuf[4] = enable;
	txbuf[5] = EOM;
	WTSerial.write(txbuf, 6);
}
*/

// **************************************************************
void SendOnlyWavTrigger::trackPlaySolo(int trk) {
  
	trackControl(trk, TRK_PLAY_SOLO);
}

// **************************************************************
void SendOnlyWavTrigger::trackPlaySolo(int trk, bool lock) {
  
	trackControl(trk, TRK_PLAY_SOLO, lock);
}

// **************************************************************
void SendOnlyWavTrigger::trackPlayPoly(int trk) {
  
	trackControl(trk, TRK_PLAY_POLY);
}

// **************************************************************
void SendOnlyWavTrigger::trackPlayPoly(int trk, bool lock) {
  
	trackControl(trk, TRK_PLAY_POLY, lock);
}

// **************************************************************
void SendOnlyWavTrigger::trackLoad(int trk) {
  
	trackControl(trk, TRK_LOAD);
}

// **************************************************************
void SendOnlyWavTrigger::trackLoad(int trk, bool lock) {
  
	trackControl(trk, TRK_LOAD, lock);
}

// **************************************************************
void SendOnlyWavTrigger::trackStop(int trk) {

	trackControl(trk, TRK_STOP);
}

// **************************************************************
void SendOnlyWavTrigger::trackPause(int trk) {

	trackControl(trk, TRK_PAUSE);
}

// **************************************************************
void SendOnlyWavTrigger::trackResume(int trk) {

	trackControl(trk, TRK_RESUME);
}

// **************************************************************
void SendOnlyWavTrigger::trackLoop(int trk, bool enable) {
 
	if (enable)
		trackControl(trk, TRK_LOOP_ON);
	else
		trackControl(trk, TRK_LOOP_OFF);
}

// **************************************************************
void SendOnlyWavTrigger::trackControl(int trk, int code) {
  
uint8_t txbuf[8];

	txbuf[0] = SOM1;
	txbuf[1] = SOM2;
	txbuf[2] = 0x08;
	txbuf[3] = CMD_TRACK_CONTROL;
	txbuf[4] = (uint8_t)code;
	txbuf[5] = (uint8_t)trk;
	txbuf[6] = (uint8_t)(trk >> 8);
	txbuf[7] = EOM;
	WTSerial.write(txbuf, 8);
}

// **************************************************************
void SendOnlyWavTrigger::trackControl(int trk, int code, bool lock) {
  
uint8_t txbuf[9];

	txbuf[0] = SOM1;
	txbuf[1] = SOM2;
	txbuf[2] = 0x09;
	txbuf[3] = CMD_TRACK_CONTROL_EX;
	txbuf[4] = (uint8_t)code;
	txbuf[5] = (uint8_t)trk;
	txbuf[6] = (uint8_t)(trk >> 8);
	txbuf[7] = lock;
	txbuf[8] = EOM;
	WTSerial.write(txbuf, 9);
}

// **************************************************************
void SendOnlyWavTrigger::stopAllTracks(void) {

uint8_t txbuf[5];

	txbuf[0] = SOM1;
	txbuf[1] = SOM2;
	txbuf[2] = 0x05;
	txbuf[3] = CMD_STOP_ALL;
	txbuf[4] = EOM;
	WTSerial.write(txbuf, 5);
}

// **************************************************************
void SendOnlyWavTrigger::resumeAllInSync(void) {

uint8_t txbuf[5];

	txbuf[0] = SOM1;
	txbuf[1] = SOM2;
	txbuf[2] = 0x05;
	txbuf[3] = CMD_RESUME_ALL_SYNC;
	txbuf[4] = EOM;
	WTSerial.write(txbuf, 5);
}

// **************************************************************
void SendOnlyWavTrigger::trackGain(int trk, int gain) {

uint8_t txbuf[9];
unsigned short vol;

	txbuf[0] = SOM1;
	txbuf[1] = SOM2;
	txbuf[2] = 0x09;
	txbuf[3] = CMD_TRACK_VOLUME;
	txbuf[4] = (uint8_t)trk;
	txbuf[5] = (uint8_t)(trk >> 8);
	vol = (unsigned short)gain;
	txbuf[6] = (uint8_t)vol;
	txbuf[7] = (uint8_t)(vol >> 8);
	txbuf[8] = EOM;
	WTSerial.write(txbuf, 9);
}

// **************************************************************
void SendOnlyWavTrigger::trackFade(int trk, int gain, int time, bool stopFlag) {

uint8_t txbuf[12];
unsigned short vol;

	txbuf[0] = SOM1;
	txbuf[1] = SOM2;
	txbuf[2] = 0x0c;
	txbuf[3] = CMD_TRACK_FADE;
	txbuf[4] = (uint8_t)trk;
	txbuf[5] = (uint8_t)(trk >> 8);
	vol = (unsigned short)gain;
	txbuf[6] = (uint8_t)vol;
	txbuf[7] = (uint8_t)(vol >> 8);
	txbuf[8] = (uint8_t)time;
	txbuf[9] = (uint8_t)(time >> 8);
	txbuf[10] = stopFlag;
	txbuf[11] = EOM;
	WTSerial.write(txbuf, 12);
}

// **************************************************************
/*
void SendOnlyWavTrigger::samplerateOffset(int offset) {

uint8_t txbuf[7];
unsigned short off;

	txbuf[0] = SOM1;
	txbuf[1] = SOM2;
	txbuf[2] = 0x07;
	txbuf[3] = CMD_SAMPLERATE_OFFSET;
	off = (unsigned short)offset;
	txbuf[4] = (uint8_t)off;
	txbuf[5] = (uint8_t)(off >> 8);
	txbuf[6] = EOM;
	WTSerial.write(txbuf, 7);
}
*/
