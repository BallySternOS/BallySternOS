 
#define CMD_GET_VERSION					1
#define CMD_GET_SYS_INFO				2
#define CMD_TRACK_CONTROL				3
#define CMD_STOP_ALL					4
#define CMD_MASTER_VOLUME				5
#define CMD_TRACK_VOLUME				8
#define CMD_AMP_POWER					9
#define CMD_TRACK_FADE					10
#define CMD_RESUME_ALL_SYNC				11
#define CMD_SAMPLERATE_OFFSET			12
#define	CMD_TRACK_CONTROL_EX			13
#define	CMD_SET_REPORTING				14
#define CMD_SET_TRIGGER_BANK			15

#define TRK_PLAY_SOLO					0
#define TRK_PLAY_POLY					1
#define TRK_PAUSE						2
#define TRK_RESUME						3
#define TRK_STOP						4
#define TRK_LOOP_ON						5
#define TRK_LOOP_OFF					6
#define TRK_LOAD						7

#define	RSP_VERSION_STRING				129
#define	RSP_SYSTEM_INFO					130
#define	RSP_STATUS						131
#define	RSP_TRACK_REPORT				132

#define MAX_MESSAGE_LEN					32
#define MAX_NUM_VOICES					14
#define VERSION_STRING_LEN				21

#define SOM1	0xf0
#define SOM2	0xaa
#define EOM		0x55


#include <HardwareSerial.h>
#define WTSerial Serial

class SendOnlyWavTrigger
{
public:
	SendOnlyWavTrigger() {;}
	~SendOnlyWavTrigger() {;}
	void start(void);
//	void update(void);
//	void flush(void);
//	void setReporting(bool enable);
//	void setAmpPwr(bool enable);
//	bool getVersion(char *pDst, int len);
//	int getNumTracks(void);
//	bool isTrackPlaying(int trk);
//	void masterGain(int gain);
	void stopAllTracks(void);
	void resumeAllInSync(void);
	void trackPlaySolo(int trk);
	void trackPlaySolo(int trk, bool lock);
	void trackPlayPoly(int trk);
	void trackPlayPoly(int trk, bool lock);
	void trackLoad(int trk);
	void trackLoad(int trk, bool lock);
	void trackStop(int trk);
	void trackPause(int trk);
	void trackResume(int trk);
	void trackLoop(int trk, bool enable);
	void trackGain(int trk, int gain);
	void trackFade(int trk, int gain, int time, bool stopFlag);
	//void samplerateOffset(int offset);

private:
	void trackControl(int trk, int code);
	void trackControl(int trk, int code, bool lock);
	char version[VERSION_STRING_LEN];
	uint16_t numTracks;
	uint8_t numVoices;
	uint8_t rxLen;
};
