typedef struct
{
	unsigned int command;
	unsigned int status;
	unsigned int track;
	unsigned int trackRegister;
	unsigned int sector;
	unsigned int data;

	int commandDone;
	int stepDir;
	double indexTime;
	int index;
	int indexHoleDetect;	// when this is 1, index hole is being detected
	int indexHoleTime;	/* this will keep track of how long the index hole is
											 detected for. According to page D.203 of Z-100 technical
											 Manual - Appendices, Misc. Timings, the index pulse
											 width is at least 20 microseconds with a 1 MHz clock */

	int commandType;
	int commandName;

	int headLoad;
	int headLoadTiming;
	int recordType;
	int recordNotFound;
	int lostData;

	int interruptNRR;
	int interruptRNR;
	int interruptIndexPulse;
	int interruptImmediate;

	int statusPortInterrupt;
	int statusPortMotorOn;
	int statusPortTPI96;
	int statusPortTwoSided;
	int statusPortReady;

	int controlLatchValue;
	int driveSelect;
	int driveType;
	int selectDrives;
	int precomp;
	int override8;
	int waitState;
	int singleDensity;

	int rate;
	double us;

	int seekpos;
	int seekdest;
	int seekstart;
	int step_stepDir;
	int step_updateReg;
	int readaddress_step;
	int readaddress_steps[6];

	int updateSSO;
	int delay;
	int swapSectorLength;
	int multipleRecords;

/*
	int immediateInterrupt;
	int spinUpCounter;
	int spinDownCounter; 	//10 to 0
	int commandPhase;	//part of current command being processed

	int intrq;		//intr request
	int drq;		//data request
	int sden;		//single density (1) enable
	int precomp_enable;	//precomp enable
	int indexpulse;		//index pulse
	int track00;		//track 0
	int wp;			//write protected
	int step;		//step out(0) in(1)
	int dir;		//direction 1=step in
	int motoron;		//motor on

	int driveselect;
	int fiveinch;
	int selectdrive;
	int eightinchmode;
	int wait_enable;
	int ninetysixtpi;
	int twosided;

	double indexTime;
	int index;
	int commandDone;
*/

} WD1797;

WD1797* newWD1797();

void resetWD1797(WD1797*);
void writeWD1797(WD1797*,unsigned int addr,unsigned int value);
unsigned int readWD1797(WD1797*,unsigned int addr);
void doWD1797Cycle(WD1797*, double cycles);
void doWD1797Command(WD1797*);
