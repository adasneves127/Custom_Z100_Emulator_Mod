#include "wd1797.h"
#include "e8259.h"
#include <stdlib.h>
#include <stdio.h>

#define RPM 300
#define ONE_ROTATION_MICROSECS 1000000/(RPM/60)
#define TIME_BETWEEN_INDEX ONE_ROTATION_MICROSECS/9
#define INDEX_MARK_TIME 500

#define FLAG_R0 1
#define FLAG_A0 1
#define FLAG_R1 2
#define FLAG_U 2
#define FLAG_V 4
#define FLAG_E 4
#define FLAG_H 8
#define FLAG_L 8
#define FLAG_T 16
#define FLAG_M 16
#define INTERRUPT 8
#define HEAD_LOAD_TIMING 50

#define CMD_FORCE_INT_NR_R 1
#define CMD_FORCE_INT_R_NR 2
#define CMD_FORCE_INT_PULSE 4
#define CMD_FORCE_INT_IMM 8

#define STATUS_1_BUSY 0
#define STATUS_1_INDEX 1
#define STATUS_1_TRACK00 2
#define STATUS_1_CRC_ERR 3
#define STATUS_1_SEEK_ERR 4
#define STATUS_1_HEAD_LOAD 5
#define STATUS_1_PROTECTED 6
#define STATUS_1_NOT_READY 7

#define STATUS_2_BUSY 0
#define STATUS_2_DRQ 1
#define STATUS_2_LOST 2
#define STATUS_2_CRC_ERR 3
#define STATUS_2_RNF_ERR 4
#define STATUS_2_RCRD_TYPE 5
#define STATUS_2_WRI_FAULT 5
#define STATUS_2_PROTECTED 6
#define STATUS_2_NOT_READY 7

#define CTL_LAT_D_SEL_MASK 3
#define CTL_LAT_D_SEL_SHFT 0
#define CTL_LAT_D_TYP_MASK 1
#define CTL_LAT_D_TYP_SHFT 2
#define CTL_LAT_D_SEN_MASK 1
#define CTL_LAT_D_SEN_SHFT 3
#define CTL_LAT_PRECO_MASK 1
#define CTL_LAT_PRECO_SHFT 4
#define CTL_LAT_OVER8_MASK 1
#define CTL_LAT_OVER8_SHFT 5
#define CTL_LAT_WAITS_MASK 1
#define CTL_LAT_WAITS_SHFT 6
#define CTL_LAT_DOUBD_MASK 1
#define CTL_LAT_DOUBD_SHFT 7

#define STEPDIRECTIONIN 0
#define STEPDIRECTIONOUT 1

#define READSECTORCOMMAND 6
#define READADDRESSCOMMAND 5
#define INTERRUPTCOMMAND 4
#define RESTORECOMMAND 3
#define SEEKCOMMAND 2
#define STEPCOMMAND 1

#define SEEK_TIME 2

extern e8259_t* e8259_slave;

FILE* zdos;

WD1797* newWD1797()
{
	WD1797* w=(WD1797*)malloc(sizeof(WD1797));
	return w;
}

void resetWD1797(WD1797* w)
{
	w->commandDone=1;
	w->status=0;
	w->track=0;
	w->trackRegister=0;
	w->sector=1;
	w->stepDir=STEPDIRECTIONIN;
	w->indexTime=0;
	w->index=0;
	w->headLoad=0;
	w->headLoadTiming=0;
	w->recordNotFound=0;
	w->recordType=0;
	w->lostData=0;
	w->interruptNRR=0;
	w->interruptRNR=0;
	w->interruptIndexPulse=0;
	w->interruptImmediate=0;

	w->statusPortInterrupt=0;
	w->statusPortMotorOn=0;
	w->statusPortTPI96=0;
	w->statusPortTwoSided=0;
	w->statusPortReady=0;

	w->command=3;
	w->commandName=RESTORECOMMAND;
	w->commandType=1;

/*	w->command=3;
	w->sector=1;
	w->status=w->track=w->data=0;
	w->immediateInterrupt=0;
	w->intrq=0;
	w->drq=0;
	w->sden=0;
	w->precomp_enable=0;
	w->wp=0;
	w->step=0;
	w->dir=0;
	w->motoron=0;
	w->driveselect=0;
	w->fiveinch=1;
	w->selectdrive=0;
	w->eightinchmode=0;
	w->wait_enable=0;
	w->ninetysixtpi=1;
	w->twosided=1;

	w->indexTime=0;
	w->index=0;
	w->commandDone=1;*/

	zdos = fopen("z-dos-1.img","rb");
}

unsigned int readWD1797(WD1797* w,unsigned int addr)
{
	unsigned int r=0;
	switch(addr)
	{
		case 0xb0:
			r=w->status;
			printf("status: %x\n",r);
			break;
		case 0xb1:
			r=w->trackRegister;
			break;
		case 0xb2:
			r=w->sector;
			break;
		case 0xb3:
			w->lostData=0;
			w->statusPortReady=0;
			r=w->data;
			if(w->commandName==READSECTORCOMMAND) {
				r=fgetc(zdos)&0xff;
				printf("read sector %x %x %x\n",w->track,w->sector,r);
			}
			break;
		case 0xb4:
			r=w->controlLatchValue;
		case 0xb5:
			//status port read
			r|=(w->statusPortInterrupt&1);
			r|=(w->statusPortMotorOn&1)<<1;
			r|=(w->statusPortTPI96&1)<<3;
			r|=(w->statusPortTwoSided&1)<<6;
			r|=(w->statusPortReady&1)<<7;
			e8259_set_irq0(e8259_slave, 0);
			w->statusPortInterrupt=0;
			break;
	}
	return r;
}


void writeWD1797(WD1797* w,unsigned int addr,unsigned int value)
{
	printf("Write to wd1797: %x %x\n",addr,value);
	switch(addr)
	{
		case 0xb0:
			//command
			w->command=value;
			w->statusPortInterrupt=0;
			doWD1797Command(w);
			break;
		case 0xb1:
			w->trackRegister=value;	break;
		case 0xb2:
			w->sector=value;	break;
		case 0xb3:
			w->data=value;	break;
		case 0xb4:
			//control latch write
			w->controlLatchValue=value;
			w->driveSelect=value&3;
			w->driveType=(value>>2)&1;
			w->selectDrives=(value>>3)&1;
			w->precomp=(value>>4)&1;
			w->override8=(value>>5)&1;
			w->waitState=(value>>6)&1;
			w->singleDensity=(value>>7)&1;
			// print contol latch values
			// printf("wd1797 control latch value: %d\n"
			// 	"drive select: %d\n"
			// 	"drive type: %d\n"
			// 	"select drives: %d\n"
			// 	"precomp: %d\n"
			// 	"override 8: %d\n"
			// 	"wait state: %d\n"
			// 	"single density: %d\n",
			// 	w->controlLatchValue,
			//  	w->driveSelect,
			// 	w->driveType,
			// 	w->selectDrives,
			// 	w->precomp,
			// 	w->override8,
			// 	w->waitState,
			// 	w->singleDensity);
			break;
	}
}


int commandStep(WD1797* w, double us)
{
	//restore
	if(w->commandName==RESTORECOMMAND)
	{
		// printf("doing RESTORE command step...\n");
		w->us+=us;
		if(w->track!=0)
		{
			if(w->us>=w->rate)
			{
				w->us-=w->rate;
				w->track--;
			}
		}
		if(w->track==0)
		{
			w->trackRegister=0;
			e8259_set_irq0 (e8259_slave, 1);
			return 1;
		}
		return 0;
	}
	else if(w->commandName==SEEKCOMMAND)
	{
		w->us+=us;

		if(w->us>=SEEK_TIME)
		{
			if(w->seekdest>w->seekpos)
			{
				w->seekpos++;
				w->track++;
			}
			else if (w->seekdest<w->seekpos)
			{
				w->seekpos--;
				w->track--;
			}
			w->trackRegister=w->seekpos;
			w->us-=SEEK_TIME;
		}
		if(w->trackRegister==w->data)
		{
			e8259_set_irq0 (e8259_slave, 1);
			return 1;
		}
		return 0;
	}
	else if (w->commandName==STEPCOMMAND)
	{
		w->us+=us;
		if(w->us>=w->rate)
		{
			w->us-=w->rate;
			if(w->step_stepDir==STEPDIRECTIONIN)
			{
				w->track++;
			}
			else
			{
				w->track--;
			}
			if(w->step_updateReg)
			{
				w->trackRegister=w->track;
			}
			return 1;
		}
		return 0;
	}
	else if (w->commandName==INTERRUPTCOMMAND)
	{
		return 1;
	}
	else if (w->commandName==READADDRESSCOMMAND)
	{
		w->us+=us;
		if(!w->statusPortReady)
		{
			if(w->readaddress_step==0)
				w->sector=w->readaddress_steps[0];
			w->data=w->readaddress_steps[w->readaddress_step];
			w->statusPortReady=1;
			w->readaddress_step++;
		}
		if(w->readaddress_step==6)
		{
			e8259_set_irq0 (e8259_slave, 1);
			return 1;
		}
		return 0;
	}
	else if (w->commandName==READSECTORCOMMAND)
	{
		w->us+=us;
		// %%%%%% WHY RETURN 0 HERE???? &&& instead of 1 like before&&&&&
		return 0;
/*
	   var numSectors = _w.Disk.GetNumSectors(_updateSSO ? 1 : 0, _w.Track);

	    if (_w.Sector > numSectors)
	    {
	        _w.RecordNotFound = true;
	        _w.Interrupt();
	        return true;
	    }
	    var cylinder = _w.Track;
	    var head = _updateSSO ? 1 : 0;
	    var sector = _w.Sector;

	    if (!_w.StatusPort.Ready)
	    {
	        _w.Data = _w.Disk.Get(cylinder, head, sector, _sectorIdx);
	        _w.StatusPort.Ready = true;
	        _sectorIdx++;
	    }

	    if (_sectorIdx == _w.Disk.GetSectorSize(head, cylinder).Size)
	    {
	        if (_multipleRecords && _w.Sector <= numSectors)
	        {
	            _w.Sector++;
	            _sectorIdx = 0;
	        }
	        else
	        {
	            if (_w.Disk.GetDeleted(cylinder, head, _w.Sector))
	                _w.RecordType = true;

	            _w.Interrupt();
	            return true;
	        }
	    }
	    return false;
*/
		return 1;
	}
	return 1;
}

void doWD1797Cycle(WD1797* w,double us)
{

	// ??
	// w->us++;

	w->indexTime+=us;
	if(w->index && w->indexTime>=INDEX_MARK_TIME)
	{
		w->index=0;
		w->indexTime=0;
	}
	else if (!w->index && w->indexTime >= TIME_BETWEEN_INDEX)
	{
		w->index=1;
		w->indexTime=0;
	}
	if(!w->commandDone)
		// printf("COMMAND is not done...do command step\n");
		w->commandDone=commandStep(w,us);

	//restore, step, interrupt
	if(w->commandType==1 || w->commandType==4)
	{
		// printf("setting status Register...\n");
		w->status =
			w->commandDone==1? 0:1 |
			w->index? 2:0 |
			w->track==0? 4:0 |
			//crc error 8
			//seek error 16
			w->headLoad? 32:0 ;
			//write protect 64:0 |
			//not ready 128:0;
	}
	//seek,read sector, read address
	else if (w->commandType==2 || w->commandType==3)
	{
		w->status =
			w->commandDone==1? 0:1 |
			w->statusPortReady? 2:0 |
			w->lostData? 4:0 |
			//crc error 8
			w->recordNotFound? 16:0 |
			w->recordType? 32:0 ;
			//write protect 64
			//not ready 128

	}
}

void computeCRC(int initialValue, int* bytes, int len, int* result)
{
	unsigned short initial=(unsigned short)initialValue;

	unsigned short temp,a;
	unsigned short table[256];
	unsigned short poly=4129;

	for(int i=0; i<256; i++)
	{
		temp=0;
		a=(unsigned short)(i<<8);
		for(int j=0; j<8; j++)
		{
			if (((temp^a)&0x8000)!=0)
				temp=(unsigned short)((temp<<1)^poly);
			else
				temp<<=1;
		}
		table[i]=temp;
	}

	unsigned short crc=initial;
	for(int i=0; i<len; i++)
	{
		crc = (unsigned short)((crc<<8)^table[((crc>>8)^(0xff & bytes[i]))]);
	}
	result[0]=crc & 0xff;
	result[1]=(crc>>8)&0xff;
}


void doWD1797Command(WD1797* w)
{
	w->commandDone=0;

	// int rates[]={3,6,10,15};
	/* this change according to page D.191 of the Z-100 Technical - Table 3
	 	STEPPING RATES. The ROM Listing version 2.5 states that T1F_5SR Flag,
		which is 0b0000 0011, corresponds to 30 msec/track. Table 3 lists this
		setting as being assciated with a 1 MHz clock */
	int rates[]={6,12,20,30};

	int rate = rates[w->command & 3];
	int hld=(w->command&8)==8;
	int verify=(w->command&4)==4;
	int updateReg=(w->command&16)==16;

	// printf("command: %d\n"
	// 	"rate: %d\n"
	// 	"head load: %d\n"
	// 	"verify: %d\n"
	// 	"updateReg: %d\n\n",
	// 	w->command,
	// 	rate,
	// 	hld,
	// 	verify,
	// 	updateReg);


	// printf("command: %x\n",w->command);

	//restore
	if((w->command >> 4) == 0)
	{
		printf("RESTORE COMMAND executing...\n");
		w->rate=rate;
		if(!hld && !verify)
			w->headLoad=0;
		else if (hld&&!verify)
			w->headLoad=1;
		else if (hld&&verify)
			w->headLoad=1;

		w->commandName=RESTORECOMMAND;
		w->commandType=1;
		w->us=0;

		// according to page D.191
		// printf("setting slave IRQ0\n");
		// e8259_set_irq0 (e8259_slave, 1);
	}
	//seek
	else if ((w->command>>4)==1)
	{
		w->rate=rate;
		if(!hld && !verify)
			w->headLoad=0;
		else if (hld&&!verify)
			w->headLoad=1;
		else if (hld&&verify)
			w->headLoad=1;

		w->seekdest=w->data;
		w->seekstart=w->trackRegister;
		w->seekpos=w->trackRegister;

		w->commandName=SEEKCOMMAND;
		w->commandType=1;
		w->us=0;
	}
	//step
	else if ((w->command>>5)==1)
	{
		w->rate=rate;
		if(!hld && !verify)
			w->headLoad=0;
		else if (hld&&!verify)
			w->headLoad=1;
		else if (hld&&verify)
			w->headLoad=1;
		w->step_stepDir=w->stepDir;
		w->step_updateReg=updateReg;

		w->commandName=STEPCOMMAND;
		w->commandType=1;
		w->us=0;
	}
	//step
	else if ((w->command>>5)==2)
	{
		w->rate=rate;
		if(!hld && !verify)
			w->headLoad=0;
		else if (hld&&!verify)
			w->headLoad=1;
		else if (hld&&verify)
			w->headLoad=1;
		w->step_stepDir=STEPDIRECTIONIN;
		w->step_updateReg=updateReg;

		w->commandName=STEPCOMMAND;
		w->commandType=1;
		w->us=0;
	}
	//step
	else if ((w->command>>5)==3)
	{
		w->rate=rate;
		if(!hld && !verify)
			w->headLoad=0;
		else if (hld&&!verify)
			w->headLoad=1;
		else if (hld&&verify)
			w->headLoad=1;
		w->step_stepDir=STEPDIRECTIONOUT;
		w->step_updateReg=updateReg;

		w->commandName=STEPCOMMAND;
		w->commandType=1;
		w->us=0;
	}
	//read sector
	else if ((w->command>>5)==4)
	{
		w->commandName=READSECTORCOMMAND;
		w->commandType=2;
		w->us=0;
		w->updateSSO=(w->command&2)==2;
		w->delay=(w->command&4)==4;
		w->swapSectorLength=(w->command&8)==8;
		w->multipleRecords=(w->command&16)==16;

		w->headLoad=1;
		w->recordNotFound=0;
		w->recordType=0;
	}
	//read address
	else if ((w->command>>3)==24)
	{
		w->commandName=READADDRESSCOMMAND;
		w->commandType=3;
		w->us=0;
		w->readaddress_step=0;
		w->headLoad=1;
		w->recordNotFound=0;
		w->readaddress_steps[0]=w->track;
		w->readaddress_steps[1]=(w->command&2)==2? 1 : 0;
		w->readaddress_steps[2]=w->sector;
		w->readaddress_steps[3]=2;

		int crc[2];
		computeCRC(0xffff, w->readaddress_steps, 4, crc);

		w->readaddress_steps[4]=crc[0];
		w->readaddress_steps[5]=crc[1];
	}
	//interrupt
	// else if ((w->command>>3)==13)
	else if ((w->command>>4)==13)
	{
		printf("force interrupt\n");
		w->interruptNRR = (w->command&1)==1;
		w->interruptRNR = (w->command&2)==2;
		w->interruptIndexPulse = (w->command&4)==4;
		w->interruptImmediate = (w->command&8)==8;

		w->commandName=INTERRUPTCOMMAND;
		w->commandType=4;

		if(w->interruptImmediate)
		{
			e8259_set_irq0 (e8259_slave, 1);
		}
	}
}
