typedef struct
{
	unsigned char fifo[17];
	unsigned int dataReg;
	int fifoHead;
	int fifoTail;

	int autoRepeatOn;
	int keyClickOn;
	int keyboardEnabled;
	int ASCIImode;
	int interruptsEnabled;

	int capsLock;

} keyboard;

void keyboardCommandWrite(keyboard*,unsigned int command);

unsigned int keyboardStatusRead(keyboard*);

unsigned int keyboardDataRead(keyboard*);

void keyboardReset(keyboard*);

keyboard* newKeyboard();

void click();
void beep();


//from UI
void keyaction(keyboard* k,char code);
void keydown(char* name);
void keyup(char* name);
