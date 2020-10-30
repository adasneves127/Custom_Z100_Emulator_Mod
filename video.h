// The Z-100 screen resolution is 640 pixels wide by 225 pixels in height
#define VWIDTH 640
#define VHEIGHT 225

typedef struct
{
	unsigned int vram[0x10000*3];

	unsigned int registers[18];
	int registerPointer;

	int io,controlA,controlB,addressLatch;
	int redenabled, blueenabled, greenenabled, flashenabled;
	int redcopy,greencopy,bluecopy;
	int vramenabled;

//	e8259_t* e8259;
} Video;

Video* newVideo();

unsigned int* generateScreen();
void renderScreen(Video*,unsigned int* pixels);

unsigned int readVideo(Video*,unsigned int addr);
void writeVideo(Video*,unsigned int addr, unsigned int data);
//void attachInterrupt(Video*,e8259_t*);
