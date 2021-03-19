// *This file implements the Z100 Main board.
// Hardware and glue logic existant on the main board is implemented here

/* mainBoard.c */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>
#include <gtk/gtk.h>
#include "debug_gui.h"
#include "8085.h"
#include "8088.h"
#include "e8253.h"
#include "e8259.h"
#include "jwd1797.h"
#include "mainBoard.h"
#include "keyboard.h"
#include "video.h"
#include "screen.h"
#include "utility_functions.h"

#define ROM_SIZE 0x4000
#define RAM_SIZE 0x30000
// (116736 cycles = 0.0233472 seconds)
// #define VSYNC_CYCLE_LIMIT 116736
/* (20 cycles = 4 microseconds, EDIT - 8253 timer is connected to a 250kHz
	clock according to page 2.80 Z-100 Technical manual (hardware) - clocked
	every 120 cycles */
#define E8253_TIMER_CYCLE_LIMIT 120
// (5 CPU cycles = 1 microsecond)
// #define JWD1797_CYCLE_LIMIT 5

enum active_processor{pr8085, pr8088};
enum active_processor active_processor;

// for determing average cycles per instruction
double total_cycles;
// rough clock to keep track of time passing (us) as instructions are executed
double total_time_elapsed;

unsigned long instructions_done;
// unsigned long cycles_done;
unsigned long breakAtInstruction;
// unsigned int VSYNC_cycle_count;
unsigned int e8253_timer_cycle_count;
// unsigned int jwd1797_cycle_count;
int last_instruction_cycles;
double last_instruction_time_us;

// debug window thread
// pthread_t dbug_window_thread;

pthread_t emulator_thread;	// this thread object runs the main emulator thread
int romOption;  // temp holder of bits 3 and 2 of memeory control latch port FC
int killParity;
int zeroParity;
int byteParity;
bool debug_mode;
bool reset_irq6;
unsigned char user_debug_mode_choice;
unsigned char switch_s101_FF;
unsigned char processor_swap_port_FE;
unsigned char memory_control_latch_FC;
unsigned char io_diag_port_F6; // not needed without external diag device

unsigned char rom[ROM_SIZE];
unsigned char ram[RAM_SIZE];
P8085 p8085;
P8088* p8088;
keyboard* keybrd;
Video* video;
unsigned int* pixels;	// holds the state of each pixel on the screen
e8253_t* e8253;
e8259_t* e8259_master;
e8259_t* e8259_slave;
JWD1797* jwd1797;

int z100_main();
void interruptFunctionCall(void*, int);
void cascadeInterruptFunctionCall(void*, int);
int getParity(unsigned int);
void print_bin8_representation(unsigned char);
unsigned int z100_memory_read_(unsigned int);
void z100_memory_write_(unsigned int, unsigned char);
void initialize_z100_ports();
void printByteArray(unsigned char*, int);
void printInt(int);
void loadrom(char*);
void timer_out_0();
void timer_ext_0();
void timer_out_1();
void timer_ext_1();
void timer_out_2();
void timer_ext_2();
// static gboolean update_debug_gui_values();

// ============== MAIN Z100 FUNCTION ==============
int z100_main() {

  // *** set up all hardware conpoenets on the main board ***

  // initialize keyboard - create a keyboard object
	keybrd = newKeyboard();
  // object that handles CRT Controller 68A45 and video parallel port
	video = newVideo();
  // this function call sets all ports to 0x00000000
  initialize_z100_ports();

  // load Z100 monitor ROM from bin file
  loadrom("zrom_444_276_1.bin");

  /* temp set ROM romOption to 0 - make the ROM appear to be repeated throughout
		memory */
  romOption = 0;

  /* setting killParity to 0 here actually enables the parity checking circuitry
 		setting this to 1 is saying "yes, kill (disable) the parity checking circuitry"
  	in the actual hardware, bit 5 of the memory control port would be set to zero
  	to disable parity ckecking */
  killParity = 0;
  /* setting this zeroParity variable to zero forces the zero parity like in the
  	hardware */
  zeroParity = 0;
  // this holds the result of calculating a byte parity
  byteParity = 0;

  // reset 8085
	reset8085(&p8085);
  printf("8085 reset\n");

	// new p8088 processor
  p8088 = new8088();
  assignCallbacks8088(p8088, z100_memory_read_, z100_memory_write_,
		z100_port_read, z100_port_write);
	reset8088(p8088);
  printf("8088 reset\n");

  // create two interrupt controller objects "master" and "slave"
	// need slave to 1. pass the self-test and 2. access JWD1797 floppy controller
	e8259_master=e8259_new("MASTER");
	e8259_slave=e8259_new("SLAVE_");
	e8259_reset(e8259_master);
	e8259_reset(e8259_slave);
  /* setup master int controller to cause 8088 interrupt on trap pin (connect interrupt
  controller to 8088 processor) */
  e8259_set_int_fct(e8259_master, NULL, interruptFunctionCall);
	/* setup slave int controller to cause IR3 pin on master to go high */
  e8259_set_int_fct(e8259_slave, NULL, cascadeInterruptFunctionCall);

  // make a timer object
  e8253 = e8253_new();
  e8253_init(e8253);
  e8253_set_gate(e8253, 0, 1);
  e8253_set_gate(e8253, 1, 1);
  e8253_set_gate(e8253, 2, 1);
	// set OUT 0 and OUT 2 functions to triggar IRQ2 on the master PIC
  e8253_set_out_fct(e8253, 0, e8259_master, e8259_set_irq2);
  e8253_set_out_fct(e8253, 2, e8259_master, e8259_set_irq2);
  e8253_set_out_fct(e8253, 1, timer_ext_1, timer_out_1);

  // set up floppy controller object
  jwd1797 = newJWD1797();
  resetJWD1797(jwd1797);

  // this creates a thread running from the debug window function
  // pthread_create(&dbug_window_thread, NULL, run_debug_window, NULL);
  // run_debug_window();
  // update values displayed in the debug gui
  // update_debug_gui_values();
  // g_main_context_invoke (NULL, update_debug_gui_values, NULL);

  // set processor selection to 8085
  active_processor = pr8085;
  // initialize all counts to zero
  instructions_done = 0;
  // cycles_done = 0;
  // VSYNC_cycle_count = 0;
  e8253_timer_cycle_count = 0;
	total_cycles = 0.0;
	total_time_elapsed = 0.0;
	last_instruction_cycles = 0;
	last_instruction_time_us = 0.0;
	// reset_irq6 = false;

  // ==== RUN THE PROCESSORS ====
  printf("Start running processors..\n");
  // --- run the processor(s) forever ---
  while(1) {

    // if active processor is the 8085
    if(active_processor == pr8085) {
      doInstruction8085(&p8085);
			last_instruction_cycles = p8085.cycles;
      instructions_done++;
			if(debug_mode && instructions_done >= breakAtInstruction) {
				printf("instructions done: %ld\n", instructions_done);
	      printf("PC = %X, opcode = %X, inst = %s\n",p8085.PC,p8085.opcode,p8085.name);
	      // int A, B, C, D, E, H, L, SP, PC;
	      printf("A = %X, B = %X, C = %X, D = %X, E = %X, H = %X, L = %X, SP = %X\n",
	        p8085.A, p8085.B, p8085.C, p8085.D, p8085.E, p8085.H, p8085.L, p8085.SP);
	      // int c, p, ac, z, s, i, m75, m65, m55;
	      printf("carry = %X, parity = %X, aux_carry = %X, zero = %X, sign = %X\n",
	        p8085.c, p8085.p, p8085.ac, p8085.z, p8085.s);
	      printf("i = %X, m75 = %X, m65 = %X, m55 = %X\n",
	        p8085.i, p8085.m75, p8085.m65, p8085.m55);
			}
    }
    // if the active processor is the 8088
    else if(active_processor == pr8088) {
      doInstruction8088(p8088);
			last_instruction_cycles = p8088->cycles;
			// add the number of cycles the last instruction took to the total cycles
			total_cycles += p8088->cycles;
      instructions_done++;
      /* increment cycles_done to add the number of cycles the current
      instruction took */
      // cycles_done = cycles_done + p8088->cycles;
      // only print processor status if debug conditions are met
      if(debug_mode && instructions_done >= breakAtInstruction) {
        printf("instructions done: %ld\n", instructions_done);
        printf("IP = %X, opcode = %X, inst = %s\n",
          p8088->IP,p8088->opcode,p8088->name_opcode);
        printf("value1 = %X, value2 = %X, result = %X cycles = %d\n",
          p8088->operand1,p8088->operand2,p8088->op_result,p8088->cycles);
        // unsigned int AL, AH, BL, BH, CL, CH, DL, DH, SP, BP, DI, SI, IP;
      	// unsigned int CS, SS, DS, ES;
        printf("AL = %X, AH = %X, BL = %X, BH = %X, CL = %X, CH = %X, DL = %X, DH = %X\n"
          "SP = %X, BP = %X, DI = %X, SI = %X\n"
          "CS = %X, SS = %X, DS = %X, ES = %X\n",
          p8088->AL, p8088->AH,p8088->BL,p8088->BH,p8088->CL,p8088->CH,p8088->DL,
          p8088->DH,p8088->SP,p8088->BP,p8088->DI,p8088->SI,p8088->CS,p8088->SS,
          p8088->DS,p8088->ES);
        // unsigned int c, p, ac, z, s, t, i, d, o;
        printf("carry = %X, parity = %X, aux_carry = %X, zero = %X, sign = %X\n",
          p8088->c,p8088->p,p8088->ac,p8088->z,p8088->s);
        printf("trap = %X, int = %X, dir = %X, overflow = %X\n",
          p8088->t,p8088->i,p8088->d,p8088->o);
      }
		}
    // printf("\n");

    /* THIS IS A BETTER WAY TO TIME THE INTERRUPT
    // update VSYNC and timer counts with 8088 cycles
    VSYNC_cycle_count = VSYNC_cycle_count + p8088->cycles;
    e8253_timer_cycle_count = e8253_timer_cycle_count + p8088->cycles;

    // simulate interrupts
    // simulate VSYNC interrupt on I6 (keyboard video display and light pen int)
    if(reset_irq6 == true) {
      // set the irq6 pin to low
      e8259_set_irq6(e8259_master, 0);
      printf("%s\n", "VSYNC interrupt reset - irq6 LOW");
      // clear VSYNC reset irq6 trigger
      reset_irq6 = false;
    }
    if(VSYNC_cycle_count >= VSYNC_CYCLE_LIMIT) {
      // set the irq6 pin on the master 8259 int controller to high
      e8259_set_irq6(e8259_master, 1);
      printf("%s\n", "VSYNC interrupt - irq6 HIGH");
      // reset VSYNC cycle count
      VSYNC_cycle_count = 0;
      // set irq6 reset trigger
      reset_irq6 = true;
    }
    // clock the 8253 timer - this should be ~ every 4 microseconds
    if(e8253_timer_cycle_count >= E8253_TIMER_CYCLE_LIMIT) {
      e8253_clock(e8253, 1);
      // reset timer cycle count
      e8253_timer_cycle_count = 0;
    }
    */

    /* THIS IS THE LESS BETTER WAY TO TIME THE INTERRUPTS */
    // simulate VSYNC interrupt on I6 (keyboard video display and light pen int)
    if(instructions_done%10000 == 10000-2) {
			// printf("%s\n", "*** KEYINT or DSPYINT/VSYNC interrupt occurred - I6 from Master PIC ***");
      // set the irq6 pin on the master 8259 int controller to high
      e8259_set_irq6(e8259_master, 1);
    }
    else if(instructions_done%10000 == 10000-1) {
      // set the irq6 pin to low
      e8259_set_irq6(e8259_master, 0);
    }

		// *** ON AVERAGE 2.284 microseconds pass with each instruction executed

    /* clock the 8253 timer - this should be ~ every 4 microseconds
     but here it happens on every instruction - although the number of cycles
     varies with each type of instruction  - EDIT: change clock cycle of timer
		 to clock every 20 cycles ~ (4us at 5MHz)
		 EDIT 2 - according to page 2.80 of Z-100 technical manual, the timer is
		 clocked by a 250kHz clock. */
		// update timer cycle count
		e8253_timer_cycle_count += last_instruction_cycles;
		if(e8253_timer_cycle_count >= E8253_TIMER_CYCLE_LIMIT) {
			e8253_clock(e8253, 1);
			e8253_timer_cycle_count = 0;
		}


		/* cycle the JWD1797. The JWD1797 is driven by a 1 MHz clock in the Z-100.
			Thus, it should be cycled every 1 microsecond or every five CPU (5 MHz)
			cycles. Considering it will be cycled after every instruction, the time
			added to the internal JWD1797 timer mechanisms will be determined
			based on how many cycles the previous instruction took.
			*/
		// calculate time passed with the last instruction execution
		// number of cycles for last instruction * 0.2 microseconds (5 MHz clock)
		last_instruction_time_us = last_instruction_cycles * 0.2;
		// increment the time total_time_elapsed based on the last instruction time
		total_time_elapsed += last_instruction_time_us;
		// printf("%s%f\n", "last instrcution time (microseconds): ", last_instruction_time_us);
		doJWD1797Cycle(jwd1797, last_instruction_time_us);

		// DEBUG
		// if(debug_mode && instructions_done >= breakAtInstruction) {
			// print index info
			// printf("\nJWD1797 commandType: %d\n"
			// 	"us: %lf\n"
			// 	"indexTime: %lf\n"
			// 	"index: %d\n",
			// 	jwd1797->commandType,
			// 	jwd1797->us,
			// 	jwd1797->indexTime,
			// 	jwd1797->index);
			// // print JWD1797 status register
			// print_bin8_representation(jwd1797->status);
    // }


    // update the screen every 100,000 instructions
    if(instructions_done%100000 == 0) {
      /* update pixel array using current VRAM state using renderScreen()
        function from video.c */
      renderScreen(video, pixels);
      // draw pixels to the GTK window using display() function from screen.c
      display();
    }

    // *** DEBUG - BOOT Command ***
    /* **SCRIPT a key press 'b' at the hand prompt to trigger the
    BOOT command this will happen very shortly after the point at which the hand
    prompt is waiting for a command entry from the user.
    ** 1093881 is the number of instructions is takes to reach the hand prompt
    and check the keyboard status one time */
    if(debug_mode && instructions_done == (1093881 + 5)) {
      keyaction(keybrd, 'b');
      // keyaction(keybrd, '\n');
    }
		// *** DEBUG - BOOT Command ***
		/* **SCRIPT a key press 'ENTER' at the hand prompt to trigger the
		BOOT command. This will happen after the "Boot" string has been written
		to the screen and the ROM prompt is waiting for either BOOT command
		parameters or an 'ENTER' keypress to initiate the default disk controller
		to boot the operating system from a disk.
		** Instruction 1098855 is a point at which the keyboard status loop has
		been reached and has been running for a significant amount of
		iterations */
		if(debug_mode && instructions_done == (1098855)) {
      keyaction(keybrd, 0x0D);
    }

    // update_debug_gui_values();

    /* if debug mode is active, wait for enter key to continue after each
    instruction */
    // if(debug_mode && ((instructions_done >= breakAtInstruction) || p8088->IP == 0x2FFC)) {
		if(debug_mode && (instructions_done >= breakAtInstruction)) {
      // printf("\n");
			printf("instructions done: %ld\n", instructions_done);
			printf("Average cycles/instruction: %f\n", (total_cycles/instructions_done));
			printf("TIME ELAPSED: %f\n", total_time_elapsed);
			printf("%s%lu\n", "JWD1797 ROTATIONAL BYTE POINTER: ",
				jwd1797->rotational_byte_pointer);
			printf("%s%02X\n", "Current Byte: ", getFDiskByte(jwd1797));
			printf("%s%f\n", "E Delay Timer: ", jwd1797->e_delay_timer);
			printByteArray(jwd1797->id_field_data, 6);
      getchar();
    }

		/* Use this code block to break when the instruction pointer is at a
		certain value */
		// if(p8088->IP == 0x3048) {
		// 	printf("instructions done: %ld\n", instructions_done);
		// 	printf("Average cycles/instruction: %f\n", (total_cycles/instructions_done));
		// 	printf("TIME ELAPSED: %f\n", total_time_elapsed);
		// 	printf("%s%lu\n", "JWD1797 ROTATIONAL BYTE POINTER: ",
		// 		jwd1797->rotational_byte_pointer);
		// 	printByteArray(jwd1797->id_field_data, 6);
    //   getchar();
    // }
  }

  // return from MAIN
  return 0;
}

//=============================================================================
//                         ** function definitions **
//=============================================================================
void interruptFunctionCall(void* v, int number) {
  if(number == 0) {
    return;
  }
  int irq = e8259_inta(e8259_master, e8259_slave);
  trap(p8088, irq);
}

void cascadeInterruptFunctionCall(void* v, int number) {
  if(number == 0) {
    return;
  }
	// printf("SLAVE 8259 PIC INT SIGNAL to MASTER 8259 PIC IRQ3\n");
  e8259_set_irq3(e8259_master, 1);
}

int getParity(unsigned int data) {
	return ((data&1)!=0) ^ ((data&2)!=0) ^ ((data&4)!=0) ^ ((data&8)!=0) ^
  ((data&16)!=0) ^ ((data&32)!=0) ^ ((data&64)!=0) ^ ((data&128)!=0) ^
  ((data&256)!=0) ^ ((data&512)!=0) ^ ((data&1024)!=0) ^ ((data&2048)!=0) ^
  ((data&4096)!=0) ^ ((data&8192)!=0) ^ ((data&16384)!=0) ^ ((data&32768)!=0);
}

unsigned int z100_memory_read_(unsigned int addr) {
  unsigned int return_value = 0x00;
  // based on memory mode adjust memory read location
  switch(romOption) {
		case 0:
      /* this is the ROM memory configuration that makes the code in ROM appear
      to be in all of memory, therefore any address is moded by 0x4000
      (size of ROM) */
      // printf("read actual ROM memory address %X\n", addr % 0x4000);
      return_value = rom[addr&(ROM_SIZE-1)]&0xff;
			break;
		case 1:
      // the ROM appears at the top of every 64K page of memory
			if((addr&0xffff) > 65536-ROM_SIZE)
				return_value = rom[addr&(ROM_SIZE-1)]&0xff;
			else if (addr < RAM_SIZE)
				return_value = ram[addr]&0xff;
			else if (addr >= 0xc0000 && addr <=0xeffff)
				return_value = video->vram[addr-0xc0000]&0xff;
			else
				return_value = 0xff;
			break;
		case 2:
      /* this is the ROM memory configuration that makes the code in ROM appear
      to be at the top of the first megabyte of memory. */
      if(addr < RAM_SIZE)
				return_value = ram[addr]&0xff;
			else if (addr >= 0xf8000)
				return_value = rom[addr&(ROM_SIZE-1)]&0xff;
			else if (addr >= 0xc0000 && addr <= 0xeffff) {
        // DEBUG message
        if(debug_mode && instructions_done >= breakAtInstruction) {
          printf("reading from video memory...\n");
        }
				return_value = video->vram[addr-0xc0000]&0xff;
      }
			else
				return_value = 0xff;
			break;
		case 3:
      // ROM is disabled - just read from RAM
			if(addr < RAM_SIZE)
				return_value = ram[addr]&0xff;
			else if (addr >= 0xc0000 && addr <= 0xeffff)
				return_value = video->vram[addr-0xc0000]&0xff;
			else
				return_value = 0xff;
			break;
		}

  // calculate parity of byte read (return_value) *** SHOULD THIS BE WHEN A BYTE IS WRITTEN?
  byteParity = getParity(return_value);
  // if we force a zero parity, enabled parity errors (by setting kill parity to false - 0),
  // and get an odd parity from byte read from memory location
  // -> throw parity interrupt by writing a 1 to line 0 of IRQ input.
  if(zeroParity==1 && killParity==0 && byteParity==1) {
    // DEBUG message
    if(debug_mode && instructions_done >= breakAtInstruction) {
      printf("THROW PARITY INTERRUPT\n");
    }
    // write 1 to IRQ line 0 (pin 0)
		e8259_set_irq0(e8259_master, 1);
	}

  // DEBUG message
  if(debug_mode && instructions_done >= breakAtInstruction) {
    printf("read %X from memory address %X\n", return_value, addr);
  }
  return return_value;
}

void z100_memory_write_(unsigned int addr, unsigned char data) {
  // DEBUG message
  if(debug_mode && instructions_done >= breakAtInstruction) {
    printf("write %X to memory address %X\n", data, addr);
  }
  if(addr < RAM_SIZE) {
    // write data to RAM at address
    ram[addr] = data&0xff;
  }
	else if(addr >= 0xc0000 && addr <= 0xeffff) {
    // write data to video memeory portion of RAM at address
    // DEBUG message
    if(debug_mode && instructions_done >= breakAtInstruction) {
      printf("Writing to video memory...\n");
    }
    // video object has its own virtual RAM to hold video data
    video->vram[addr - 0xc0000] = data&0xff;
  }
}

unsigned int z100_port_read(unsigned int address) {
  unsigned char return_value;
  // the appropriate port reads will happen here
  // printf("Value read from port at address %X\n", address);
  // read port based on incoming port address
  switch(address) {
    // Z-217 Secondary Winchester Drive Controller Status Port (0xAA)
    case 0xAA:
      // printf("reading from Z-217 Secondary Winchester Drive Controller Status Port %X [NO HARDWARE IMPLEMENTATION]\n", address);
      /* this returned status value is hardcoded to indicate the absence of the
      Winchester Drive Controller S-100 card */
      return_value = 0xfe;
      break;
    // Z-217 Secondary Winchester Drive Controller Command Port (0xAB)
    case 0xAB:
      // printf("reading from Z-217 Secondary Winchester Drive Controller Command Port %X [NO HARDWARE IMPLEMENTATION]\n", address);
      /* this returned command value is hardcoded to indicate the absence of the
      Winchester Drive controller */
      return_value = 0x0;
      break;
    // Z-217 Primary Winchester Drive Controller Status Port (0xAE)
    case 0xAE:
      // printf("reading from Z-217 Primary Winchester Drive Controller Status Port %X [NO HARDWARE IMPLEMENTATION]\n", address);
      /* this returned status value is hardcoded to indicate the absence of the
      Winchester Drive Controller S-100 card */
      return_value = 0xfe;
      break;
    // Z-217 Primary Winchester Drive Controller Command Port (0xAF)
    case 0xAF:
      // printf("reading from Z-217 Primary Winchester Drive Controller Command Port %X [NO HARDWARE IMPLEMENTATION]\n", address);
      /* this returned command value is hardcoded to indicate the absence of the
      Winchester Drive controller */
      return_value = 0x0;
      break;
    /* FD179X-02 Floppy Disk Formatter/Controller (0xB0-0xB5) */
    case 0xB0:
      // Z-207 Primary Floppy Drive Controller Status Port
      // printf("reading from Z-207 Primary Floppy Drive Controller Status Port %X\n",
      //  address);
      return_value = readJWD1797(jwd1797, address);
      break;
    case 0xB1:
      // Z-207 Primary Floppy Drive Controller Track Port
      // printf("reading from Z-207 Primary Floppy Drive Controller Track Port %X\n",
      //  address);
      return_value = readJWD1797(jwd1797, address);
      break;
    case 0xB2:
      // Z-207 Primary Floppy Drive Controller Sector Port
      // printf("reading from Z-207 Primary Floppy Drive Controller Sector Port %X\n",
      //   address);
      return_value = readJWD1797(jwd1797, address);
      break;
    case 0xB3:
      // Z-207 Primary Floppy Drive Controller Data Port
      // printf("reading from Z-207 Primary Floppy Drive Controller Data Port %X\n",
      //   address);
      return_value = readJWD1797(jwd1797, address);
      break;
    case 0xB4:
      // Z-207 Primary Floppy Drive Controller CNTRL Control Port
      // printf("reading from Z-207 Primary Floppy Drive Controller CNTRL Control Port %X\n",
      //   address);
      return_value = readJWD1797(jwd1797, address);
      break;
    case 0xB5:
      // Z-207 Primary Floppy Drive Controller CNTRL Status Port
      // printf("reading from Z-207 Primary Floppy Drive Controller CNTRL Status Port %X\n",
      //   address);
      return_value = readJWD1797(jwd1797, address);
      break;
    // Video Commands - 68A21 parallel port
    case 0xD8:
      // printf("reading from Video Command - 68A21 parallel port %X\n", address);
      return_value = readVideo(video, address)&0xff;
      break;
    // Video Command Control - 68A21 parallel port
    case 0xD9:
      // printf("reading from Video Command Control - 68A21 parallel port %X\n", address);
      return_value = readVideo(video, address)&0xff;
      break;
    // Video RAM Mapping Module Data - 68A21 parallel port
    case 0xDA:
      // printf("reading from Video RAM Mapping Module Data - 68A21 parallel port %X\n",
      //   address);
      return_value = readVideo(video, address)&0xff;
      break;
    // Video RAM Mapping Module Control - 68A21 parallel port
    case 0xDB:
      // printf("reading from Video RAM Mapping Module Control - 68A21 parallel port %X\n",
      //   address);
      return_value = readVideo(video, address)&0xff;
      break;
    // CRT Controller 68A45 Register Select port 0xDC
    case 0xDC:
      // printf("reading from 68A45 CRT Controller Register Select port %X\n", address);
      return_value = readVideo(video, address)&0xff;
      break;
    // CRT Controller 68A45 Register Value port 0xDD
    case 0xDD:
      // printf("reading from 68A45 CRT Controller Register Value port %X\n", address);
      return_value = readVideo(video, address)&0xff;
      break;
    // parallel port (68A21 - Peripheral Interface Adapter (PIA))
    // ports 0xE0-0xE3 ***WHY SHOULD ALL THE 68A21 PORT READS RETURN 0x40??**
    case 0xE0:
      // general data port
      // printf("reading from 68A21 general data port %X [0x40 hardcoded return]\n",
      //  address);
      // hardcoded return value
      return_value = 0x40;
      break;
    case 0xE1:
      // general control port
      // printf("reading from 68A21 general control port %X [0x40 hardcoded return]\n",
      //  address);
      // hardcoded return value
      return_value = 0x40;
      break;
    case 0xE2:
      // printer data port
      // printf("reading from 68A21 printer data port %X [0x40 hardcoded return]\n",
      //   address);
      // hardcoded return value
      return_value = 0x40;
      break;
    case 0xE3:
      // printer control port
      // printf("reading from 68A21 printer control port %X [0x40 hardcoded return]\n",
      //   address);
      // hardcoded return value
      return_value = 0x40;
      break;
    case 0xE4:
      // 8253 timer counter 0
      // printf("reading from 8253 timer counter 0 port %X\n", address);
      return_value = e8253_get_uint8(e8253, address&3)&0xff;
      break;
    case 0xE5:
      // 8253 timer counter 1
      // printf("reading from 8253 timer counter 1 port %X\n", address);
      return_value = e8253_get_uint8(e8253, address&3)&0xff;
      break;
    case 0xE6:
      // 8253 timer counter 2
      // printf("reading from 8253 timer counter 2 port %X\n", address);
      return_value = e8253_get_uint8(e8253, address&3)&0xff;
      break;
    case 0xE7:
      // 8253 timer control port
      // printf("reading from 8253 control port %X\n", address);
      return_value = e8253_get_uint8(e8253, address&3)&0xff;
      break;
    // read IRR register of 8259 slave interrupt controller (PIC)
    case 0xF0:
		case 0xF1:
      // printf("reading slave interrupt controller port %X\n", address);
			return_value = e8259_get_uint8(e8259_slave, address&1)&0xff;
      break;
    // read IRR register of 8259 master interrupt controller (PIC)
    case 0xF2:
		case 0xF3:
      // printf("reading master interrupt controller port %X\n", address);
			return_value = e8259_get_uint8(e8259_master, address&1)&0xff;
      break;
    // keyboard data
    case 0xF4:
      // printf("reading keyboard data port %X\n", address);
      return_value = keyboardDataRead(keybrd);
      break;
    // keyboard status
    case 0xF5:
      // printf("reading keyboard status port %X\n", address);
      return_value = keyboardStatusRead(keybrd);
      break;
    // IO_DIAG port F6
    case 0xF6:
      // printf("reading from IO_DIAG port %X\n", address);
      return_value = io_diag_port_F6;
      break;
    case 0xFB:
      // 8253 timer status port
      // printf("reading from 8253 status port %X\n", address);
      return_value = e8253_get_status(e8253)&0xff;
      break;
    case 0xFC:
      // memory control latch port FC;
      // printf("reading from memory control latch port %X\n", address);
      return_value = memory_control_latch_FC;
      break;
		case 0xFE:
      // processor swap port FE;
      // printf("reading from processor swap port %X\n", address);
      return_value = processor_swap_port_FE;
      break;
    // S101 DIP Switch - (Page 2.8 in Z100 technical manual for pin defs)
    case 0xFF:
      // printf("reading from DIP_switch_s101_FF port %X\n", address);
      return_value = switch_s101_FF;
      break;
    default:
  		printf("READING FROM UNIMPLEMENTED PORT %X\n",address);
  		return_value = 0x00;
      break;
  }
  // printInt(return_value);
  // printf("value read: %X\n", return_value);
  return return_value;
}

void z100_port_write(unsigned int address, unsigned char data) {
  // the appropriate port reads will happen here
  // printf("Value %X written to port at address %X\n", data, address);
  switch(address) {
    // Z-217 Secondary Winchester Drive Controller Status Port (0xAA)
    case 0xAA:
      // printf("writing %X to Z-217 Secondary Winchester Drive Controller Status Port %X [NO HARDWARE IMPLEMENTATION]\n",
      //   data, address);
      /* NO Winchester Drive Controller S-100 card present */
      break;
    // Z-217 Secondary Winchester Drive Controller Command Port (0xAB)
    case 0xAB:
      // printf("writing %X to Z-217 Secondary Winchester Drive Controller Command Port %X [NO HARDWARE IMPLEMENTATION]\n",
      //   data, address);
      /* NO Winchester Drive Controller S-100 card present */
      break;
    // Z-217 Primary Winchester Drive Controller Status Port (0xAE)
    case 0xAE:
      // printf("writing %X to Z-217 Primary Winchester Drive Controller Status Port %X [NO HARDWARE IMPLEMENTATION]\n",
      //   data, address);
      /* NO Winchester Drive Controller S-100 card present */
      break;
    // Z-217 Primary Winchester Drive Controller Command Port (0xAF)
    case 0xAF:
      // printf("writing %X to Z-217 Primary Winchester Drive Controller Command Port %X [NO HARDWARE IMPLEMENTATION]\n",
      //   data, address);
      /* NO Winchester Drive Controller S-100 card present */
      break;
    /* FD179X-02 Floppy Disk Formatter/Controller (0xB0-0xB5) */
    case 0xB0:
      // Z-207 Primary Floppy Drive Controller Command Port
      // printf("writing %X to Z-207 Primary Floppy Drive Controller Command Port %X\n",
      //   data, address);
      writeJWD1797(jwd1797, address, data&0xff);
      break;
    case 0xB1:
      // Z-207 Primary Floppy Drive Controller Track Port
      // printf("writing %X to Z-207 Primary Floppy Drive Controller Track Port %X\n",
      //   data, address);
      writeJWD1797(jwd1797, address, data&0xff);
      break;
    case 0xB2:
      // Z-207 Primary Floppy Drive Controller Sector Port
      // printf("writing %X to Z-207 Primary Floppy Drive Controller Sector Port %X\n",
      //   data, address);
      writeJWD1797(jwd1797, address, data&0xff);
      break;
    case 0xB3:
      // Z-207 Primary Floppy Drive Controller Data Port
      // printf("writing %X to Z-207 Primary Floppy Drive Controller Data Port %X\n",
      //   data, address);
      writeJWD1797(jwd1797, address, data&0xff);
      break;
    case 0xB4:
      // Z-207 Primary Floppy Drive Controller CNTRL Control Port
      // printf("writing %X to Z-207 Primary Floppy Drive Controller CNTRL Control Port %X\n",
      //   data, address);
      writeJWD1797(jwd1797, address, data&0xff);
      break;
    case 0xB5:
      // Z-207 Primary Floppy Drive Controller CNTRL Status Port
      // printf("writing %X to Z-207 Primary Floppy Drive Controller CNTRL Status Port %X\n",
      //   data, address);
      writeJWD1797(jwd1797, address, data&0xff);
      break;
    // Video Commands - 68A21 parallel port
    case 0xD8:
      // printf("writing %X to Video Command - 68A21 parallel port %X\n",
      // 	data, address);
      writeVideo(video, address, data&0xff);
      break;
    // Video Command Control - 68A21 parallel port
    case 0xD9:
      // printf("writing %X to Video Command Control - 68A21 parallel port %X\n",
      // 	data, address);
      writeVideo(video, address, data&0xff);
      break;
    // Video RAM Mapping Module Data - 68A21 parallel port
    case 0xDA:
      // printf("writing %X to Video RAM Mapping Module Data - 68A21 parallel port %X\n",
      // 	data, address);
      writeVideo(video, address, data&0xff);
      break;
    // Video RAM Mapping Module Control - 68A21 parallel port
    case 0xDB:
      // printf("writing %X to Video RAM Mapping Module Control - 68A21 parallel port %X\n",
      // 	data, address);
      writeVideo(video, address, data&0xff);
      break;
    // CRT Controller 68A45 Register Select port 0xDC
    case 0xDC:
      // printf("writing %X to 68A45 CRT Controller Register Select port %X\n",
      // 	data, address);
      writeVideo(video, address, data&0xff);
      break;
    // CRT Controller 68A45 Register Value port 0xDD
    case 0xDD:
      // printf("writing %X to 68A45 CRT Controller Register Value port %X\n",
      // 	data, address);
      writeVideo(video, address, data&0xff);
      break;
    // parallel port (68A21 - Peripheral Interface Adapter (PIA))
    // ports 0xE0-0xE3
    case 0xE0:
      // general data port
      // printf("writing %X to 68A21 general data port %X [NO HARDWARE IMPLEMENTATION]\n",
      //  data, address);
      break;
    case 0xE1:
      // general control port
      // printf("writing %X to 68A21 general control port %X [NO HARDWARE IMPLEMENTATION]\n",
      //   data, address);
      break;
    case 0xE2:
      // printer data port
      // printf("writing %X to 68A21 printer data port %X [NO HARDWARE IMPLEMENTATION]\n",
      //   data, address);
      break;
    case 0xE3:
      // printer control port
      // printf("writing %X to 68A21 printer control port %X [NO HARDWARE IMPLEMENTATION]\n",
      //   data, address);
      break;
    case 0xE4:
      // 8253 timer counter 0
      // printf("writing %X to 8253 timer counter 0 port %X\n", data, address);
      e8253_set_uint8(e8253, address&3, data&0xff);
      break;
    case 0xE5:
      // 8253 timer counter 1
      // printf("writing %X to 8253 timer counter 1 port %X\n", data, address);
      e8253_set_uint8(e8253, address&3, data&0xff);
      break;
    case 0xE6:
      // 8253 timer counter 2
      // printf("writing %X to 8253 timer counter 2 port %X\n", data, address);
      e8253_set_uint8(e8253, address&3, data&0xff);
      break;
    case 0xE7:
      // 8253 timer control port
      // printf("writing %X to 8253 timer control port %X\n", data, address);
      e8253_set_uint8(e8253, address&3, data&0xff);
      break;
    case 0xFB:
      // 8253 timer status port
      // printf("writing %X to 8253 timer status port %X\n", data, address);
      e8253_set_status(e8253, data&0xff);
      break;
    // memory control latch port FC;
    case 0xFC:
      // printf("writing %X to memory control latch port %X\n", data, address);
			memory_control_latch_FC = data;
      /* extract bits 3 and 2 to determine which ROM memory configuration
       will be set */
      int bit2 = (data >> 2) & 0x01;
      int bit3 = (data >> 3) & 0x01;
      if (bit3 == 0 && bit2 == 0) {
        romOption = 0;
      }
      else if (bit3 == 0 && bit2 == 1) {
        romOption = 1;
      }
      else if (bit3 == 1 && bit2 == 0) {
        romOption = 2;
      }
      else if (bit3 == 1 && bit2 == 1) {
        romOption = 3;
      }
      // check the zero parity mode bit by examining bit 4 -
      // if the zero party bit is 0, the parity for every byte is forced to zero
      // -- this mode is used to force a parity error to check the parity logic
      zeroParity = ((data>>4)&1)==0;
      // check the kill parity bit by examing bit 5 -
      // if this bit is a 0, the parity checking circuitry is trurned off
			int tokillparity = ((data>>5)&1)==0;
      // if parity checking circuitry is enabled with a 1 written to bit 5 and
      // killParity variable is a 1 which means the circuitry is off (actual hardware
      // memory control port bit 5 = 0)
			if(!tokillparity && killParity) {
				printf("CLEAR PARITY ERROR\n");
				e8259_set_irq0(e8259_master, 0);
			}
			killParity = tokillparity;
      break;
    // slave interrupt controler (8259) ports F0 and F1
    case 0xF0:
    case 0xF1:
      // printf("writing %X to slave interrupt controller port %X\n", data, address);
      e8259_set_uint8(e8259_slave, address&1, data);
      break;
    // master interrupt controler (8259) ports F2 and F3
    case 0xF2:
    case 0xF3:
      // printf("writing %X to master interrupt controller port %X\n", data, address);
      e8259_set_uint8(e8259_master, address&1, data);
      break;
    // keyboard command port F5
    case 0xF5:
      // printf("writing %X to keyboard command port %X\n", data, address);
      keyboardCommandWrite(keybrd, data);
      break;
    // IO_DIAG port F6
    case 0xF6:
      // printf("writing %X to IO_DIAG port %X\n", data, address);
      io_diag_port_F6 = data;
      break;
    // processor switch port FE
    case 0xFE:
      processor_swap_port_FE = data;
      // printf("Value %X written to processor_swap_port_FE port at address %X\n",
      //  data, address);
      // select proper processor based on 7th bit in port data
      // if the 7th bit is 1, select 8088 processor
      if(((processor_swap_port_FE >> 7) & 0x01) == 0x01) {
        active_processor = pr8088;
      }
      // if the 7th bit is 0, select 8085 processor
      else if (((processor_swap_port_FE >> 7) & 0x01) == 0x00) {
        active_processor = pr8085;
      }
			// generate interrupt signal (I1 on master 8259A) if bit 1 is set high
			if(((processor_swap_port_FE >> 1) & 0x01) == 0x01) {
				e8259_set_irq1(e8259_master, 1);
			}
			/* force a swap to the 8088 if bit 0 is high. This option is used when
			an interrupt is called when the 8085 is active but the interrupt must
			be handled by the 8088. Otherwise (if bit 0 is low) interrupts are handled
			by whatever processor is active. */
			if(((processor_swap_port_FE) & 0x01) == 0x01) {
				active_processor = pr8088;
			}
      break;
    // S101 DIP Switch - (Page 2.8 in Z100 technical manual for pin defs)
    // NOTE: since this is a PHYSICAL DIP switch - this case should never be called
    case 0xFF:
      // printf("ERROR: CAN NOT WRITE to DIP_switch_s101_FF port at address %X\n",
			// 	address);
      break;
    // unimplemented port
    default:
  		printf("WRITING TO UNIMPLEMENTED PORT %X\n",address);
      break;
  }
}

// open the bin file for the Z100 ROM and load it into the rom array
// modified from Michael Black's Z100 implementation (z100.c)
void loadrom(char* fname) {
	FILE* f = fopen(fname,"rb");
	for(int i = 0; i < ROM_SIZE; i++) {
    rom[i] = fgetc(f);
  }
	fclose(f);
}

void initialize_z100_ports() {
  // this will be the S101 Switch - selects functions to be run during start-up
  // and master reset. (Page 2.8 in Z100 technical manual for pin defs)
  switch_s101_FF = 0b00000000;
  // processor swap ports
  processor_swap_port_FE = 0b00000000;
	// memeory control latch
  memory_control_latch_FC = 0b00000000;
  // io_diag_port
  io_diag_port_F6 = 0b11111111;
}

// the timer functions below serve as stand-in functions
void timer_out_0() {
  printf("timer out 0\n");
}
void timer_ext_0() {
  printf("timer ext 0\n");
}
void timer_out_1() {
  printf("timer out 1\n");
}
void timer_ext_1() {
  printf("timer ext 1\n");
}
void timer_out_2() {
  printf("timer out 2\n");
}
void timer_ext_2() {
  printf("timer ext 2\n");
}

// --------------------------------------------
/* MAIN FUNCTIONS */
// --------------------------------------------

// emulator thread function
void* mainBoardThread(void* arg) {
  // start the z100_main function defined here in mainBoard.c. This starts the
  // actual emulation program.
  z100_main();
}

// MAIN FUNCTION - ENTRY
int main(int argc, char* argv[]) {

  char user_input_string[100];
  char valid_enter_key_press;
  /* user chooses DEBUG mode */
  // loop until a valid entry is input by the user ('1', '2', or '3')
  while(1) {
    printf("%s", "\nChoose a mode:\n\n  1. Normal\n  2. DEBUG\n  3. EXIT\n\n>> ");
    fgets(user_input_string, 100, stdin);
    // check that the first character in the user's entry is '1', '2', or '3'
    // also check that only one character was entered followed by a '\n'
    if((user_input_string[0] < '1' || user_input_string[0] > '3') ||
      user_input_string[1] != '\n') {
      printf("%s\n", "INVALID ENTRY!\n");
      // back to top of loop to reprint menu
      continue;
    }
    /* entry valid - terminate loop with user_input_string[0] holding user's
    menu choice */
    break;
  }

  // process user choice
  switch(user_input_string[0]) {
    case '1' :
      debug_mode = FALSE;
      printf("%s\n", "\nNORMAL MODE");
      break;
    case '2' :
      printf("%s\n", "\nDEBUG MODE");
      debug_mode = TRUE;

      // get instruction break point from user
      while(1) {
        printf("\nEnter an instruction number to break at.\n");
        printf("%s", "(Negative integers will give undesired break points.)\n\n>> ");
        fgets(user_input_string, 100, stdin);
        // https://stackoverflow.com/questions/4072190/check-if-input-is-integer-type-in-c
        if(sscanf(user_input_string, "%lu%c", &breakAtInstruction, &valid_enter_key_press) != 2 ||
          valid_enter_key_press != '\n') {
            printf("INVALID ENTRY: Enter a valid integer greater or equal to 0 (zero)!\n");
            // back to top of loop to display prompt again
            continue;
        }
        // valid entry - break from loop
        break;
      }
      break;
    case '3' :
      printf("%s\n", "\nEXITING PROGRAM...\n");
      exit(0);
      break;
    default :
      printf("\nERROR - undefined behavior - EXITING PROGRAM...\n\n" );
      exit(0);
  }

  // allocate memory for array and initialize each pixel element to 0
  // (uses generateScreen() function from video.c to set up the pixel array)
  pixels = generateScreen();
  // call initialization function defined in screen.c to set up a gtk window
  screenInit(&argc, &argv);
  // start main emulator thread
  pthread_create(&emulator_thread, NULL, mainBoardThread, NULL);
  // start GTK window thread
  screenLoop();
}
