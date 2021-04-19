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
/* (20 cycles = 4 microseconds, EDIT - 8253 timer is connected to a 250kHz
	clock according to page 2.80 Z-100 Technical manual (hardware) - clocked
	every 120 cycles */
#define E8253_TIMER_CYCLE_LIMIT 20
/* 83,333 cycles = 16,666.6 microseconds, VSYNC occurs at 60 Hz
 	along with the display refresh rate
	(page 4.46 Z-100 Technical manual (hardware)) */
#define VSYNC_TIMER_CYCLE_LIMIT 83333

enum active_processor{pr8085, pr8088};
enum active_processor active_processor;
int processor_wait_state;

// rough clock to keep track of time passing (us) as instructions are executed
double total_time_elapsed;
unsigned long instructions_done;
unsigned long breakAtInstruction;
unsigned int vsync_timer_cycle_count;
unsigned int vsync_timer_overage;
unsigned int e8253_timer_cycle_count;
unsigned int e8253_timer_overage;
int last_instruction_cycles;
double last_instruction_time_us;

// debug window thread
// pthread_t dbug_window_thread;

pthread_t emulator_thread;	// this thread object runs the main emulator thread
int romOption;  // temp holder of bits 3 and 2 of memeory control latch port FC
int killParity;
int zeroParity;
int byteParity;
char debug_mode;
int debug_mode_2_active;
bool reset_irq6;
unsigned char user_debug_mode_choice;
unsigned char switch_s101_FF;
unsigned char processor_swap_port_FE;
unsigned char memory_control_latch_FC;
unsigned char io_diag_port_F6; // not needed without external diag device
// DEBUG
unsigned char debug_test_port;
unsigned char debug_timer2;
unsigned char debug_int6;
int tkp_debug;

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
void handleDebug2Mode();
void handle8085InstructionCycle();
void handle8088InstructionCycle();
void updateElapsedVirtualTime();
void simulateVSYNCInterrupt();
void handle8253TimerClockCycle();
void updateZ100Screen();
void handleDebugOutput();
void fD1797DebugOutput();

void loadimage();

/*
 ============== MAIN Z100 FUNCTION ==============
*/
int z100_main() {

  /*
		*** set up all hardware conpoenets on the main board ***
	*/

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
  e8253_set_gate(e8253, 0, 1);
  e8253_set_gate(e8253, 1, 1);
  e8253_set_gate(e8253, 2, 1);
	/* set OUT 0 and OUT 2 functions to triggar IRQ2 on the master PIC
	 OUT 0 also clocks timer channel 1 in the Z-100 */
  e8253_set_out_fct(e8253, 0, NULL, timer_out_0);
	e8253_set_out_fct(e8253, 1, NULL, timer_out_1);
  e8253_set_out_fct(e8253, 2, NULL, timer_out_2);

  // set up floppy controller object
  jwd1797 = newJWD1797();
	// load current disk file (hardcoded) here in reset function
  resetJWD1797(jwd1797);

  /* this creates a thread running from the debug window function */
  // pthread_create(&dbug_window_thread, NULL, run_debug_window, NULL);
  // run_debug_window();
  // update values displayed in the debug gui
  // update_debug_gui_values();
  // g_main_context_invoke (NULL, update_debug_gui_values, NULL);

  /*
		***** initialize all counts to zero *****
	*/
  instructions_done = 0;
	last_instruction_cycles = 0;
	last_instruction_time_us = 0.0;
	total_time_elapsed = 0.0;
	vsync_timer_cycle_count = 0;
	vsync_timer_overage = 0;
	e8253_timer_cycle_count = 0;
	e8253_timer_overage = 0;
	processor_wait_state = 0;
	// turn debug mode 2 off until program counter target has been reached
	debug_mode_2_active = 0;
	// set processor selection to 8085
  active_processor = pr8085;

	debug_test_port = 0x00;
	debug_timer2 = 0x00;
	debug_int6 = 0x00;
	tkp_debug = 0;
  /*
	 >>>> RUN THE PROCESSORS <<<<
	*/
  printf("\n\nStart running processors..\n\n");
  // --- run the processor(s) forever ---
  while(1) {
		// check if instruction pointer (IP) has been reached for dubug mode 2
		handleDebug2Mode();
    // if active processor is the 8085
    if(active_processor == pr8085) {
			handle8085InstructionCycle();
    }
    // if the active processor is the 8088
    else if(active_processor == pr8088) {
			handle8088InstructionCycle();
		}

		// calculate virtual time passed with the last instruction execution
		updateElapsedVirtualTime();

    /* simulate VSYNC interrupt on I6 (keyboard video display and light pen int)
			roughly every 10,000 instructions - This satisfies BIOS diagnostics,
			but the interrupt routine is not used to display the Z-100 screen
		*/
    simulateVSYNCInterrupt();

    /* clock the 8253 timer - this should be ~ every 4 microseconds
		 - according to page 2.80 of Z-100 technical manual, the timer is
		 clocked by a 250kHz clock (every 20 cycles of the 5 Mhz main clock)
		 The e8253 timer is incremented based on the last instruction cycle count.
		*/
		handle8253TimerClockCycle();

		/* cycle the JWD1797. The JWD1797 is driven by a 1 MHz clock in the Z-100.
			Thus, it should be cycled every 1 microsecond or every five CPU (5 MHz)
			cycles. Instead, time slices added to the internal JWD1797 timer
			mechanisms will be determined by how many cycles the previous instruction
			took.
		*/
		doJWD1797Cycle(jwd1797, last_instruction_time_us);

    // update the screen every 100,000 instructions
    updateZ100Screen();

    // update_debug_gui_values();

    /* if debug mode is active, wait for enter key to continue after each
    instruction */
		handleDebugOutput();

		if(instructions_done == 1797613) {
			// loadimage();
		}
  }
  // return from z100_main() (will not return because of processor loop)
  return 0;
}


//=============================================================================
//										***** FUNCTION DEFINITIONS *****
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
	printf("SLAVE 8259 PIC INT SIGNAL to MASTER 8259 PIC IRQ3\n");
  e8259_set_irq3(e8259_master, 1);
}

/* since the Z-100 timer circuitry clocks the 8253 chnnel 1 timer with the
 output of the channel 0 timer, this function clocks timer channel 1 when
 channel 0's out goes high (Z-100 Technical Manual (Hardware) - Page 10.8) */
void timer_out_0(void* v, int number) {
	// printf("timer out 0\n");
	e8259_set_irq2(e8259_master, 1);
	e8253_cascade_clock_ch1(e8253, 1);
}
// void timer_ext_0() {printf("timer ext 0\n");}
void timer_out_1(void* v, int number) {
	printf("timer out 1\n");
}
// void timer_ext_1() {printf("timer ext 1\n");}
void timer_out_2(void* v, int number) {
	printf("timer out 2\n");
	e8259_set_irq2(e8259_master, 1);
	debug_timer2 = 1;
}
// void timer_ext_2() {printf("timer ext 2\n");}

int getParity(unsigned int data) {
	return ((data&1)!=0) ^ ((data&2)!=0) ^ ((data&4)!=0) ^ ((data&8)!=0) ^
  ((data&16)!=0) ^ ((data&32)!=0) ^ ((data&64)!=0) ^ ((data&128)!=0) ^
  ((data&256)!=0) ^ ((data&512)!=0) ^ ((data&1024)!=0) ^ ((data&2048)!=0) ^
  ((data&4096)!=0) ^ ((data&8192)!=0) ^ ((data&16384)!=0) ^ ((data&32768)!=0);
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

int pr8085_FD1797WaitStateCondition(unsigned char opCode, unsigned char port_num) {
	// if 8085 "in" instruction and reading from WD1797 data register (port 0xB3)
	if((opCode == 0xdb) && (port_num == 0xb3)) {
		return 1;
	}
	return 0;
}

int pr8088_FD1797WaitStateCondition(unsigned char opCode, unsigned char port_num) {
	// if 8088 "in" instruction and reading from WD1797 data register (port 0xB3)
	/* MUST ALSO INCLUDE PORT 0xB5!! COMPUTER MUST WAIT FOR AN INTRQ signal from
		FD-1797 when reading the status port dusring the SEEK command! */
	if(((opCode == 0xe4) || (opCode == 0xe5) || (opCode == 0xec) || (opCode == 0xed))
		&& (port_num == 0xb3)) {
		return 1;
		}
	return 0;
}

void handleDebug2Mode() {

	if(debug_mode == '2') {
		if((active_processor == pr8085) && (p8085.PC == breakAtInstruction)) {
			debug_mode_2_active = 1;
		}
		else if((active_processor == pr8088) && (p8088->IP == breakAtInstruction)) {
			debug_mode_2_active = 1;
		}
	}

	// if(jwd1797->id_field_data[0] == 1 && jwd1797->id_field_data[1] == 1 &&
	// 	jwd1797->id_field_data[2] == 4 && jwd1797->intrq == 1) {
	// 		debug_mode_2_active = 1;
	// 	}

	// if(p8088->investigate_opcode == 0x64) {
	// 		debug_mode_2_active = 1;
	// }

	// if(jwd1797->commandRegister == 0x8A && jwd1797->sectorRegister == 0x04 &&
	// 	jwd1797->controlLatch == 0x18 && jwd1797->controlStatus == 0x03 &&
	// 	jwd1797->id_field_data[0] == 0x01) {
	// 		debug_mode_2_active = 1;
	// }

	if(jwd1797->intrq == 1 && debug_test_port == 0x14 && debug_timer2 == 1) {
			// debug_mode_2_active = 1;
	}

	// 02 01 01 02 01 01 ********* 18 to Z-207 Primary Floppy Drive Controller CNTRL Control Port B4

	if(p8088->CS==0x40 && p8088->IP == 0x817) {
		// debug_mode_2_active = 1;
	}

	// HARDCODE AL REGISTER TO GET PAST Z-DOS INTERRUPT LOOP
	if(p8088->IP == 0xECF) {
		// p8088->AL = 0xFF;
	}

	// debug to get to "Error LOADING COMMAND.COM"
	if(p8088->IP == 0xED6) {
		p8088->z = 0;
	}

	/* DEBUG: This condition is used to check interrupts' effect on Z-dos
	infinite loop */
	if(p8088->IP == 0xEE7) {
		// printf("SET ALL INTERRUPTS HIGH!\n");
		// e8259_set_irq0(e8259_master, 1);
		// e8259_set_irq1(e8259_master, 1);
		// e8259_set_irq2(e8259_master, 1);
		// e8259_set_irq3(e8259_master, 1);
		// e8259_set_irq4(e8259_master, 1);
		// e8259_set_irq5(e8259_master, 1);
	 	// e8259_set_irq6(e8259_master, 1);
		// e8259_set_irq7(e8259_master, 1);
		//
		// e8259_set_irq0(e8259_slave, 1);
		// e8259_set_irq1(e8259_slave, 1);
		// e8259_set_irq2(e8259_slave, 1);
		// e8259_set_irq3(e8259_slave, 1);
		// e8259_set_irq4(e8259_slave, 1);
		// e8259_set_irq5(e8259_slave, 1);
	 	// e8259_set_irq6(e8259_slave, 1);
		// e8259_set_irq7(e8259_slave, 1);
	}
}

void handle8085InstructionCycle() {
	// update data request signal from JWD1797 drq pin
	p8085.ready_ = jwd1797->drq;
	doInstruction8085(&p8085);
	// if processor is NOT in a wait state
	if(p8085.wait_state == 0) {
		instructions_done++;
		last_instruction_cycles = p8085.cycles;
	}
	/* processor is in a wait state
		- no instruction done
		- 1 clock cycle passes (200 ns for 5 MHz clock) */
	else {
		last_instruction_cycles = 1;
	}
	if((debug_mode == '1' && (instructions_done >= breakAtInstruction)) ||
		(debug_mode_2_active == 1)) {
		printf("PC = %X, opcode = %X, inst = %s\n",p8085.PC,p8085.opcode,p8085.name);
		printf("A = %X, B = %X, C = %X, D = %X, E = %X, H = %X, L = %X, SP = %X\n",
			p8085.A, p8085.B, p8085.C, p8085.D, p8085.E, p8085.H, p8085.L, p8085.SP);
		printf("carry = %X, parity = %X, aux_carry = %X, zero = %X, sign = %X\n",
			p8085.c, p8085.p, p8085.ac, p8085.z, p8085.s);
		printf("i = %X, m75 = %X, m65 = %X, m55 = %X\n",
			p8085.i, p8085.m75, p8085.m65, p8085.m55);
	}
}

void handle8088InstructionCycle() {
	// update data request signal from JWD1797 drq pin
	p8088->ready_x86_ = jwd1797->drq;
	doInstruction8088(p8088);
	// if processor is NOT in a wait state
	if(p8088->wait_state_x86 == 0) {
		instructions_done++;
		last_instruction_cycles = p8088->cycles;
	}
	/* processor is in a wait state
		- no instruction done
		- 1 clock cycle passes (200 ns for 5 MHz clock) */
	else {
		last_instruction_cycles = 1;
	}
	/* increment cycles_done to add the number of cycles the current
	instruction took */
	// cycles_done = cycles_done + p8088->cycles;
	// only print processor status if debug conditions are met
	if((debug_mode == '1' && (instructions_done >= breakAtInstruction)) ||
		(debug_mode_2_active == 1)) {
		printf("IP = %X, opcode = %X, inst = %s\n",
			p8088->IP,p8088->opcode,p8088->name_opcode);
		printf("value1 = %X, value2 = %X, result = %X cycles = %d\n",
			p8088->operand1,p8088->operand2,p8088->op_result,p8088->cycles);
		printf("AL = %X, AH = %X, BL = %X, BH = %X, CL = %X, CH = %X, DL = %X, DH = %X\n"
			"SP = %X, BP = %X, DI = %X, SI = %X\n"
			"CS = %X, SS = %X, DS = %X, ES = %X\n",
			p8088->AL, p8088->AH,p8088->BL,p8088->BH,p8088->CL,p8088->CH,p8088->DL,
			p8088->DH,p8088->SP,p8088->BP,p8088->DI,p8088->SI,p8088->CS,p8088->SS,
			p8088->DS,p8088->ES);
		printf("carry = %X, parity = %X, aux_carry = %X, zero = %X, sign = %X\n",
			p8088->c,p8088->p,p8088->ac,p8088->z,p8088->s);
		printf("trap = %X, int = %X, dir = %X, overflow = %X\n",
			p8088->t,p8088->i,p8088->d,p8088->o);
	}
}

void updateElapsedVirtualTime() {
	// number of cycles for last instruction * 0.2 microseconds (5 MHz clock)
	last_instruction_time_us = last_instruction_cycles * 0.2;
	// increment the time total_time_elapsed based on the last instruction time
	total_time_elapsed += last_instruction_time_us;
}

void simulateVSYNCInterrupt() {
	// update VSYNC cycle count
	vsync_timer_cycle_count += last_instruction_cycles;

	if(vsync_timer_cycle_count >= VSYNC_TIMER_CYCLE_LIMIT) {
		// this will account for any additional cycles not used for this clock cycle
		vsync_timer_overage = vsync_timer_cycle_count - VSYNC_TIMER_CYCLE_LIMIT;
		// printf("%s\n", "*** KEYINT or DSPYINT/VSYNC interrupt occurred - I6 from Master PIC ***");
		// set the irq6 pin on the master 8259 int controller to high
		e8259_set_irq6(e8259_master, 1);
		debug_int6 = 1;
		vsync_timer_cycle_count = vsync_timer_overage;
	}

	// reset VSYNC INT if pulse high
	if(debug_int6 == 1) {
		// set the irq6 pin to low
		e8259_set_irq6(e8259_master, 0);
		debug_int6 = 0;
	}
}

void handle8253TimerClockCycle() {
	// update timer cycle count
	e8253_timer_cycle_count += last_instruction_cycles;
	// printf("%s%d\n", "last_instruction_cycles: ", last_instruction_cycles);
	// printf("%s%d\n", "e8253_timer_cycle_count: ", e8253_timer_cycle_count);
	if(e8253_timer_cycle_count >= E8253_TIMER_CYCLE_LIMIT) {
		// this will account for any additional cycles not used for this clock cycle
		e8253_timer_overage = e8253_timer_cycle_count - E8253_TIMER_CYCLE_LIMIT;
		// printf("%s%d\n", "e8253_timer_overage: ", e8253_timer_overage);
		e8253_clock(e8253, 1);
		e8253_timer_cycle_count = e8253_timer_overage;
	}
}

void updateZ100Screen() {
	if(instructions_done%100000 == 0) {
		/* update pixel array using current VRAM state using renderScreen()
			function from video.c */
		renderScreen(video, pixels);
		// draw pixels to the GTK window using display() function from screen.c
		display();
	}
}

void handleDebugOutput() {
	if((debug_mode == '1' && (instructions_done >= breakAtInstruction)) ||
		(debug_mode_2_active == 1)) {
		printf("instructions done: %ld\n", instructions_done);
		printf("%s%f\n", "last instruction time (us): ", last_instruction_time_us);
		printf("TOTAL TIME ELAPSED (us): %f\n", total_time_elapsed);
		// fD1797DebugOutput();

		while(getchar() != '\n') {};
	}
}

void fD1797DebugOutput() {
	// DEBUG FD-1797 Floppy Disk Controller
	printf("%s%lu\n", "JWD1797 ROTATIONAL BYTE POINTER: ",
		jwd1797->rotational_byte_pointer);
	printf("%s%d\n", "JWD1797 ROTATIONAL BYTE TIMER (ns): ",
		jwd1797->rotational_byte_read_timer);
	printf("%s%d\n", "JWD1797 ROTATIONAL BYTE TIMER OVR (ns): ",
		jwd1797->rotational_byte_read_timer_OVR);
	printf("%s%02X\n", "Current Byte: ", getFDiskByte(jwd1797));
	printf("%s%f\n", "HEAD LOAD Timer: ", jwd1797->HLT_timer);
	printf("%s%f\n", "E Delay Timer: ", jwd1797->e_delay_timer);
	printf("%s", "FD-1797 Status Reg.: " );
	print_bin8_representation(jwd1797->statusRegister);
	printf("%s", "Disk ID Field Data: " );
	printByteArray(jwd1797->id_field_data, 6);
	printf("%s", "data a1 byte count: ");
  printf("%d\n", jwd1797->data_a1_byte_counter);
  printf("%s", "data AM search count: ");
  printf("%d\n", jwd1797->data_mark_search_count);
  printf("%s", "Data AM found: ");
  printf("%d\n", jwd1797->data_mark_found);
  printf("%s", "Sector length count: ");
  printf("%d\n", jwd1797->intSectorLength);
  printf("%s%d\n", "DRQ: ", jwd1797->drq);
  printf("%s%02X\n", "DATA REGISTER: ", jwd1797->dataRegister);
  printf("%s", "SECTOR REGISTER: ");
  print_bin8_representation(jwd1797->sectorRegister);
  printf("%s", "");
  printf("%s", "TYPE II STATUS REGISTER: ");
  print_bin8_representation(jwd1797->statusRegister);
  printf("%s\n", "");
}


// ============================================================================
														/* MAIN FUNCTIONS (ENTRY) */
// ============================================================================

// emulator thread function
void* mainBoardThread(void* arg) {
  // start the z100_main function defined here in mainBoard.c. This starts the
  // actual emulation program.
  z100_main();
}

// MAIN FUNCTION - ENTRY
int main(int argc, char* argv[]) {

	printf("\n\n%s\n\n",
		" ===================================\n"
		" |\tZENITH Z-100 EMULATOR\t   |\n"
		" |\tby: Joe Matta\t\t   |\n"
		" |\t8/2020 - 4/2021\t\t   |\n"
		" ===================================");

  char user_input_string[100];
  char valid_enter_key_press;
	debug_mode = '0';
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
      debug_mode = '0';
      printf("%s\n\n", "\nNORMAL MODE");
      break;
    case '2' :
      printf("%s\n\n", "\nDEBUG MODE");
			while(1) {
				// check that the first character in the user's entry is '1' or '2'
		    // also check that only one character was entered followed by a '\n'
		    printf(
					"%s", "\nChoose a DEBUG mode:\n\n  1. Break at Instruction number\n  "
					"2. Break at Instruction Pointer (IP) value\n  3. EXIT\n\n>> ");
		    fgets(user_input_string, 100, stdin);
				if((user_input_string[0] < '1' || user_input_string[0] > '3') ||
		      user_input_string[1] != '\n') {
		      printf("%s\n", "INVALID ENTRY!\n");
		      // back to top of loop to reprint menu
		      continue;
		    }
				break;
			}

      debug_mode = user_input_string[0];

			switch(debug_mode) {
				case '1' :
					while(1) {
		        printf("\nEnter an INSTRUCTION NUMBER to break at (integer >= 0).\n");
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
				case '2' :
					while(1) {
						printf("\nEnter an INSTRUCTION POINTER (IP) value to break at"
							" (hexadecimal value without \"0x\" >= 0. MAX 8 hex digits).\n");
						printf("%s", "(Negative values will give undesired break points.)\n\n>> ");
						fgets(user_input_string, 100, stdin);
						// convert hex string to int
						int ip_break_at = hex2int(user_input_string);
						// https://stackoverflow.com/questions/4072190/check-if-input-is-integer-type-in-c
						if(ip_break_at == -1) {
							printf("INVALID ENTRY: Enter a valid hex value greater or equal to 0 (zero)!\n");
							// back to top of loop to display prompt again
							continue;
						}
						// valid entry - break from loop
						breakAtInstruction = ip_break_at;
						break;
					}
					break;
				case '3' :
		      printf("%s\n", "\nEXITING Z-100 EMULATOR PROGRAM...\n");
		      exit(0);
		      break;
		    default :
		      printf("\nERROR - undefined behavior - EXITING Z-100 EMULATOR PROGRAM...\n\n" );
		      exit(0);
			}
			break;
    case '3' :
      printf("%s\n", "\nEXITING Z-100 EMULATOR PROGRAM...\n");
      exit(0);
      break;
    default :
      printf("\nERROR - undefined behavior - EXITING Z-100 EMULATOR PROGRAM...\n\n" );
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


/*
		================================
		***** READ/WRITE FUNCTIONS *****
		================================
*/

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
        if((debug_mode == '1' && (instructions_done >= breakAtInstruction)) ||
					(debug_mode_2_active == 1)) {printf("reading from video memory...\n");}
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
    if((debug_mode == '1' && (instructions_done >= breakAtInstruction)) ||
			(debug_mode_2_active == 1)) {
      printf("THROW PARITY INTERRUPT\n");
    }
    // write 1 to IRQ line 0 (pin 0)
		e8259_set_irq0(e8259_master, 1);
	}

  // DEBUG message
  if((debug_mode == '1' && (instructions_done >= breakAtInstruction)) ||
		(debug_mode_2_active == 1)) {
    printf("read %X from memory address %X\n", return_value, addr);
  }
  return return_value;
}

void z100_memory_write_(unsigned int addr, unsigned char data) {
  // DEBUG message
  if((debug_mode == '1' && (instructions_done >= breakAtInstruction)) ||
		(debug_mode_2_active == 1)) {
    printf("write %X to memory address %X\n", data, addr);
  }
  if(addr < RAM_SIZE) {
    // write data to RAM at address
    ram[addr] = data&0xff;
  }
	else if(addr >= 0xc0000 && addr <= 0xeffff) {
    // write data to video memeory portion of RAM at address
    // DEBUG message
    if((debug_mode == '1' && (instructions_done >= breakAtInstruction)) ||
			(debug_mode_2_active == 1)) {
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
      // address);
      return_value = readJWD1797(jwd1797, address);
      break;
    case 0xB1:
      // Z-207 Primary Floppy Drive Controller Track Port
      printf("reading from Z-207 Primary Floppy Drive Controller Track Port %X\n",
       address);
      return_value = readJWD1797(jwd1797, address);
      break;
    case 0xB2:
      // Z-207 Primary Floppy Drive Controller Sector Port
      printf("reading from Z-207 Primary Floppy Drive Controller Sector Port %X\n",
        address);
      return_value = readJWD1797(jwd1797, address);
      break;
    case 0xB3:
      // Z-207 Primary Floppy Drive Controller Data Port
      // printf("reading from Z-207 Primary Floppy Drive Controller Data Port %X\n",
      //  address);
      return_value = readJWD1797(jwd1797, address);
      break;
    case 0xB4:
      // Z-207 Primary Floppy Drive Controller CNTRL Control Port
      printf("reading from Z-207 Primary Floppy Drive Controller CNTRL Control Port %X\n",
        address);
			printf("CS = %X IP = %X\n", p8088->CS, p8088->IP);
      return_value = readJWD1797(jwd1797, address);
      break;
    case 0xB5:
			// Z-207 Primary Floppy Drive Controller CNTRL Status Port
			return_value = readJWD1797(jwd1797, address);
      // printf("reading %X from Z-207 Primary Floppy Drive Controller CNTRL Status Port %X\n",
      //   return_value, address);
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
      printf("writing %X to Z-207 Primary Floppy Drive Controller Command Port %X\n",
        data, address);
      writeJWD1797(jwd1797, address, data&0xff);
      break;
    case 0xB1:
      // Z-207 Primary Floppy Drive Controller Track Port
      printf("writing %X to Z-207 Primary Floppy Drive Controller Track Port %X\n",
        data, address);
      writeJWD1797(jwd1797, address, data&0xff);
      break;
    case 0xB2:
      // Z-207 Primary Floppy Drive Controller Sector Port
      printf("writing %X to Z-207 Primary Floppy Drive Controller Sector Port %X\n",
        data, address);
      writeJWD1797(jwd1797, address, data&0xff);
      break;
    case 0xB3:
      // Z-207 Primary Floppy Drive Controller Data Port
      printf("writing %X to Z-207 Primary Floppy Drive Controller Data Port %X\n",
        data, address);
      writeJWD1797(jwd1797, address, data&0xff);
      break;
    case 0xB4:
      // Z-207 Primary Floppy Drive Controller CNTRL Control Port
      printf("writing %X to Z-207 Primary Floppy Drive Controller CNTRL Control Port %X\n",
        data, address);
			printf("CS = %X IP = %X\n", p8088->CS, p8088->IP);
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
		case 0xEF:
      // Serial B (modem port)
      printf("writing %X to Serial B modem port %X\n", data, address);
			// DEBUG
      debug_test_port = data;
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
			tkp_debug = tokillparity;
			if(!tokillparity && killParity) {
				printf("CLEAR PARITY ERROR\n");
				printf("CS = %X IP = %X\n", p8088->CS, p8088->IP);
				e8259_set_irq0(e8259_master, 0);
			}
			killParity = tokillparity;
      break;
    // slave interrupt controler (8259) ports F0 and F1
    case 0xF0:
    case 0xF1:
      printf("writing %X to slave interrupt controller port %X\n", data, address);
      e8259_set_uint8(e8259_slave, address&1, data);
      break;
    // master interrupt controler (8259) ports F2 and F3
    case 0xF2:
    case 0xF3:
      printf("writing %X to master interrupt controller port %X\n", data, address);
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
      printf("ERROR: CAN NOT WRITE to DIP_switch_s101_FF port at address %X\n",
				address);
      break;
    // unimplemented port
    default:
  		printf("WRITING %X TO UNIMPLEMENTED PORT %X\n", data, address);
      break;
  }
}

void loadimage()
{
	const char* name="z100_memdmp.bin";
	FILE* f=fopen(name,"rb");
	for(int i=0; i<RAM_SIZE; i++)
		ram[i]=fgetc(f)&0xff;
	// for(int i=0; i<0x30000; i++)
	// 	video->vram[i]=fgetc(f)&0xff;
/*	for(int i=0; i<0x10000-0x2000; i++)
		fgetc(f);
	for(int i=0; i<0x2000; i++)
	{
		int r=fgetc(f);
		printf("%x ",r);
		rom[i]=r&0xff;
	}
*/
//	p8088->AX=0x0000;
//	p8088->BX=0x0000;
//	p8088->CX=0x007A;
//	p8088->DX=0x0000;
	p8088->AH=0x00;
	p8088->BH=0x00;
	p8088->CH=0x00;
	p8088->DH=0x00;
	p8088->AL=0x00;
	p8088->BL=0x00;
	p8088->CL=0x7A;
	p8088->DL=0x00;
	p8088->SP=0x0064;
	p8088->BP=0x0000;
	p8088->SI=0x0010;
	p8088->DI=0x0000;
	p8088->DS=0x0000;
	p8088->ES=0x08E9;
	p8088->SS=0x08EA;
	p8088->CS=0x08E2;
//	p8088->CS=0x0734;
	p8088->IP=0x0061;
	p8088->o=0;
	p8088->d=0;
	p8088->i=1;
	p8088->s=0;
	p8088->z=1;
	p8088->ac=0;
	p8088->p=1;
	p8088->c=1;
}
