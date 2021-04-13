#define PREFETCH_SIZE 4
typedef void (*store_function) (unsigned int addr, unsigned char val);
typedef unsigned int (*load_function) (unsigned int addr);

typedef struct
{
	unsigned int AL, AH, BL, BH, CL, CH, DL, DH, SP, BP, DI, SI, IP;
	unsigned int CS, SS, DS, ES;
	unsigned int c, p, ac, z, s, t, i, d, o;

	int interrupt_deferred, interrupts;
	int halt;
	int enable_interrupts;
	int ready_x86_;
	int wait_state_x86;

	unsigned int opcode, value, operand1, operand2, immediate, cycles;
	int op_result;
	char* name_opcode;

	unsigned char* memory;

	unsigned int prefetch[PREFETCH_SIZE];
	int prefetch_counter;

	store_function memory_write_x86;
	store_function port_write_x86;
	load_function memory_read_x86;
	load_function port_read_x86;

	// debug
	unsigned char investigate_opcode;

} P8088;

P8088* new8088();
void doInstruction8088(P8088*);
void reset8088(P8088*);
void trap(P8088*,unsigned int);
void prefetch_flush(P8088*);

void assignCallbacks8088(P8088*,load_function,store_function,load_function,store_function);
