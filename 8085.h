typedef struct
{
	int A, B, C, D, E, H, L, SP, PC;
	int c, p, ac, z, s, i, m75, m65, m55;
	int flags;
	int interrupt_deferred, interrupts;
	int halted;

	int opcode, value, immediate, cycles;
	char* name;

	/* signal for external device data read request (used for FD-1797 Floppy Disk
		controller */
	int ready_;

	int wait_state;

	unsigned char* memory;
} P8085;

void doInstruction8085();
void reset8085();
void throwInterrupt8085(int);
