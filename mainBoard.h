unsigned int z100_port_read(unsigned int);
void z100_port_write(unsigned int, unsigned char);
unsigned int z100_memory_read_(unsigned int);
void z100_memory_write_(unsigned int, unsigned char);
int pr8085_FD1797WaitStateCondition(unsigned char, unsigned char);
int pr8088_FD1797WaitStateCondition(unsigned char, unsigned char);

unsigned int* pixels;
