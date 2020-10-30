//utility functions
#include <stdio.h>

// this is a helper function to display numbrs in binary format
void print_bin8_representation(unsigned char val) {
  for(int i=7; i>=0; i--)  {
    printf("%d",(val>>i)&1);
  }
  printf("\n");
}

void printByteArray(unsigned char *array, int size) {
  for (int i = 0; i < size; i++) {
      if (i > 0) printf(" ");
      printf("%02X", array[i]);
  }
  printf("\n");
}

void printInt(int num) {
  int length;
  char output[20];
  length = sprintf(output, "%d", num);
  printf("%s\n", output);
}

void intToString(int num, char* output_array) {
  int length;
  length = sprintf(output_array, "%d", num);
  // return output_array;
}

void intToHexString(int num, char* output_array) {
  int length;
  length = sprintf(output_array, "%X", num);
  // return output_array;
}
