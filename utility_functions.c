//utility functions
#include <stdio.h>
#include <stdint.h>

// this is a helper function to display numbers in binary format
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

// https://stackoverflow.com/questions/10156409/convert-hex-string-char-to-int
/**
 * hex2int
 * take a hex string and convert it to a 32bit number (max 8 hex digits)
 */
int32_t hex2int(char *hex) {
    int32_t val = 0;
    while (*hex) {
        // get current character then increment
        uint8_t byte = *hex++;
        // transform hex character to the 4bit equivalent number, using the ascii table indexes
        if (byte >= '0' && byte <= '9') byte = byte - '0';
        else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;
        // stdin input text may include '\n' as its final character
        else if (byte == '\n') continue;
        // if an invalid hex char is detected, return -1 (we expect a positive value)
        else return -1;
        // shift 4 to make space for new digit, and add the 4 bits of the new digit
        val = (val << 4) | (byte & 0xF);
    }
    return val;
}
