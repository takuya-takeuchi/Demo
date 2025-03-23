#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char* argv[])
{
  char flags[9];
  flags[0] = 0x79;
  flags[1] = 0x61;
  flags[2] = 0x6b;
  flags[3] = 0x69;
  flags[4] = 0x6e;
  flags[5] = 0x69;
  flags[6] = 0x6b;
  flags[7] = 0x75;
  flags[8] = 0x21;
  printf("%s", flags);
}
