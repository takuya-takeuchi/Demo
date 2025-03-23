#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char* argv[])
{
  char flags[28];
  flags[0] = 0x7a;
  flags[1] = 0x69;
  flags[2] = 0x78;
  flags[3] = 0x6e;
  flags[4] = 0x62;
  flags[5] = 0x6f;
  flags[6] = 0x7c;
  flags[7] = 0x6b;
  flags[8] = 0x77;
  flags[9] = 0x78;
  flags[10] = 0x74;
  flags[0xb] = 0x38;
  flags[0xc] = 0x38;
  flags[0xd] = 100;
  char* var2 = flags + 0xe;
  for (int i = 0xe; i != 0; i -= 1)
  {
    *var2 = 0;
    var2 = var2 + 1;
  }
  for (int i = 0; i < 0xe; i += 1)
  {
    flags[i + 0xe] = flags[i] ^ 0x19;
  }

  printf("%s", flags);
}
