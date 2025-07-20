# Q23.[Reversing]またやらかした！

## How to resolve?

````bash
$ file rev200 
rev200: ELF 32-bit LSB executable, Intel 80386, version 1 (SYSV), dynamically linked, interpreter /lib/ld-linux.so.2, for GNU/Linux 2.6.24, BuildID[sha1]=e87140105d6b5c8ea9b0193380ab3b79bfdcd85b, not stripped
````

Watch your step.
This module is 32bit binary and you can not run it on x86_&4 machine.
So you must install `lib32stdc++6` and `lib32z1`.
Otherwise, you will see `bash: ./rev200: No such file or directory` message.

But you can see the following message in spite of installing the above pacakges.

````bash
$ ./rev200
````

This program does not output but it is specification.
Therefore, you need to disassemble [rev200](./rev200).
For example, [Ghidra](https://ghidra-sre.org/).

<img src="./images/image.png" />

In other words?