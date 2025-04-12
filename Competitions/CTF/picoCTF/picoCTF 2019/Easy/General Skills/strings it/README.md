# strings it

## How to resolve?

You need to get flag without running [string](./strings).
This file is dynamic link library file.

````bash
$ file string
strings: ELF 64-bit LSB shared object, x86-64, version 1 (SYSV), dynamically linked, interpreter /lib64/ld-linux-x86-64.so.2, for GNU/Linux 3.2.0, BuildID[sha1]=047a5079a5f563cd0e540d28f42a37161093ffda, not stripped
````

But this file could has flag.
So you must find flag which be store as string symbols from this file by using `strings` command.

````bash
$ strings strings | grep pico
````

In other words?