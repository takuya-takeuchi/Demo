# GDB baby step 2

## How to resolve?

Debug [debugger0_b](debugger0_b) by `gdb`.

This problem says `Can you figure out what is in the eax register at the end of the main function?`.
We have to know value in `eax` register.

`gdb` may be litte difficult.
This program need you using the following commands.

* `disass`
  * Disassemble specified symbol. For example, `disass main`
* `b`
  * set break point to specified address or symobl. For example, `b *main+59`
* `run`
  * execute program
* `p`
  * print register value. For example, `p $eax`

In other words?