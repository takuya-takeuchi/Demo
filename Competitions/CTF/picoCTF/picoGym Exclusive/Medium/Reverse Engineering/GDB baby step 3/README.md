# GDB baby step 3

## How to resolve?

Debug [debugger0_c](debugger0_c) by `gdb`.

`gdb` may be litte difficult.
This program need you using the following commands.

* `disass`
  * Disassemble specified symbol. For example, `disass main`
* `b`
  * set break point to specified address or symobl. For example, `b *main+22`
* `run`
  * execute program
* `x/4xb`
  * dump memory for specified address. For example, `x/4xb $rbp-0x4`

In other words?