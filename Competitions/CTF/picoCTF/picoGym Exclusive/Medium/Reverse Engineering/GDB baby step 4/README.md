# GDB baby step 4

## How to resolve?

Debug [debugger0_d](debugger0_d) by `gdb`.

This problem says `main calls a function that multiplies eax by a constant. The flag for this challenge is that constant in decimal base.`.

`gdb` may be litte difficult.
This program need you using the following commands.

* `disass`
  * Disassemble specified symbol. For example, `disass main`
* `run`
  * execute program
* `n`
  * Step over
* `s`
  * Step in

This program has `func1` and you must check this symbol.
You will see constant value in this function.

In other words?