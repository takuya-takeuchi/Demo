# Picker III

## How to resolve?

At first, launch instance and download source code.
Check soruce code [picker-III.py](./picker-III.py).

Then, connect by `nc saturn.picoctf.net 56856`.

You need to invoke `win` function.
But program can not accept `win` as input.

You will find that program use `func_table` to invoke defined functions.
And you can override defined variable.
If you can fool `check_table` function, you will invoke function you want to.

In other words?