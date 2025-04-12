# Picker IV

## How to resolve?

At first, launch instance and download source code.
Check soruce code [picker-IV.c](./picker-IV.c).

Then, connect by `nc saturn.picoctf.net 56856`.

You need to invoke `win` function.
But this code is written C language.
Program requires you to input memory address.

You must know adress of `win` function.
Therefore, you need to analyze binary [picker-IV](./picker-IV) by `nm`.

In other words?