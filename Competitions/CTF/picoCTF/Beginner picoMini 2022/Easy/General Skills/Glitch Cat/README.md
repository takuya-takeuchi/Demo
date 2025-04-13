# Glitch Cat

## How to resolve?

````bash
$ nc saturn.picoctf.net 57209
'picoCTF{gl17ch_m3_n07_' + chr(0x61) + chr(0x34) + chr(0x33) + chr(0x39) + chr(0x32) + chr(0x64) + chr(0x32) + chr(0x65) + '}'
````

We can only convert `chr(0xXX)` to Ascii value.

````bash
$ python3 solver.py
````

In other words?