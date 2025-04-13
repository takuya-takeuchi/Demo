# endianness

## How to resolve?

````bash
$ nc titan.picoctf.net 51182
Welcome to the Endian CTF!
You need to find both the little endian and big endian representations of a word.
If you get both correct, you will receive the flag.
Word: jvrib
Enter the Little Endian representation:
````

At first, we must convert to hex representation.
And confirm acceptable input format ofr [flag.c](./flag.c).

````bash
$ python3 solver.py
````

In other words?