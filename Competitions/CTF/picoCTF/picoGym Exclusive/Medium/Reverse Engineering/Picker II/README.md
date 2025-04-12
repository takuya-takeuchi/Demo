# Picker II

## How to resolve?

At first, launch instance and download source code.
Check soruce code [picker-II.py](./picker-II.py).

Then, connect by `nc saturn.picoctf.net 65440`.
Program can not accept `win` as input because `filter` function rejects it.

But `exec` function still exists.
So you can open and print content of `flag.txt` by input command.

In other words?