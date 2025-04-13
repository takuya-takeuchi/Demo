# -*- coding: utf-8 -*-

ret = ""
with open("input.txt") as f:
    for line in f.readlines():
        ret += chr(int(line))
print(ret)