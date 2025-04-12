# -*- coding: utf-8 -*-

import sys

len = len(sys.argv)

ret = ""
for i, c in enumerate(sys.argv):
    if i == 0:
        continue
    ret += chr(int(c, 16))
print(ret)