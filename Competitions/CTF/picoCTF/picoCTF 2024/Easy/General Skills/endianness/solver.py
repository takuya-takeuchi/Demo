# -*- coding: utf-8 -*-

ret = []
for c in 'jvrib':
    ret.append(hex(ord(c)).replace("0x", ""))

ret.reverse()
print(f"LSB: {''.join(ret)}")
ret.reverse()
print(f"MSB: {''.join(ret)}")