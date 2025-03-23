# -*- coding: utf-8 -*-

def solveCaesarCipher(text: str, shitf: int):
    ret = ""
    for c in text:
        if c.isalpha():
            s = int(ord(c))
            c = int(s + shitf)
            ret += str(chr(c))
        else:
            ret += c
    return ret

prefix = solveCaesarCipher("fsdz", -3)
postfix = solveCaesarCipher("Fdhvdu_flskhu_lv_fodvvlfdo_flskhu", -3)
print(f"{prefix}{{{postfix}}}")