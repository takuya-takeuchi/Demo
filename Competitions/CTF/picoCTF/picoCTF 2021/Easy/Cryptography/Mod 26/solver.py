# -*- coding: utf-8 -*-

def solveRot13(text: str):
    shitf = 13
    ret = ""
    text = text.lower()
    for c in text:
        if c.isalpha():
            s = int(ord(c))
            c = int(s + shitf)
            # 0x7a is z
            if c > 0x7a:
                c = 0x60 + c - 0x7a
            ret += str(chr(c))
        else:
            ret += c
    return ret

plaintext = solveRot13("cvpbPGS{arkg_gvzr_V'yy_gel_2_ebhaqf_bs_ebg13_nSkgmDJE}")
print(f"{plaintext}")