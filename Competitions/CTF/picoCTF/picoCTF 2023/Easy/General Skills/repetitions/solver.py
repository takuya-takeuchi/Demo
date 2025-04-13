# -*- coding: utf-8 -*-

import base64

ret = ""
with open("enc_flag") as f:
    for line in f.readlines():
        ret += line

while (True):
    ret = base64.b64decode(ret).decode()
    if ret.startswith("picoCTF"):
        print(ret)
        break