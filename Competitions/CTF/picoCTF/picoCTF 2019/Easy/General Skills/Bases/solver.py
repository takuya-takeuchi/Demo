# -*- coding: utf-8 -*-

import base64

decoded = base64.b64decode("bDNhcm5fdGgzX3IwcDM1").decode('ascii')
print(f"picoctf{{{decoded}}}")