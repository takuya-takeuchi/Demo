# -*- coding: utf-8 -*-

numbers = [
    "16",
    "9",
    "3",
    "15",
    "3",
    "20",
    "6",
    "{",
    "20",
    "8",
    "5",
    "14",
    "21",
    "13",
    "2",
    "5",
    "18",
    "19",
    "13",
    "1",
    "19",
    "15",
    "14",
    "}"    
]

ret = ""
for number in numbers:
    if number.isnumeric():
        num = int(number) + 0x40
        ret += chr(num)
    else:
        ret += number
print(ret.lower())

