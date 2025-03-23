# -*- coding: utf-8 -*-

print(f"Find satisfies both math formula.")
print(f"\tx ≡ 32134 (mod 1584891)")
print(f"\tx ≡ 193127 (mod 3438478)")

print("start find....")
k = 1
while(True):
    x = 193127 * 3438478 * k
    if x % 1584891 == 32134:
        print(f"x = {x}")
        break
    k += 1