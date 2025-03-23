# -*- coding: utf-8 -*-

from numba import njit

print(f"Find satisfies both math formula.")
print(f"\tx ≡ 32134 (mod 1584891)")
print(f"\tx ≡ 193127 (mod 3438478)")

print("start find....")

@njit
def task():
    k = 1
    while(True):
        x = 193127 + 3438478 * k
        if x % 1584891 == 32134:
            return x
        k += 1

x = task()
print(f"x = {x}")
print(f"x mod 1584891 = {x % 1584891}")
print(f"x mod 3438478 = {x % 3438478}")