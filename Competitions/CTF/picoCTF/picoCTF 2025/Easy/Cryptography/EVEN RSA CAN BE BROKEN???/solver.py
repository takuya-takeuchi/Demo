from Crypto.Util.number import inverse, long_to_bytes
from sympy import factorint

N = 24333469497314269964517721800457638381450532645164468811554055083976440971677211295325167601682791622781741573155217458388077776276374180150703211069636902
e = 65537
cyphertext = 16495239692262059115339136365104050211474117435494010199079411539963884804551723002752339848124907947287076830992196602874784181146653837684424052921006419

factors = factorint(N)
if len(factors) != 2:
    raise ValueError("N is not semiprime (p * q)")

p, q = list(factors.keys())
print(f"p={p}, q={q}")

# φ(N) = (p-1)(q-1)
phi = (p - 1) * (q - 1)

# privatekey d = e^-1 mod φ(N)
d = inverse(e, phi)

# m = cyphertext^d mod N
m = pow(cyphertext, d, N)
plaintext = long_to_bytes(m)
print(f"plaintext: {plaintext}")