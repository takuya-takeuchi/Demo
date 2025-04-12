# Bit-O-Asm-1

## How to resolve?

[disassembler-dump0_a.txt](disassembler-dump0_a.txt) is

````txt
<+0>:     endbr64 
<+4>:     push   rbp
<+5>:     mov    rbp,rsp
<+8>:     mov    DWORD PTR [rbp-0x4],edi
<+11>:    mov    QWORD PTR [rbp-0x10],rsi
<+15>:    mov    eax,0x30
<+20>:    pop    rbp
<+21>:    ret
````

We have to know value in `eax` register.
The `mov` instruction copies the `src` operand to the `dest` operand.

In other words?