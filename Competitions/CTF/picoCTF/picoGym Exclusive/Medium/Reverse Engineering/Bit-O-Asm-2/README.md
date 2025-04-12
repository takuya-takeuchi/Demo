# Bit-O-Asm-2

## How to resolve?

[disassembler-dump0_b.txt](disassembler-dump0_b.txt) is

````txt
<+0>:     endbr64 
<+4>:     push   rbp
<+5>:     mov    rbp,rsp
<+8>:     mov    DWORD PTR [rbp-0x14],edi
<+11>:    mov    QWORD PTR [rbp-0x20],rsi
<+15>:    mov    DWORD PTR [rbp-0x4],0x9fe1a
<+22>:    mov    eax,DWORD PTR [rbp-0x4]
<+25>:    pop    rbp
<+26>:    ret
````

We have to know value in `eax` register.
The `mov` instruction copies the `src` operand to the `dest` operand.
`mov` instruction read value from [rdp-0x4].

In other words?