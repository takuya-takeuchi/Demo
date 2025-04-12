# Bit-O-Asm-3

## How to resolve?

[disassembler-dump0_c.txt](disassembler-dump0_c.txt) is

````txt
<+0>:     endbr64 
<+4>:     push   rbp
<+5>:     mov    rbp,rsp
<+8>:     mov    DWORD PTR [rbp-0x14],edi
<+11>:    mov    QWORD PTR [rbp-0x20],rsi
<+15>:    mov    DWORD PTR [rbp-0xc],0x9fe1a
<+22>:    mov    DWORD PTR [rbp-0x8],0x4
<+29>:    mov    eax,DWORD PTR [rbp-0xc]
<+32>:    imul   eax,DWORD PTR [rbp-0x8]
<+36>:    add    eax,0x1f5
<+41>:    mov    DWORD PTR [rbp-0x4],eax
<+44>:    mov    eax,DWORD PTR [rbp-0x4]
<+47>:    pop    rbp
<+48>:    ret
````

We have to know value in `eax` register.
But `eax` looks like to be calculated value by `add` and `imul` instruction.

The `add` instruction copies result of `src` + `dest` operand to `dest` operand.
The `imul` instruction copies result of `src` * `dest` operand to `dest` operand.

* rdp-0xc=0x9fe1a
* rbp-0x8=0x4

In other words?