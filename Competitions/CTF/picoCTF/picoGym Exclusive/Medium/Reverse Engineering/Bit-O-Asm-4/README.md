# Bit-O-Asm-4

## How to resolve?

[disassembler-dump0_d.txt](disassembler-dump0_d.txt) is

````txt
<+0>:     endbr64 
<+4>:     push   rbp
<+5>:     mov    rbp,rsp
<+8>:     mov    DWORD PTR [rbp-0x14],edi
<+11>:    mov    QWORD PTR [rbp-0x20],rsi
<+15>:    mov    DWORD PTR [rbp-0x4],0x9fe1a
<+22>:    cmp    DWORD PTR [rbp-0x4],0x2710
<+29>:    jle    0x55555555514e <main+37>
<+31>:    sub    DWORD PTR [rbp-0x4],0x65
<+35>:    jmp    0x555555555152 <main+41>
<+37>:    add    DWORD PTR [rbp-0x4],0x65
<+41>:    mov    eax,DWORD PTR [rbp-0x4]
<+44>:    pop    rbp
<+45>:    ret
````

We have to know value in `eax` register.
But `eax` looks like to be calculated value by `add` and `imul` instruction.

The `cmp` instruction subtracts two operands, but does not store the result, but reflects the result in the flag registers (EFLAGS).
The `jle` instruction jump if EFLAGS is Less than or Equal (signed).
The `sub` instruction copies result of `src` - `dest` operand to `dest` operand.
The `jmp` instruction jump to specified address or label without any condition.

1. rbp-0x4=0x9fe1a
2. compare rbp-0x4 - 0x2710
3. Check EFLAGS and jump if need
4. rbp-0x4 - 0x65
5. jump
6. rbp-0x4 + 0x65

In other words?