# Useful tools

## Network

#### aircrack-ng

Aircrack-ng is a complete suite of tools to assess WiFi network security.

````bash
$ aircrack-ng wpa-ing_out.pcap -w rockyou.txt
````

## Binary

#### strings

Command to display the readable portion of a binary or data file.

````bash
$ $ strings asciiftw 
/lib64/ld-linux-x86-64.so.2
libc.so.6
__stack_chk_fail
printf
__cxa_finalize
__libc_start_main
GLIBC_2.2.5
GLIBC_2.4
_ITM_deregisterTMCloneTable
__gmon_start__
_ITM_registerTMCloneTable
u+UH
...
````

#### objdump

Command to display content of binary object.

````bash
$ objdump -d asciiftw
...
0000000000001169 <main>:
    1169:       f3 0f 1e fa             endbr64 
    116d:       55                      push   %rbp
    116e:       48 89 e5                mov    %rsp,%rbp
    1171:       48 83 ec 30             sub    $0x30,%rsp
    1175:       64 48 8b 04 25 28 00    mov    %fs:0x28,%rax
    ..
````

#### nm

Command to display symbols information of binary object.

````bash
$ nm asciiftw 
0000000000004010 B __bss_start
0000000000004010 b completed.8061
                 w __cxa_finalize@@GLIBC_2.2.5
0000000000004000 D __data_start
0000000000004000 W data_start
00000000000010b0 t deregister_tm_clones
0000000000001120 t __do_global_dtors_aux
0000000000003db8 d __do_global_dtors_aux_fini_array_entry
0000000000004008 D __dso_handle
0000000000003dc0 d _DYNAMIC
0000000000004010 D _edata
0000000000004018 B _end
00000000000012b8 T _fini
0000000000001160 t frame_dummy
    ..
````

#### checksec

Checksec checks the properties of executables (like PIE, RELRO, Canaries, ASLR, Fortify Source).

````bash
$ checksec --file=vuln
RELRO           STACK CANARY      NX            PIE             RPATH      RUNPATH      Symbols         FORTIFY Fortified       Fortifiable  FILE
Full RELRO      Canary found      NX enabled    PIE enabled     No RPATH   No RUNPATH   78 Symbols     Yes      0               1       vuln
````

## Password list

* [rockyou.txt](https://github.com/brannondorsey/naive-hashcat/releases/download/data/rockyou.txt)

## Hash cracking

* [MD5Hashing.net](https://md5hashing.net/hash)