# AES

## Abstracts

* Use AES-256 Encryption by Crypto Next Genaration (CNG) API

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++17

### Windows

* Visual Studio

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to use?

````bat
$ install\win\bin\Demo.exe "This is a secret message." password
             Plain Text: This is a secret message.
       Plain Text (Hex): 54 68 69 73 20 69 73 20 61 20 73 65 63 72 65 74 20 6d 65 73 73 61 67 65 2e
               Password: password
      Password (Hashed): 5e 88 48 98 da 28 04 71 51 d0 e5 6f 8d c6 29 27 73 60 3d 0d 6a ab bd d6 2a 11 ef 72 1d 15 42 d8
                     IV: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
         Encrypted Text: ff c1 33 a3 98 17 e1 98 b1 7c 25 80 5f 10 80 c8 03 0f 68 82 fd 8e 71 6f ab fe dc 49 19 c5 62 da
         Decrypted Text: This is a secret message.
   Decrypted Text (Hex): 54 68 69 73 20 69 73 20 61 20 73 65 63 72 65 74 20 6d 65 73 73 61 67 65 2e
````