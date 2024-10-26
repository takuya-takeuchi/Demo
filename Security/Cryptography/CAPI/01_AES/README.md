# AES

## Abstracts

* Use AES-256 Encryption by Cryptographic API (CryptoAPI, CAPI)

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
$ install\win\bin\Demo.exe test password 0
    Plain Text: test
      Password: password
            IV: N/A
Encrypted Text: 49 17 f4 45 38 35 7a c5 4a 38 f6 b1 22 f9 2f 30
Decrypted Text: test

$ install\win\bin\Demo.exe test password 1
    Plain Text: test
      Password: password
            IV: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
Encrypted Text: e5 02 df 2a 2b 19 b9 9f 23 6c 71 8c 0f c1 7f 9e
Decrypted Text: test
````