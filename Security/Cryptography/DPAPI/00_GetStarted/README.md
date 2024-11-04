# Encrypt/Decrypt file

## Abstracts

* Encrypt/Decrypt specified file by Crypto Next Genaration (CNG) API

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

#### Encryption

This is input data `test.txt`.

````txt
Hello, world!!
こんにちは、世界!!
````

````bat
$ .\install\win\bin\Demo.exe 0 test.txt output.data
[Info]   Plained Text: 48 65 6c 6c 6f 2c 20 77 6f 72 6c 64 21 21 0d 0a e3 81 93 e3 82 93 e3 81 ab e3 81 a1 e3 81 af e3 80 81 e4 b8 96 e7 95 8c 21 21 
[Info] Encrypted Text: 01 00 00 00 d0 8c 9d df 01 15 d1 11 8c 7a 00 c0 4f c2 97 eb 01 00 00 00 6f 4d c5 12 86 1f 9a 40 b4 a1 4f 51 76 f5 81 af 00 00 00 00 02 00 00 00 00 00 10 66 00 00 00 01 00 00 20 00 00 00 fb 3b d7 3a 20 e4 c5 58 c0 fb f8 74 30 28 5f 61 e4 32 44 31 4e 11 14 c1 5d 33 60 b2 2e 34 68 88 00 00 00 00 0e 80 00 00 00 02 00 00 20 00 00 00 c9 05 3b 48 67 8c 8d 90 88 28 f5 f4 e7 79 ab 0d c5 e2 74 3e f1 11 3a 58 62 1b 54 90 9d 4d 5f 14 30 00 00 00 23 b4 56 4e ed 2c 4c 34 33 b9 b1 5a 46 a5 51 f9 49 40 87 ea b2 e1 a3 3b 5d e2 86 ee ff 24 b1 0e a3 e7 e8 67 25 51 de e9 94 f1 b0 96 83 74 e6 26 40 00 00 00 37 5c 76 82 ad ba 63 d5 ef 77 53 61 a9 bd 59 4c 41 64 86 37 6b 0c b5 5f 35 2f 6b 86 90 14 4d b3 01 0e 60 b6 50 de a3 ef 6e 6f 44 91 3c f5 bc f7 de 67 7f b8 4b ee 58 d8 90 e2 f2 a9 9d ea ed 97
````

#### Decryption

````bat
$ .\install\win\bin\Demo.exe 1 output.data decoded.txt
[Info] Encrypted Text: 01 00 00 00 d0 8c 9d df 01 15 d1 11 8c 7a 00 c0 4f c2 97 eb 01 00 00 00 6f 4d c5 12 86 1f 9a 40 b4 a1 4f 51 76 f5 81 af 00 00 00 00 02 00 00 00 00 00 10 66 00 00 00 01 00 00 20 00 00 00 fb 3b d7 3a 20 e4 c5 58 c0 fb f8 74 30 28 5f 61 e4 32 44 31 4e 11 14 c1 5d 33 60 b2 2e 34 68 88 00 00 00 00 0e 80 00 00 00 02 00 00 20 00 00 00 c9 05 3b 48 67 8c 8d 90 88 28 f5 f4 e7 79 ab 0d c5 e2 74 3e f1 11 3a 58 62 1b 54 90 9d 4d 5f 14 30 00 00 00 23 b4 56 4e ed 2c 4c 34 33 b9 b1 5a 46 a5 51 f9 49 40 87 ea b2 e1 a3 3b 5d e2 86 ee ff 24 b1 0e a3 e7 e8 67 25 51 de e9 94 f1 b0 96 83 74 e6 26 40 00 00 00 37 5c 76 82 ad ba 63 d5 ef 77 53 61 a9 bd 59 4c 41 64 86 37 6b 0c b5 5f 35 2f 6b 86 90 14 4d b3 01 0e 60 b6 50 de a3 ef 6e 6f 44 91 3c f5 bc f7 de 67 7f b8 4b ee 58 d8 90 e2 f2 a9 9d ea ed 97
[Info]   Plained Text: 48 65 6c 6c 6f 2c 20 77 6f 72 6c 64 21 21 0d 0a e3 81 93 e3 82 93 e3 81 ab e3 81 a1 e3 81 af e3 80 81 e4 b8 96 e7 95 8c 21 21
````

If try to decrypt on another machine, `CryptUnprotectData` returns false.

````bat
$ .\install\win\bin\Demo.exe 1 output.data decoded.txt
[Error] Failed to decrypt
````