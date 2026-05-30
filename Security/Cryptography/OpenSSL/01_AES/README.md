# AES

## Abstracts

* Use AES Encryption by OpenSSL's EVP (The Digital EnVeloPe library) functions.

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++17

### Windows

* Visual Studio 2022

### Linux

* g++

### OSX

* Xcode

## How to build?

At first, you must build OpenSSL on [OpenSSL](..) directory.

````shell
$ pwsh build.ps1 <Debug/Release> <x86_64/x86/arm64>
````
Once time you built opnessl, you need not to do again.

## How to use?

### Windows

````bat
$ .\install\openssl-3.6.2\win\x86_64\static\Release\bin\Demo.exe "This is a secret message." password 128
OpenSSL version text: OpenSSL 3.6.2 7 Apr 2026

              Algorithm: AES-128
             Plain Text: This is a secret message.
       Plain Text (Hex): 54 68 69 73 20 69 73 20 61 20 73 65 63 72 65 74 20 6d 65 73 73 61 67 65 2e
               Password: password
      Password (Hashed): 5e 88 48 98 da 28 04 71 51 d0 e5 6f 8d c6 29 27
                     IV: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
         Encrypted Text: d6 28 0a 9c c7 00 96 c4 45 ad 24 f6 85 77 c8 bf 30 b4 e9 51 2f c3 64 1c 5d 2b 77 7e 57 e9 61 92
         Decrypted Text: This is a secret message.
   Decrypted Text (Hex): 54 68 69 73 20 69 73 20 61 20 73 65 63 72 65 74 20 6d 65 73 73 61 67 65 2e

$ .\install\openssl-3.6.2\win\x86_64\static\Release\bin\Demo.exe "This is a secret message." password 256
OpenSSL version text: OpenSSL 3.6.2 7 Apr 2026

              Algorithm: AES-256
             Plain Text: This is a secret message.
       Plain Text (Hex): 54 68 69 73 20 69 73 20 61 20 73 65 63 72 65 74 20 6d 65 73 73 61 67 65 2e
               Password: password
      Password (Hashed): 5e 88 48 98 da 28 04 71 51 d0 e5 6f 8d c6 29 27 73 60 3d 0d 6a ab bd d6 2a 11 ef 72 1d 15 42 d8
                     IV: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
         Encrypted Text: ff c1 33 a3 98 17 e1 98 b1 7c 25 80 5f 10 80 c8 03 0f 68 82 fd 8e 71 6f ab fe dc 49 19 c5 62 da
         Decrypted Text: This is a secret message.
   Decrypted Text (Hex): 54 68 69 73 20 69 73 20 61 20 73 65 63 72 65 74 20 6d 65 73 73 61 67 65 2e
````

### Linux

````bash
$ ./install/openssl-3.6.2/linux/x86_64/static/Release/bin/Demo "This is a secret message." password 128
OpenSSL version text: OpenSSL 3.6.2 7 Apr 2026

              Algorithm: AES-128
             Plain Text: This is a secret message.
       Plain Text (Hex): 54 68 69 73 20 69 73 20 61 20 73 65 63 72 65 74 20 6d 65 73 73 61 67 65 2e
               Password: password
      Password (Hashed): 5e 88 48 98 da 28 04 71 51 d0 e5 6f 8d c6 29 27
                     IV: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
         Encrypted Text: d6 28 0a 9c c7 00 96 c4 45 ad 24 f6 85 77 c8 bf 30 b4 e9 51 2f c3 64 1c 5d 2b 77 7e 57 e9 61 92
         Decrypted Text: This is a secret message.
   Decrypted Text (Hex): 54 68 69 73 20 69 73 20 61 20 73 65 63 72 65 74 20 6d 65 73 73 61 67 65 2e

$ ./install/openssl-3.6.2/linux/x86_64/static/Release/bin/Demo "This is a secret message." password 256
OpenSSL version text: OpenSSL 3.6.2 7 Apr 2026

              Algorithm: AES-256
             Plain Text: This is a secret message.
       Plain Text (Hex): 54 68 69 73 20 69 73 20 61 20 73 65 63 72 65 74 20 6d 65 73 73 61 67 65 2e
               Password: password
      Password (Hashed): 5e 88 48 98 da 28 04 71 51 d0 e5 6f 8d c6 29 27 73 60 3d 0d 6a ab bd d6 2a 11 ef 72 1d 15 42 d8
                     IV: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
         Encrypted Text: ff c1 33 a3 98 17 e1 98 b1 7c 25 80 5f 10 80 c8 03 0f 68 82 fd 8e 71 6f ab fe dc 49 19 c5 62 da
         Decrypted Text: This is a secret message.
   Decrypted Text (Hex): 54 68 69 73 20 69 73 20 61 20 73 65 63 72 65 74 20 6d 65 73 73 61 67 65 2e
````

### OSX

````bash
$ ./install/openssl-3.6.2/osx/arm64/static/Release/bin/Demo "This is a secret message." password 128
OpenSSL version text: OpenSSL 3.6.2 7 Apr 2026

              Algorithm: AES-128
             Plain Text: This is a secret message.
       Plain Text (Hex): 54 68 69 73 20 69 73 20 61 20 73 65 63 72 65 74 20 6d 65 73 73 61 67 65 2e
               Password: password
      Password (Hashed): 5e 88 48 98 da 28 04 71 51 d0 e5 6f 8d c6 29 27
                     IV: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
         Encrypted Text: d6 28 0a 9c c7 00 96 c4 45 ad 24 f6 85 77 c8 bf 30 b4 e9 51 2f c3 64 1c 5d 2b 77 7e 57 e9 61 92
         Decrypted Text: This is a secret message.
   Decrypted Text (Hex): 54 68 69 73 20 69 73 20 61 20 73 65 63 72 65 74 20 6d 65 73 73 61 67 65 2e

$ ./install/openssl-3.6.2/osx/arm64/static/Release/bin/Demo "This is a secret message." password 256
OpenSSL version text: OpenSSL 3.6.2 7 Apr 2026

              Algorithm: AES-256
             Plain Text: This is a secret message.
       Plain Text (Hex): 54 68 69 73 20 69 73 20 61 20 73 65 63 72 65 74 20 6d 65 73 73 61 67 65 2e
               Password: password
      Password (Hashed): 5e 88 48 98 da 28 04 71 51 d0 e5 6f 8d c6 29 27 73 60 3d 0d 6a ab bd d6 2a 11 ef 72 1d 15 42 d8
                     IV: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
         Encrypted Text: ff c1 33 a3 98 17 e1 98 b1 7c 25 80 5f 10 80 c8 03 0f 68 82 fd 8e 71 6f ab fe dc 49 19 c5 62 da
         Decrypted Text: This is a secret message.
   Decrypted Text (Hex): 54 68 69 73 20 69 73 20 61 20 73 65 63 72 65 74 20 6d 65 73 73 61 67 65 2e
````
