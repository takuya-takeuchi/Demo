# OpenSSL

## Abstracts

* Build OpenSSL
* Sample and experimental codes

## Requirements

### Common

* Powershell 7 or later
* C++ Compiler

### Windows

* Visual Studio 2022
* [Strawberry Perl](https://strawberryperl.com/)
  * GNU General Public License or the Artistic License
* [NASM](https://www.nasm.us/)
  * BSD-2-Clause License

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [OpenSSL](https://www.openssl.org/)
  * 3.0 or later: Apache License 2.0
  * 1.x and earlier: OpenSSL License

## How to use?

At first, you have to download openssl source code.

````shell
$ pwsh download.ps1 <OpenSSL version>
````

Then, you can start building.
It takes a long time.

#### Windows

````shell
$ set PERLPATH=D:\Works\Lib\Strawberry Perl\5.26.2.1\perl\bin
$ set NASMPATH=D:\Works\Lib\NASM\2.13.03\x64
$ pwsh build.ps1 <OpenSSL version> <Debug/Release>
````

#### Linux

````shell
$ pwsh build.ps1 <OpenSSL version> <Debug/Release>
````

#### OSX

````shell
$ pwsh build.ps1 <OpenSSL version> <Debug/Release>
````