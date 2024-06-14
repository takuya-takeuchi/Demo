# libiconv

## Abstracts

* Use libiconv and convert shit-jis text to utf-8 text

## Requirements

### Common

* Powershell 7 or later
* CMake 3.0.0 or later
* C++ Compiler supports C++14

### Windows

* Visual Studio
* libiconv
  * via `vcpkg` command
    * `vcpkg install tesseract --triplet x64-windows-static`

### Ubuntu

* g++
* install `libiconv` from source code

````shell
$ wget https://ftp.gnu.org/pub/gnu/libiconv/libiconv-1.17.tar.gz
$ tar xvfzp libiconv-1.17.tar.gz
$ pushd libiconv-1.17
$ ./configure
$ make
$ sudo make install
$ popd
````

### OSX

* Xcode
* libiconv
  * via `brew` command
    * `brew install libiconv`

## Dependencies

* [tesseract](https://github.com/tesseract-ocr/tesseract)
  * 5.4.0
  * Apache-2.0 license
* [tessdata](https://github.com/tesseract-ocr/tessdata)
  * 4.1.0
  * Apache-2.0 license

## How to use?

````shell
$ pwsh build.ps1 <Debug/Release>
[Info] iconv_open
[Info] iconv_close
````