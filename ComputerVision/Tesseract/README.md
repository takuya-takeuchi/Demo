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
* install the following packages

````shell
$ sudo apt install -y libicu-dev libcairo2-dev libsdl-pango-dev
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
* [leptonica](https://github.com/danbloomberg/leptonica)
  * 1.84.1
  * BSD 2-clause license

## How to use?

````shell
$ pwsh build.ps1 <Debug/Release>
[Info] iconv_open
[Info] iconv_close
````