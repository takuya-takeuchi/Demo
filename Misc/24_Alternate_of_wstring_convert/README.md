# Alternate of wstring_convert

## Abstracts

* std::wstring_convert is deprecated since C++ 17 so replace it with other functions
  * Windows: WideCharToMultiByte/MultiByteToWideChar
  * Others: iconv

## Requirements

### Common

* Powershell 7 or later
* CMake 3.0.0 or later
* C++ Compiler supports C++17

### Windows

* Visual Studio

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

* [libiconv](https://www.gnu.org/software/libiconv/)
  * GNU Lesser General Public License

## How to use?

### Windows

````shell
$ pwsh build.ps1 <Debug/Release>
  UTF-8 String: こんにちは、世界！ (27)
WCHAR_T String: こんにちは、世界！ (9)
````

### OSX

````shell
$ pwsh build.ps1 <Debug/Release>
  UTF-8 String: こんにちは、世界！ (27)
WCHAR_T String: こんにちは、世界！ (9)
````

### Linux

You may have to do.

````shell
$ sudo locale-gen ja_JP.UTF-8
$ sudo update-locale LANG=ja_JP.UTF-8
$ sudo dpkg-reconfigure console-setup
````

But result of `WCHAR_T` is mojibake.

````shell
$ pwsh build.ps1 <Debug/Release>
  UTF-8 String: こんにちは、世界！ (27)
WCHAR_T String: S�kaoL (9)
````