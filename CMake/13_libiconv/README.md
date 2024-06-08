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
    * `vcpkg install libiconv --triplet x64-windows`

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

````shell
$ pwsh build.ps1 <Debug/Release>
[Info] iconv_open
[Info] iconv_close
````