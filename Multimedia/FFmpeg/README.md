# FFmpeg

## Abstracts

* Build FFmpeg

## Requirements

### Common

* Powershell 7 or later
* CMake
  * 3.16 or higher

### Windows

* Visual Studio 2022

##### NOTE

Not supported.

### Ubuntu

* g++
* `nasm`
  * install by `apt install nasm`

### OSX

* Xcode

## Dependencies

* [FFmpeg](https://github.com/FFmpeg/FFmpeg)
  * n8.1
  * GNU Lesser General Public License 2.0, 3.0, GNU General Public License 2.0 or 3.0

## How to use?

````shell
$ pwsh build-abseil.ps1 <Debug/Release>
$ pwsh build-zlib.ps1 <Debug/Release>
$ pwsh build-protobuf.ps1 <Debug/Release>
$ pwsh build.ps1 <Debug/Release>
````