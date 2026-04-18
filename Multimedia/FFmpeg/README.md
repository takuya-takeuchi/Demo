# FFmpeg

## Abstracts

* Build FFmpeg and change build configuration

## Requirements

### Common

* Powershell 7 or later

### Windows

* [MSYS2](https://www.msys2.org/)

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
$ pwsh build.ps1 <Debug/Release>
````

If you want to change build configuration, you can modify [build-config.json](./build-config.json).
For example, you want to use **OpenH264** instead of **libx264**, set `true` flag property on `--enable-libopenh264` line.

Currently, these scripts support only OpenH264.
Please refer [./scripts/libopenh264.ps1](./scripts/libopenh264.ps1)