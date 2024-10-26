# Get serial number of attached storage device

## Abstracts

* Get serial number of attached storage device via `udev`

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++17

### Linux

* g++
* libudev
  * `apt install libudev-dev`

## Dependencies

* [libudev](https://github.com/systemd/systemd/tree/main/src/libudev)
  * GNU Lesser General Public License

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to use?

````shell
$ ./install/linux/bin/Demo sda
      ID_SERIAL: WDC_WD20EFRX-68EUZN0_WD-WCC4M2RS7K1R
ID_SERIAL_SHORT: WD-WCC4M2RS7K1R
$ ./install/linux/bin/Demo sdb
Cannot find device: sdb
````