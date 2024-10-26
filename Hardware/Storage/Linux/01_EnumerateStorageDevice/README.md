# Enumerate attached storage devices

## Abstracts

* Enumerate storage devices via `udev`

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
$ ./install/linux/bin/Demo 
Device Node Path: /dev/sda, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/sda1, Device Subsystem: block, Device Type: partition
Device Node Path: /dev/sda2, Device Subsystem: block, Device Type: partition
Device Node Path: /dev/sda3, Device Subsystem: block, Device Type: partition
Device Node Path: /dev/sdb, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/sdc, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/sr0, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop0, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop1, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop10, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop11, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop12, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop13, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop14, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop15, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop16, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop2, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop3, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop4, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop5, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop6, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop7, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop8, Device Subsystem: block, Device Type: disk
Device Node Path: /dev/loop9, Device Subsystem: block, Device Type: disk
````