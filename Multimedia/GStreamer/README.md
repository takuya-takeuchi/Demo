# GStreamer

## Abstracts

* Build GStreamer

## Requirements

### Common

* Powershell 7 or later
* Python3

### Windows

* Visual Studio 2022

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [GStreamer](https://gstreamer.freedesktop.org/)
  * 1.26.11
  * GNU General Public License (GPL) version 2.1

### Dependencies for development

##### Windows

* [pkg-config](https://download.gnome.org)
  * 0.23-2
  * GNU General Public License
* [glib](https://download.gnome.org)
  * 2.26.1-1
  * GNU Lesser General Public License 2.01
* [gettext](https://download.gnome.org)
  * 0.18.1.1-2
  * GNU General Public License

## How to use?

````shell
$ pwsh build.ps1 <Debug/Release>
````

For windows, you need to kick [download-pkg-config.ps1](./download-pkg-config.ps1) to use `gstreamer` for cmake project.