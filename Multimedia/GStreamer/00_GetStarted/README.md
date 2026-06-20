# Get started

## Abstracts

* Link gstreamer binary
* Show gstreamer version

## Requirements

### Common

* Powershell 7 or later
* CMake
  * 3.26 or higher

### Windows

* Visual Studio 2022

### Ubuntu

* g++

### OSX

* Xcode
* pkg-config
  * You can install `brew install pkg-config`

## Dependencies

* [GStreamer](https://gstreamer.freedesktop.org/)
  * GNU General Public License (GPL) version 2.1

## How to build?

### GStreamer

Go to [GStreamer](..).

Once time you built `GStreamer`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ .\install\win\bin\Demo.exe
GStreamer 1.28.2
````

#### Linux

````bat
$ export GSTREAMER_VERSION=1.28.2
$ export GST_PLUGIN_SCANNER=../install/linux/gstreamer/${GSTREAMER_VERSION}$/Release/libexec/gstreamer-1.0/gst-plugin-scanner
$ export GST_PLUGIN_SYSTEM_PATH=../install/linux/gstreamer/${GSTREAMER_VERSION}/Release/lib/x86_64-linux-gnu/gstreamer-1.0
$ export LD_LIBRARY_PATH=../install/linux/gstreamer/${GSTREAMER_VERSION}/Release/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
$ ./install/linux/bin/Demo
GStreamer 1.28.2
````

#### OSX

````shell
$ ./install/osx/bin/Demo
GStreamer 1.28.2
````