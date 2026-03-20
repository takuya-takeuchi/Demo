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
GStreamer 1.26.11
````

#### Linux

````bat
$ ./install/linux/bin/Demo testdata/mnist.onnx Input3 1 28 28 Plus214_Output_0
[Info] output tensor: {1, 10}
````

#### OSX

````shell
$ ./install/osx/bin/Demo
GStreamer 1.26.11
````