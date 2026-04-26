# Use OpenCV to decode h264 by FFMPEG backend with libopenh264

## Abstracts

* Decode h264 movie file by opencv with ffmpeg and libopenh264

## Requirements

### Common

* Powershell 7 or later
* CMake 3.20.0 or later
* C++ Compiler supports C++17

### Windows

* Visual Studio

### Linux

* g++

### OSX

* Xcode

## Dependencies

* [opencv](https://github.com/opencv/opencv)
  * 4.13.0
  * Apache-2.0 License

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

## Test Data

* [INTER-STREAM®サポートページ](https://inter-stream.jp/interstream_support/ems/08_05.html)
  * bun33s.mp4 (h264)

## How to build?

### FFMPEG and OpenCV

Go to [FFMPEG](..).
Edit [build-config.json](../build-config.json) and enable `--enable-libopenh264` before start build.
And build opencv after complete building FFMPEG.

````shell
$ pwsh build-opencv.ps1 <Debug/Release>
````

For windows, you need to kick [download-pkg-config.ps1](./download-pkg-config.ps1) to use `FFMPEG` for cmake project.

Last,

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

### Windows

````bat
$ .\install\win\bin\Demo.exe bun33s.mp4
````

### Linux

````shell
$ LD_LIBRARY_PATH=./install/linux/Release/bin ./install/linux/Release/bin/Demo ./install/linux/Release/bin/bun33s.mp4 
````

### OSX

````shell
$ DYLD_LIBRARY_PATH=./install/osx/Release/bin ./install/osx/Release/bin/Demo ./install/osx/Release/bin/bun33s.mp4 
````