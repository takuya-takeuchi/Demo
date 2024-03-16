# Decode H.264 by not using ffmpeg

## Abstracts

* Play h264 movie file by opencv without ffmpeg backend
  * Use OS decoder to avoid patent violation
    * But I'm not sure this approach can avoid paying patent fee

## Requirements

### Common

* Powershell 7 or later
* CMake 3.0.0 or later
* C++ Compiler supports C++17

### Windows

* Visual Studio

### Ubuntu

* g++

:warning: I could not find a way to play h264 file by OS decoder so this program does not support on linux.

### OSX

* Xcode

## Dependencies

* [opencv](https://github.com/opencv/opencv)
  * 4.7.0
  * Apache-2.0 License

## Test Data

* [INTER-STREAM®サポートページ](https://inter-stream.jp/interstream_support/ems/08_05.html)
  * bun33s.mp4 (h264)

## How to build?

### OpenCV 4

Go to [OpenCV](..).

````shell
$ pwsh build-disable-ffmpeg.ps1 <Debug/Release>
````

Once time you built `opencv4`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

### Windows

Use `Microsoft Media Foundation` and `DirectShow` backend instead of `ffmpeg` backend.

````bat
$ .\install\win\bin\Demo.exe bun33s.mp4
[Info] Succeeded to fail opening 'bun33s.mp4' by using ffmpeg backend.
[Info] Succeeded to open'bun33s.mp4' by using Any backend.
[Info] Succeeded to read frame from 'bun33s.mp4' by using Any backend.
[Info] Succeeded to open'bun33s.mp4' by using Microsoft Media Foundation backend.
[Info] Succeeded to read frame from 'bun33s.mp4' by using Microsoft Media Foundation backend.
[Warning] Failed to find 'bun33s.mp4' by using DirectShow backend. But System does not have proper codec to play video for DirectShow.
````

### OSX

Use `AVFoundation` backend instead of `ffmpeg` backend.

````shell
$ ./install/osx/bin/Demo bun33s.mp4 
[Info] Succeeded to fail opening 'bun33s.mp4' by using ffmpeg backend.
[Info] Succeeded to open'bun33s.mp4' by using Any backend.
[Info] Succeeded to read frame from 'bun33s.mp4' by using Any backend.
[Info] Succeeded to open'bun33s.mp4' by using AVFoundation backend.
[Info] Succeeded to read frame from 'bun33s.mp4' by using AVFoundation backend.
````