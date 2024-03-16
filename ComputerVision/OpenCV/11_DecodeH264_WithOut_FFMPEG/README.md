# Resize image with padding

## Abstracts

* Play h264 movie file by opencv without ffmpeg backend
  * Use OS decoder to avoid patent violation 

## Requirements

### Common

* Powershell 7 or later
* CMake 3.0.0 or later
* C++ Compiler supports C++17

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [opencv](https://github.com/opencv/opencv)
  * 4.7.0
  * Apache-2.0 License

## Test Data

* [INTER-STREAM®サポートページ](https://inter-stream.jp/interstream_support/ems/08_05.html)
  * bun33s.mp4

## How to build?

### OpenCV 4

Go to [OpenCV](..).

````shell
$ pwsh build-disable_ffmpeg.ps1 <Debug/Release>
````

Once time you built `opencv4`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

````bat
$ .\install\win\bin\Demo.exe bun33s.mp4
[Info] Succeeded to fail opening 'bun33s.mp4' by using ffmpeg backend.
[Info] Succeeded to open'bun33s.mp4' by using Any backend.
[Info] Succeeded to read frame from 'bun33s.mp4' by using Any backend.
[Info] Succeeded to open'bun33s.mp4' by using Microsoft Media Foundation backend.
[Info] Succeeded to read frame from 'bun33s.mp4' by using Microsoft Media Foundation backend.
[Warning] Failed to find 'bun33s.mp4' by using DirectShow backend. But System does not have proper codec to play video for DirectShow.
````