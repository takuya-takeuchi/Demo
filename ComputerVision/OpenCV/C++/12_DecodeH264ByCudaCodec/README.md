# Decode H.264 by NVIDIA Gpu

## Abstracts

* Decode H.264 file by using `NVIDIA VIDEO CODEC SDK`

## Requirements

### Common

* Powershell 7 or later
* CMake 3.0.0 or later
* C++ Compiler supports C++17
* CUDA 12.0

### Windows

* Visual Studio
* NVIDIA VIDEO CODEC SDK 12.1
  * Install all header files of NVIDIA VIDEO CODEC SDK into `include` of `CUDA_PATH`
  * Install all library files of NVIDIA VIDEO CODEC SDK into `lib` of `CUDA_PATH`

## Dependencies

* [opencv](https://github.com/opencv/opencv)
  * 4.7.0
  * Apache-2.0 License
* [opencv_contrib](https://github.com/opencv/opencv_contrib)
  * 4.7.0
  * Apache-2.0 License

## Test Data

* [INTER-STREAM®サポートページ](https://inter-stream.jp/interstream_support/ems/08_05.html)
  * bun33s.mp4 (h264)

## How to build?

### OpenCV 4

Go to [OpenCV](..).

````shell
$ pwsh build-enable-cudacodec.ps1 <Debug/Release>
````

Once time you built `opencv4`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

````bat
$ .\install\win\bin\Demo.exe bun33s.mp4
[Info] Available GPUs: 1
[Info]  width: 480
[Info] height: 272
[Info]    fps: 0
[Info]  codec: H264
[Info] frames: 812
[Info]     ms: 26 (per frame)
````