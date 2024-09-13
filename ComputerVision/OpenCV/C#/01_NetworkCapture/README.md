# Network Camer capture

## Abstracts

* Use OpenCvSharp to capture network camera.
* Check issue [VideoCapture(rtsp) work incorrectly in linux #1053](https://github.com/shimat/opencvsharp/issues/1053) is wrong.
  * Default OpenCVSharp can connect to network camera by using FFMPEG backend.

## Requirements

### Common

* .NET SDK 8

### Windows

* N/A

### Ubuntu

* g++
* docker
  * for building WASM

### OSX

* Xcode
* docker
  * for building WASM

## Dependencies

* [opencv](https://github.com/opencv/opencv)
  * 4.7.0
  * Apache-2.0 License

## How to use?

````shell
$ git submodule update --init --recursive .
$ pwsh build.ps1  <Debug/Release>
````

Then you can try samples. For example [10_ResizeWithPadding](./10_ResizeWithPadding).


````bash
$ dotnet run -c Release -- http://root:password@192.168.11.40/axis-cgi/mjpg/video.cgi FFMPEG
2024-09-13 23:37:05.1107 [INFO ]     Url: http://root:password@192.168.11.40/axis-cgi/mjpg/video.cgi 
2024-09-13 23:37:05.1527 [INFO ] Backend: FFMPEG 
2024-09-13 23:37:05.4760 [INFO ] Available frame: 1920x1080 
2024-09-13 23:37:05.5081 [INFO ] Available frame: 1920x1080 
2024-09-13 23:37:05.5315 [INFO ] Available frame: 1920x1080 
````

````bash
dotnet Demo.dll rtsp://root:password@192.168.11.40/axis-media/media.amp?videocodec=h264 FFMPEG
2024-09-13 23:29:39.5578 [INFO ]     Url: rtsp://root:password@192.168.11.40/axis-media/media.amp?videocodec=h264 
2024-09-13 23:29:39.6110 [INFO ] Backend: FFMPEG 
2024-09-13 23:29:40.8943 [INFO ] Available frame: 1920x1080 
2024-09-13 23:29:40.9042 [INFO ] Available frame: 1920x1080 
2024-09-13 23:29:40.9061 [INFO ] Available frame: 1920x1080 
````