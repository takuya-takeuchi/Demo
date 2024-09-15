# Network Camer capture

## Abstracts

* Use OpenCvSharp to capture network camera.
* Check issue [VideoCapture(rtsp) work incorrectly in linux #1053](https://github.com/shimat/opencvsharp/issues/1053) is wrong.
  * Default OpenCVSharp can connect to network camera by using FFMPEG backend for linux.
    * You must install some package
    * You need not to rebuild OpenCvSharp and install gstreamer!!

## Requirements

### Common

* .NET SDK 8

### Windows

* N/A

### Ubuntu

* Ubunbut 20.04
  * tesseract-ocr libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev libopenexr-dev

## Dependencies

* [OpenCvSharp4](https://github.com/shimat/opencvsharp)
  * 4.9.0.20240103
  * Apache-2.0 License
* [OpenCvSharp4.runtime.win](https://github.com/shimat/opencvsharp)
  * 4.9.0.20240103
  * Apache-2.0 License
* [OpenCvSharp4_.runtime.ubuntu.20.04-x64](https://github.com/shimat/opencvsharp)
  * 4.9.0.20240103
  * Apache-2.0 License  

## How to use?

````bash
$ cd sources/Demo
$ dotnet run -c Release -- http://root:password@192.168.11.40/axis-cgi/mjpg/video.cgi FFMPEG
2024-09-13 23:37:05.1107 [INFO ]     Url: http://root:password@192.168.11.40/axis-cgi/mjpg/video.cgi 
2024-09-13 23:37:05.1527 [INFO ] Backend: FFMPEG 
2024-09-13 23:37:05.4760 [INFO ] Available frame: 1920x1080 
2024-09-13 23:37:05.5081 [INFO ] Available frame: 1920x1080 
2024-09-13 23:37:05.5315 [INFO ] Available frame: 1920x1080 
````

````bash
$ cd sources/Demo
$ dotnet run -c Release -- rtsp://root:password@192.168.11.40/axis-media/media.amp?videocodec=h264 FFMPEG
2024-09-13 23:29:39.5578 [INFO ]     Url: rtsp://root:password@192.168.11.40/axis-media/media.amp?videocodec=h264 
2024-09-13 23:29:39.6110 [INFO ] Backend: FFMPEG 
2024-09-13 23:29:40.8943 [INFO ] Available frame: 1920x1080 
2024-09-13 23:29:40.9042 [INFO ] Available frame: 1920x1080 
2024-09-13 23:29:40.9061 [INFO ] Available frame: 1920x1080 
````