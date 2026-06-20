# Recerive Stream

## Abstracts

* How to recerive stream like RTSP and persist every frame into storage as jpeg
  * And drop corrupted frame until key frame come.

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

Use ffmpeg and MediaMTX to stream vidoe file.
You can setup them by using scripts in [../../MediaMTX](../../MediaMTX).
Kick [../../MediaMTX/01_StreamingServer/run.ps1](../../MediaMTX/01_StreamingServer/run.ps1) after setup.

#### Windows

````bat
````bat
$ set GSTREAMER_VERSION=1.28.2
$ set GST_PLUGIN_SCANNER=..\install\win\gstreamer\%GSTREAMER_VERSION%\Release\libexec\gstreamer-1.0\gst-hotdoc-plugins-scanner.exe
$ set GST_PLUGIN_SYSTEM_PATH=..\install\win\gstreamer\%GSTREAMER_VERSION%\Release\lib\gstreamer-1.0
$ .\install\win\bin\Demo rtsp://192.168.11.102:12345/mystream
Pipeline: rtspsrc name=src location=rtsp://192.168.11.102:12345/mystream protocols=tcp latency=500 src. ! application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse name=parser config-interval=-1 ! openh264dec ! videoconvert ! jpegenc quality=100 ! appsink name=mysink emit-signals=true sync=false max-buffers=1 drop=true
Streaming started. Press Ctrl+C to stop.
Drop delta frame before first keyframe
Drop delta frame before first keyframe
...
Drop delta frame before first keyframe
Drop delta frame before first keyframe
Drop delta frame before first keyframe
First keyframe detected
on_new_sample
Saved: output/frame_0.jpg (1656972 bytes)
on_new_sample
Saved: output/frame_1.jpg (1684625 bytes)
on_new_sample
Saved: output/frame_2.jpg (1713791 bytes)
on_new_sample
Saved: output/frame_3.jpg (1703287 bytes)
on_new_sample
Saved: output/frame_4.jpg (1744020 bytes)
on_new_sample
Saved: output/frame_5.jpg (1706627 bytes)
on_new_sample
Saved: output/frame_6.jpg (1718862 bytes)
on_new_sample
Saved: output/frame_7.jpg (1713308 bytes)
````

#### Linux

````bat
$ export GSTREAMER_VERSION=1.28.2
$ export GST_PLUGIN_SCANNER=../install/linux/gstreamer/${GSTREAMER_VERSION}$/Release/libexec/gstreamer-1.0/gst-plugin-scanner
$ export GST_PLUGIN_SYSTEM_PATH=../install/linux/gstreamer/${GSTREAMER_VERSION}/Release/lib/x86_64-linux-gnu/gstreamer-1.0
$ export LD_LIBRARY_PATH=../install/linux/gstreamer/${GSTREAMER_VERSION}/Release/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
$ ./install/linux/bin/Demo rtsp://192.168.11.102:12345/mystream
Pipeline: rtspsrc name=src location=rtsp://192.168.11.102:12345/mystream protocols=tcp latency=500 src. ! application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse name=parser config-interval=-1 ! openh264dec ! videoconvert ! jpegenc quality=100 ! appsink name=mysink emit-signals=true sync=false max-buffers=1 drop=true
Streaming started. Press Ctrl+C to stop.
Drop delta frame before first keyframe
Drop delta frame before first keyframe
...
Drop delta frame before first keyframe
Drop delta frame before first keyframe
Drop delta frame before first keyframe
First keyframe detected
on_new_sample
Saved: output/frame_0.jpg (1670193 bytes)
on_new_sample
Saved: output/frame_1.jpg (1699509 bytes)
on_new_sample
Saved: output/frame_2.jpg (1724545 bytes)
on_new_sample
Saved: output/frame_3.jpg (1723146 bytes)
on_new_sample
Saved: output/frame_4.jpg (1776495 bytes)
on_new_sample
Saved: output/frame_5.jpg (1720442 bytes)
on_new_sample
Saved: output/frame_6.jpg (1735359 bytes)
on_new_sample
Saved: output/frame_7.jpg (1728795 bytes)
on_new_sample
````

#### OSX

````shell
$ export GSTREAMER_VERSION=1.28.2
$ export GST_PLUGIN_SCANNER=../install/osx/gstreamer/${GSTREAMER_VERSION}$/Release/libexec/gstreamer-1.0/gst-plugin-scanner
$ export GST_PLUGIN_SYSTEM_PATH=../install/osx/gstreamer/${GSTREAMER_VERSION}/Release/lib/gstreamer-1.0
$ export DYLD_LIBRARY_PATH=../install/osx/gstreamer/${GSTREAMER_VERSION}/Release/lib:$DYLD_LIBRARY_PATH
$ ./install/osx/bin/Demo rtsp://192.168.11.102:12345/mystream
Pipeline: rtspsrc name=src location=rtsp://192.168.11.102:12345/mystream protocols=tcp latency=500 src. ! application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse name=parser config-interval=-1 ! openh264dec ! videoconvert ! jpegenc quality=100 ! appsink name=mysink emit-signals=true sync=false max-buffers=1 drop=true
Streaming started. Press Ctrl+C to stop.
Drop delta frame before first keyframe
Drop delta frame before first keyframe
...
Drop delta frame before first keyframe
Drop delta frame before first keyframe
Drop delta frame before first keyframe
First keyframe detected
on_new_sample
Saved: output/frame_0.jpg (1702872 bytes)
on_new_sample
Saved: output/frame_1.jpg (1752651 bytes)
on_new_sample
Saved: output/frame_2.jpg (1787726 bytes)
on_new_sample
Saved: output/frame_3.jpg (1783853 bytes)
on_new_sample
Saved: output/frame_4.jpg (1838364 bytes)
on_new_sample
Saved: output/frame_5.jpg (1767013 bytes)
on_new_sample
Saved: output/frame_6.jpg (1791758 bytes)
on_new_sample
````