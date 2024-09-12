

````bash
$ dotnet Demo.dll http://root:password@192.168.11.40/axis-cgi/mjpg/video.cgi
[ WARN:0@1.871] global cap_gstreamer.cpp:1777 open OpenCV | GStreamer warning: Cannot query video position: status=1, value=-1, duration=-1
2024-09-13 00:02:48.0711 [INFO ] Available frame: 1920x1080 
2024-09-13 00:02:48.0854 [INFO ] Available frame: 1920x1080 
2024-09-13 00:02:48.0916 [INFO ] Available frame: 1920x1080 
2024-09-13 00:02:48.0988 [INFO ] Available frame: 1920x1080 
````

````bash
sudo apt install gstreamer1.0-rtsp
dotnet Demo.dll rtsp://root:password@192.168.11.40/axis-media/media.amp?videocodec=h264
2024-09-13 00:28:10.0392 [INFO ] Url: rtsp://root:password@192.168.11.40/axis-media/media.amp?videocodec=h264 
[ WARN:0@0.377] global cap_gstreamer.cpp:1750 open OpenCV | GStreamer warning: frame count is estimated by duration and fps
2024-09-13 00:28:10.5174 [INFO ] Available frame: 1920x1080 
2024-09-13 00:28:10.5322 [INFO ] Available frame: 1920x1080 
2024-09-13 00:28:10.5366 [INFO ] Available frame: 1920x1080 
2024-09-13 00:28:10.5472 [INFO ] Available frame: 1920x1080 
````