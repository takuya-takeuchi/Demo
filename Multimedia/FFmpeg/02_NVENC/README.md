# Use OpenCV to decode h264 by FFMPEG backend with libopenh264

## Abstracts

* Encode by `NVENC`

## Requirements

### Windows

* N/A

### Linux

* N/A

### OSX

* Not supported

## Dependencies

* N/A

## How to build?

### FFMPEG and OpenCV

Go to [FFMPEG](..).
Edit [build-config.json](../build-config.json) and enable `--enable-cuda-llvm`, `--enable-nvenc` and `--enable-cuvid` before start build.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

### Windows

````bat
$ set OPENCV_FFMPEG_CAPTURE_OPTIONS="video_codec;libopenh264" 
$ \install\win\Release\bin\Demo.exe .\install\win\Release\bin\bun33s.mp4
[Info] Video has finished.
[Info] Total frame count: 812
````

### Linux

This is our environmentals.

````bash
$ nvidia-smi
Sat Aug 29 20:39:02 2026       
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 580.173.02             Driver Version: 580.173.02     CUDA Version: 13.0     |
+-----------------------------------------+------------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M. |
|                                         |                        |               MIG M. |
|=========================================+========================+======================|
|   0  Quadro M6000 24GB              Off |   00000000:23:00.0 Off |                  Off |
| 25%   43C    P8             17W /  250W |       4MiB /  24576MiB |      0%      Default |
|                                         |                        |                  N/A |
+-----------------------------------------+------------------------+----------------------+

+-----------------------------------------------------------------------------------------+
| Processes:                                                                              |
|  GPU   GI   CI              PID   Type   Process name                        GPU Memory |
|        ID   ID                                                               Usage      |
|=========================================================================================|
|  No running processes found                                                             |
+-----------------------------------------------------------------------------------------+
````

First, launch `nvidia-smi` to check usage of gpu

````bash
$  nvidia-smi dmon -s u
````

##### Encode

````shell
$ export LD_LIBRARY_PATH=../install/linux/ffmpeg/n8.1/dynamic/Release/lib
$ ../install/linux/ffmpeg/n8.1/dynamic/Release/bin/ffmpeg -f lavfi -i testsrc2=size=1920x1080:rate=30 -t 30 \
                                                          -pix_fmt yuv420p -c:v h264_nvenc -y test.mp4

ffmpeg version n8.1 Copyright (c) 2000-2026 the FFmpeg developers
  built with gcc 15 (Ubuntu 15.2.0-16ubuntu1)
  configuration: --enable-optimizations --disable-debug --enable-shared --disable-static --disable-logging --fatal-warnings --enable-pic --disable-doc --disable-htmlpages --disable-manpages --disable-podpages --disable-txtpages --disable-gpl --enable-cuda-llvm --enable-nvenc --enable-cuvid --prefix=/data/work/oss/Demo/Multimedia/FFmpeg/install/linux/ffmpeg/n8.1/dynamic/Release
  libavutil      60. 26.100 / 60. 26.100
  libavcodec     62. 28.100 / 62. 28.100
  libavformat    62. 12.100 / 62. 12.100
  libavdevice    62.  3.100 / 62.  3.100
  libavfilter    11. 14.100 / 11. 14.100
  libswscale      9.  5.100 /  9.  5.100
  libswresample   6.  3.100 /  6.  3.100
Input #0, lavfi, from 'testsrc2=size=1920x1080:rate=30':
  Duration: N/A, start: 0.000000, bitrate: N/A
  Stream #0:0: Video: wrapped_avframe, yuv420p, 1920x1080 [SAR 1:1 DAR 16:9], 30 fps, 30 tbr, 30 tbn
Stream mapping:
  Stream #0:0 -> #0:0 (wrapped_avframe (native) -> h264 (h264_nvenc))
Press [q] to stop, [?] for help
Output #0, mp4, to 'test.mp4':
  Metadata:
    encoder         : Lavf62.12.100
  Stream #0:0: Video: h264 (Main) (avc1 / 0x31637661), yuv420p(tv, progressive), 1920x1080 [SAR 1:1 DAR 16:9], q=2-31, 2000 kb/s, 30 fps, 15360 tbn
    Metadata:
      encoder         : Lavc62.28.100 h264_nvenc
    Side data:
      CPB properties: bitrate max/min/avg: 0/0/2000000 buffer size: 4000000 vbv_delay: N/A
[out#0/mp4 @ 0x6141f38dcd80] video:7899KiB audio:0KiB subtitle:0KiB other streams:0KiB global headers:0KiB muxing overhead: 0.145659%
frame=  900 fps=196 q=37.0 Lsize=    7911KiB time=00:00:29.90 bitrate=2167.4kbits/s speed=6.52x elapsed=0:00:04.58
````

When running `ffmpeg`, you will see stats of gpu like that

````bash
$  nvidia-smi dmon -s u
# gpu     sm    mem    enc    dec    jpg    ofa 
# Idx      %      %      %      %      %      % 
    0      0      0      0      0      -      - 
    0      0      0      0      0      -      - 
    0      6      4     11      0      -      - 
    0      6      4     23      0      -      - 
    0      7      4     31      0      -      - 
# gpu     sm    mem    enc    dec    jpg    ofa 
# Idx      %      %      %      %      %      % 
    0      6      4     36      0      -      - 
    0      0      0     33      0      -      - 
    0      0      0     23      0      -      - 
    0      0      0     16      0      -      - 
    0      0      0     11      0      -      - 
    0      0      0      8      0      -      - 
    0      0      0      5      0      -      - 
    0      0      0      4      0      -      - 
    0      0      0      3      0      -      - 
    0      0      0      2      0      -      - 
    0      0      0      1      0      -      -
````

##### Decode

````shell
$ export LD_LIBRARY_PATH=../install/linux/ffmpeg/n8.1/dynamic/Release/lib
$ ../install/linux/ffmpeg/n8.1/dynamic/Release/bin/ffmpeg -hwaccel cuda -hwaccel_output_format cuda \
                                                          -i test.mp4 -an -f null -

ffmpeg version n8.1 Copyright (c) 2000-2026 the FFmpeg developers
  built with gcc 15 (Ubuntu 15.2.0-16ubuntu1)
  configuration: --enable-optimizations --disable-debug --enable-shared --disable-static --disable-logging --fatal-warnings --enable-pic --disable-doc --disable-htmlpages --disable-manpages --disable-podpages --disable-txtpages --disable-gpl --enable-cuda-llvm --enable-nvenc --enable-cuvid --prefix=/data/work/oss/Demo/Multimedia/FFmpeg/install/linux/ffmpeg/n8.1/dynamic/Release
  libavutil      60. 26.100 / 60. 26.100
  libavcodec     62. 28.100 / 62. 28.100
  libavformat    62. 12.100 / 62. 12.100
  libavdevice    62.  3.100 / 62.  3.100
  libavfilter    11. 14.100 / 11. 14.100
  libswscale      9.  5.100 /  9.  5.100
  libswresample   6.  3.100 /  6.  3.100
Input #0, mov,mp4,m4a,3gp,3g2,mj2, from 'test.mp4':
  Metadata:
    major_brand     : isom
    minor_version   : 512
    compatible_brands: isomiso2avc1mp41
    encoder         : Lavf62.12.100
  Duration: 00:00:30.00, start: 0.000000, bitrate: 2160 kb/s
  Stream #0:0[0x1](und): Video: h264 (Main) (avc1 / 0x31637661), yuv420p(progressive), 1920x1080 [SAR 1:1 DAR 16:9], 2156 kb/s, 30 fps, 30 tbr, 15360 tbn (default)
    Metadata:
      handler_name    : VideoHandler
      encoder         : Lavc62.28.100 h264_nvenc
Stream mapping:
  Stream #0:0 -> #0:0 (h264 (native) -> wrapped_avframe (native))
Press [q] to stop, [?] for help
Output #0, null, to 'pipe:':
  Metadata:
    major_brand     : isom
    minor_version   : 512
    compatible_brands: isomiso2avc1mp41
    encoder         : Lavf62.12.100
  Stream #0:0(und): Video: wrapped_avframe, cuda(progressive), 1920x1080 [SAR 1:1 DAR 16:9], q=2-31, 200 kb/s, 30 fps, 30 tbn (default)
    Metadata:
      encoder         : Lavc62.28.100 wrapped_avframe
      handler_name    : VideoHandler
[out#0/null @ 0x55b35a0cc580] video:373KiB audio:0KiB subtitle:0KiB other streams:0KiB global headers:0KiB muxing overhead: unknown
frame=  900 fps=397 q=-0.0 Lsize=N/A time=00:00:30.00 bitrate=N/A speed=13.2x elapsed=0:00:02.26
````

When running `ffmpeg`, you will see stats of gpu like that

````bash
$ nvidia-smi dmon -s u
# gpu     sm    mem    enc    dec    jpg    ofa 
# Idx      %      %      %      %      %      % 
    0      0      0      0      0      -      - 
    0      0      0      0      0      -      - 
    0      2      5      0     25      -      - 
    0      3      6      0     48      -      - 
    0      0      0      0     36      -      - 
    0      0      0      0     25      -      - 
    0      0      0      0     17      -      - 
    0      0      0      0     12      -      - 
    0      0      0      0      8      -      - 
    0      0      0      0      6      -      - 
    0      0      0      0      4      -      - 
    0      0      0      0      3      -      - 
    0      0      0      0      2      -      -
````