# Streaming mp4 file as RTSP

## Abstracts

* Use MediaMTX and FFmpeg to stream mp4 file as RTSP

## Requirements

### Common

* Powershell 7 or later

## Dependencies

* [FFmpeg](https://www.ffmpeg.org/)
  * 7.0.2
  * GNU Lesser General Public License/GNU General Public License
  * Prebuid binaries are from [FFmpeg-Builds](https://github.com/BtbN/FFmpeg-Builds)
* [MediaMTX](https://github.com/bluenviron/mediamtx)
  * v1.9.0
  * MIT license

#### Data

* [Sample MP4 video files](https://samplelib.com/sample-mp4.html)
  * No license
  * [sample-30s.mp4](https://download.samplelib.com/mp4/sample-30s.mp4)

## How to install?

At first, you must kick the following commands on [MediaMTX](..).

````shell
$ pwsh download-ffmpeg.ps1
$ pwsh donwload.ps1
````

Then, run this command

````shell
$ pwsh run.ps1

2024/09/15 04:44:05 INF MediaMTX v1.9.0
2024/09/15 04:44:05 INF configuration loaded from D:\Works\OpenSource\Demo\Multimedia\MediaMTX\01_StreamingServer\mediamtx.yml
2024/09/15 04:44:05 INF [path mystream] runOnInit command started
2024/09/15 04:44:05 INF [RTSP] listener opened on :12345 (TCP), :8000 (UDP/RTP), :8001 (UDP/RTCP)
2024/09/15 04:44:05 INF [RTMP] listener opened on :1935
2024/09/15 04:44:05 INF [HLS] listener opened on :8888
2024/09/15 04:44:05 INF [WebRTC] listener opened on :8889 (HTTP), :8189 (ICE/UDP)
2024/09/15 04:44:05 INF [SRT] listener opened on :8890 (UDP)
2024/09/15 04:44:05 INF [RTSP] [conn [::1]:64622] opened
2024/09/15 04:44:05 INF [RTSP] [session 50d5f383] created by [::1]:64622
2024/09/15 04:44:05 INF [RTSP] [session 50d5f383] is publishing to path 'mystream', 2 tracks (H264, MPEG-4 Audio)
````

And you can listen by media player (e.g. VLC).

<img src="./images/movie.gif" />