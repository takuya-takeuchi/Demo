# Publish vidoe stream to AWS KVS

## Abstracts

* How to publis video stream from mp4 file

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
* [Amazon Kinesis Video Streams C++ Producer, kvssink GStreamer Plugin](https://github.com/awslabs/amazon-kinesis-video-streams-producer-sdk-cpp/)
  * Apache-2.0 license

## TestData

* [sample-5s.mp4](./sample-5s.mp4)
  * [Test video of a road in a city](https://samplelib.com/sample-mp4.html)
  * License: [License](https://samplelib.com/license.html)

## How to build?

### GStreamer

Go to [GStreamer](..).

Once time you built `GStreamer (with some plugins)` and `Amazon Kinesis Video Streams C++ Producer, kvssink GStreamer Plugin`, you need not to do again.

````shell
$ pwsh build-kvs.ps1 <Debug/Release>
$ pwsh build-amazon-kinesis-video-streams-producer-sdk-cpp.ps1 <Debug/Release>
````

## How to test?

At first, you need to create KVS stream.


````bat
$ aws kinesisvideo create-stream --stream-name "MyKVSStream" --data-retention-in-hours 1 --profile <your-profile-name>
{                                                                                                                                                                  
    "StreamARN": "arn:aws:kinesisvideo:ap-northeast-1:111111111111:stream/MyKVSStream/1234567890123"
}
````

#### Windows

````bat
$ set AWS_ACCESS_KEY_ID=<your-aws-access-key>
$ set AWS_SECRET_ACCESS_KEY=<your-aws-secret-access-key>
$ set AWS_DEFAULT_REGION=<your-kvs-region>

$ pwsh run.ps1 Release MyKVSStream
````

#### Linux

````bash
$ export AWS_ACCESS_KEY_ID=<your-aws-access-key>
$ export AWS_SECRET_ACCESS_KEY=<your-aws-secret-access-key>
$ export AWS_DEFAULT_REGION=<your-kvs-region>

$ pwsh run.ps1 Release MyKVSStream
Launching pipeline: filesrc location="sample-5s.mp4" ! qtdemux name=demux demux.video_0 ! queue ! h264parse ! openh264dec ! videoconvert ! video/x-raw,format=I420 ! openh264enc rate-control=bitrate bitrate=2000000 gop-size=60 ! h264parse config-interval=-1 ! video/x-h264,stream-format=avc,alignment=au ! identity sync=true ! kvssink stream-name="MyKVSStream" log-config="./kvs_log_configuration" framerate=30 fragment-duration=2000
log4cplus:ERROR PropertyConfigurator::configureLogger()- Invalid appender: WARN
2026-07-18 18:17:32 INFO  - Logger config being used: ./kvs_log_configuration
2026-07-18 18:17:32 INFO  - User agent string: AWS-SDK-KVS-CPP-CLIENT/3.6.0
2026-07-18 09:17:32.429 INFO    createKinesisVideoClient(): Creating Kinesis Video Client
2026-07-18 18:17:32 INFO  - heapInitialize(): Initializing native heap with limit size 134217728, spill ratio 0% and flags 0x00000001
2026-07-18 18:17:32 INFO  - heapInitialize(): Creating AIV heap.
2026-07-18 18:17:32 INFO  - heapInitialize(): Heap is initialized OK
2026-07-18 18:17:32 INFO  - createDeviceResultEvent(): Create device result event.
2026-07-18 18:17:32 INFO  - GStreamer caps: video/x-h264, stream-format=(string)avc, alignment=(string)au, width=(int)[ 16, 2147483647 ], height=(int)[ 16, 2147483647 ], parsed=(boolean)true
2026-07-18 18:17:32 INFO  - Try creating stream for MyKVSStream
2026-07-18 18:17:32 INFO  - Creating Kinesis Video Stream MyKVSStream
2026-07-18 18:17:32 INFO  - createKinesisVideoStream(): Creating Kinesis Video Stream.
2026-07-18 18:17:32 INFO  - logStreamInfo(): SDK version: a6efdf5d2fe1fcc5be85048a0c1b6a90ab80ff72
2026-07-18 18:17:32 INFO  - writeHeaderCallback(): RequestId: a5c32946-c40f-4c9a-a9af-90b588457bb9
2026-07-18 18:17:32 INFO  - describeStreamResultEvent(): Describe stream result event.
2026-07-18 18:17:32 WARN  - describeStreamResult(): [MyKVSStream] Retention period (1640261632) returned from the DescribeStream call doesn't match the one specified in the StreamInfo: (-1014444032)
2026-07-18 18:17:32 WARN  - describeStreamResult(): [MyKVSStream] Content type (1648407494) returned from the DescribeStream call doesn't match the one specified in the StreamInfo: (-654182756)
2026-07-18 18:17:32 INFO  - writeHeaderCallback(): RequestId: 072b121d-6a00-44d5-b31d-90c584875bf4
2026-07-18 18:17:32 INFO  - getStreamingEndpointResultEvent(): Get streaming endpoint result event.
2026-07-18 18:17:32 INFO  - getStreamingTokenResultEvent(): Get streaming token result event.
Stream is ready
Streaming to KVS stream: MyKVSStream...
2026-07-18 18:17:32 INFO  - kinesisVideoStreamFormatChanged(): Stream format changed.
2026-07-18 18:17:32 INFO  - putStreamResultEvent(): Put stream result event. New upload handle 0
2026-07-18 18:17:32 WARN  - notifyDataAvailable(): [MyKVSStream] Failed to un-pause curl with error: 43. Curl object 0x7eb65c479a20
2026-07-18 18:17:33 INFO  - writeHeaderCallback(): RequestId: f9c8733c-f400-d52f-9b1d-35d0e8b8f487
2026-07-18 18:17:46 INFO  - EOS Event received in sink for MyKVSStream
2026-07-18 18:17:46 INFO  - Sending EOFR for MyKVSStream
2026-07-18 18:17:46 WARN  - GStreamer buffer is invalid for MyKVSStream
2026-07-18 18:17:46 INFO  - Stopping kvssink for MyKVSStream
2026-07-18 18:17:46 INFO  - stopKinesisVideoStreamSync(): Synchronously stopping Kinesis Video Stream 0000600ed901d5d0.
2026-07-18 18:17:46 INFO  - getStreamData(): [MyKVSStream] Handle 0 waiting for last persisted ack with ts 17843662569070000
2026-07-18 18:17:46 INFO  - getStreamData(): [MyKVSStream] Indicating an EOS after last persisted ACK is received for stream upload handle 0
2026-07-18 18:17:46 INFO  - postReadCallback(): Reported end-of-stream for stream MyKVSStream. Upload handle: 0
2026-07-18 18:17:46 INFO  - Stopped kvssink for MyKVSStream
2026-07-18 18:17:46 INFO  - Pipeline state changed to NULL in kvssink
Streaming session ended
````

#### OSX

````bash
$ export AWS_ACCESS_KEY_ID=<your-aws-access-key>
$ export AWS_SECRET_ACCESS_KEY=<your-aws-secret-access-key>
$ export AWS_DEFAULT_REGION=<your-kvs-region>

$ pwsh run.ps1 Release MyKVSStream
Launching pipeline: filesrc location="sample-5s.mp4" ! qtdemux name=demux demux.video_0 ! queue ! h264parse ! openh264dec ! videoconvert ! video/x-raw,format=I420 ! openh264enc rate-control=bitrate bitrate=2000000 gop-size=60 ! h264parse config-interval=-1 ! video/x-h264,stream-format=avc,alignment=au ! identity sync=true ! kvssink stream-name="MyKVSStream" log-config="./kvs_log_configuration" framerate=30 fragment-duration=2000
log4cplus:ERROR PropertyConfigurator::configureLogger()- Invalid appender: WARN
2026-07-18 18:56:40 INFO  - Logger config being used: ./kvs_log_configuration
2026-07-18 18:56:40 INFO  - User agent string: AWS-SDK-KVS-CPP-CLIENT/3.6.0
2026-07-18 09:56:40.299 INFO    createKinesisVideoClient(): Creating Kinesis Video Client
2026-07-18 18:56:40 INFO  - heapInitialize(): Initializing native heap with limit size 134217728, spill ratio 0% and flags 0x00000001
2026-07-18 18:56:40 INFO  - heapInitialize(): Creating AIV heap.
2026-07-18 18:56:40 INFO  - heapInitialize(): Heap is initialized OK
2026-07-18 18:56:40 INFO  - createDeviceResultEvent(): Create device result event.
2026-07-18 18:56:40 INFO  - GStreamer caps: video/x-h264, stream-format=(string)avc, alignment=(string)au, width=(int)[ 16, 2147483647 ], height=(int)[ 16, 2147483647 ], parsed=(boolean)true
2026-07-18 18:56:40 INFO  - Try creating stream for MyKVSStream
2026-07-18 18:56:40 INFO  - Creating Kinesis Video Stream MyKVSStream
2026-07-18 18:56:40 INFO  - createKinesisVideoStream(): Creating Kinesis Video Stream.
2026-07-18 18:56:40 INFO  - logStreamInfo(): SDK version: a6efdf5d2fe1fcc5be85048a0c1b6a90ab80ff72
2026-07-18 18:56:40 INFO  - writeHeaderCallback(): RequestId: e2f545c1-2647-44d2-a6ed-df6dbb957d9e
2026-07-18 18:56:40 INFO  - describeStreamResultEvent(): Describe stream result event.
2026-07-18 18:56:40 WARN  - describeStreamResult(): [MyKVSStream] Retention period (1640261632) returned from the DescribeStream call doesn't match the one specified in the StreamInfo: (-1014444032)
2026-07-18 18:56:40 WARN  - describeStreamResult(): [MyKVSStream] Content type (1845970150) returned from the DescribeStream call doesn't match the one specified in the StreamInfo: (-1797582372)
2026-07-18 18:56:40 INFO  - writeHeaderCallback(): RequestId: eee2b92f-58e0-4857-970a-7e4d28fe829d
2026-07-18 18:56:40 INFO  - getStreamingEndpointResultEvent(): Get streaming endpoint result event.
2026-07-18 18:56:40 INFO  - getStreamingTokenResultEvent(): Get streaming token result event.
Stream is ready
0:00:01.740248209      31397      0x1f985dd80 WARN                pipeline gstpipeline.c:753:gst_pipeline_do_latency:<pipeline0> did not really configure latency of 0:00:00.000000000
Streaming to KVS stream: MyKVSStream...
0:00:01.743621084      31397      0x16e103000 FIXME           videodecoder gstvideodecoder.c:1196:gst_video_decoder_drain_out:<openh264dec0> Sub-class should implement drain()
2026-07-18 18:56:40 INFO  - kinesisVideoStreamFormatChanged(): Stream format changed.
2026-07-18 18:56:40 INFO  - putStreamResultEvent(): Put stream result event. New upload handle 0
2026-07-18 18:56:40 WARN  - notifyDataAvailable(): [MyKVSStream] Failed to un-pause curl with error: 43. Curl object 0x894df1c00
2026-07-18 18:56:46 INFO  - EOS Event received in sink for MyKVSStream
2026-07-18 18:56:46 INFO  - Sending EOFR for MyKVSStream
2026-07-18 18:56:46 WARN  - GStreamer buffer is invalid for MyKVSStream
2026-07-18 18:56:46 INFO  - Stopping kvssink for MyKVSStream
2026-07-18 18:56:46 INFO  - stopKinesisVideoStreamSync(): Synchronously stopping Kinesis Video Stream 00000008950d6a30.
2026-07-18 18:56:46 INFO  - getStreamData(): [MyKVSStream] Handle 0 waiting for last persisted ack with ts 17843686046130000
2026-07-18 18:56:46 INFO  - writeHeaderCallback(): RequestId: d3166ad3-6b93-255b-b1c3-3acb133663d4
2026-07-18 18:56:46 INFO  - getStreamData(): [MyKVSStream] Indicating an EOS after last persisted ACK is received for stream upload handle 0
2026-07-18 18:56:46 INFO  - postReadCallback(): Reported end-of-stream for stream MyKVSStream. Upload handle: 0
2026-07-18 18:56:46 INFO  - Stopped kvssink for MyKVSStream
2026-07-18 18:56:46 INFO  - Pipeline state changed to NULL in kvssink
Streaming session ended.
````