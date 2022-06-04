# Onvif

* Generate onvif class definitions

## Requirements

* Visual Studio 2019 16.5.4 or later

## How to build

### Preparation

````shell
dotnet tool install --global dotnet-svcutil
dotnet svcutil https://raw.githubusercontent.com/onvif/specs/21.12/wsdl/ver10/media/wsdl/media.wsdl -n "*,Onvif.Media" -o Service\Media.cs
dotnet svcutil https://raw.githubusercontent.com/onvif/specs/21.12/wsdl/ver10/device/wsdl/devicemgmt.wsdl -n "*,Onvif.DeviceManagement" -o Service\DeviceManagement.cs
````

### Build and Run

The following command demonstates how to connect to axis camera.

````shell
dotnet run -c Release -- http://192.168.11.40/onvif/services root password
[Info]      Url: http://192.168.11.40/onvif/services
[Info]     User: root
[Info] Password: password

[Info] Invoke GetSystemDateAndTimeAsync
[Info]  2021/11/19 02:18:42.000

[Info] Invoke GetCapabilitiesAsync
[Info]  Media.XAddr: http://192.168.11.40/onvif/services
[Info]              StreamingCapabilities
[Info]                       RTPMulticast: True
[Info]                       RTP_RTSP_TCP: True
[Info]                            RTP_TCP: True
[Info]              RTPMulticastSpecified: True
[Info]              RTP_RTSP_TCPSpecified: True
[Info]                   RTP_TCPSpecified: True

[Info] Invoke GetProfilesAsync
[Info]        Name: profile_1 h264
[Info]               token: profile_1_h264
[Info]        Name: profile_1 jpeg
[Info]               token: profile_1_jpeg

[Info] Invoke GetSnapshotUriAsync
[Info]         Uri: http://192.168.11.40/onvif-cgi/jpg/image.cgi?resolution=1920x1080&compression=30

[Info] Invoke GetStreamUriAsync
[Info]         Uri: http://192.168.11.40/onvif-media/media.amp?profile=profile_1_h264&sessiontimeout=60&streamtype=unicast
````