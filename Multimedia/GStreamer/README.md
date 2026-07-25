# GStreamer

## Abstracts

* Build GStreamer

## Requirements

### Common

* Powershell 7 or later
* Python3

### Windows

* Visual Studio 2022

##### Amazon Kinesis Video Streams C++ Producer

For Windows user, you SHALL move this repo to shallow directory!!
Otherwise, you would face to build error due to long path issue.
Refere to https://developercommunity.visualstudio.com/t/Allow-building-running-and-debugging-a/351628

* Perl
* Support long path (more than 260)
  * OS
    * `HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem\LongPathsEnabled`
  * git
    * `git config --global core.longpaths true`

### Ubuntu

* g++
* Packages
  * `flex`
  * `bison`
  * `nasm`

### OSX

* Xcode

## Dependencies

* [GStreamer](https://gstreamer.freedesktop.org/)
  * 1.28.2
    * Build error 'ERROR: Requested variable "plugins_cache_generator" not found' occurs since 1.28.3
  * GNU General Public License (GPL) version 2.1
* [Amazon Kinesis Video Streams C++ Producer, kvssink GStreamer Plugin](https://github.com/awslabs/amazon-kinesis-video-streams-producer-sdk-cpp/)
  * 3.6.0
  * Apache-2.0 license

### Dependencies for development

##### Windows

* [pkg-config](https://download.gnome.org)
  * 0.23-2
  * GNU General Public License
* [glib](https://download.gnome.org)
  * 2.26.1-1
  * GNU Lesser General Public License 2.01
* [gettext](https://download.gnome.org)
  * 0.18.1.1-2
  * GNU General Public License

## How to use?

````shell
$ pwsh build.ps1 <Debug/Release>
````

For windows, you need to kick [download-pkg-config.ps1](./download-pkg-config.ps1) to use `gstreamer` for cmake project.