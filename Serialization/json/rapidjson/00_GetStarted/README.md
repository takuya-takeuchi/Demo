# Get started

## Abstracts

* How to read json file

## Requirements

### Common

* Powershell 7 or later
* CMake
  * 3.0 or higher

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [rapidjson](https://github.com/Tencent/rapidjson)
  * 1.1.0
  * MIT license

## How to build?

### rapidjson

Go to [rapidjson](..).

Once time you built `rapidjson`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ .\install\win\bin\Demo.exe testdata\test.json
[Info] json_path: testdata\test.json
[Info]           name: test
[Info]           year: 2024
[Info]        version: 1.2
[Info]          error: false
[Info]   dependencies:
[Info]           - a.lib
[Info]           - b.lib
````

#### Linux

````bat
$ ./install/linux/bin/Demo testdata/test.json 
[Info] json_path: testdata/test.json
[Info]           name: test
[Info]           year: 2024
[Info]        version: 1.2
[Info]          error: false
[Info]   dependencies: 
[Info]           - a.lib
[Info]           - b.lib
````

#### OSX

````shell
$ ./install/osx/bin/Demo testdata/test.json 
[Info] json_path: testdata/test.json
[Info]           name: test
[Info]           year: 2024
[Info]        version: 1.2
[Info]          error: false
[Info]   dependencies: 
[Info]           - a.lib
[Info]           - b.lib
````