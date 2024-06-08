# Read json file with comment

## Abstracts

* How to read json file with comment
  * Original specification of json (RFC 4627) does not allow to use comment but JSON5 allow it. It is JSONC(JSON with comments) but there is no RFC.

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
````

#### Linux

````bat
$ ./install/linux/bin/Demo testdata/test.json 
[Info] json_path: testdata/test.json
[Info]           name: test
````

#### OSX

````shell
$ ./install/osx/bin/Demo testdata/test.json 
[Info] json_path: testdata/test.json
[Info]           name: test
````