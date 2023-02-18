# PUT

## Abstracts

* POST sample program
  * Invoke /put of https://httpbin.org/

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++11

### Windows

* Visual Studio
* cpprestsdk
  * via `vcpkg` command with `--triplet x64-windows-static`

### Ubuntu

* g++
* libcpprest-dev
  * via `apt` command

### OSX

* Xcode
* cpprestsdk
  * via `brew` command

## Dependencies

* [cpprestsdk](https://github.com/microsoft/cpprestsdk)
  * 2.10.18
  * MIT License

## How to build?

You have to set `VCPKG_ROOT_DIR` envrironmental variable, like `C:\vcpkg` before build cpprestsdk on windows machine.

````shell
$ pwsh build.ps1 <Debug/Release>
{
  "args": {},
  "data": "{\"message\":\"Hello http\"}",
  "files": {},
  "form": {},
  "headers": {
    "Accept": "application/json",
    "Content-Length": "24",
    "Content-Type": "application/json",
    "Host": "httpbin.org",
    "User-Agent": "cpprestsdk/2.10.18",
    "X-Amzn-Trace-Id": "Root=1-63d573fb-366f60bd4aee43bc3b3bba3b"
  },
  "json": {
    "message": "Hello http"
  },
  "origin": "XXX.XXX.XXX.XXX",
  "url": "https://httpbin.org/put"
}
````

## How to test?

````bat
$ install\win\bin\Test "https://<your-bucket-name>.s3.ap-northeast-1.amazonaws.com/lenna.jpg?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=XXXXXXXXXXXXXXXXXXX%2F20230218%2Fap-northeast-1%2Fs3%2Faws4_request&X-Amz-Date=20230218T134137Z&X-Amz-Expires=600&X-Amz-SignedHeaders=host&X-Amz-Signature=4b5d4337e3ca087ec5526335d8081e2620c2de4466aac56b6361e63a4939ca0e" install\win\bin\lenna.jpg  
````