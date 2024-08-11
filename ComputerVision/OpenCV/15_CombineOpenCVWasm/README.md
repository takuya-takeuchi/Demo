# Sample Library by docker build

## Abstracts

* Build simple wasm library by docker

## Requirements

### Common

* Powershell 7 or later
* CMake 3.10.0 or later
* Docker
  * If use on Linux, you must add priviledge for user to execute docker without priviledge

## How to build?

### Nginx

````shell
$ pwsh BuildNginx.ps1
````

Once time you built `nginx`, you need not to do again.

Then,

````shell
$ pwsh Build.ps1 <Debug/Release>
````

## How to test?

````shell
$ pwsh Run.ps1
````

Then, `nginx` listen `8080`.

<img src="./images/edge.png" />

## Note

* When use `std::cout`, comiler throw build error message like 

> wasm-ld: error: CMakeFiles/Demo.dir/library.cpp.o: relocation R_WASM_MEMORY_ADDR_SLEB cannot be used against symbol `std::__2::cout`; recompile with -fPIC