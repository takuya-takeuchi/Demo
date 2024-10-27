# Report Memory Leak of CRT (for Windows only)

## Abstracts

* Report memory leaks due to `new` and `malloc`
* Enable report on stdout so you need not to use Visual Studio

## Requirements

### Common

* Powershell 7 or later
* CMake 3.0.0 or later

### Windows

* Visual Studio

## How to build?

````shell
$ pwsh build.ps1 Debug
````

You can specify `Release` but memory leak report will not be present.

## How to use?

````shell
$ install\win\bin\Demo.exe
Set flag to _CrtSetDbgFlag
Check for memory leaks in the output window after the program ends.
Detected memory leaks!
Dumping objects ->
{154} normal block at 0x000002119BF8D3F0, 400 bytes long.
 Data: <                > CD CD CD CD CD CD CD CD CD CD CD CD CD CD CD CD
D:\Works\OpenSource\Demo\Misc\25_CheckVSMemoryLeak\main.cpp(26) : {153} normal block at 0x000002119BF92FE0, 160000 bytes long.
 Data: <                > CD CD CD CD CD CD CD CD CD CD CD CD CD CD CD CD
Object dump complete.
````