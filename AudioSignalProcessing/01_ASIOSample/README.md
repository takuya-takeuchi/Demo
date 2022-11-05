# Minimul sample of ASIO driver

## Abstracts

* How to embed ASIO SDK (from Steinberg)
  * Used `asiosdk_2.3.3_2019-06-14`

## Requirements

* CMake 3.0 or later
* PowerShell Core

## Dependencies

* [ASIO SDK](https://www.steinberg.net/developers/)
  * Steinberg ASIO SDK License

## How to use?

At first, you must download ASIO SDK and extract it into `asiosdk` directory.
After this, you can build it.

````bat
$ pwsh Build.ps1 Release
````

And you can see sample program in `install` directory.

## Result

````bat
$ cd install\windows\Release
$ AsioSample.exe
Realtek ASIO
````