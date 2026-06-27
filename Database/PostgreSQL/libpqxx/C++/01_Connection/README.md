# Establish connection

## Abstracts

* Link libpqxx binary
* Establish connection to local PostgreSQL server

## Requirements

### Common

* Powershell 7 or later
* CMake 3.12 or higher

### Windows

* Visual Studio 2022

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [libpqxx](https://github.com/jtv/libpqxx)
  * BSD-3-Clause license

## How to build?

### GStreamer

Go to [libpqxx](..).

Once time you built `libpqxx`, you need not to do again.

````shell
$ pwsh download-libpq.ps1
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
````

#### Linux

````shell
$ export LD_LIBRARY_PATH=./install/linux/bin
$ ./install/linux/bin/Demo 
qxx::connection
Standard exception: connection to server on socket "/var/run/postgresql/.s.PGSQL.5432" failed: No such file or directory
        Is the server running locally and accepting connections on that socket?
````

#### OSX

````shell
$ export DYLD_LIBRARY_PATH=./install/osx/bin
$ ./install/osx/bin/Demo 
libpqxx Version: 7.10.4
````