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
$ .\install\win\bin\Demo.exe
pqxx::connection
Standard exception: connection to server at "localhost" (::1), port 5432 failed: Connection refused (0x0000274D/10061)
        Is the server running on that host and accepting TCP/IP connections?
connection to server at "localhost" (127.0.0.1), port 5432 failed: Connection refused (0x0000274D/10061)
        Is the server running on that host and accepting TCP/IP connections?
````

#### Linux

````shell
# If libpqxx is built as dynamic library
$ export LD_LIBRARY_PATH=./install/linux/bin
$ ./install/linux/bin/Demo 
qxx::connection
Standard exception: connection to server on socket "/var/run/postgresql/.s.PGSQL.5432" failed: No such file or directory
        Is the server running locally and accepting connections on that socket?
````

#### OSX

````shell
# If libpqxx is built as dynamic library
$ export DYLD_LIBRARY_PATH=./install/osx/bin
$ ./install/osx/bin/Demo 
pqxx::connection
Standard exception: connection to server on socket "/tmp/.s.PGSQL.5432" failed: No such file or directory
        Is the server running locally and accepting connections on that socket?
````