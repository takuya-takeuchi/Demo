# Sample

## Abstracts

* Simple CRUD test

## Requirements

* .NET 8.0

## Dependencies

* [NLog](https://github.com/NLog/NLog)
  * BSD-3-Clause License
  * 5.4.0
* [Npgsql Entity Framework Core provider for PostgreSQL](https://github.com/npgsql/efcore.pg)
  * PostgreSQL license
  * 9.0.4

## How to run

````bat
$ dotnet run -c Release
2025-03-30 13:14:03.9173 [INFO ] Added: John Doe, j-doe@example.com 
2025-03-30 13:14:04.3896 [INFO ] Entity List: 
2025-03-30 13:14:04.3896 [INFO ] 1: John Doe (j-doe@example.com) 
2025-03-30 13:14:04.4447 [INFO ] Updated: Jane Doe 
2025-03-30 13:14:04.4829 [INFO ] Deleted: Jane Doe
````