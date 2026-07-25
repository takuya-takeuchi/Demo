# JWT Authorization

## Abstracts

* Demonstration of JWT
  * Added Authorization button on Swagger UI

## Requirements

* .NET 10.0 SDK

## Dependencies

* [Microsoft.AspNetCore.Authentication.JwtBearer](https://github.com/dotnet/dotnet)
  * MIT license
* [Microsoft.Extensions.Configuration.Binder](https://github.com/dotnet/dotnet)
  * MIT license
* [System.IdentityModel.Tokens.Jwt](https://github.com/AzureAD/azure-activedirectory-identitymodel-extensions-for-dotnet)
  * MIT license

## How to use?

### via CLI

At first, launch app.

````bash
$ dotnet run -c Release
````

### Login

#### Windows

````bat
$ curl -X "POST" "http://localhost:5234/api/Authorization/login" -H "accept: application/json" -H "Content-Type: application/json" -d "{ \"userName\": \"test\", \"password\": \"password\"}"

{"accessToken":"eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJ1c2VyLTAwMSIsImp0aSI6IjdiZDcyYTExMGRhNDQ1ZTU5ODNlOWFkMThkZTJjMTExIiwiaWF0IjoxNzg0OTg2MDg4LCJuYmYiOjE3ODQ5ODYwODgsImV4cCI6MTc4NDk4NjA5OCwiaXNzIjoiVGh1bmRlcmluZ0hlcmREZW1vIiwiYXVkIjoiVGh1bmRlcmluZ0hlcmREZW1vQ2xpZW50In0.bOjGqwgfhNSabMJ10Bl4FI6464g9PQfC0IiTOlUA--Y","accessTokenExpiresAt":"2026-07-25T13:28:18.4947919+00:00","refreshToken":"jPvr+YyvRz5gl+r+yAS4dc/hNBJPMa7iBeBIbTRUTF8=","refreshTokenExpiresAt":"2026-07-25T13:38:08.4947919+00:00"}
````

##### Linux or OSX

````bash
$ curl -X "POST" "http://localhost:5234/api/Authorization/login" -H "accept: application/json" -H "Content-Type: application/json" -d '{ "userName": "test", "password": "password"}'
````

### Refresh

#### Windows

````bat
$ curl -X "POST" "http://localhost:5234/api/Authorization/refresh" -H "accept: application/json" -H "Content-Type: application/json" -d "{ \"refreshToken\": \"e0rvOykEk6Kc8LCn9aC6e911c//uWnLxWBCiqEulRKM=\"}"

{"accessToken":"eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJ1c2VyLTAwMSIsImp0aSI6ImI3NWNlMDlmNDgxMDRmODM5NmIwODVjODU2MzBkYzRkIiwiaWF0IjoxNzg0OTg2MzMzLCJuYmYiOjE3ODQ5ODYzMzMsImV4cCI6MTc4NDk4NjM0MywiaXNzIjoiVGh1bmRlcmluZ0hlcmREZW1vIiwiYXVkIjoiVGh1bmRlcmluZ0hlcmREZW1vQ2xpZW50In0.pagPb1dgxj9Utrf64_dpBcN8eWAgQ9qtMpCfCTBC7Cs","accessTokenExpiresAt":"2026-07-25T13:32:23.2939602+00:00","refreshToken":"grhTtYQgah5BN9M5mF3Aw9mfUUWefc3iiqfalS46xwk=","refreshTokenExpiresAt":"2026-07-25T13:42:13.2939602+00:00"}
````

##### Linux or OSX

````bash
$ curl -X "POST" "http://localhost:5234/api/Authorization/login" -H "accept: application/json" -H "Content-Type: application/json" -d '{ "userName": "test", "password": "password"}'
````

### Test

Requires Bearer token.

#### Windows

````bat
$ curl -v -X "GET" "http://localhost:5234/api/Test" -H "accept: application/json" -H "Authorization: Bearer "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJ1c2VyLTAwMSIsImp0aSI6IjBhOTI3YzgwMmRhMDRhNDU4OTFmNjRiNzZiNGRmMmU4IiwiaWF0IjoxNzg0OTkyODE3LCJuYmYiOjE3ODQ5OTI4MTcsImV4cCI6MTc4NDk5MzExNywiaXNzIjoiVGh1bmRlcmluZ0hlcmREZW1vIiwiYXVkIjoiVGh1bmRlcmluZ0hlcmREZW1vQ2xpZW50In0.v1SH2orz6YKhKH0pTVDdW6hMpqM3xKZiiyfljCsUAJ4"
Note: Unnecessary use of -X or --request, GET is already inferred.
* Host localhost:5234 was resolved.
* IPv6: ::1
* IPv4: 127.0.0.1
*   Trying [::1]:5234...
* Connected to localhost (::1) port 5234
* using HTTP/1.x
> GET /api/Test HTTP/1.1
> Host: localhost:5234
> User-Agent: curl/8.13.0
> accept: application/json
> Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJ1c2VyLTAwMSIsImp0aSI6IjBhOTI3YzgwMmRhMDRhNDU4OTFmNjRiNzZiNGRmMmU4IiwiaWF0IjoxNzg0OTkyODE3LCJuYmYiOjE3ODQ5OTI4MTcsImV4cCI6MTc4NDk5MzExNywiaXNzIjoiVGh1bmRlcmluZ0hlcmREZW1vIiwiYXVkIjoiVGh1bmRlcmluZ0hlcmREZW1vQ2xpZW50In0.v1SH2orz6YKhKH0pTVDdW6hMpqM3xKZiiyfljCsUAJ4
> 
< HTTP/1.1 200 OK
< Content-Length: 0
< Date: Sat, 25 Jul 2026 15:20:52 GMT
< Server: Kestrel
< 
* Connection #0 to host localhost left intact
````

##### Linux or OSX

````bash
$ curl -v -X "GET" "http://localhost:5234/api/Test" -H "accept: application/json" -H "Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJ1c2VyLTAwMSIsImp0aSI6ImUxNGI1N2E1ODgwYTRjMjBhNmEzNmMxOGQ5NjExYTA1IiwiaWF0IjoxNzg0OTk4NTEzLCJuYmYiOjE3ODQ5OTg1MTMsImV4cCI6MTc4NDk5ODgxMywiaXNzIjoiVGh1bmRlcmluZ0hlcmREZW1vIiwiYXVkIjoiVGh1bmRlcmluZ0hlcmREZW1vQ2xpZW50In0.Gv1qs1jwgGzdhKN4Hm9IdZWL8ulGEsOVMk77AaeAsMs"
Note: Unnecessary use of -X or --request, GET is already inferred.
* Host localhost:5234 was resolved.
* IPv6: ::1
* IPv4: 127.0.0.1
*   Trying [::1]:5234...
* Connected to localhost (::1) port 5234
> GET /api/Test HTTP/1.1
> Host: localhost:5234
> User-Agent: curl/8.7.1
> accept: application/json
> Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJ1c2VyLTAwMSIsImp0aSI6ImUxNGI1N2E1ODgwYTRjMjBhNmEzNmMxOGQ5NjExYTA1IiwiaWF0IjoxNzg0OTk4NTEzLCJuYmYiOjE3ODQ5OTg1MTMsImV4cCI6MTc4NDk5ODgxMywiaXNzIjoiVGh1bmRlcmluZ0hlcmREZW1vIiwiYXVkIjoiVGh1bmRlcmluZ0hlcmREZW1vQ2xpZW50In0.Gv1qs1jwgGzdhKN4Hm9IdZWL8ulGEsOVMk77AaeAsMs
> 
* Request completely sent off
< HTTP/1.1 200 OK
< Content-Length: 0
< Date: Sat, 25 Jul 2026 16:55:35 GMT
< Server: Kestrel
< 
* Connection #0 to host localhost left intact
````

### via Swagger UI

<img src="images/image1.png" />

<img src="images/image2.png" />

<img src="images/image3.png" />

<img src="images/image4.png" />