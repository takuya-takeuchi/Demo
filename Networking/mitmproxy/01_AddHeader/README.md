# Add Header

## Abstracts

* Add `count` property to response header

## Dependencies

* [mitmproxy](https://github.com/mitmproxy/mitmproxy)
  * MIT license
  * 8.1.1

## How to use?

````bash
$ cd 01_AddHeader
$ mitmproxy -s script.py --set addheader=true
````

#### Client

Client must install ca certificate from [htps://mitm.it](htps://mitm.it).

````bash
$ curl -i -X GET https://httpbin.org/get
HTTP/1.1 200 OK
Date: Sun, 23 Feb 2025 15:41:08 GMT
Content-Type: application/json
Content-Length: 252
Connection: keep-alive
Server: gunicorn/19.9.0
Access-Control-Allow-Origin: *
Access-Control-Allow-Credentials: true

{
  "args": {},
  "headers": {
    "Accept": "*/*",
    "Host": "httpbin.org",
    "User-Agent": "curl/8.9.1",
    "X-Amzn-Trace-Id": "Root=1-67bb4194-1d732278782414cf664138f7"
  },
  "origin": "xx.xx.xx.xx",
  "url": "https://httpbin.org/get"
}
````

Via `mitmproxy` on proxyserver.

````bash
$ curl -i -X GET https://httpbin.org/get -x http://<proxyserver>:8080
HTTP/1.1 200 Connection established

HTTP/1.1 200 OK
Date: Sun, 23 Feb 2025 15:39:28 GMT
Content-Type: application/json
Content-Length: 252
Connection: keep-alive
Server: gunicorn/19.9.0
Access-Control-Allow-Origin: *
Access-Control-Allow-Credentials: true
count: 1

{
  "args": {},
  "headers": {
    "Accept": "*/*",
    "Host": "httpbin.org",
    "User-Agent": "curl/8.9.1",
    "X-Amzn-Trace-Id": "Root=1-67bb4130-294dcca93632f57404e91086"
  },
  "origin": "xx.xx.xx.xx",
  "url": "https://httpbin.org/get"
}
````