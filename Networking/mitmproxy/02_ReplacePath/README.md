# Replace path

## Abstracts

* Replace path of url

## Dependencies

* [mitmproxy](https://github.com/mitmproxy/mitmproxy)
  * MIT license
  * 8.1.1

## How to use?

#### Server

````bash
$ cd 02_ReplacePath
$ mitmproxy -s script.py --set domain=httpbin.org --set old=/get --set new=/json
````

#### Client

Client must install ca certificate from [htps://mitm.it](htps://mitm.it).

````bash
$ curl -X GET https://httpbin.org/get
{
  "args": {},
  "headers": {
    "Accept": "*/*",
    "Host": "httpbin.org",
    "User-Agent": "curl/8.9.1",
    "X-Amzn-Trace-Id": "Root=1-67bb40a5-13c0b4eb0d1d27766dc688b0"
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
Date: Sun, 23 Feb 2025 15:31:53 GMT
Content-Type: application/json
Content-Length: 429
Connection: keep-alive
Server: gunicorn/19.9.0
Access-Control-Allow-Origin: *
Access-Control-Allow-Credentials: true

{
  "slideshow": {
    "author": "Yours Truly",
    "date": "date of publication",
    "slides": [
      {
        "title": "Wake up to WonderWidgets!",
        "type": "all"
      },
      {
        "items": [
          "Why <em>WonderWidgets</em> are great",
          "Who <em>buys</em> WonderWidgets"
        ],
        "title": "Overview",
        "type": "all"
      }
    ],
    "title": "Sample Slide Show"
  }
}
````

