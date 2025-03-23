# Q16.[Network+Forensic]HTTP Traffic

## How to resolve?

You should use wireshark.
At first, you can find that there is are `301 Moved Permanently`.
Then you will see html source code on `HTTP/1.1 200 OK`.
You can find that javascript find is missing.
It means that you must check response data of javascript.