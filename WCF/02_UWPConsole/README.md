


netsh http add urlacl url=http://+:8080/ user=tekk


* [Console App (Universal) Project Templates](https://marketplace.visualstudio.com/items?itemName=AndrewWhitechapelMSFT.ConsoleAppUniversal)


````bat
2024-04-27 21:45:29.7631 [INFO ] Start
2024-04-27 21:45:30.0774 [INFO ] Connect to http://localhost:5001
2024-04-27 21:45:31.0280 [ERROR] Error
        Message: An error occurred while receiving the HTTP response to http://localhost:5001/. This could be due to the service endpoint binding not using the HTTP protocol. This could also be due to an HTTP request context being aborted by the server (possibly due to the service shutting down). See server logs for more details., HResult: 0x80131500
        Message: An error occurred while sending the request., HResult: 0x80072EFF
        Message: このエラー コードに関連付けられたテキストが見つかりませんでした。

サーバーとの接続がリセットされました。
, HResult: 0x80072EFF
````