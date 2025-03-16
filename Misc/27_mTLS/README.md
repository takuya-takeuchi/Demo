# Mutual TLS

## Abstracts

* How to use Mutual TLS (mTLS) for nginx

## Requirements

### Common

* Powershell 7 or later
* .NET 8.0 or later

## How to run?

At first, you must create client certificates from server certificates.
Please refer [WebApplication/nginx/01_MutualTLS](../../WebApplication/nginx/01_MutualTLS).

### Windows

You must install Root CA certificate into **Trusted Root Certification Authorities**.
In other words, you will error message `The remote certificate is invalid because of errors in the certificate chain: PartialChain`.

````bash
$ cd sources\Demo
$ dotnet run -c Release -- https://192.168.11.100 client.pfx password
2025-03-16 17:00:15.6607 [INFO ] <!DOCTYPE html>
<html>
<head>
<title>Welcome to nginx!</title>
<style>
html { color-scheme: light dark; }
body { width: 35em; margin: 0 auto;
font-family: Tahoma, Verdana, Arial, sans-serif; }
</style>
</head>
<body>
<h1>Welcome to nginx!</h1>
<p>If you see this page, the nginx web server is successfully installed and
working. Further configuration is required.</p>

<p>For online documentation and support please refer to
<a href="http://nginx.org/">nginx.org</a>.<br/>
Commercial support is available at
<a href="http://nginx.com/">nginx.com</a>.</p>

<p><em>Thank you for using nginx.</em></p>
</body>
</html>
````

### Linux or MacOS

You must install Root CA certificate into **Keychain** or **ca-certificates**.
In other words, you will error message like `The remote certificate is invalid according to the validation procedure: RemoteCertificateNameMismatch`.

````bash
$ cd sources/Demo
$ dotnet run -c Release -- https://192.168.11.100 client.crt client.decrypted.key
2025-03-16 17:50:14.9521 [INFO ] <!DOCTYPE html>
<html>
<head>
<title>Welcome to nginx!</title>
<style>
html { color-scheme: light dark; }
body { width: 35em; margin: 0 auto;
font-family: Tahoma, Verdana, Arial, sans-serif; }
</style>
</head>
<body>
<h1>Welcome to nginx!</h1>
<p>If you see this page, the nginx web server is successfully installed and
working. Further configuration is required.</p>

<p>For online documentation and support please refer to
<a href="http://nginx.org/">nginx.org</a>.<br/>
Commercial support is available at
<a href="http://nginx.com/">nginx.com</a>.</p>

<p><em>Thank you for using nginx.</em></p>
</body>
</html>
````