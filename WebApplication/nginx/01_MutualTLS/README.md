# Mutual TLS

## Abstracts

* How to use Mutual TLS (mTLS) for nginx

## Requirements

* curl
  * If you run on windows for test, I does not recommend using built-in curl. You shoule download from [curl for Windows](https://curl.se/windows/).

## Dependencies

* [nginx](https://github.com/nginx/nginx)
  * 1.26.1
  * BSD 2-Clause License
* [OpenSSL](https://github.com/PrismLibrary/Prism)
  * OpenSSL License/SSLeay License
    * Download 1.0.2u from https://indy.fulgan.com/SSL/
    * openssl.cnf is obtained from source code

## How to use?

#### 01. Create certificates

First, you need to create server and client certificates.
You can use these scripts

* [Misc/20_PrivateCertificateAuthority](../../../Misc/20_PrivateCertificateAuthority)
* [Misc/21_NewCertificateSigningRequest](../../../Misc/21_NewCertificateSigningRequest)

Then, copy all generated files (ca.crt, ca.key, server.crt, server.csr and server.key) into here.

#### 02. Create client certificates

````bash
$ pwsh GenerateClientCert.ps1
Create client key:
Generating RSA private key, 4096 bit long modulus
...................................................................................................................................................................................................++++
........................................................................................................................................................................................................................................................................++++
unable to write 'random state'
e is 65537 (0x10001)
Enter pass phrase for client.key:
Verifying - Enter pass phrase for client.key:
Create a CSR:
Enter pass phrase for client.key:
Sign the CSR:
Signature ok
subject=/C=JP/ST=Osaka/L=Osaka-shi/O=Contoso Asia/OU=Docs,Contoso Asia
Getting CA Private Key
unable to write 'random state'
Decrypt client key:
Enter pass phrase for client.key:
writing RSA key
Create pfx:
Enter Export Password:
Verifying - Enter Export Password:
unable to write 'random state'
````

You will see client client.crt, client.csr, client.decrypted.key and client.key.

#### 03. Deploy nginx

Download and deploy nginx.

````json
$ pwsh ../DownloadNginx.ps1
````

And start nginx.

##### Windows

````bash
$ cd nginx
$ start nginx.exe
````

##### Linux and MacOS

````bash
$ cd nginx
$ start nginx.exe
````

#### 04. Try mTLS by curl

````bash
$ curl https://192.168.11.21 --cacert ca.crt --key client.decrypted.key --cert client.crt
<!DOCTYPE html>
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

If you didn't specify client key and certificate, you will see

````bash
$ curl https://192.168.11.21 --cacert ca.crt             
<html>                          
<head><title>400 No required SSL certificate was sent</title></head>
<body>
<center><h1>400 Bad Request</h1></center>
<center>No required SSL certificate was sent</center>
<hr><center>nginx/1.26.1</center>
</body>
</html>
````

And if you use built-in `curl` on windows, you may see the following error in spite of using same command.

````cmd
$ curl https://192.168.11.21 --cacert ca.crt --key client.decrypted.key --cert client.crt
curl: (58) schannel: Failed to import cert file client.crt, last error is 0x80092002
````