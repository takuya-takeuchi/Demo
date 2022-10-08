# Generate SSL certificate for IP address

## Abstacts

* Generate SSL certificate supports subjectAltName for IP address

## Requirements

* PowerShell Core

## Dependencies

* [OpenSSL](https://github.com/PrismLibrary/Prism)
  * OpenSSL License/SSLeay License
    * Download 1.0.2u from https://indy.fulgan.com/SSL/
    * openssl.cnf is obtained from source code

## How to use

````bat
$ pwsh Generate.ps1 192.168.11.21
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Generating a RSA private key
.......+++++
.........+++++
writing new private key to 'server.key'
-----
No value provided for Subject Attribute C, skipped
No value provided for Subject Attribute ST, skipped
No value provided for Subject Attribute L, skipped
No value provided for Subject Attribute O, skipped
No value provided for Subject Attribute OU, skipped
````

You can see server.key and server.crt