# Create certificate signing request

## Abstracts

* Generate server's certificate signing request

## Requirements

* PowerShell Core

## Dependencies

* [OpenSSL](https://github.com/openssl/openssl)
  * OpenSSL License/SSLeay License
    * Download 3.3.0 from https://kb.firedaemon.com/support/solutions/articles/4000121705

## How to use

Modify these files in [CreateCSR.ps1](./CreateCSR.ps1).

````powershell
$C="JP"                # Country Name
$ST="Tokyo"            # State or Province Name
$L="Minato-ku"         # Locality Name
$O="Contoso"           # Organization Name
$OU="Docs,Contoso"     # Organizational Unit Name
````

Then, kick script with fqdn or your name as argument.

````bat
$ pwsh CreateCSR.ps1 www.contoso.com
Create private key ...
.......+.....+++++++++++++++++++++++++++++++++++++++*......+++++++++++++++++++++++++++++++++++++++*.+.+.....+...+.+...........+....+......+.....+......+...+..........+.....+......+..........+...+...............++++++
......+....+......+...+++++++++++++++++++++++++++++++++++++++*........+.+++++++++++++++++++++++++++++++++++++++*.........+..........+......+........+.+..+.......+.....+...+....+......+.........+.....+.+..+..................+.........+....+..+............+............+....+..............+..........+.....+.+.....+......+.........+.+...........+.......+..................+.........+........+...+....+......+...+.....+....+..+............................+...........+.+......+..................+..............+......................+...+..+.........+.+........+.+..+............+.......+.....+.........+......+....+...........+...+.+......+......+..+.+..+...+.+......+...+......+........+..........+..+...+......+....+...+.....+...+.........+..........+.....+..................+...+.+...+......+.....+.+........+...+...+.....................+.+...+............+............+..+...+...+....+...........+.......+......+......+.....++++++
Create certificate signing request ...
````

You will obtain `server.csr` and `server.pem`.
`server.pem` is privete key and it can be used for generating pfx file.