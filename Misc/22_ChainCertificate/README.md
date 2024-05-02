# Chain Certitifacte

## Abstracts

* Generate Root CA, Intermediate CA and Server certificates

## Requirements

* PowerShell Core

## Dependencies

* [OpenSSL](https://github.com/openssl/openssl)
  * OpenSSL License/SSLeay License
    * Download 3.3.0 from https://kb.firedaemon.com/support/solutions/articles/4000121705

## How to use

#### 0. Setup

At first, copy or rename these files

* rename `root-ca.conf.in` to `root-ca.conf`
* rename `intermediate-ca.conf.in` to `intermediate-ca.conf`
* rename `server.conf.in` to `server.conf`

Then, modify thems.
For example, 

###### root-ca.conf

Most of root ca (e.g. ISRG Root X1) has only 3 items so this repo follows it.

````txt
[req_dn]
countryName = "JP"
organizationName ="Contoso"
commonName = "Contoso Root CA"
````

###### intermediate-ca.conf

Most of intermediate ca (e.g. Let's Encrypt) has only 3 items so this repo follows it.

````txt
[req_dn]
countryName = "JP"
organizationName ="Contoso"
commonName = "Contoso Intermediate CA"
````

###### server.conf

Most of intermediate ca (e.g. www.google.com) has only 1 items so this repo follows it.

````txt
[req_dn]
commonName = "devpc2.taktak.com"

[ alternate_names ]
DNS.1 = devpc2.taktak.com
````

#### 1. Initialize

You should kick this script only 1 time.

````bat
$ pwsh 01_Initialize.ps1
````

#### 2. Create certificates

You can kick this script again and again.
But root ca and intermediate ca are not re-generated after kicked script one time.
You have to remove [pki](./pki) directory and kick [01_Initialize.ps1](./01_Initialize.ps1) again if you want to re-generate them.
On other words, you can only modify `server.conf` if you want to create new server certificate.

````bat
$ pwsh 02_Generate.ps1

[Root CA]
Create private key...
Create certificate signing request...
Do Self-Sign certificate signing request...
Using configuration from E:\Works\OpenSource\Demo2\Misc\22_ChainCertificate copy\root-ca.conf
Check that the request matches the signature
Signature ok
Certificate Details:
Certificate:
    Data:
        Version: 1 (0x0)
        Serial Number:
            3a:6b:63:dd:cf:6c:dd:99:09:28:54:a3:dc:ff:74:20
        ...
Certificate is to be certified until Apr 30 21:01:50 2034 GMT (3650 days)

Write out database with 1 new entries
Database updated
Create certificate...

[Intermediate CA]
Create private key...
Create certificate signing request...
Do Sign certificate signing request...
Using configuration from E:\Works\OpenSource\Demo2\Misc\22_ChainCertificate copy\intermediate-ca.conf
Check that the request matches the signature
Signature ok
Certificate Details:
Certificate:
    Data:
        Version: 1 (0x0)
        Serial Number:
            c5:73:20:85:e8:57:bd:34:c3:45:54:04:9b:4e:40:97
        ...
Certificate is to be certified until May  1 21:01:50 2029 GMT (1825 days)

Write out database with 1 new entries
Database updated
Create certificate...

[Server]
Create private key...
Create certificate signing request...
Do Sign certificate signing request...
Using configuration from E:\Works\OpenSource\Demo2\Misc\22_ChainCertificate copy\server.conf
Check that the request matches the signature
Signature ok
Certificate Details:
Certificate:
    Data:
        Version: 1 (0x0)
        Serial Number:
            c2:d7:e5:0f:f1:b7:ea:7a:33:79:b7:2f:fc:90:ff:14
        ...
Certificate is to be certified until May  2 21:01:51 2025 GMT (365 days)

Write out database with 1 new entries
Database updated
Create certificate...

Create certificate chain...

Verify server certification by certification chain...
E:\Works\OpenSource\Demo2\Misc\22_ChainCertificate copy\pki\Server\server.crt: OK
````

All certificates are generated into there.

* [pki/CA](./pki/CA)
  * Root CA certificate files
* [pki/ICA](./pki/ICA)
  * Intermediate CA certificate files
* [pki/Server](./pki/Server)
  * Server certificate files

#### 3. Create PFX file (optional)

You can generate pfx file from `**.crt`.

````bat
$ pwsh 03_CreatePFX.ps1
Enter Password: ***********
Confirm Password: ***********

[Root CA]
Create personal information exchange...

[Intermediate CA]
Create personal information exchange...

[Server]
Create personal information exchange...
````