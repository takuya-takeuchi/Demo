# Generate SSL certificate for custom domain

## Abstracts

* Generate SSL certificate supports subjectAltName for custom domain

## Requirements

* PowerShell Core

## Dependencies

* [OpenSSL](https://github.com/PrismLibrary/Prism)
  * OpenSSL License/SSLeay License
    * Download 1.0.2u from https://indy.fulgan.com/SSL/
    * openssl.cnf is obtained from source code

## How to use

````bat
$ pwsh .\Initialize.ps1
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
unable to write 'random state'
````

````bat
$ pwsh .\GenerateRootCA.ps1
Create private key and certificate signing request...
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Generating a RSA private key
.........................................................++++
..................................++++
unable to write 'random state'
writing new private key to 'E:\Works\OpenSource\Demo2\Misc\20_PrivateCertificateAuthority\pki\CA\private\root-ca.key'
-----
Do Self-Sign certificate signing request...
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Using configuration from E:\Works\OpenSource\Demo2\Misc\20_PrivateCertificateAuthority\root-ca.conf
Check that the request matches the signature
Signature ok
Certificate Details:
Certificate:
    Data:
        Version: 3 (0x2)
        Serial Number:
            bc:2a:fd:81:83:51:a1:e0:6a:1a:30:45:5d:b9:12:6e
        Issuer:
            countryName               = JP
            organizationName          = Contoso
            commonName                = Contoso Root CA
        Validity
            Not Before: Apr 28 17:31:29 2024 GMT
            Not After : Apr 26 17:31:29 2034 GMT
        Subject:
            countryName               = JP
            organizationName          = Contoso
            commonName                = Contoso Root CA
        Subject Public Key Info:
            Public Key Algorithm: rsaEncryption
                Public-Key: (4096 bit)
                Modulus:
                    00:c3:fb:4b:48:94:89:fe:3c:da:82:07:8a:22:a0:
                    93:2f:36:b7:ff:fb:2b:2a:a8:1d:c6:48:18:cf:e8:
                    50:74:60:0d:b5:36:9d:c0:37:b4:13:21:fd:b9:ac:
                    3f:cb:03:14:4e:40:b0:28:2d:9d:82:34:50:71:4e:
                    fb:62:7e:bb:06:40:e0:6d:e2:b2:d1:c9:3f:2b:63:
                    20:0d:c0:38:0a:2d:92:fa:90:2f:f0:d5:4a:1d:14:
                    3c:88:01:77:c3:30:63:9d:e2:cc:77:10:6b:b6:b2:
                    c3:d0:02:0c:40:fb:fd:fd:52:25:cb:79:4b:2e:67:
                    88:58:f8:08:79:35:b9:ff:de:32:ab:7e:39:0b:7d:
                    fe:5c:5a:b0:40:e1:2e:e7:8d:d5:ef:af:ae:44:79:
                    77:5b:2d:0e:b7:c8:e5:d9:28:11:17:f9:4b:72:93:
                    bb:67:a8:10:47:d7:b5:76:21:44:24:68:72:cb:64:
                    b7:c8:c7:dc:ca:9b:a8:a4:b0:bf:0b:75:74:ca:07:
                    fe:57:56:cf:5f:8d:df:84:23:ed:7a:2d:04:9e:9e:
                    10:1e:e0:4b:7f:3e:91:cd:50:1f:9a:d1:90:1d:fe:
                    7a:b5:6f:7d:3e:70:23:2d:45:da:cb:e3:4d:4a:61:
                    a2:bc:23:9b:b3:52:75:17:a7:2c:6b:a4:14:fb:8c:
                    22:11:ee:51:ab:d0:55:25:6a:d9:3a:4c:20:bc:46:
                    43:8e:65:42:fc:dd:b4:2b:61:01:94:aa:0b:4c:a4:
                    c0:da:66:ea:7b:cd:5f:78:e6:04:e9:33:8d:7c:fc:
                    e1:4e:77:d1:d2:37:1b:5e:4d:b8:64:ec:07:db:ac:
                    58:e5:c7:eb:4f:37:22:db:a8:01:ce:79:87:c5:7c:
                    fe:7c:2f:71:cb:c2:8a:2c:9a:49:6f:93:20:df:cd:
                    df:09:a3:a2:9c:b5:29:da:08:7b:68:f6:0a:23:71:
                    1a:3c:14:fe:de:76:85:fc:f0:17:7d:c8:e0:6b:28:
                    f2:2e:f2:de:32:59:53:0f:0f:6d:50:b9:cb:42:33:
                    49:35:d4:b3:6e:d5:f0:68:e9:63:c0:e6:87:b7:52:
                    6e:81:aa:9e:a5:5b:79:ad:fe:da:ed:c1:56:e6:e3:
                    8f:f9:43:12:c7:09:a3:57:84:ab:2a:51:2e:4c:e1:
                    9e:45:3a:c4:ad:a0:2e:b5:d0:ef:1e:e5:51:7d:e0:
                    ee:74:7c:0f:c2:f5:71:d4:ee:73:07:23:dc:6d:60:
                    9a:5c:46:6e:4f:1d:ad:2b:aa:0a:3a:64:7c:d7:99:
                    f2:65:7f:99:c9:27:5b:97:c5:5a:16:e8:b4:c3:48:
                    8b:f7:66:ee:9a:2e:68:5e:9e:a7:63:fa:46:81:ee:
                    75:99:77
                Exponent: 65537 (0x10001)
        X509v3 extensions:
            X509v3 Basic Constraints: critical
                CA:TRUE
            X509v3 Key Usage: critical
                Certificate Sign
            X509v3 Subject Key Identifier:
                40:45:16:72:6B:CC:0A:A8:4F:F6:75:F4:11:5A:04:59:78:3C:F5:4E
Certificate is to be certified until Apr 26 17:31:29 2034 GMT (3650 days)
Sign the certificate? [y/n]:y


1 out of 1 certificate requests certified, commit? [y/n]y
Write out database with 1 new entries
Data Base Updated
unable to write 'random state'
````



Lastly, install `root-ca.crt` into OS certification manager,