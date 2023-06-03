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
$ pwsh Generate.ps1 www.example.com
Create your CA crt and key:
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Generating a RSA private key
.......................++++
..............++++
writing new private key to 'ca.key'
-----
Create a CSR:
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Generating a RSA private key
................................+++++
.........+++++
writing new private key to 'server.key'
-----
Sign the CSR, resulting in CRT and add the v3 SAN extension:
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Signature ok
subject=/C=JP/ST=Tokyo/L=Minato-ku/O=Contoso/OU=Docs,Contoso/CN=www.example.com
Getting CA Private Key
unable to write 'random state'
Check contents of CSR:
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Unable to load config info from /usr/local/ssl/openssl.cnf
Certificate Request:
    Data:
        Version: 0 (0x0)
        Subject: C=JP, ST=Tokyo, L=Minato-ku, O=Contoso, OU=Docs,Contoso, CN=www.example.com
        Subject Public Key Info:
            Public Key Algorithm: rsaEncryption
                Public-Key: (2048 bit)
                Modulus:
                    00:c5:0d:14:63:6e:cc:af:84:ea:2b:1c:ff:49:45:
                    1a:ca:2d:ab:22:13:7e:23:49:86:76:f6:2f:d3:25:
                    c4:ec:3a:b3:f5:a0:43:8b:bf:c6:75:b9:0c:fd:f6:
                    5c:77:4f:ec:0d:7e:cb:05:72:5b:16:94:d3:58:83:
                    01:cc:7f:00:3b:7e:84:df:d0:3d:33:9b:48:f7:35:
                    b2:cc:1b:1c:ff:d3:20:4f:42:d7:23:0d:2d:b8:4a:
                    ac:56:09:06:c1:9a:d8:cb:2e:2c:65:ce:1d:83:67:
                    b9:52:9d:de:b2:b5:9f:61:38:1b:92:51:d1:a1:39:
                    71:ee:ed:94:2c:dc:5c:df:c3:d7:c5:19:7e:4d:25:
                    7e:20:ad:cc:76:fd:db:95:a3:a4:ae:1c:4b:e2:60:
                    02:b7:40:f5:33:19:d8:40:ce:51:54:d7:33:70:29:
                    9e:b8:02:67:54:08:37:79:af:60:f5:24:d2:66:ae:
                    b4:c5:38:05:af:5b:fa:78:b6:2a:c7:23:ca:60:e7:
                    84:20:84:8b:7f:f7:56:a7:4b:50:5a:0a:2e:d0:8a:
                    6f:fb:99:26:f2:3d:bd:fc:71:ae:d8:90:78:41:9d:
                    26:4c:3d:e2:d7:bb:b6:0d:7c:95:ea:57:a5:fb:27:
                    4a:b9:a2:2b:78:8c:4a:e8:ea:a1:b2:7e:87:85:44:
                    b1:c1
                Exponent: 65537 (0x10001)
        Attributes:
            a0:00
    Signature Algorithm: sha256WithRSAEncryption
         5a:b9:c8:f9:e4:b0:61:10:a7:10:df:da:a4:9e:64:8e:0b:19:
         54:dc:64:41:14:c8:98:df:32:ef:e8:6c:7e:e8:53:0e:f3:45:
         90:23:1e:62:dd:85:8c:03:e7:5f:7e:d1:7e:a7:c3:0a:f5:64:
         30:2e:47:9f:d5:e2:8e:9c:fd:38:08:12:c0:d6:be:d0:0b:2a:
         60:0e:fc:29:76:51:f3:d1:b1:5b:cd:42:85:a5:80:2f:80:b2:
         4e:2f:21:c2:ff:5c:14:be:0e:6a:cd:91:73:44:10:2d:f7:45:
         2d:e8:ae:4c:41:18:d2:50:83:45:75:f2:2d:6e:96:05:61:42:
         42:0a:ec:0c:2a:31:08:ca:c2:63:33:43:b0:14:0a:1a:61:5a:
         14:df:40:93:22:8d:24:57:a3:84:37:89:0e:68:ab:a2:82:df:
         ab:51:aa:78:55:47:e7:15:f3:64:ca:51:7f:74:cf:a9:31:6a:
         f8:7a:83:a2:c8:0d:0b:33:3a:28:a0:df:bd:54:90:d8:2d:19:
         7c:52:ed:1a:f3:e3:d6:3e:14:c3:d5:76:a6:96:b9:89:9e:5c:
         5a:d6:f5:48:85:01:7d:2b:e8:a8:4f:16:c5:29:87:87:ad:17:
         6e:0f:2f:69:f0:0d:02:90:ab:e1:4e:e9:b7:d4:42:47:af:8a:
         8b:fc:8f:eb
Check contents of CRT
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Certificate:
    Data:
        Version: 1 (0x0)
        Serial Number:
            af:27:af:43:7f:81:54:ee
    Signature Algorithm: sha256WithRSAEncryption
        Issuer: C=JP, ST=Tokyo, L=Minato-ku, O=Contoso, OU=Docs,Contoso, CN=Contoso Insecure Certificate Authority
        Validity
            Not Before: Jun  3 11:55:17 2023 GMT
            Not After : May 31 11:55:17 2033 GMT
        Subject: C=JP, ST=Tokyo, L=Minato-ku, O=Contoso, OU=Docs,Contoso, CN=www.example.com
        Subject Public Key Info:
            Public Key Algorithm: rsaEncryption
                Public-Key: (2048 bit)
                Modulus:
                    00:c5:0d:14:63:6e:cc:af:84:ea:2b:1c:ff:49:45:
                    1a:ca:2d:ab:22:13:7e:23:49:86:76:f6:2f:d3:25:
                    c4:ec:3a:b3:f5:a0:43:8b:bf:c6:75:b9:0c:fd:f6:
                    5c:77:4f:ec:0d:7e:cb:05:72:5b:16:94:d3:58:83:
                    01:cc:7f:00:3b:7e:84:df:d0:3d:33:9b:48:f7:35:
                    b2:cc:1b:1c:ff:d3:20:4f:42:d7:23:0d:2d:b8:4a:
                    ac:56:09:06:c1:9a:d8:cb:2e:2c:65:ce:1d:83:67:
                    b9:52:9d:de:b2:b5:9f:61:38:1b:92:51:d1:a1:39:
                    71:ee:ed:94:2c:dc:5c:df:c3:d7:c5:19:7e:4d:25:
                    7e:20:ad:cc:76:fd:db:95:a3:a4:ae:1c:4b:e2:60:
                    02:b7:40:f5:33:19:d8:40:ce:51:54:d7:33:70:29:
                    9e:b8:02:67:54:08:37:79:af:60:f5:24:d2:66:ae:
                    b4:c5:38:05:af:5b:fa:78:b6:2a:c7:23:ca:60:e7:
                    84:20:84:8b:7f:f7:56:a7:4b:50:5a:0a:2e:d0:8a:
                    6f:fb:99:26:f2:3d:bd:fc:71:ae:d8:90:78:41:9d:
                    26:4c:3d:e2:d7:bb:b6:0d:7c:95:ea:57:a5:fb:27:
                    4a:b9:a2:2b:78:8c:4a:e8:ea:a1:b2:7e:87:85:44:
                    b1:c1
                Exponent: 65537 (0x10001)
    Signature Algorithm: sha256WithRSAEncryption
         00:fd:11:13:73:aa:a2:cf:88:25:22:d9:68:3a:88:12:56:e6:
         89:1e:f1:dd:2a:00:27:00:b8:59:4d:a3:59:f3:85:db:b9:50:
         87:d4:02:8c:66:93:18:0d:fd:53:a0:ea:e8:49:cd:ca:93:0a:
         be:15:13:45:f8:49:6f:61:9f:5d:64:bc:0f:86:91:41:8e:13:
         66:6c:ed:3d:1a:82:53:68:d2:ba:10:71:b7:a3:b3:48:51:25:
         4f:f8:58:ae:92:09:5a:51:db:8f:c9:12:31:cd:5d:d2:dd:d7:
         ce:2e:d9:a4:76:09:b6:a7:79:18:ec:77:2f:3c:9f:fd:c9:a2:
         b7:7d:e1:54:55:a2:60:d7:40:ef:f2:e9:8c:fc:e4:83:fa:ab:
         17:f4:56:59:d1:3e:04:10:e8:2e:42:cf:26:f9:0a:74:c1:1b:
         4e:fe:06:08:c6:c8:b3:0c:11:2d:48:00:26:cc:ff:aa:ce:64:
         98:de:a9:53:da:08:ff:46:ef:37:17:4c:7c:b4:01:2b:5e:12:
         82:76:82:e0:e2:de:04:38:db:23:d4:1e:2d:17:37:8e:82:7a:
         5a:ee:66:a9:ce:c2:ae:5f:3b:42:0c:40:c8:98:ad:21:5c:a5:
         0f:35:8f:68:97:b8:05:79:6f:84:a1:de:87:da:ee:73:40:d0:
         1f:ad:7d:6d:2e:67:88:65:0f:60:21:e7:5d:87:f1:00:0f:c2:
         48:e2:5e:9e:df:42:cf:44:55:bc:cb:2a:f6:65:9a:9c:6b:9c:
         de:3a:08:71:fa:91:ae:6e:52:52:de:15:2b:77:22:cc:8a:04:
         ca:9e:cd:da:86:0f:db:86:33:0e:6b:66:ce:89:74:30:27:cb:
         c9:43:87:5c:72:15:fd:47:5e:da:07:f9:a9:54:2d:a5:24:8c:
         f5:a4:14:9a:ac:b8:4a:f8:52:c7:9b:ae:d8:27:d7:ee:1a:11:
         4b:7c:fe:e5:c9:34:c4:ee:85:da:96:00:80:c6:5c:ea:de:09:
         37:a6:63:61:d0:28:89:4a:a0:ab:c9:24:09:59:b2:f3:98:c8:
         06:66:29:7a:a6:bf:62:1e:72:6f:b9:15:7c:b2:7a:c5:9e:67:
         d3:5e:35:cf:76:bf:e8:87:d5:76:54:97:49:37:ef:19:da:c8:
         9c:33:46:a4:44:e6:4d:ca:c9:c5:3b:8f:66:fa:2f:fd:a1:13:
         3f:34:ed:e4:bb:e3:b6:a4:d3:84:9e:37:f4:0a:cc:82:7b:fb:
         d8:72:bc:fe:2e:fa:9f:10:ac:c2:23:18:0b:b0:94:48:c0:36:
         28:d8:10:80:db:bd:7f:fd:30:b4:fe:00:75:b9:57:75:0e:19:
         59:bb:ba:cc:5d:5f:8a:af
````

You can see the following files.

* ca.crt
* ca.key
* server.crt
* server.csr
* server.key