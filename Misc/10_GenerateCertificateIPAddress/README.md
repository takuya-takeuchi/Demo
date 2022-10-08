# Generate SSL certificate for IP address

## Abstracts

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
Create your CA crt and key:
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Generating a RSA private key
...................................................................................................++++
............................................................................++++
writing new private key to 'ca.key'
-----
Create a CSR:
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Generating a RSA private key
...............................................................................+++++
.............................................................................................................+++++
writing new private key to 'server.key'
-----
Sign the CSR, resulting in CRT and add the v3 SAN extension:
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Signature ok
subject=/C=JP/ST=Tokyo/L=Minato-ku/O=Contoso/OU=Docs,Contoso/CN=192.168.11.21
Getting CA Private Key
Check contents of CSR:
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Unable to load config info from /usr/local/ssl/openssl.cnf
Certificate Request:
    Data:
        Version: 0 (0x0)
        Subject: C=JP, ST=Tokyo, L=Minato-ku, O=Contoso, OU=Docs,Contoso, CN=192.168.11.21
        Subject Public Key Info:
            Public Key Algorithm: rsaEncryption
                Public-Key: (2048 bit)
                Modulus:
                    00:b2:ef:6f:d3:b0:dd:cf:6a:65:a9:28:13:de:7a:
                    c9:7b:67:31:80:0b:5b:d5:1b:56:ac:9c:b9:83:ee:
                    51:bc:d7:e3:88:3a:bd:fa:df:5c:d1:21:af:50:fe:
                    c7:66:dc:dc:c7:07:f6:3f:5b:f2:45:a4:4b:1f:bb:
                    23:81:c3:c9:78:27:69:b9:59:82:ef:60:e9:49:9d:
                    5a:7a:5f:6a:fd:f1:b9:e9:ed:1f:f4:84:98:84:1d:
                    9b:7c:61:8e:79:94:7f:f5:03:a4:e6:08:ef:01:d3:
                    65:ef:b3:67:e0:40:e0:29:9f:f9:e3:82:18:7b:06:
                    6d:03:64:09:50:87:aa:fd:30:b0:2b:bf:ec:31:62:
                    59:80:56:59:6d:63:af:63:21:6a:76:81:ab:71:0f:
                    0a:13:4a:51:ca:81:36:ee:3b:b7:d1:83:f6:28:9c:
                    ba:70:6a:67:f1:6b:3d:e9:89:51:a5:0c:16:88:65:
                    95:b7:c0:53:a3:c6:eb:05:df:b5:1f:cf:3a:e8:d2:
                    19:d1:03:4c:35:17:dd:53:ea:54:b6:8f:23:3e:46:
                    75:ae:ed:32:a9:75:68:54:b8:2b:2a:14:a3:2b:35:
                    4e:00:15:58:0d:75:15:b8:15:0e:33:ff:21:ae:47:
                    4b:e3:df:b1:b5:93:00:14:69:88:18:96:0c:b1:82:
                    0a:b3
                Exponent: 65537 (0x10001)
        Attributes:
            a0:00
    Signature Algorithm: sha256WithRSAEncryption
         25:05:06:6d:93:74:44:3f:47:53:39:9c:a5:aa:df:8f:a2:cb:
         e2:f3:75:0b:0f:1a:f0:13:0c:a7:13:8a:5c:78:7a:95:ba:ab:
         2f:02:3f:f1:a8:1e:b1:5b:56:15:6c:ce:d6:c3:c0:70:02:32:
         a7:74:ea:a6:c5:24:a4:9f:b6:b7:ee:ed:d4:7c:3d:36:71:12:
         19:45:24:92:5b:84:5e:e4:57:67:91:ab:e4:64:40:87:a1:f6:
         72:6d:d3:e2:a9:ee:fb:21:ea:5b:a7:5a:70:7b:5c:30:5a:43:
         6c:c2:9c:ba:9c:d9:56:c5:f9:2e:3a:82:44:96:ce:90:43:1d:
         cf:1a:be:f0:0b:40:05:95:e3:71:f9:a6:ee:b4:91:24:2a:78:
         9d:0e:dd:b5:03:a1:38:aa:27:3a:d8:eb:b8:35:3f:7d:0f:ec:
         da:b0:06:b9:ba:5d:3f:ee:bc:e3:fb:31:ea:f4:f0:22:9f:fd:
         de:c5:8c:36:69:88:9a:46:65:ea:f9:f4:6d:83:be:5d:7d:65:
         4e:aa:6d:c7:64:25:24:a0:49:15:0e:87:03:a1:66:81:92:1b:
         bb:1c:4e:b7:a5:39:c1:1e:c8:46:ab:16:8e:cc:17:84:aa:5e:
         34:b2:fb:83:a0:2d:7c:56:10:a7:1e:9f:84:a8:63:61:5f:27:
         d8:9f:75:15
Check contents of CRT
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Certificate:
    Data:
        Version: 3 (0x2)
        Serial Number:
            84:85:50:93:73:00:c4:c5
    Signature Algorithm: sha256WithRSAEncryption
        Issuer: C=JP, ST=Tokyo, L=Minato-ku, O=Contoso, OU=Docs,Contoso, CN=Contoso Insecure Certificate Authority
        Validity
            Not Before: Oct  8 16:20:43 2022 GMT
            Not After : Oct  5 16:20:43 2032 GMT
        Subject: C=JP, ST=Tokyo, L=Minato-ku, O=Contoso, OU=Docs,Contoso, CN=192.168.11.21
        Subject Public Key Info:
            Public Key Algorithm: rsaEncryption
                Public-Key: (2048 bit)
                Modulus:
                    00:b2:ef:6f:d3:b0:dd:cf:6a:65:a9:28:13:de:7a:
                    c9:7b:67:31:80:0b:5b:d5:1b:56:ac:9c:b9:83:ee:
                    51:bc:d7:e3:88:3a:bd:fa:df:5c:d1:21:af:50:fe:
                    c7:66:dc:dc:c7:07:f6:3f:5b:f2:45:a4:4b:1f:bb:
                    23:81:c3:c9:78:27:69:b9:59:82:ef:60:e9:49:9d:
                    5a:7a:5f:6a:fd:f1:b9:e9:ed:1f:f4:84:98:84:1d:
                    9b:7c:61:8e:79:94:7f:f5:03:a4:e6:08:ef:01:d3:
                    65:ef:b3:67:e0:40:e0:29:9f:f9:e3:82:18:7b:06:
                    6d:03:64:09:50:87:aa:fd:30:b0:2b:bf:ec:31:62:
                    59:80:56:59:6d:63:af:63:21:6a:76:81:ab:71:0f:
                    0a:13:4a:51:ca:81:36:ee:3b:b7:d1:83:f6:28:9c:
                    ba:70:6a:67:f1:6b:3d:e9:89:51:a5:0c:16:88:65:
                    95:b7:c0:53:a3:c6:eb:05:df:b5:1f:cf:3a:e8:d2:
                    19:d1:03:4c:35:17:dd:53:ea:54:b6:8f:23:3e:46:
                    75:ae:ed:32:a9:75:68:54:b8:2b:2a:14:a3:2b:35:
                    4e:00:15:58:0d:75:15:b8:15:0e:33:ff:21:ae:47:
                    4b:e3:df:b1:b5:93:00:14:69:88:18:96:0c:b1:82:
                    0a:b3
                Exponent: 65537 (0x10001)
        X509v3 extensions:
            X509v3 Subject Alternative Name:
                DNS:localhost, IP Address:192.168.11.21, IP Address:127.0.0.1
            X509v3 Basic Constraints:
                CA:FALSE
            X509v3 Key Usage:
                Digital Signature, Non Repudiation, Key Encipherment
    Signature Algorithm: sha256WithRSAEncryption
         be:87:a3:a7:db:01:0c:68:7d:f7:5e:cc:9b:57:5b:7b:3f:d0:
         cc:60:06:d7:1d:fd:fa:58:7a:c9:aa:dd:4f:a8:62:5d:a6:3a:
         26:58:29:b2:fa:c2:b3:4e:1a:6f:ca:c3:0a:fc:a9:15:56:fe:
         84:35:3e:e9:b1:63:b4:e2:23:75:53:3b:66:ae:2b:2a:fa:b7:
         95:41:66:f6:a1:6c:87:ad:9f:f7:64:c7:7d:c2:0f:f8:4a:b2:
         b0:0d:a8:26:3c:bd:52:2e:84:7a:b9:0d:ff:b6:f4:ed:ce:da:
         01:7d:b8:de:3a:f7:b4:2e:99:a1:67:74:f2:05:95:83:88:f7:
         da:1e:3b:3f:5e:fe:d8:50:cb:27:84:16:7c:c5:d9:ae:a0:0f:
         ad:d2:f9:17:21:ac:4c:7a:64:c7:b0:4e:36:99:18:73:1f:9e:
         79:26:15:b6:86:93:3c:67:03:dd:25:46:75:6d:6e:1a:12:d5:
         5f:6b:51:bd:ba:a7:7c:09:83:6f:35:a1:21:e5:e3:72:57:1a:
         3a:18:2c:44:ec:63:69:02:1b:2a:d8:60:e1:77:de:ac:bc:3d:
         ee:93:78:11:32:ea:63:bd:7c:7f:0b:ac:5a:21:3a:cd:ab:03:
         ab:8c:cd:5b:c8:09:9b:e3:d1:be:dc:5f:01:37:3b:fe:d8:1d:
         d2:18:49:83:bf:a7:6d:d0:a7:a3:68:c2:5e:3a:c4:2c:32:63:
         12:f0:de:6d:96:b7:2a:24:b2:27:4b:12:e3:a7:d2:02:d1:6f:
         04:e5:e8:95:7f:73:cf:5a:0f:79:ff:03:be:38:ee:cf:ff:2b:
         12:84:22:32:dc:a9:15:e3:32:2a:c7:ab:fa:55:49:c2:35:7e:
         41:58:63:e2:59:1e:78:89:9a:87:ed:3e:93:ef:f9:99:44:31:
         05:ad:9a:79:b3:e8:c6:4b:1b:ee:fa:ca:5e:a6:13:75:dd:72:
         9f:02:c1:62:cd:23:2b:82:00:4f:99:d1:bc:a5:00:a8:ac:af:
         80:83:04:ac:2f:bb:4b:79:6e:24:9e:ab:57:d2:da:98:67:db:
         aa:c3:97:17:de:43:02:ca:37:e8:e8:a1:31:c0:0a:fe:86:b0:
         f7:d6:61:83:36:00:59:e6:cc:47:c0:6e:d4:32:58:8e:94:87:
         ce:7e:74:81:fd:8c:60:6f:65:b6:e6:e6:98:3d:93:2a:7c:f4:
         84:8d:19:dc:97:7d:eb:b0:5b:76:17:15:de:4d:bf:bc:8a:67:
         5d:e3:b8:0a:5a:f2:6a:cf:8d:fa:09:b0:cd:17:a2:2b:61:e7:
         25:2a:8e:ff:17:bf:0c:ad:8f:d8:fd:53:b4:7e:b1:a6:a6:87:
         54:40:83:55:54:1a:25:7e
````

You can see the following files.

* ca.crt
* ca.key
* server.crt
* server.csr
* server.key