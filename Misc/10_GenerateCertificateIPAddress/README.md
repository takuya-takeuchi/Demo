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
Can't load /home/t-takeuchi/.rnd into RNG
139724968452544:error:2406F079:random number generator:RAND_load_file:Cannot open file:../crypto/rand/randfile.c:88:Filename=/home/t-takeuchi/.rnd
Generating a RSA private key
...........+++++
...................................................................+++++
writing new private key to 'server.key'
-----
Certificate:
    Data:
        Version: 3 (0x2)
        Serial Number:
            35:84:25:8e:0b:22:31:7d:94:f4:d3:c2:b0:f5:c6:2e:c0:b1:dd:82
        Signature Algorithm: sha256WithRSAEncryption
        Issuer: C = JP, ST = Tokyo, L = Minato-ku, O = Contoso, OU = "Docs,Contoso", CN = 192.168.11.21
        Validity
            Not Before: Oct  8 14:44:44 2022 GMT
            Not After : Oct  5 14:44:44 2032 GMT
        Subject: C = JP, ST = Tokyo, L = Minato-ku, O = Contoso, OU = "Docs,Contoso", CN = 192.168.11.21
        Subject Public Key Info:
            Public Key Algorithm: rsaEncryption
                RSA Public-Key: (2048 bit)
                Modulus:
                    00:df:0e:88:3a:3e:75:14:4c:49:82:36:0d:87:ca:
                    17:0a:68:fb:d1:6f:9c:65:62:a2:8d:de:0d:41:2d:
                    47:e6:f8:d7:53:30:84:3a:ec:2b:94:4d:6a:1a:64:
                    c5:51:c1:f6:50:dc:37:31:b8:14:4f:a4:4b:11:52:
                    0f:ae:79:b2:c5:63:93:4c:e0:f4:58:e9:43:50:e4:
                    f9:d1:3a:09:4c:e7:37:a6:13:51:19:7e:e1:ac:cc:
                    a2:47:ac:60:60:91:29:ca:87:90:ef:3a:6c:50:8e:
                    86:87:ae:54:ef:fc:e1:3c:29:59:79:a6:0f:1f:d6:
                    ac:81:11:e3:b6:3f:4c:9a:73:da:58:50:b3:04:11:
                    4b:c2:07:39:2a:4d:44:c2:fb:7c:5b:1e:7c:f6:ff:
                    a6:7c:1a:7b:78:7d:a1:cb:a5:84:77:0b:22:6d:2d:
                    43:5a:92:bf:65:ef:25:58:44:08:31:eb:df:13:41:
                    e0:c3:b6:a4:9b:0b:13:61:40:16:53:4c:45:3b:c7:
                    74:2f:04:a3:d5:04:55:e0:9d:0a:e2:21:8e:ca:c8:
                    c3:32:4d:bd:27:42:ea:9a:cb:5d:49:5d:ad:06:be:
                    af:3d:41:0a:2d:2d:dd:99:28:3c:07:08:25:e6:1d:
                    65:87:40:3f:70:3f:30:87:41:12:7c:50:3a:63:ad:
                    66:0b
                Exponent: 65537 (0x10001)
        X509v3 extensions:
            X509v3 Basic Constraints: 
                CA:FALSE
            X509v3 Key Usage: 
                Digital Signature, Non Repudiation, Key Encipherment
    Signature Algorithm: sha256WithRSAEncryption
         45:f2:0c:ee:b1:af:99:af:e1:40:c4:39:4d:37:90:21:10:24:
         af:88:a8:1b:e1:0c:73:cc:35:15:72:fb:ad:aa:6a:ee:58:38:
         ca:3b:aa:18:0d:ad:32:73:73:f2:22:df:d7:2a:65:73:e8:0a:
         da:03:c2:dc:bf:fd:2d:12:45:0e:7d:6b:4b:f6:ae:83:63:6c:
         f9:1e:7e:b0:f4:b7:f1:0a:97:01:b7:91:49:51:bf:c7:98:f5:
         31:1d:b7:40:20:3d:41:0a:d8:e4:2e:13:6a:7c:1e:f5:4e:f9:
         c0:7d:14:0d:44:c8:9f:f7:1d:f7:0e:ef:40:8e:5f:01:ad:1a:
         f5:6f:6c:16:25:e9:ae:04:54:f9:a2:34:e6:07:86:2b:ef:4d:
         a3:b5:51:89:e1:dd:3c:30:c7:ef:98:6d:4e:ef:8f:ed:14:73:
         4e:2b:0b:22:29:84:61:b7:56:94:a3:0f:51:8f:bc:d6:23:0a:
         af:42:98:e9:ad:c4:a1:f4:10:09:7a:58:89:2a:1f:82:93:1e:
         34:8e:b4:63:05:29:1e:eb:af:38:c7:48:a3:bc:6a:cf:64:82:
         52:09:8e:31:dc:7c:0b:ef:45:25:0c:b9:37:39:ad:4a:d8:94:
         2d:3f:27:5f:27:49:12:6c:7e:04:60:95:fe:ab:30:3c:fb:77:
         de:77:1a:75
````

You can see server.key and server.crt