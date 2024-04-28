# Private Certificate Authority

## Abstracts

* Generate private Certificate Authority (CA)
* Support signing private server's certificate signing request

## Requirements

* PowerShell Core

## Dependencies

* [OpenSSL](https://github.com/PrismLibrary/Prism)
  * OpenSSL License/SSLeay License
    * Download 1.0.2u from https://indy.fulgan.com/SSL/
    * openssl.cnf is obtained from source code

## How to use

#### 0. Setup

At first, copy or rename `root-ca.conf.in` to `root-ca.conf`.
And modify it.
For example, you can set your information to `[req_dn]`.

````txt
[req_dn]
countryName = "JP"
stateOrProvinceName = "Tokyo"
localityName = "Minato-ku"
organizationName ="Contoso"
organizationalUnitName = "Docs, Contoso"
commonName = "Contoso Root CA"
emailAddress = "ca@contso.com"
````

Then, you kick only powershell scripts.

#### 1. Initialize

You should kick this script only 1 time.

````bat
$ pwsh 01_Initialize.ps1
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
unable to write 'random state'
````

#### 2. Creating Root CA private key and certificate signing request

````bat
$ pwsh 02_GenerateRootCA.ps1
Create private key and certificate signing request...
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Generating a RSA private key
......................................................................................................................................................++++
...++++
unable to write 'random state'
writing new private key to 'E:\Works\OpenSource\Demo2\Misc\21_Test\pki\CA\private\root-ca.key'
-----
Do Self-Sign certificate signing request...
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Using configuration from E:\Works\OpenSource\Demo2\Misc\21_Test\root-ca.conf
Check that the request matches the signature
Signature ok
Certificate Details:
Certificate:
    Data:
        Version: 3 (0x2)
        Serial Number:
            f7:ea:40:85:61:19:db:5e:b2:d7:f8:d6:eb:57:f8:f6
        Issuer:
            countryName               = JP
            stateOrProvinceName       = Tokyo
            organizationName          = Contoso
            organizationalUnitName    = Docs, Contoso
            localityName              = Minato-ku
            commonName                = Contoso Root CA
            emailAddress              = ca@contso.com
        Validity
            Not Before: Apr 28 18:26:24 2024 GMT
            Not After : Apr 26 18:26:24 2034 GMT
        Subject:
            countryName               = JP
            stateOrProvinceName       = Tokyo
            organizationName          = Contoso
            organizationalUnitName    = Docs, Contoso
            localityName              = Minato-ku
            commonName                = Contoso Root CA
            emailAddress              = ca@contso.com
        Subject Public Key Info:
            Public Key Algorithm: rsaEncryption
                Public-Key: (4096 bit)
                Modulus:
                    00:bc:e5:0e:b8:b2:3b:cc:79:74:35:62:d6:f8:33:
                    5d:16:36:26:6a:10:38:15:02:fa:4b:01:17:68:66:
                    b8:13:61:77:12:fe:64:2d:11:86:20:01:52:43:35:
                    10:b5:e0:e5:17:a2:b8:bd:cb:de:6b:71:52:02:5c:
                    ac:b3:6d:d1:7b:3e:c2:1e:be:6e:f0:a0:3a:45:85:
                    e2:fb:62:35:f4:87:c9:c2:58:7c:5c:29:0d:4f:0b:
                    15:4f:f1:63:58:ec:69:75:b7:53:32:8a:2d:fc:16:
                    b2:37:08:51:2d:b1:31:4b:d7:c9:62:ef:98:ef:58:
                    ac:e0:f1:9e:f1:6e:3b:23:56:0c:1f:04:ef:a8:6b:
                    6f:6e:1f:37:91:a9:48:fb:81:97:d8:2c:e8:50:85:
                    84:00:36:f4:ea:a4:e0:0f:54:95:5d:ae:13:bf:ab:
                    20:3e:33:26:dd:28:a7:ef:89:e4:3d:fc:3b:2d:f4:
                    63:15:2d:13:ff:e3:4e:73:df:f8:18:dc:16:f2:5f:
                    7b:7a:d9:9c:bb:fc:2e:6d:5e:69:14:61:b6:3e:08:
                    ad:00:94:b6:41:15:f3:85:dc:92:34:80:cb:31:38:
                    9c:1c:d5:9d:c1:48:f4:5f:78:90:92:53:84:19:3d:
                    8e:56:a8:a6:e0:1e:62:c0:9b:be:b4:eb:f8:e7:b5:
                    05:61:23:03:83:d1:dc:36:ef:bd:0a:e5:d9:e4:ac:
                    3c:70:96:20:6d:2e:c7:1e:36:d8:fe:4c:5b:50:39:
                    60:13:64:59:90:0e:bc:91:dc:21:6b:6d:22:85:ec:
                    66:92:62:d7:b3:3a:21:35:a1:dd:88:6b:25:04:2f:
                    c6:b5:6e:9a:75:29:bd:56:93:2a:3c:d5:55:60:e0:
                    ff:cc:27:3b:79:70:57:85:48:47:9f:45:da:c1:ba:
                    4f:8e:47:70:a1:9a:9f:b3:91:91:0d:e2:45:6b:18:
                    00:81:2f:e6:56:79:62:ad:df:85:42:5d:6b:1a:e5:
                    e9:fd:9f:78:b5:c5:70:fd:9b:fd:5d:33:2a:33:17:
                    eb:5e:21:2a:fc:c3:ce:22:4a:72:c6:cb:3a:15:47:
                    c9:77:e8:51:e6:b7:fd:32:6f:30:56:34:4b:5b:ce:
                    63:f9:18:bd:72:8f:35:37:0e:b2:d6:5c:9b:0e:85:
                    fa:6b:f0:bf:53:47:b7:e0:19:65:0f:1b:ea:5d:68:
                    eb:03:6c:14:72:8b:1b:49:dd:fc:c3:86:58:45:26:
                    5c:a3:68:83:f8:d5:54:c8:00:83:be:43:13:f0:65:
                    48:7b:af:09:70:17:55:81:5b:c6:e4:e0:79:55:86:
                    c2:b6:86:ad:21:48:62:c4:c9:8a:89:c9:36:57:79:
                    64:e3:15
                Exponent: 65537 (0x10001)
        X509v3 extensions:
            X509v3 Basic Constraints: critical
                CA:TRUE
            X509v3 Key Usage: critical
                Certificate Sign
            X509v3 Subject Key Identifier:
                DC:9D:B3:F6:5F:B8:F5:16:CC:E1:8C:BC:A1:B7:23:4F:6B:B8:53:25
Certificate is to be certified until Apr 26 18:26:24 2034 GMT (3650 days)
Sign the certificate? [y/n]:y


1 out of 1 certificate requests certified, commit? [y/n]y
Write out database with 1 new entries
Data Base Updated
unable to write 'random state'
````

#### 3. Check server's certificate signing request (optional)

You can see content of server's certificate signing request.
This sample is VMWare Esxi's certificate signing request.

````bat
$ pwsh 03_CheckServerCSR.ps1
Check server's certificate signing request...
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Unable to load config info from /usr/local/ssl/openssl.cnf
Certificate Request:
    Data:
        Version: 0 (0x0)
        Subject: C=US, ST=California, L=Palo Alto, O=VMware, Inc, OU=VMware ESX Server Default Certificate/emailAddress=ssl-certificates@vmware.com, CN=192.168.11.43/unstructuredName=1672285880,564d7761726520496e632e
        Subject Public Key Info:
            Public Key Algorithm: rsaEncryption
                Public-Key: (2048 bit)
                Modulus:
                    00:cd:30:a1:16:0f:2a:da:2b:ea:6e:f9:26:e2:68:
                    1d:29:20:9c:02:5d:33:40:f7:7e:aa:68:88:cf:08:
                    b4:f2:2c:38:f2:ae:de:59:6a:06:17:2c:98:07:f0:
                    e2:29:2f:74:1a:5a:7f:c4:50:8f:cb:7d:1f:12:12:
                    ad:e3:03:18:f3:d1:fa:79:01:d6:c2:32:71:2f:cd:
                    e3:70:d7:ab:b5:66:2a:4f:11:c0:1f:ad:06:de:db:
                    3d:5c:5e:49:0a:85:a2:06:fb:3e:68:25:d2:7d:8a:
                    04:7a:46:a5:e0:72:d3:35:eb:c3:20:e3:3c:0f:dd:
                    38:63:d5:70:3c:3c:b1:22:91:24:f6:13:45:f3:43:
                    59:87:22:14:d1:21:d3:d3:e0:8c:7e:58:4f:fa:03:
                    4e:25:90:13:3b:6b:b0:e7:27:36:c3:1c:15:aa:d0:
                    9d:92:a8:4d:06:2f:18:02:e0:10:3a:af:81:82:e4:
                    55:d6:86:57:22:11:a9:62:e0:0f:59:52:e2:eb:68:
                    03:b2:90:d9:b0:c6:e1:ab:aa:cc:c3:54:ea:d6:43:
                    68:70:08:f0:db:01:dc:ab:d1:e3:7e:fa:d5:9b:52:
                    73:78:8b:e6:e4:fb:89:79:2d:33:a5:8c:62:ce:74:
                    2e:5c:ba:ad:34:34:f0:a6:6d:5f:84:8b:8a:36:dc:
                    9c:29
                Exponent: 65537 (0x10001)
        Attributes:
        Requested Extensions:
            X509v3 Subject Alternative Name:
                IP Address:192.168.11.43
    Signature Algorithm: sha256WithRSAEncryption
         cb:3f:e3:6e:fc:7b:75:76:ac:6a:b7:fc:7f:cb:88:93:19:a4:
         b4:d6:57:f8:37:3d:07:f0:b5:0b:7f:ec:97:e4:8e:a8:4f:81:
         fb:3a:bf:1f:c2:6a:ec:3f:d0:6c:45:eb:f7:7a:12:fe:df:8f:
         9b:ff:33:39:2e:e7:18:1f:3c:2d:ae:9b:b0:62:f5:97:22:73:
         6c:a4:b1:9e:21:7f:b7:c8:c9:d3:3e:e6:7b:a8:35:de:2a:83:
         a0:40:3c:8e:70:88:ed:9e:ae:f3:cd:b1:88:a8:ee:37:a0:8c:
         62:3f:90:ba:cb:b4:86:9e:a2:f7:0a:7e:8a:18:66:40:d0:0a:
         d5:2f:a7:ea:f2:e5:00:39:ac:58:ed:ed:56:dd:d9:75:c9:e6:
         89:60:fb:98:ff:d6:35:c4:2f:a2:31:0c:c4:c6:f4:6a:b4:6b:
         17:73:49:07:08:3b:68:dc:4b:3e:48:c7:d5:10:54:0a:fc:d4:
         7a:14:7b:c9:ed:e7:a5:6f:60:f3:92:59:0e:13:d7:85:c3:b6:
         7c:4d:bf:1e:8e:12:e8:0d:31:60:8e:90:27:80:ff:41:40:ab:
         b2:c4:98:f2:23:07:cd:26:54:b5:40:ad:68:df:77:0f:18:32:
         b1:73:a7:42:4c:1b:c9:3a:4f:69:4a:e6:4e:1e:cc:e8:5a:de:
         9e:8a:3c:94
````

#### 4. Sign server's certificate signing request by root CA private key (optional)

Sign server's certificate signing request and create signed certificate.
You will see `server.csr` and deploy it to server.

````bat
$ pwsh 04_SignServerCSR.ps1
Do sign server's certificate signing request...
WARNING: can't open config file: /usr/local/ssl/openssl.cnf
Using configuration from sign-server.conf
Check that the request matches the signature
Signature ok
Certificate Details:
Certificate:
    Data:
        Version: 3 (0x2)
        Serial Number:
            f7:ea:40:85:61:19:db:5e:b2:d7:f8:d6:eb:57:f8:f7
        Issuer:
            countryName               = JP
            stateOrProvinceName       = Tokyo
            organizationName          = Contoso
            organizationalUnitName    = Docs, Contoso
            localityName              = Minato-ku
            commonName                = Contoso Root CA
            emailAddress              = ca@contso.com
        Validity
            Not Before: Apr 28 18:26:34 2024 GMT
            Not After : Apr 28 18:26:34 2025 GMT
        Subject:
            countryName               = US
            stateOrProvinceName       = California
            organizationName          = VMware, Inc
            organizationalUnitName    = VMware ESX Server Default Certificate
            localityName              = Palo Alto
            commonName                = 192.168.11.43
            emailAddress              = ssl-certificates@vmware.com
        Subject Public Key Info:
            Public Key Algorithm: rsaEncryption
                Public-Key: (2048 bit)
                Modulus:
                    00:cd:30:a1:16:0f:2a:da:2b:ea:6e:f9:26:e2:68:
                    1d:29:20:9c:02:5d:33:40:f7:7e:aa:68:88:cf:08:
                    b4:f2:2c:38:f2:ae:de:59:6a:06:17:2c:98:07:f0:
                    e2:29:2f:74:1a:5a:7f:c4:50:8f:cb:7d:1f:12:12:
                    ad:e3:03:18:f3:d1:fa:79:01:d6:c2:32:71:2f:cd:
                    e3:70:d7:ab:b5:66:2a:4f:11:c0:1f:ad:06:de:db:
                    3d:5c:5e:49:0a:85:a2:06:fb:3e:68:25:d2:7d:8a:
                    04:7a:46:a5:e0:72:d3:35:eb:c3:20:e3:3c:0f:dd:
                    38:63:d5:70:3c:3c:b1:22:91:24:f6:13:45:f3:43:
                    59:87:22:14:d1:21:d3:d3:e0:8c:7e:58:4f:fa:03:
                    4e:25:90:13:3b:6b:b0:e7:27:36:c3:1c:15:aa:d0:
                    9d:92:a8:4d:06:2f:18:02:e0:10:3a:af:81:82:e4:
                    55:d6:86:57:22:11:a9:62:e0:0f:59:52:e2:eb:68:
                    03:b2:90:d9:b0:c6:e1:ab:aa:cc:c3:54:ea:d6:43:
                    68:70:08:f0:db:01:dc:ab:d1:e3:7e:fa:d5:9b:52:
                    73:78:8b:e6:e4:fb:89:79:2d:33:a5:8c:62:ce:74:
                    2e:5c:ba:ad:34:34:f0:a6:6d:5f:84:8b:8a:36:dc:
                    9c:29
                Exponent: 65537 (0x10001)
        X509v3 extensions:
            X509v3 Authority Key Identifier:
                keyid:DC:9D:B3:F6:5F:B8:F5:16:CC:E1:8C:BC:A1:B7:23:4F:6B:B8:53:25

            X509v3 Basic Constraints: critical
                CA:FALSE
            X509v3 Extended Key Usage:
                TLS Web Server Authentication
            X509v3 Key Usage: critical
                Digital Signature, Key Encipherment
            X509v3 Subject Key Identifier:
                6B:AC:76:08:D3:A6:BC:B5:88:14:08:74:1D:6D:77:9B:05:6F:95:FF
            X509v3 Subject Alternative Name:
                IP Address:192.168.11.43
Certificate is to be certified until Apr 28 18:26:34 2025 GMT (365 days)
Sign the certificate? [y/n]:y


1 out of 1 certificate requests certified, commit? [y/n]y
Write out database with 1 new entries
Data Base Updated
unable to write 'random state'
````

#### 5. Install Root CA certificate to client (optional)

Install `root-ca.crt` into `Trusted Root CA` store in system certification manager.
Otherwise, client machine can not trust server's certification if not install `root-ca.crt`.
So web brower keep showing certification warning.