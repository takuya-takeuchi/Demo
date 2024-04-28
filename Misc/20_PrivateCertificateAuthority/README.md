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
...............................................++++
...++++
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
            d7:6e:25:55:87:5a:96:91:45:12:f4:23:b5:1a:a0:0e
        Issuer:
            countryName               = JP
            stateOrProvinceName       = Aichi
            organizationName          = Takuya Takeuchi
            organizationalUnitName    = Takuya Takeuchi
            localityName              = Togo-cho
            commonName                = Takuya Takeuchi Root CA
            emailAddress              = spitzbergen@gmail.com
        Validity
            Not Before: Apr 28 18:06:14 2024 GMT
            Not After : Apr 26 18:06:14 2034 GMT
        Subject:
            countryName               = JP
            stateOrProvinceName       = Aichi
            organizationName          = Takuya Takeuchi
            organizationalUnitName    = Takuya Takeuchi
            localityName              = Togo-cho
            commonName                = Takuya Takeuchi Root CA
            emailAddress              = spitzbergen@gmail.com
        Subject Public Key Info:
            Public Key Algorithm: rsaEncryption
                Public-Key: (4096 bit)
                Modulus:
                    00:c0:69:d4:c8:76:91:58:b1:31:83:8f:6b:f8:03:
                    57:f5:ca:d6:fa:41:6a:40:66:54:f0:a3:45:68:e2:
                    ea:0b:d0:fb:30:3b:e8:c0:f1:04:60:87:52:20:53:
                    07:cd:d0:9d:67:95:b8:b5:8c:44:de:d1:4b:11:dd:
                    4f:21:c6:95:5b:e8:e6:90:ab:21:9e:3b:2b:e1:d8:
                    a7:14:eb:30:64:46:de:ba:a9:16:10:ed:be:57:fc:
                    61:a4:65:7e:76:c2:f8:66:99:27:61:95:5b:a2:c4:
                    61:c5:d3:32:64:0e:78:db:15:63:93:00:7e:d1:a0:
                    77:8b:89:55:d1:cd:04:23:06:ed:5c:cf:d6:41:b9:
                    0b:f2:b3:3e:65:e5:ed:3a:97:1e:56:4c:98:a7:fb:
                    92:eb:46:4e:98:21:84:72:2d:a7:38:6f:53:4f:6d:
                    6d:f5:c1:d3:4e:ad:54:86:88:79:6b:f1:99:ea:56:
                    e1:ce:c9:45:50:db:fb:ef:de:a0:ae:03:65:f4:b3:
                    2a:79:ec:c6:20:cf:9a:92:fb:96:03:1e:dc:71:65:
                    29:c1:65:6e:bd:e6:9e:bf:f0:f1:03:89:eb:90:ec:
                    ee:e3:45:d9:ec:b7:3e:b2:a1:dd:1e:ed:81:1a:fb:
                    0c:f2:24:e6:0c:75:6c:4c:5f:8f:a9:da:ef:63:fb:
                    a3:c9:46:ed:52:b9:c2:64:57:c1:0c:a3:c5:4e:08:
                    de:53:c9:79:54:79:7d:34:7c:bb:de:64:43:31:00:
                    78:cb:5e:37:a0:a4:f7:d3:3b:2c:23:6b:38:af:63:
                    ab:ac:5c:6c:d8:22:5a:29:f9:49:1b:b2:65:05:c2:
                    d1:52:39:e8:6f:e3:93:d0:ea:81:73:e8:57:65:ad:
                    8e:70:6b:91:45:0e:66:f0:f6:b0:ca:4c:6d:c0:1b:
                    9d:7c:2c:3b:12:69:94:22:c2:ba:d8:2d:ed:25:cb:
                    10:91:de:d7:83:c8:bf:50:e4:35:1d:17:c4:2f:43:
                    04:8c:c3:a7:82:0c:b2:22:b5:8a:53:05:95:79:5a:
                    1f:5d:c5:c0:2f:de:45:8f:f7:83:2a:df:be:c6:cc:
                    a4:dc:14:a6:66:34:44:98:28:66:02:a2:04:9e:b8:
                    8d:7f:ce:d2:5e:93:10:fc:f0:7d:09:41:d2:92:0e:
                    21:9f:bc:a4:a4:b4:ff:c9:c9:94:7b:e5:63:7c:a3:
                    78:b8:47:77:da:5d:13:2e:43:8d:58:f6:10:53:1e:
                    4f:01:af:69:d2:19:51:b2:1a:4c:82:2b:bb:f2:9f:
                    c6:4f:05:ef:cf:ea:2d:dd:4a:06:2f:73:46:41:d3:
                    77:61:ba:bb:6f:61:21:37:bf:a2:30:8b:ff:98:6d:
                    6a:76:5d
                Exponent: 65537 (0x10001)
        X509v3 extensions:
            X509v3 Basic Constraints: critical
                CA:TRUE
            X509v3 Key Usage: critical
                Certificate Sign
            X509v3 Subject Key Identifier:
                18:11:01:FF:C7:10:E9:97:3E:CA:05:C7:F0:7A:7B:45:65:2F:CB:86
Certificate is to be certified until Apr 26 18:06:14 2034 GMT (3650 days)
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
            d7:6e:25:55:87:5a:96:91:45:12:f4:23:b5:1a:a0:0f
        Issuer:
            countryName               = JP
            stateOrProvinceName       = Aichi
            organizationName          = Takuya Takeuchi
            organizationalUnitName    = Takuya Takeuchi
            localityName              = Togo-cho
            commonName                = Takuya Takeuchi Root CA
            emailAddress              = spitzbergen@gmail.com
        Validity
            Not Before: Apr 28 18:06:46 2024 GMT
            Not After : Apr 28 18:06:46 2025 GMT
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
                keyid:18:11:01:FF:C7:10:E9:97:3E:CA:05:C7:F0:7A:7B:45:65:2F:CB:86

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
Certificate is to be certified until Apr 28 18:06:46 2025 GMT (365 days)
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