# IPA Over The Air

## Abstracts

* How to setup OTA (Over Tha Air) for IPA file

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

* [Misc/10_GenerateCertificateIPAddress](../../../Misc/10_GenerateCertificateIPAddress)

Then, copy all generated files (ca.crt, ca.key, server.crt, server.csr and server.key) into here.

#### 02. Deploy nginx

Download and deploy nginx.

````json
$ pwsh ../ConfigureNginx.ps1
````

#### 03. Deploy IPA file

Copy ***.ipa** file and **manifest.plist** into [WebServer/nginx/03_IPAOverTheAir/nginx/html](WebServer/nginx/03_IPAOverTheAir/nginx/html).

#### 04. Start nginx

````bash
$ pwsh Run.ps1
````

#### 05. Install Root CA file

1. Clie **1. Install self-signed certificate**

<img src="./images/IMG_0237.png" height="400" />

2. Select device (Thid dialog may be not present)

<img src="./images/IMG_0238.png" height="400" />
<br>
<img src="./images/IMG_0239.png" height="400" />

3. Open **Setting -> General -> VPN & Device Management** and select downloaded profile

<img src="./images/IMG_0240.png" height="400" />

4. Tap **Install**

<img src="./images/IMG_0241.png" height="400" />

5. Tap **Install**

<img src="./images/IMG_0242.png" height="400" />

6. Input passcode

<img src="./images/IMG_0243.png" height="400" />

7. Tap **Install**

<img src="./images/IMG_0244.png" height="400" />

8. Tap check button

<img src="./images/IMG_0245.png" height="400" />

9. Open **Setting -> General -> About -> Certificate Trust Settings** and select installed profile

<img src="./images/IMG_0246.png" height="400" />

10. Tap **Continue**

<img src="./images/IMG_0247.png" height="400" />

11. Installed profile is enabled

<img src="./images/IMG_0248.png" height="400" />

12. Tap **2. Install iOS app via OTA** and select **Install**

<img src="./images/IMG_0250.png" height="400" />

If tap **2. Install iOS app via OTA** in http page, you will see

<img src="./images/IMG_0249.png" height="400" />