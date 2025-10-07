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

First, you need to create certificates.
You can use these scripts

* [Misc/10_GenerateCertificateIPAddress](../../../Misc/10_GenerateCertificateIPAddress)

Then, copy all generated files (ca.crt, ca.key, server.crt, server.csr and server.key) into [WebServer/nginx/03_IPAOverTheAir](WebServer/nginx/03_IPAOverTheAir).

You can not change ip address after create certificate.

#### 03. Generate IPA files

1. Select **Custom**

<img src="./images/xcode/xcode-01.png" height="400" />

2. Select **Release Testing**

<img src="./images/xcode/xcode-02.png" height="400" />

3. Check **Include manifest for over-the-air installation** and select **Next**

<img src="./images/xcode/xcode-03.png" height="400" />

4. Input **Name**, **App URL**, **Display imaeg URL** and  **Full Size imaeg URL** and select **Next**
IP Address SHALL be same value as created certififacate.

<img src="./images/xcode/xcode-04.png" height="400" />

5. Choise **Automatically manage signing** or **Manually manage signing** and select **Next**

<img src="./images/xcode/xcode-05.png" height="400" />

6. Choise **Automatically manage signing** or **Manually manage signing** and select **Next**

<img src="./images/xcode/xcode-06.png" height="400" />

7. (Case if you choised Manually manage signing) Select proper certificate and app, then select **Next**

<img src="./images/xcode/xcode-07.png" height="400" />

8. Select **Export**

<img src="./images/xcode/xcode-08.png" height="400" />

After this, you will see ***.ipa** file and **manifest.plist**.

#### 03. Deploy nginx

Download and deploy nginx.
This script download nginx and deploy it.

````json
$ pwsh ConfigureNginx.ps1
````

#### 04. Deploy IPA files

Copy ***.ipa** file and **manifest.plist** into [WebServer/nginx/03_IPAOverTheAir/nginx/html](WebServer/nginx/03_IPAOverTheAir/nginx/html).

#### 05. Start nginx

````bash
$ pwsh Run.ps1
````

#### 06. Install Root CA file

1. Tap **1. Install self-signed certificate**

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