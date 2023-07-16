# DeepLink 

## Abstracts

* How to implement Deeplink
* Implement platform service and consume via DryIoc

## Requirements

* Visual Studio 2022
* .NET 7.0

### iOS

* Apple Developer Program

## Dependencies

* [Prism.Maui](https://github.com/PrismLibrary/Prism)
  * MIT License
* [Prism.DryIoc.Maui](https://github.com/PrismLibrary/Prism)
  * MIT License
* [Prism.Maui.Rx](https://github.com/PrismLibrary/Prism)
  * MIT License

## Preparation

### Android

#### 1. Generete KeyStore

Run [GenereteKey.ps1](../GenereteKey.ps1)

````cmd
$ pwsh GenerateKey.ps1
keytool: 'C:\Program Files\Java\jdk-17.0.6\bin\keytool.exe'
Delete Key...
Generate Key...
10,950日間有効な2,048ビットのRSAのキー・ペアと自己署名型証明書(SHA256withRSA)を生成しています
        ディレクトリ名: CN=Android Debug, O=Android, C=US
[debug.keystoreを格納中]
Show keystore list ...
キーストアのパスワードを入力してください:
キーストアのタイプ: PKCS12
キーストア・プロバイダ: SUN

キーストアには1エントリが含まれます

別名: androiddebugkey
作成日: 2023/07/16
エントリ・タイプ: PrivateKeyEntry
証明書チェーンの長さ: 1
証明書[1]:
所有者: CN=Android Debug, O=Android, C=US
発行者: CN=Android Debug, O=Android, C=US
シリアル番号: fd03a206c0b221c4
有効期間の開始日: Sun Jul 16 22:34:58 JST 2023終了日: Tue Jul 08 22:34:58 JST 2053
証明書のフィンガプリント:
         SHA1: 65:34:18:50:5C:A1:96:9B:86:97:D8:40:5A:28:65:DE:DF:FE:51:63
         SHA256: 8D:E7:B8:B8:DC:0D:03:58:34:50:C2:75:B7:12:C6:B1:93:A3:20:55:40:02:E2:B2:A3:E2:43:ED:EF:08:B7:11
署名アルゴリズム名: SHA256withRSA
サブジェクト公開キー・アルゴリズム: 2048ビットRSAキー
バージョン: 3

拡張:

#1: ObjectId: 2.5.29.14 Criticality=false
SubjectKeyIdentifier [
KeyIdentifier [
0000: 5C EE 6E 4C CF D1 AF 35   2B 81 CC 19 18 8F E5 0A  \.nL...5+.......
0010: A8 5F 32 1F                                        ._2.
]
]



*******************************************
*******************************************
````

You will obtain SHA256 value.

#### 2. Edit Target.csproj

Edit [Target.csproj](./sources/Target/Target.csproj) to set proper value to `AndroidSigningKeyStore`.
It shall be absolute path.

#### 3. Modify assetlinks.json

Replace `sha256_cert_fingerprints` with SHA256 value of generated key store in [assetlinks.json](./assetlinks.json)

#### 4. Deploy assetlinks.json

Deploy assetlinks.json to .well-known/assetlinks.json on your web server with your domain.
This domain shall support https.

#### 5. Check assetlinks.json

We can verify assetlinks.json by using Digital Asset Links API.

````shell
$ curl "https://digitalassetlinks.googleapis.com/v1/statements:list?source.web.site=https://taktak.jp&relation=delegate_permission/common.handle_all_urls"
{
 "statements": [
   {
     "source": {
       "web": {
         "site": "https://taktak.jp."
       }
     },
     "relation": "delegate_permission/common.handle_all_urls",
     "target": {
       "androidApp": {
         "packageName": "takuyatakeuchi.demo.applinkstarget",
         "certificate": {
           "sha256Fingerprint": "73:41:E5:FA:67:F0:E0:59:EA:66:CB:05:B2:8A:34:29:08:26:B2:A9:2B:A7:36:07:0E:8B:25:2F:F2:D4:46:9C"
         }
       }
     }
   }
 ],
 "maxAge": "88.876309048s"
}
````

#### 6. Deploy apps

Build and deploy Taget and Source into device or simulator.

### iOS

#### 1. Modify apple-app-site-association

Replace appIDs with modified Team and Bundle Identifier in [apple-app-site-association](./apple-app-site-association)

#### 2. Modify 

Modify [Entitlements.plist](./sources/Target/Platforms/iOS/Entitlements.plist).
This domain shall be your domain.

#### 3. Deploy apple-app-site-association

Deploy apple-app-site-association to .well-known/apple-app-site-association on your web server with your domain.
This domain shall support https.

#### 4. Validate apple-app-site-association

You can use the following validator to check whether apple-app-site-association file is deployed correctly.

* https://branch.io/resources/aasa-validator/
* https://yurl.chayev.com/

#### 5. Check Apple CDN

You can check whether apple-app-site-association is cached by Apple CDN. For example,

````shell
curl -v https://app-site-association.cdn-apple.com/a/v1/taktak.jp

*   Trying [2001:1900:2389:1f::1fc]:443...
* Connected to app-site-association.cdn-apple.com (2001:1900:2389:1f::1fc) port 443 (#0)
* ALPN: offers h2
* ALPN: offers http/1.1
*  CAfile: /etc/ssl/cert.pem
*  CApath: none
* (304) (OUT), TLS handshake, Client hello (1):
* (304) (IN), TLS handshake, Server hello (2):
* (304) (IN), TLS handshake, Unknown (8):
* (304) (IN), TLS handshake, Certificate (11):
* (304) (IN), TLS handshake, CERT verify (15):
* (304) (IN), TLS handshake, Finished (20):
* (304) (OUT), TLS handshake, Finished (20):
* SSL connection using TLSv1.3 / AEAD-AES256-GCM-SHA384
* ALPN: server accepted http/1.1
* Server certificate:
*  subject: CN=app-site-association.cdn-apple.com; O=Apple Inc.; ST=California; C=US
*  start date: Aug 31 17:34:05 2022 GMT
*  expire date: Sep 30 17:34:04 2023 GMT
*  subjectAltName: host "app-site-association.cdn-apple.com" matched cert's "app-site-association.cdn-apple.com"
*  issuer: CN=Apple Public Server RSA CA 12 - G1; O=Apple Inc.; ST=California; C=US
*  SSL certificate verify ok.
> GET /a/v1/taktak.jp HTTP/1.1
> Host: app-site-association.cdn-apple.com
> User-Agent: curl/7.86.0
> Accept: */*
> 
* Mark bundle as not supporting multiuse
< HTTP/1.1 200 OK
< Date: Fri, 02 Jun 2023 16:52:22 GMT
< Content-Type: application/json
< Content-Length: 1203
< Connection: keep-alive
< Apple-From: https://taktak.jp/.well-known/apple-app-site-association
< Apple-Origin-Format: json
< Cache-Control: public, max-age=3600
< X-Cache: MISS
< CDNUUID: 90db5ad6-95bd-4779-a1b0-e549cc00a1cc-1323606289
< CDN-Server: lumn
< Age: 28
< Accept-Ranges: bytes
< 
{
   "applinks": {
      "details": [
         {
            "appIDs": [
               "U27JXXX492.takuyatakeuchi.demo.universallinktarget"
            ],
            "components": [
               {
                  "#": "no_universal_links",
                  "exclude": true,
                  "comment": "Matches any URL with a fragment that equals no_universal_links and instructs the system not to open it as a universal link."
               },
               {
                  "/": "/help/website/*",
                  "exclude": true,
                  "comment": "Matches any URL with a path that starts with /help/website/ and instructs the system not to open it as a universal link."
               },
               {
                  "/": "/help/*",
                  "?": { "articleNumber": "????" },
                  "comment": "Matches any URL with a path that starts with /help/ and that has a query item with name 'articleNumber' and a value of exactly four characters."
               },
               {
                  "/": "/buy/*",
                  "comment": "Matches any URL with a path that starts with /buy/."
               }
            ]
         }
      ]
   }
* Connection #0 to host app-site-association.cdn-apple.com left intact
}% 
````

#### 7. Deploy apps

Build and deploy Target and Source into device or simulator.

## Result

|Android|iOS|
|---|---|
|<img src="images/android.gif?raw=true" title="ios" width="296"/>|<img src="images/ios.gif?raw=true" title="android"/>|
