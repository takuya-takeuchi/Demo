# Get My Number and Kihon 4 Information

## Abstracts

* Get My Number and Kihon 4 Information by using `Windows API`

## Requirements

### Windows

* Windows 10.0.19041.0 or later
* .NET 8.0 SDK
* SmartCard Reader
* Japan My Number Card

## Dependencies

* [NLog](https://github.com/NLog/NLog)
  * 5.2.8
  * BSD-3-Clause License

## Note

After invoked `Windows.Devices.SmartCards.SmartCardReader.FromIdAsync`, it slows appliction shutdown process.
This take 10-20 sec.
Refer: [Windows.Devices.SmartCards SmartCardReader slows appliction shutdown process](https://stackoverflow.com/questions/77278900/windows-devices-smartcards-smartcardreader-slows-appliction-shutdown-process)

According to Microsoft Community, this deley is specification.
Community member asked to Microsoft and he got answer

> Japanese: Microsoftに問い合わせたところ、ご推察の通りタイムアウト処理が実装されているとのことでした。（ワーカースレッドがSmartCardReaderより先に終了した際に20秒のタイムアウト処理が実行される）
> English:  We contacted Microsoft, and as you may have guessed, they have implemented a timeout process. (A 20 second timeout process is executed when a worker thread exits before SmartCardReader)

Refer: [Windows.Devices.SmartCards.SmartCardReader.FromIdAsync(string deviceId)メソッドを呼ぶとExeプロセスの終了が遅延する](https://learn.microsoft.com/ja-jp/answers/questions/1634385/windows-devices-smartcards-smartcardreader-fromida)

## How to build?

#### Verify Succeeded

````bat
$ dotnet run --project .\sources\Demo -c Release
2024-04-07 16:04:57.2783 [INFO ] SmartCardReader: SONY FeliCa Port/PaSoRi 4.0 0 
2024-04-07 16:04:57.3635 [INFO ] Press any key When put card on device 
2024-04-07 16:04:59.1956 [INFO ]        SELECT FILE 券面入力補助AP: True [90-00] 
2024-04-07 16:04:59.2255 [INFO ]        SELECT FILE 券面入力補助用PIN: True [90-00] 
2024-04-07 16:04:59.2255 [INFO ] Press enter 認証用PIN (4 digit) 
****
2024-04-07 16:05:00.4125 [INFO ]        VERIFY: [90-00] 
2024-04-07 16:05:00.4125 [INFO ]        OK 
2024-04-07 16:05:00.4476 [INFO ]        SELECT FILE マイナンバー: True [90-00] 
2024-04-07 16:05:00.4844 [INFO ]        READ BINARY マイナンバー: True [FF-10-0C-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-FF-FF-90-00] 
2024-04-07 16:05:00.4844 [INFO ]                マイナンバー: 123456789012
2024-04-07 16:05:00.5204 [INFO ]        SELECT FILE 基本4情報: True [90-00] 
2024-04-07 16:05:00.5616 [INFO ]        READ BINARY 基本4情報 (長さ): True [82-90-00] 
2024-04-07 16:05:00.5967 [INFO ]        READ BINARY 基本4情報 ヘッダー: True [FF-20-82-00-89-90-00] 
2024-04-07 16:05:00.6492 [INFO ]        READ BINARY 基本4情報: True [FF-20-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX-XX]
2024-04-07 03:54:01.7958 [INFO ]                名前: 山田　太郎
2024-04-07 03:54:01.7958 [INFO ]                住所: 東京都港区芝公園４丁目２−８
2024-04-07 03:54:01.7958 [INFO ]                生年月日: 19581223
2024-04-07 03:54:01.7958 [INFO ]                性別: 1
2024-04-07 03:54:01.7958 [INFO ] Press any key to exit.
````