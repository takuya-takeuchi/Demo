# Get Authentication PIN Retry Count

## Abstracts

* Get Authentication PIN Retry Count by using `Windows API`

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

#### Devices found

````bat
$ dotnet run --project .\sources\Demo -c Release
2024-04-06 21:53:02.6519 [INFO ] SmartCardReader: SONY FeliCa Port/PaSoRi 4.0 0 
2024-04-06 21:53:02.7341 [INFO ] Press any key When put card on device 
2024-04-06 21:53:04.0925 [INFO ]        SELECT FILE JPKI: True [90-00] 
2024-04-06 21:53:04.1250 [INFO ]        SELECT FILE PIN: True [90-00] 
2024-04-06 21:53:04.1524 [INFO ]        RETRY COUNT: 3 
2024-04-06 21:53:04.1608 [INFO ] Press any key to exit. 
````