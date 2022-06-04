# OCR on Windows 10 (for .NET Framework)

## Abstacts

* How to use OCR engine embeded on Windows 10
* Check how much influence is accuracy with or without Language Pack

## Requirements

* Visual Studio 2022
* .NET Framework 4.8
* Windows 10
  * 19041 or later
* Language Pack
  * e.g. Install english language if you want to recognize english text

## How to usage?

You can build demo program by Visual Studio.
After build it, you can run from command prompt.

````cmd
$ sources\Demo\bin\Release\Demo.exe ja testdata\ja.png
2022-06-04 22:51:54.4947 [INFO ] TrySetupOcrEngine: 2 ms
2022-06-04 22:51:54.5227 [INFO ] GetImageBinary: 0 ms
2022-06-04 22:51:54.5584 [INFO ] ConvertToSoftwareBitmap: 28 ms
2022-06-04 22:51:54.7014 [INFO ] Result: 昭 和 ニ 十 - 年 憲 法 日 本 国 憲 法 日 本 国 民 は 、 正 当 に 選 挙 さ れ た 国 会 に お け る 代 表 者 を  通 し て 行 動 し 、 わ れ ら と わ れ ら の 子 孫 の た め に 、 諸 国 民 と の 協 和 に よ る 成 果 と 、 わ が 国 全 土 に わ た っ て 自 由 の も た ら す 恵 沢 を 確 保 し 、 政 府 の 行 為 に よ っ て 再 び 戦 争 の 惨 禍 が 起 る こ と の な い や う に す る こ と を 決 意 し 、 こ こ に 主 権  が 国 民 に 存 す る こ と を 目 言 し 、 こ の 法 を 確 定 す る 。 そ も そ も 国 政 は 、 国 民 の 廠 粛 な 信 託 に よ る も の で あ っ て 、 そ の 権 威 は 国 民 に 由 来 し 、 そ の 権 力 は 国 民 の 代 表 者 が こ れ を 行 使 し 、 そ の 福 利 は 国 民 が こ れ を 享 受 す る 。 こ れ は 人 類  普 遍 の 原 理 で あ り 、 こ の 法 は 、 か か る 原 理 に 基 く も の で あ る 。 わ れ ら は 、 こ れ に 反 す る - 切 の 法 、 法 令 及 び 詔 勅 を 排 除 す る 。 日 本 国 民 は 、 恒 久 の 平 和 を 念 願 し 、 人 間 相 互 の 閂 係 を 支 配 す る 崇 高 な 理 想 を 深 く 自 覚 す る の で あ っ て 、 平 和 を 愛 す る 諸 国 民 の 公 正 と 信 義 に 信 頼 し て 、 わ れ ら の 安 全 と 生 存 を 保 持 し よ う と 決 意 し た 。 わ れ ら は 、 平 和 を  維 持 し 、 専 制 と 隷 従 、 圧 迫 と 偏 狭 を 地 上 か ら 永 遠 に 除 去 し よ う と 努 め て ゐ る 国 社 会 に お い て 、 名 誉 あ る 地 位 を 占 め た い と 思 ふ 。 わ れ ら は 、 全 世 界 の 国 民 が 、 ひ と し く 恐 怖 と 欠 乏 か ら 免 か れ 、 平 和 の う ち に 生 存 す る 権 利 を 有 す る  こ と を 確 認 す る 。 わ れ ら は 、 い づ れ の 国 家 も 、 自 国 の こ と の み に 等 念 し て 他 国 を 無 視 し て は な ら な い の で あ っ て 、 政 治 道 徳 の 法 則 は 、 普 遍 的 な も の で あ り 、 こ の 法 則 に 従 ふ こ と は 、 自 国 の 主 権 を 維 持 し 、 他 国 と 対 等 閂 係 に 立 た  う と す る 各 国 の き 務 で あ る と 信 す る 。 日 本 国 民 は 、 国 家 の 名 誉 に か け 、 全 力 を あ け て こ の 崇 高 な 理 想 と 目 的 を 達 成 す る こ と を ふ 。
2022-06-04 22:51:54.7014 [INFO ] RunOcr: 141 ms
````

The 1st argument is laguage tag being used for text recognition. For example, `ja` or `en`.
The 2nd argument is file path of image file.

## How much accuracy with or without Language Pack?

Windows 10 OCR engine depends on Language Pack.
`OcrEngine.TryCreateFromLanguage` may return null.
Please refer [OcrEngine.TryCreateFromLanguage(Language) Method](https://docs.microsoft.com/ja-jp/uwp/api/windows.media.ocr.ocrengine.trycreatefromlanguage?view=winrt-18362) and [Language matching](https://docs.microsoft.com/en-us/previous-versions/windows/apps/jj673578(v=win.10))

You can find that OCR engine generates abosolutely different result with or without Language Pack.
Author uses Windows 10 Japanese and English Language Pack is not installed default.

### Before install English Language Pack

You can see program fails to generate OCR engine if language pack is not installed.

````cmd
$ sources\Demo\bin\Release\Demo.exe en testdata\en.png
2022-06-04 23:01:11.8062 [ERROR] Failed to create ocr engine because it could be lack of language pack.
2022-06-04 23:01:11.8357 [INFO ] TrySetupOcrEngine: 31 ms
````

### Use Japanese OCR engine for English text

You can use OCR engine for other Language.
But it should generate bad result like this.

````cmd
$ sources\Demo\bin\Release\Demo.exe ja testdata\en.png
2022-06-04 23:07:14.7779 [INFO ] TrySetupOcrEngine: 2 ms
2022-06-04 23:07:14.8040 [INFO ] GetImageBinary: 0 ms
2022-06-04 23:07:14.8375 [INFO ] ConvertToSoftwareBitmap: 29 ms
2022-06-04 23:07:14.9762 [INFO ] Result: " The Constitution Of the United states Of America ( 1787 ) (See Note 1 ) Wethe People ofthe united States ′ in 0 「 de 「 t0f0n11 a more perfect Union, establishJustice, insure domesticTranquility, provideforthe common defence, promotethe general Welfare, and secu 「 e the Blessings of Liberty to ou 「 se ⅳ es and ou 「 Posterity, do ordain and establish this Constitution 「 0 「 the United States 0 「 America. Sectlon 1. Alllegislative Powers he 「 ein granted shall be vested in a Congress 0 「 the United States, which shall consist Of a Senate and House Of Representatives.
2022-06-04 23:07:14.9772 [INFO ] RunOcr: 137 ms
````

### After install English Language Pack

````cmd
$ sources\Demo\bin\Release\Demo.exe en testdata\en.png
2022-06-04 23:05:57.2636 [INFO ] TrySetupOcrEngine: 5 ms
2022-06-04 23:05:57.2906 [INFO ] GetImageBinary: 0 ms
2022-06-04 23:05:57.3296 [INFO ] ConvertToSoftwareBitmap: 34 ms
2022-06-04 23:05:57.3826 [INFO ] Result: The Constitution of the United States of America (1787) (See Note 1) We the People of the united States, in Order to form a more perfect Union, establish Justice, insure domestic Tranquility, provide for the common defence, promote the general Welfare, and secure the Blessings of Liberty to ourselves and our Posterity, do ordain and establish this Constitution for the United States of America. Section 1. All legislative Powers herein granted shall be vested in a Congress of the United States, which shall consist of a Senate and House of Representatives.
2022-06-04 23:05:57.3826 [INFO ] RunOcr: 50 ms
````