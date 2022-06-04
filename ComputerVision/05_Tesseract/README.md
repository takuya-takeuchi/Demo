# Tesseract

## Abstacts

* How to use Tesseract

## Requirements

* .NET 6.0

## Dependencies

* [NLog](https://github.com/NLog/NLog)
  * BSD-3-Clause license
* [TesseractOCR](https://github.com/Sicos1977/TesseractOCR)
  * Apache-2.0 license
* [tessdata](https://github.com/tesseract-ocr/tessdata)
  * Apache-2.0 license
* [tessdata_fast – Fast integer versions of trained models](https://github.com/tesseract-ocr/tessdata_fast)
  * Apache-2.0 license

## How to usage?

At first, you run the following command to init submodule. 
It will take time because [tessdata](https://github.com/tesseract-ocr/tessdata) is huge repository.

````cmd
$ git submodule update --init --recursive .
````

After this, you can get Tesseract model files

* tessdata
* tessdata_fast

Next, you can try OCR by the following command.

````cmd
$ cd sources\Demo
$ dotnet run -c Release -- ..\..\tessdata Japanese TesseractAndLstm ..\..\testdata\ja.png
2022-06-05 02:56:33.6743 [INFO ] GetImageBinary: 0 ms
2022-06-05 02:56:34.4534 [INFO ] CreateEngine: 708 ms
2022-06-05 02:56:34.4625 [INFO ] LoadTesseractImage: 5 ms
2022-06-05 02:56:35.1218 [INFO ] Mean confidence: 0.87
2022-06-05 02:56:35.1549 [INFO ] Text:
昭 和 ニ ナ ー 年 憲 法

日 本 国 憲 法
日 本 国 民 は 、 正 当 に 選 挙 さ れ た 国 会 に お け る 代 表 者 を 通 じ て 行 動 し 、 わ れ ら と わ れ ら の 子 捜 の た め に 、 諾 国 民 と の 協 和 に よ る 成 果 と 、 わ が 国 全 土 に わ た つ て 自 由 の ち
た ら す 恵 沢 を 確 尿 し 、 政 府 の 行 為 に よ つ て 再 ひ 戦 争 の 倉 福 が 記 る こ と の な い や う に す る こ と を 決 意 し 、 こ こ に 主 権 が 国 民 に 存 す る こ と を 宜 言 し 、 こ の 憲 法 を 確 定 す る 。

そ も そ も 国 攻 は 、 国 民 の 厳 目 な 信 託 に よ る ち の で あ つ て 、 そ の 権 或 は 国 民 に 由 来 し 、 そ の 権 力 は 国 民 の 代 表 老 が こ れ を 行 使 し 、 そ の 福 初 は 国 民 が こ れ を 享 受 す る 。 こ れ
は 人 類 智 逢 の 原 理 で あ り 、 こ の 患 法 は 、 か か る 原 理 に 慰 く ち の で あ る 。 わ れ ら は 、 こ れ に 反 す る 一 切 の 惟 法 、 法 令 及 び 詠 動 を 排 除 す る 。

日 本 団 民 は 、 恒 久 の 平 和 を き 顧 し 、 人 間 相 世 の 関 係 を 支 配 す る 岩 高 な 理 想 を 深 く 自 覚 す る の で あ つ て 、 平 和 を 恒 す る 諾 国 民 の 公 正 信 饅 に 信 頻 し て 、 わ れ ら の 妄 全 と
生 存 を 保 持 し よ う と ど 決 情 し た 。 わ れ ら は 、 平 和 を 綱 持 し 、 封 剣 と 隷 径 、 圧 迫 と 側 狩 を 地 上 か ら 永 遠 に 除 な し よ う と 烏 め て ゐ る 国 際 社 会 に お い て 、 名 誉 あ る 地 介 を 占 め た
い と 思 ふ 。 わ れ ら は 、 全 形 界 の 国 民 が 、 ひ と し く 悪 伸 と 欠 之 か ら 免 か れ 、 平 和 の う ち に 生 存 す る 権 利 を 有 す る こ と を 確 訟 す る 。

わ れ ら は 、 い づ れ の 国 容 ち 、 自 国 の こ と の み に 専 念 し て 他 国 を 無 視 し て は な ら な い の で あ つ て 、 政 治 道 待 の 法 則 は 、 普 途 的 な ち の で あ り 、 こ の 法 則 に 炒 ふ こ と は 、 自
国 の 主 権 を 綱 持 し 、 他 国 と 対 電 関 係 に た う と す る 各 国 の 貫 効 で あ る と 信 ず る 。

日 本 団 民 は 、 国 治 の 名 誉 に か け 、 全 力 を あ げ て こ の 崇 高 な 理 想 と 目 的 を 違 成 す る こ と を 誌 。


2022-06-05 02:56:35.1724 [INFO ] RunOcr: 708 ms
````

Or you can specify [tessdata_fast – Fast integer versions of trained models](https://github.com/tesseract-ocr/tessdata_fast). 
In this case, you can specify only **LstmOnly** for 3rd argument.

````cmd
$ cd sources\Demo
$ dotnet run -c Release -- ..\..\tessdata_fast Japanese LstmOnly ..\..\testdata\ja.png
2022-06-05 03:01:08.0326 [INFO ] GetImageBinary: 0 ms
2022-06-05 03:01:08.2293 [INFO ] CreateEngine: 161 ms
2022-06-05 03:01:08.2293 [INFO ] LoadTesseractImage: 4 ms
2022-06-05 03:01:08.9736 [INFO ] Mean confidence: 0.87
2022-06-05 03:01:08.9973 [INFO ] Text:
昭和ニー年吉法
日本国憲法

日本国民は、正当に選挙された|
たらす恵沢を確保し、政府の行為によつて再び戦争の惨褐が起ることのないやうにすることを決意し、こごこごに主権が国民に存すること:
そもそも国政は、国民の上厳整な   によるものであつて、その権威は国民に由来し、その権力は国民の代表者がこれを行使し、その福利
は人類普遍の原理であり、ごの一法は、かかる原理に基くものである。われらは、こごれに反する一切の憲法、法令及び語勅を排除する。

日本国民は、恒久の平和を念願し、人間相互の関係を支配する崇高な理想を深く自覚するのであつて、平和を愛する諸国民の公正と信義に信頼して、われらの安全と
生存を保持しようと決意した。われらは、平和を維持し、専制と隷従、圧迫と偏狭を地上から永遠に除去しようと努めてゐる国際社会において、名准ある地位を占めた
いと思ふ。われらは、全世界の国民が、ひとしく門怖と欠乏から免かれ、平和のうちに生存する権利を有することを確認する。

 われらは、いづれの国家も、   (のことのみに専念して他国を無視してはならないのであつて、政治道徳の法則は、普遍的なものであり、こごの法則に従ふことは、自
国の主権を維持し、他国と対等関係に立たうとする各国の責務であると信ずる。

日本国民は、国家の名注にかけ、全力をあげてこの崇高な理想と目的を達成することを将ふ。

国会における代表者を通じて行動し、われらとわれらの子孫のために、諸国民との協和による成果と、わが国全土にわたつて自由のも
、ごの一法を確定する。
がこれを享受する。ごこれ


2022-06-05 03:01:09.0094 [INFO ] RunOcr: 772 ms
````