# Upper-Lower Case Sensitive of XmlSerializer for C# and VB.NET

## Abstacts

* Demonstration of Upper-Lower Case Sensitive for XmlSerializer

## Dependencies

* [NLog](https://github.com/NLog/NLog)
  * MIT license

## How to use?

````cmd
$ pwsh Run.ps1
Run DemoCSharp
2022-07-05 21:50:25.7322 [ERROR] Failed to deserialize DemoCSharp.Test from TestUpper.xml
2022-07-05 21:50:25.7703 [INFO ] Succeed to deserialize DemoCSharp.Test from TestLower.xml
2022-07-05 21:50:25.7742 [INFO ] Succeed to deserialize DemoCSharp.TEST from TestUpper.xml
2022-07-05 21:50:25.7742 [ERROR] Failed to deserialize DemoCSharp.TEST from TestLower.xml
Run DemoVisualBasic
2022-07-05 21:50:27.5853 [ERROR] Failed to deserialize DemoVisualBasic.Test from TestUpper.xml
2022-07-05 21:50:27.6247 [INFO ] Succeed to deserialize DemoVisualBasic.Test from TestLower.xml
````