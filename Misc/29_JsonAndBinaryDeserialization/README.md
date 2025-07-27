# Json and Binary Deserialization Behaviors

## Abstracts

* Compare deserialize of Newtonsoft.Json, System.Text.Json and System.Runtime.Serialization.Formatters.Binary.BinaryFormatter
  * Default behavior of Newtonsoft.Json's deserialize uses constractors with most parameters. So results of deserialize are different.

## Requirements

### Windows

* Visual Studio
* .NET 6
  * 7 or lator can not run this program

## Dependencies

* [Newtonsoft.Json](https://www.newtonsoft.com/json)
  * 13.0.3
  * MIT License
* [NLog](https://github.com/NLog/NLog)
  * 6.0.2
  * BSD-3-Clause License

## How to run?

Build sources by Visual Studio.
Next kick built program.

````bat
$ Demo.exe
2025-07-27 22:40:46.4800 [INFO ] Start
2025-07-27 22:40:46.8466 [INFO ]                       BinaryFormatter.List.Count: 1
2025-07-27 22:40:46.8466 [INFO ]     NewtonsoftJson.List.Count (Used Constractor): 2
2025-07-27 22:40:46.8466 [INFO ] NewtonsoftJson.List.Count (Not used Constractor): 1
2025-07-27 22:40:46.8466 [INFO ]                            SystemJson.List.Count: 1
````