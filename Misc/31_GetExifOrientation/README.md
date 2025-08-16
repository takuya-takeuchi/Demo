# Get Exif Orientation

## Abstracts

* How to get Exif Orientation tag by System.Drawing.Image and ExifLib

## Requirements

### Windows

* Visual Studio
* .NET Framework 4.8
* Windows 23H2 or prior
  * If use Windows 24hH2, Build 26000.2454 or later
    * Refer to [EXIF data misreading in WinForms VB application on Windows 11 24H2 #12338](https://github.com/dotnet/winforms/issues/12338)

## Dependencies

* [ExifLib](https://www.nuget.org/packages/ExifLib)
  * 1.7.0
  * Code Project Open License
* [NLog](https://github.com/NLog/NLog)
  * 6.0.3
  * BSD-3-Clause License

## Test Data

* [output_orientation_4_RotateNoneFlipY.jpg](./sources/Demo/output_orientation_4_RotateNoneFlipY.jpg)
  * from https://sipi.usc.edu/database/database.php?volume=misc&image=13#top
  * Added Exif value by [30_GenerateExifRotatedImage](../30_GenerateExifRotatedImage/)

## How to run?

Build sources by Visual Studio.
Next kick built program.

````bat
$ Demo.exe output_orientation_4_RotateNoneFlipY.jpg
2025-08-16 11:29:55.4697 [INFO ] ExifLib found Orientation Tag: 4 
2025-08-16 11:29:55.4997 [INFO ] System.Drawing found Orientation Tag: 4
````