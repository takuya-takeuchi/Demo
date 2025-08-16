# Generate rotated imgaes by Exif Orientation tag

## Abstracts

* How to generate rotated images by Exif Orientation tag

## Requirements

### Windows

* Visual Studio
* .NET Framework 4.8

## Dependencies

* [ExifLibrary](https://github.com/devedse/exiflibrary)
  * 1.0.13
  * IT license
* [NLog](https://github.com/NLog/NLog)
  * 6.0.3
  * BSD-3-Clause License

## Test Data

* [4.2.07.jpg](./sources/Demo/4.2.07.jpg)
  * from https://sipi.usc.edu/database/database.php?volume=misc&image=13#top

## How to run?

Build sources by Visual Studio.
Next kick built program.

````bat
$ Demo.exe 4.2.07.jpg
2025-08-15 05:55:00.9673 [INFO ] Saved: output_no_exif.jpg
2025-08-15 05:55:01.0561 [INFO ] Saved: output_orientation_1_RotateNone.jpg
2025-08-15 05:55:01.0781 [INFO ] Saved: output_orientation_2_RotateNoneFlipX.jpg
2025-08-15 05:55:01.0991 [INFO ] Saved: output_orientation_3_Rotate180FlipNone.jpg
2025-08-15 05:55:01.1201 [INFO ] Saved: output_orientation_4_RotateNoneFlipY.jpg
2025-08-15 05:55:01.1411 [INFO ] Saved: output_orientation_5_Rotate90FlipX.jpg
2025-08-15 05:55:01.1620 [INFO ] Saved: output_orientation_6_Rotate270FlipNone.jpg
2025-08-15 05:55:01.1829 [INFO ] Saved: output_orientation_7_Rotate270FlipX.jpg
2025-08-15 05:55:01.2029 [INFO ] Saved: output_orientation_8_Rotate90FlipNone.jpg
````

These images have been applied with inverse affine transformation so that they appear upright based on the rotation value in Exif.

| Value | Image| Meaning      | Rotation              | Mirror (Flip)     | Description                                  |
| ----- | -----|------------- | --------------------- | ----------------- | -------------------------------------------- |
| 1     |<img src="./images/output_orientation_1_RotateNone.jpg" width="256" />| Top-left     | None                  | None              | Default orientation (no rotation or flip)    |
| 2     |<img src="./images/output_orientation_2_RotateNoneFlipX.jpg" width="256"/>| Top-right    | None                  | Horizontal mirror | Mirrored horizontally (like a reflection)    |
| 3     |<img src="./images/output_orientation_3_Rotate180FlipNone.jpg" width="256" />| Bottom-right | 180° rotation         | None              | Upside down                                  |
| 4     |<img src="./images/output_orientation_4_RotateNoneFlipY.jpg" width="256" />| Bottom-left  | 180° rotation         | Horizontal mirror | Upside down and mirrored                     |
| 5     |<img src="./images/output_orientation_5_Rotate90FlipX.jpg" width="256" />| Left-top     | 90° clockwise         | Horizontal mirror | Rotated right and mirrored                   |
| 6     |<img src="./images/output_orientation_6_Rotate270FlipNone.jpg" width="256" />| Right-top    | 90° clockwise         | None              | Rotated 90° to the right (portrait)          |
| 7     |<img src="./images/output_orientation_7_Rotate270FlipX.jpg" width="256" />| Right-bottom | 90° counter-clockwise | Horizontal mirror | Rotated left and mirrored                    |
| 8     |<img src="./images/output_orientation_8_Rotate90FlipNone.jpg" width="256" />| Left-bottom  | 90° counter-clockwise | None              | Rotated 90° to the left (portrait, reversed) |