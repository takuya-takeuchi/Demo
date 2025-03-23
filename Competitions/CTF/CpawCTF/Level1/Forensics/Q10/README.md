# Q10.[Forensics] River

## How to resolve?

Check EXIF from [river.jpg](./river.jpg).
You can use `exiftool` for Linux or file property for Windows and OSX.
If you ask me, fiver name is not stored in file directly.

````bash
$ exiftool -s river.jpg
````

You will ser `GPSPosition`.
This value is represent by DMS format.