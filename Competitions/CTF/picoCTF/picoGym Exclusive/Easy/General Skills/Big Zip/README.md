# Big Zip

## How to resolve?

Big zip means that zip file may have extream large file.
It occurs a lack of disk space if expands it.
So you should check size of contents without expanding zip.

````bash
$ zipinfo -t big-zip-files.zip
9090 files, 740217 bytes uncompressed, 622028 bytes compressed:  16.0%
````

Ok. There is no risk of zip bomb attack.

````bash
$ find big-zip-files -type f  | xargs grep -i picoctf
````

In other words?