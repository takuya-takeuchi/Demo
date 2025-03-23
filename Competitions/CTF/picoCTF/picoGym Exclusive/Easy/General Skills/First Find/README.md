# First Find

## How to resolve?

You need to find `uber-secret.txt` from [files.zip](./files.zip).

````bash
$ zipinfo -l files.zip  | grep uber-secret.txt
-rw-rw-r--  3.0 unx       31 tx       31 stor 22-May-14 05:17 files/adequate_books/more_books/.secret/deeper_secrets/deepest_secrets/uber-secret.txt
````

OK. You can get target path.
So you can show content of specified path without expanding zip.

````bash
$ unzip -p files.zip files/adequate_books/more_books/.secret/deeper_secrets/deepest_secrets/uber-secret.txt
````

In other words?