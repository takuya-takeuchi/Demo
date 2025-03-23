# Q28.[Network] Can you login？

## How to resolve?

You can use wireshark or tshark.
You can find that `ftp` is present.
FTP protocol uses plain text as password.
So you should look at `Request` packet.
After retrieve password, you can login ftp server by ftp client which support PASV mode.
And you should make enable to show hidden files.

In other words?