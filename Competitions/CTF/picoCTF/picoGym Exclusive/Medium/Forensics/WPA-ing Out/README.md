# WPA-ing Out

## How to resolve?

Text of an question says `but it turns out that passwords passed over the AIR can be CRACKED` and `rockyou.txt`.
`rockyou.txt` is based on approximately 32 million passwords that were leaked when the social networking application “RockYou” was hacked in December 2009.
So you may be able to do brute force attack by using `rockyou.txt`.
In addition, you have already got [wpa-ing_out.pcap](./wpa-ing_out.pcap).
So you will get secret if you can do brute force against this packet capture file. 

You can download `rockyou.txt` from [rockyou.txt](https://github.com/brannondorsey/naive-hashcat/releases/download/data/rockyou.txt).
Or you cau use preinstalled file if you use Kali Linux.

Fortunately, it is easy to do brute force attack for packet capture file by using `aircrack-ng`. 
Like this,

````bash
$ aircrack-ng wpa-ing_out.pcap -w rockyou.txt
````

In other words?