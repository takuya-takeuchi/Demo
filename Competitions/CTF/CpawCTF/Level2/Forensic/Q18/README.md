# Q18.[Forensic]leaf in forest

## How to resolve?

At first, you must know what [misc100](./misc100]) is.
But you can not get any information by `file` command.

````bash
$ file misc100 
misc100: pcap capture file, microsecond ts (little-endian) - version 0.0 (linktype#1768711542, capture length 1869357413)
$ tsark -r misc100
tshark: The file "misc100" contains record data that TShark doesn't support.
(pcap: major version 0 unsupported)
````

Than you should use binary editor and find this file has strange content.
Repeat removing strange word and you will find `CCC`, `PPP`, `AAA` and `WWW`.

In other words?