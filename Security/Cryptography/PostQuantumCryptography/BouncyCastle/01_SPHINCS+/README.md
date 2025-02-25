# SPHINCS+

## Abstract

* SPHINCS+ is a stateless hash-based signature scheme, which was submitted to the NIST post-quantum crypto project.
* Demonstrate the following SPHINCS+'s methods
  * SPHINCS+-SHAKE256
  * SPHINCS+-SHA-256
  * SPHINCS+-Haraka

## Requirements

### Common

* .NET 8.0 SDK
* PowerShell 7 or later

## Dependencies

* [BouncyCastle.Cryptography](https://github.com/bcgit/bc-csharp)
  * 2.5.1
  * MIT License
* [NLog](https://github.com/NLog/NLog)
  * 5.0.0
  * MIT License

## How to usage?

````shell
$ dotnet run hello shake_128s --project sources\Demo\Demo.csproj
2025-02-26 01:07:20.4461 [INFO ]                    Message: hello 
2025-02-26 01:07:20.4811 [INFO ]                     Method: shake_128s 
2025-02-26 01:07:20.4811 [INFO ]        Public key (length): 32 bytes
2025-02-26 01:07:20.4811 [INFO ]          Alice Public key : FF30B62E6602C71CB54682074592AF21135C35D160550A97E8D78880A68C66F0
2025-02-26 01:07:20.4811 [INFO ]       Private key (length): 64 bytes
2025-02-26 01:07:20.4811 [INFO ]          Alice Private key: EB52984B1CE130FBDD817FAFB271489C676E6B770B2652ACAC939044C82294B1FF30B62E6602C71CB54682074592AF21135C35D160550A97E8D78880A68C66F0
2025-02-26 01:07:20.4811 [INFO ]         Signature (length): 7856 bytes
2025-02-26 01:07:20.4811 [INFO ] Signature (first 50 bytes): C610533F6743EB463196BD8CDB6E7FA945D90CB65E474BBE4C6FAFD3384CC47A03A3A250DB2CF883BFEA423BEC3E02770136
2025-02-26 01:07:20.4890 [INFO ]                   Verified: True
````

````shell
$ dotnet run hello haraka_128s --project sources\Demo\Demo.csproj
2025-02-26 01:08:55.6346 [INFO ]                    Message: hello 
2025-02-26 01:08:55.6981 [INFO ]                     Method: haraka_128s 
2025-02-26 01:08:55.6981 [INFO ]        Public key (length): 32 bytes
2025-02-26 01:08:55.6981 [INFO ]          Alice Public key : 420D5D10E37EC24D271B51A1BF903425FD47CE62888C2673C4BDBC49C1B8A6E8
2025-02-26 01:08:55.6981 [INFO ]       Private key (length): 64 bytes
2025-02-26 01:08:55.6981 [INFO ]          Alice Private key: C6DF31B1CC015148B06F9A546C2D20979EFEE3C3A52680F82E4BF8D794D76C87420D5D10E37EC24D271B51A1BF903425FD47CE62888C2673C4BDBC49C1B8A6E8
2025-02-26 01:08:55.6981 [INFO ]         Signature (length): 7856 bytes
2025-02-26 01:08:55.6981 [INFO ] Signature (first 50 bytes): 7F6F9E155526FB077C344E27F4C0CFAC5B7950D2F6FC2367D5B0610D833257AC8E1A4EC01ECE02D80A072920FF5A394B07B5
2025-02-26 01:08:55.6981 [INFO ]                   Verified: True
````