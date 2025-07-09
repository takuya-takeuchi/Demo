# Get started

## Abstracts

* Enumerate device name

## Requirements

### Common

* Powershell 7 or later
* CMake 3.12 or later
* C++ Compiler

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [Npcap](https://npcap.com/#download)
  * 1.15
  * [License](https://npcap.com/oem/)
* [PcapPlusPlus](https://github.com/seladb/PcapPlusPlus)
  * v25.05
  * Unlicense License

## How to build?

### PcapPlusPlus

Go to [PcapPlusPlus++](..).
Once time you built `PcapPlusPlus`, you need not to do again.

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ install\win\static\bin\Demo.exe
\Device\NPF_{9028DAC1-72EF-4C5D-AC22-2D533071F3C8} : WAN Miniport (Network Monitor)
\Device\NPF_{83CAED62-B3C4-434D-AD0D-24BCE3E44211} : WAN Miniport (IPv6)
\Device\NPF_{6909B09A-1DDC-4EB8-AEB5-BA445475CEDC} : WAN Miniport (IP)
\Device\NPF_{D75176DE-904D-424E-B952-6284DF3D2106} : Killer E2500 Gigabit Ethernet Controller
\Device\NPF_{486B3DBF-910E-4B2A-A280-27D8956B3DC6} : Hyper-V Virtual Ethernet Adapter #16
\Device\NPF_{280E52A1-AA25-4D8C-A0D7-34A0696DE7C3} : Hyper-V Virtual Ethernet Adapter #15
\Device\NPF_{505A476C-6703-4FAC-B7A1-FAA57A43986E} : Hyper-V Virtual Ethernet Adapter #14
\Device\NPF_{505436A6-92E9-41A8-ABF7-1464D2018400} : Hyper-V Virtual Ethernet Adapter #13
\Device\NPF_{F636CF78-7421-4F59-A143-D16D7D09EA82} : Hyper-V Virtual Ethernet Adapter #12
\Device\NPF_{22AB4492-3DF6-41DD-B9D5-C4BD5ED1EAA4} : Hyper-V Virtual Ethernet Adapter #11
\Device\NPF_{B0EF6A4C-BAAE-410D-93C5-B40F5FD6BE0B} : Hyper-V Virtual Ethernet Adapter #10
\Device\NPF_{265C8517-65BD-4C6E-9763-CE6366293F47} : Hyper-V Virtual Ethernet Adapter #9
\Device\NPF_{F214E570-4DC7-40D2-904C-8D10A6DA4971} : Hyper-V Virtual Ethernet Adapter #7
\Device\NPF_{097FBBF9-28DF-4DFA-8A20-7A6DAD23C0BF} : Hyper-V Virtual Ethernet Adapter #2
\Device\NPF_{C33FBF46-6A0A-419B-BAFC-5F5702A4F62B} : Hyper-V Virtual Ethernet Adapter
\Device\NPF_{DB27D49D-AE9D-4A2E-BFF8-770681B80467} : Bluetooth Device (Personal Area Network) #4
\Device\NPF_{BAA6EE77-1895-4CB0-8E16-89F91C4B3623} : VMware Virtual Ethernet Adapter for VMnet8
\Device\NPF_{E1C2FE3E-DF84-417A-BE0C-28DB2C9AAF34} : VMware Virtual Ethernet Adapter for VMnet1
\Device\NPF_{C02BDA0E-300C-49AE-B281-C37631262622} : Microsoft Wi-Fi Direct Virtual Adapter #4
\Device\NPF_{B9CF988B-3EA5-4E01-B500-D4ADA765380A} : Microsoft Wi-Fi Direct Virtual Adapter #3
\Device\NPF_{3237D77E-9542-4AF2-8F9A-E75619B3A4C7} : Killer Wireless-n/a/ac 1535 Wireless Network Adapter
\Device\NPF_{EAA77FCE-2A2B-4BDA-9651-ADB28DEF2B05} : Hyper-V Virtual Ethernet Adapter #8
\Device\NPF_{E7616615-1787-4E93-B82C-18AA4A41E964} : VirtualBox Host-Only Ethernet Adapter
\Device\NPF_{E71B69DB-824E-4CC2-93B9-99669BD2A833} : Hyper-V Virtual Ethernet Adapter #6
\Device\NPF_{2EAAC34E-04F2-44D0-9C39-E1B38084D436} : Hyper-V Virtual Ethernet Adapter #5
\Device\NPF_{85B623E8-03E7-45E4-A76F-D911B2B4C141} : Hyper-V Virtual Ethernet Adapter #4
\Device\NPF_{4175C5AB-487A-47A8-A273-38768D8656CE} : Hyper-V Virtual Ethernet Adapter #3
\Device\NPF_Loopback : Adapter for loopback traffic capture
````