#include <iomanip>
#include <iostream>
#include <string>
#include <sstream>

#include <windows.h>

#pragma pack(push)
#pragma pack(1)
typedef struct _RawSMBIOSData
{
	BYTE	Used20CallingMethod;
	BYTE	SMBIOSMajorVersion;
	BYTE	SMBIOSMinorVersion;
	BYTE	DmiRevision;
	DWORD	Length;
	PBYTE	SMBIOSTableData;
} RawSMBIOSData, *PRawSMBIOSData;

typedef struct _SMBIOSHEADER_
{
	BYTE Type;
	BYTE Length;
	WORD Handle;
} SMBIOSHEADER, *PSMBIOSHEADER;


typedef struct _TYPE_1_ {
	SMBIOSHEADER	Header;
	UCHAR	Manufacturer;
	UCHAR	ProductName;
	UCHAR	Version;
	UCHAR	SN;
	UCHAR	UUID[16];
	UCHAR	WakeUpType;
	UCHAR	SKUNumber;
	UCHAR	Family;
} SystemInfo, *PSystemInfo;

const char* LocateStringA(const char* str, const uint32_t length)
{
	static const char strNull[] = "Null String";

	auto i = length;
	if (0 == i || 0 == *str)
		return strNull;

	while (--i)
		str += strlen((char*)str) + 1;

	return str;
}

template <class Type>
Type FindStructure(LPBYTE p, const DWORD length, const int32_t type)
{
	const LPBYTE lastAddress = p + length;
	for (;;)
	{
		auto pHeader = (PSMBIOSHEADER)p;
		if (pHeader->Type == type)
			return (Type)p;

		if ((pHeader->Type == 127) && (pHeader->Length == 4))
			break;

		LPBYTE nt = p + pHeader->Length;
		while (0 != (*nt | *(nt + 1)))
			nt++;
		nt += 2;

		if (nt >= lastAddress)
			break;

		p = nt;
	}

	return nullptr;
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 1)
    {
        std::cout << "Usage:" << std::endl;
        std::cout << "\tTest" << std::endl;
        return -1;
    }

	// 'ACPI'	The ACPI firmware table provider.
	// 'FIRM'	The raw firmware table provider. Not supported for UEFI systems; use 'RSMB' instead.
	// 'RSMB'	The raw SMBIOS firmware table provider.
	const DWORD Signature = 'RSMB';
	const DWORD needBufferSize = GetSystemFirmwareTable(Signature,
                                                        0,
                                                        NULL,
                                                        0);

	LPBYTE pBuffer = (LPBYTE)malloc(needBufferSize);
    std::unique_ptr<std::remove_pointer<LPBYTE>::type, void(*)(LPBYTE)> buffer
    {
        pBuffer, [](LPBYTE p)
        {
            free(p);
        }
    };

	if (!buffer.get())
	{
        std::cout << "Failed to allocate buffer. size: " << needBufferSize << std::endl;
		return -1;
	}

	GetSystemFirmwareTable(Signature, 0, buffer.get(), needBufferSize);

	const PRawSMBIOSData pDMIData = (PRawSMBIOSData)buffer.get();
	std::cout <<             "SMBIOS version: " << (int32_t)pDMIData->SMBIOSMajorVersion << "." << (int32_t)pDMIData->SMBIOSMinorVersion << std::endl;
	std::cout << std::hex << "  DMI Revision: " << (int32_t)pDMIData->DmiRevision << std::endl;
	std::cout << std::dec << "  Total length: " << pDMIData->Length << std::endl;
	std::cout << std::hex << "DMI at address: " << &pDMIData->SMBIOSTableData << std::endl;
	std::cout << std::endl;

	PSystemInfo pSystemInfo = FindStructure<PSystemInfo>((LPBYTE)(&pDMIData->SMBIOSTableData), pDMIData->Length, 1);
	if (!pSystemInfo)
	{
		std::cout << "Failed to find System Information (Type 1)" << std::endl;
		return -1;
	}

	auto str = (char*)pSystemInfo + ((PSMBIOSHEADER)pSystemInfo)->Length;
	std::cout << "System Information (Type 1)" << std::endl;
	std::cout << "\t Manufacturer: " << LocateStringA(str, pSystemInfo->Manufacturer) << std::endl;
	std::cout << "\t Product Name: " << LocateStringA(str, pSystemInfo->ProductName) << std::endl;
	std::cout << "\t      Version: " << LocateStringA(str, pSystemInfo->Version) << std::endl;
	std::cout << "\tSerial Number: " << LocateStringA(str, pSystemInfo->SN) << std::endl;

	// for v2.1 and later
	if (pSystemInfo->Header.Length > 0x08)
	{
		std::cout << "\t         UUID: ";
		std::cout << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pSystemInfo->UUID[0];
		std::cout << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pSystemInfo->UUID[1];
		std::cout << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pSystemInfo->UUID[2];
		std::cout << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pSystemInfo->UUID[3];
		std::cout << "-";
		std::cout << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pSystemInfo->UUID[4];
		std::cout << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pSystemInfo->UUID[5];
		std::cout << "-";
		std::cout << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pSystemInfo->UUID[6];
		std::cout << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pSystemInfo->UUID[7];
		std::cout << "-";
		std::cout << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pSystemInfo->UUID[8];
		std::cout << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pSystemInfo->UUID[9];
		std::cout << "-";
		std::cout << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pSystemInfo->UUID[10];
		std::cout << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pSystemInfo->UUID[11];
		std::cout << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pSystemInfo->UUID[12];
		std::cout << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pSystemInfo->UUID[13];
		std::cout << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pSystemInfo->UUID[14];
		std::cout << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pSystemInfo->UUID[15];
		std::cout << std::endl;
	}

	// for v2.4 and later
	if (pSystemInfo->Header.Length > 0x19)
	{
		std::cout << "\t   SKU Number: " << LocateStringA(str, pSystemInfo->SKUNumber) << std::endl;
		std::cout << "\t       Family: " << LocateStringA(str, pSystemInfo->Family) << std::endl;
	}

    return 0;
}