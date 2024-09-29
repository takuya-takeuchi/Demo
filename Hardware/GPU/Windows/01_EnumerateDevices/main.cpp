#include <dxgi.h>
#include <iostream>
#include <wrl.h>

using namespace Microsoft::WRL;

int main()
{
    ComPtr<IDXGIFactory> pFactory;
    HRESULT hr = CreateDXGIFactory(__uuidof(IDXGIFactory), reinterpret_cast<void**>(pFactory.GetAddressOf()));
    if (FAILED(hr))
	{
        std::cerr << "Failed to create DXGIFactory." << std::endl;
        return -1;
    }

    UINT i = 0;
    ComPtr<IDXGIAdapter> pAdapter;
    DXGI_ADAPTER_DESC adapterDesc;

    std::cout << "Enumerating GPUs:" << std::endl;
    while (pFactory->EnumAdapters(i, &pAdapter) != DXGI_ERROR_NOT_FOUND)
	{
        pAdapter->GetDesc(&adapterDesc);

        std::wcout << L"Adapter " << i << L": " << adapterDesc.Description << std::endl;
        std::wcout << L"\tVendorId: " << adapterDesc.VendorId << std::endl;
        std::wcout << L"\tDeviceId: " << adapterDesc.DeviceId << std::endl;
        std::wcout << L"\tDedicated Video Memory: " << adapterDesc.DedicatedVideoMemory / (1024 * 1024) << L" MB" << std::endl;
        std::wcout << L"\tDedicated System Memory: " << adapterDesc.DedicatedSystemMemory / (1024 * 1024) << L" MB" << std::endl;
        std::wcout << L"\tShared System Memory: " << adapterDesc.SharedSystemMemory / (1024 * 1024) << L" MB" << std::endl;

        i++;
    }

    return 0;
}