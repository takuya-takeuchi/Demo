#include <iostream>

#include <dxgi.h>
#include <d3d12.h>
#include <dxgi1_6.h>
#include <wrl.h>

using namespace Microsoft::WRL;

int main()
{
    ComPtr<IDXGIFactory6> factory;
    HRESULT hr = CreateDXGIFactory1(IID_PPV_ARGS(&factory));
    if (FAILED(hr))
    {
        std::cerr << "Failed to create IDXGIFactory6." << std::endl;
        return -1;
    }

    UINT i = 0;
    ComPtr<IDXGIAdapter1> adapter;
    DXGI_ADAPTER_DESC adapterDesc;

    std::cout << "Enumerating GPUs:" << std::endl;
    while (factory->EnumAdapters1(i, &adapter) != DXGI_ERROR_NOT_FOUND)
	{
        adapter->GetDesc(&adapterDesc);

        std::wcout << L"Adapter " << i << L": " << adapterDesc.Description << std::endl;
        std::wcout << L"\tVendorId: " << adapterDesc.VendorId << std::endl;
        std::wcout << L"\tDeviceId: " << adapterDesc.DeviceId << std::endl;
        std::wcout << L"\tSubSysId: " << adapterDesc.SubSysId << std::endl;
        std::wcout << L"\tRevision: " << adapterDesc.Revision << std::endl;
        std::wcout << L"\tAdapterLuid: " << adapterDesc.AdapterLuid.LowPart << "-" << adapterDesc.AdapterLuid.HighPart << std::endl;
        std::wcout << L"\tDedicated Video Memory: " << adapterDesc.DedicatedVideoMemory / (1024 * 1024) << L" MB" << std::endl;
        std::wcout << L"\tDedicated System Memory: " << adapterDesc.DedicatedSystemMemory / (1024 * 1024) << L" MB" << std::endl;
        std::wcout << L"\tShared System Memory: " << adapterDesc.SharedSystemMemory / (1024 * 1024) << L" MB" << std::endl;

        ComPtr<ID3D12Device> device;
        hr = D3D12CreateDevice(adapter.Get(), D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&device));
        if (FAILED(hr))
        {
            std::cerr << "Failed to create D3D12 device." << std::endl;
            return -1;
        }

        // D3D12のオプション機能を確認
        D3D12_FEATURE_DATA_D3D12_OPTIONS options = {};
        hr = device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS, &options, sizeof(options));
        if (FAILED(hr))
        {
            std::cerr << "Failed to query feature support." << std::endl;
            return -1;
        }

        std::wcout << L"\tFP16 (half precision) operations are supported: " << ((options.MinPrecisionSupport & D3D12_SHADER_MIN_PRECISION_SUPPORT_16_BIT) ? "Yes" : "No") << std::endl;

        D3D12_FEATURE_DATA_D3D12_OPTIONS1 options1 = {};
        hr = device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS1, &options1, sizeof(options1));
        if (FAILED(hr))
        {
            std::cerr << "Failed to query feature support for D3D12_OPTIONS1." << std::endl;
            return -1;
        }

        std::wcout << L"\tINT8 operations (wave operations) operations are supported: " << (options1.WaveOps ? "Yes" : "No") << std::endl;

        i++;
    }

    return 0;
}