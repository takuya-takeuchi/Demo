#include <charconv>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <windows.h>
#include <wincrypt.h>

#define MODE_ENCRYPT 0
#define MODE_DECRYPT 1

bool ProtectData(const std::vector<uint8_t>& data, std::vector<BYTE>& encryptedData)
{
    DATA_BLOB inputData;
    inputData.pbData = (BYTE*)data.data();
    inputData.cbData = (DWORD)data.size();

    DATA_BLOB outputData;
    if (!CryptProtectData(&inputData,
                          nullptr,
                          nullptr,
                          nullptr,
                          nullptr,
                          0,
                          &outputData))
    {
        return false;
    }

    encryptedData.clear();
    encryptedData.resize(outputData.cbData);
    std::memcpy(encryptedData.data(), outputData.pbData, outputData.cbData);

    LocalFree(outputData.pbData);

    return true;
}

bool UnprotectData(const std::vector<uint8_t>& encryptedData, std::vector<BYTE>& decryptedData)
{
    DATA_BLOB inputData;
    inputData.pbData = (BYTE*)encryptedData.data();
    inputData.cbData = (DWORD)encryptedData.size();

    DATA_BLOB outputData;
    if (!CryptUnprotectData(&inputData,
                            nullptr,
                            nullptr,
                            nullptr,
                            nullptr,
                            0,
                            &outputData))
    {
        return false;
    }

    decryptedData.clear();
    decryptedData.resize(outputData.cbData);
    std::memcpy(decryptedData.data(), outputData.pbData, outputData.cbData);

    LocalFree(outputData.pbData);

    return true;
}

bool ReadFile(const std::filesystem::path& path, std::vector<uint8_t>& data)
{
    std::ifstream file(path.wstring(), std::ios::binary);
    if (!file.is_open())
        return false;

    file.unsetf(std::ios::skipws);

    std::streampos fileSize;
    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    data.reserve(fileSize);
    data.insert(data.begin(),
                std::istream_iterator<uint8_t>(file),
                std::istream_iterator<uint8_t>());

    return true;
}

void WriteFile(const std::filesystem::path& path, std::vector<uint8_t>& data)
{
    std::ofstream file(path.wstring(), std::ios::out | std::ios::binary);
    file.write((char*)&data[0], data.size() * sizeof(data[0]));
    file.close();
}

std::string OutputBinaryAsHex(const std::vector<uint8_t>& data)
{
    std::ostringstream oss;
    for (const BYTE& byte : data)
        oss << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(byte) << " ";    
    return oss.str();
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 4)
    {
        std::cerr << "[Error] Demo <mode: 0 is encrypt, 1 is decrypt> </path/to/input> </path/to/output>" << std::endl;
        return -1;
    }

    int mode;
    std::string str = argv[1];
    if (std::from_chars(str.data(), str.data() + str.size(), mode).ec != std::errc() || (mode != MODE_ENCRYPT && mode != MODE_DECRYPT))
    {
        std::cerr << "[Error] " << argv[1] << " is not integer, 0 or 1" << std::endl;
        return -1;
    }

    const std::filesystem::path input = std::filesystem::path(argv[2]);
    if (!std::filesystem::exists(input) || !std::filesystem::is_regular_file(input))
    {
        std::cerr << "[Error] " << argv[2] << " does not exist or not file" << std::endl;
        return -1;
    }

    try
    {
        std::vector<uint8_t> data;
        if (!ReadFile(input, data))
        {
            std::cerr << "[Error] Failed to open " << argv[2] << std::endl;
            return -1;
        }

        std::vector<uint8_t> outputData;
        if (mode == MODE_ENCRYPT)
        {
            if (!ProtectData(data, outputData))
            {
                std::cerr << "[Error] Failed to encrypt" << std::endl;
                return -1;
            }

            std::cout << "[Info]   Plained Text: " << OutputBinaryAsHex(data) << std::endl;
            std::cout << "[Info] Encrypted Text: " << OutputBinaryAsHex(outputData) << std::endl;
        }
        else
        {
            if (!UnprotectData(data, outputData))
            {
                std::cerr << "[Error] Failed to decrypt" << std::endl;
                return -1;
            }

            std::cout << "[Info] Encrypted Text: " << OutputBinaryAsHex(data) << std::endl;
            std::cout << "[Info]   Plained Text: " << OutputBinaryAsHex(outputData) << std::endl;
        }

        const std::filesystem::path output = std::filesystem::path(argv[3]);
        WriteFile(output, outputData);

        std::cout << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Error] " << e.what() << std::endl;
    }

    return 0;
}