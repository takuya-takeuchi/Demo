#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <windows.h>
#include <wincrypt.h>

#define AES_BLOCK_SIZE 16

struct CryptContextDeleter {
    void operator()(HCRYPTPROV* phProv) const {
        if (phProv && *phProv) { CryptReleaseContext(*phProv, 0); delete phProv; }
    }
};

struct CryptHashDeleter {
    void operator()(HCRYPTHASH* phHash) const {
        if (phHash && *phHash) { CryptDestroyHash(*phHash); delete phHash; }
    }
};

struct CryptKeyDeleter {
    void operator()(HCRYPTKEY* phKey) const {
        if (phKey && *phKey) { CryptDestroyKey(*phKey); delete phKey; }
    }
};

bool EncryptData(ALG_ID algId, const std::string& plainText, std::vector<BYTE>& cipherText, const std::string& password, const BYTE* iv)
{
    std::unique_ptr<HCRYPTPROV, CryptContextDeleter> hProv(new HCRYPTPROV(0));
    std::unique_ptr<HCRYPTHASH, CryptHashDeleter> hHash(new HCRYPTHASH(0));
    std::unique_ptr<HCRYPTKEY, CryptKeyDeleter> hKey(new HCRYPTKEY(0));

    if (!CryptAcquireContext(hProv.get(), nullptr, nullptr, PROV_RSA_AES, CRYPT_VERIFYCONTEXT))
        return false;

    if (!CryptCreateHash(*hProv.get(), CALG_SHA_256, 0, 0, hHash.get()))
        return false;

    // Compute hash from password string
    if (!CryptHashData(*hHash.get(), (BYTE*)password.data(), (DWORD)password.size(), 0))
        return false;

    // Create hash as key
    if (!CryptDeriveKey(*hProv.get(), algId, *hHash.get(), 0, hKey.get()))
        return false;

    // if is nullptr, IV is 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    if (iv != nullptr && !CryptSetKeyParam(*hKey.get(), KP_IV, iv, 0))
        return false;

    DWORD dataLen = (DWORD)plainText.size();
    cipherText.assign(plainText.begin(), plainText.end());
    cipherText.resize(dataLen + AES_BLOCK_SIZE);

    if (!CryptEncrypt(*hKey.get(), 0, TRUE, 0, cipherText.data(), &dataLen, (DWORD)cipherText.size()))
        return false;

    cipherText.resize(dataLen);

    return true;
}

bool DecryptData(ALG_ID algId, const std::vector<BYTE>& cipherText, std::string& plainText, const std::string& password, const BYTE* iv)
{
    std::unique_ptr<HCRYPTPROV, CryptContextDeleter> hProv(new HCRYPTPROV(0));
    std::unique_ptr<HCRYPTHASH, CryptHashDeleter> hHash(new HCRYPTHASH(0));
    std::unique_ptr<HCRYPTKEY, CryptKeyDeleter> hKey(new HCRYPTKEY(0));
    std::vector<BYTE> buffer = cipherText;

    if (!CryptAcquireContext(hProv.get(), nullptr, nullptr, PROV_RSA_AES, CRYPT_VERIFYCONTEXT))
        return false;

    if (!CryptCreateHash(*hProv.get(), CALG_SHA_256, 0, 0, hHash.get()))
        return false;

    // Compute hash from password string
    if (!CryptHashData(*hHash.get(), (BYTE*)password.data(), (DWORD)password.size(), 0))
        return false;

    // Create hash as key
    if (!CryptDeriveKey(*hProv.get(), algId, *hHash.get(), 0, hKey.get()))
        return false;

    // if is nullptr, IV is 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    if (iv != nullptr && !CryptSetKeyParam(*hKey.get(), KP_IV, iv, 0))
        return false;

    DWORD dataLen = (DWORD)buffer.size();
    if (!CryptDecrypt(*hKey.get(), 0, TRUE, 0, buffer.data(), &dataLen))
        return false;

    plainText.assign(buffer.begin(), buffer.begin() + dataLen);

    return true;
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 4)
    {
        std::cerr << "Demo <plainText> <password> <use IV: 0 or 1>" << std::endl;
        return -1;
    }

    const std::string plainText = argv[1];
    const std::string password = argv[2];
    const bool useIV = std::stoi(argv[3]) != 0;

    std::vector<BYTE> cipherText;
    std::string decryptedText;
    ALG_ID algId = CALG_AES_256;
    BYTE iv[AES_BLOCK_SIZE] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
    
    std::cout << "    Plain Text: " << plainText << std::endl;
    std::cout << "      Password: " << password << std::endl;
    std::cout << "            IV: ";

    if (useIV)
        for (const BYTE& byte : iv)
            std::cout << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(byte) << " ";
    else
        std::cout << "N/A";
    std::cout << std::dec << std::endl;

    if (!EncryptData(CALG_AES_256, plainText, cipherText, password, useIV ? iv : nullptr))
    {
        std::cerr << "Encryption failed." << std::endl;
        return -1;
    }

    if (!DecryptData(CALG_AES_256, cipherText, decryptedText, password, useIV ? iv : nullptr))
    {
        std::cerr << "Decryption failed." << std::endl;
        return -1;
    }

    std::cout << "Encrypted Text: ";
    for (const BYTE& byte : cipherText)
        std::cout << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(byte) << " ";
    std::cout << std::dec << std::endl;
    std::cout << "Decrypted Text: " << decryptedText << std::endl;

    return 0;
}