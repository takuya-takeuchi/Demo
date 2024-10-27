#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <windows.h>
#include <ntstatus.h>
#include <Bcrypt.h>

struct BCryptCloseAlgorithmProviderDeleter {
    void operator()(BCRYPT_ALG_HANDLE* phAlg) const {
        if (phAlg && *phAlg) { BCryptCloseAlgorithmProvider(*phAlg, 0); delete phAlg; }
    }
};

struct BCryptDestroyKeyDeleter {
    void operator()(BCRYPT_KEY_HANDLE* phKey) const {
        if (phKey && *phKey) { BCryptDestroyKey(*phKey); delete phKey; }
    }
};

struct BCryptDestroyHashDeleter {
    void operator()(BCRYPT_KEY_HANDLE* phHash) const {
        if (phHash && *phHash) { BCryptDestroyHash(*phHash); delete phHash; }
    }
};

std::string OutputBinaryAsHex(const std::vector<BYTE>& data)
{
    std::ostringstream oss;
    for (const BYTE& byte : data)
        oss << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(byte) << " ";    
    return oss.str();
}

bool ComputeHash(const std::vector<BYTE>& data, std::vector<BYTE>& hash)
{
    std::unique_ptr<BCRYPT_ALG_HANDLE, BCryptCloseAlgorithmProviderDeleter> hAlg(new BCRYPT_ALG_HANDLE(nullptr));
    std::unique_ptr<BCRYPT_HASH_HANDLE, BCryptDestroyHashDeleter> hHash(new BCRYPT_HASH_HANDLE(nullptr));

    if (BCryptOpenAlgorithmProvider(hAlg.get(),
                                    BCRYPT_SHA256_ALGORITHM,
                                    nullptr,
                                    0) != 0)
    {
        std::cerr << "Failed to open algorithm hash provider" << std::endl;
        return false;
    }

    DWORD result = 0;
    DWORD hashObjectSize = 0;
    if (BCryptGetProperty(*hAlg.get(),
                          BCRYPT_OBJECT_LENGTH,
                          (PUCHAR)&hashObjectSize,
                          sizeof(DWORD),
                          &result, 
                          0) != STATUS_SUCCESS)
    {
        std::cerr << "Failed to get key object size" << std::endl;
        return false;
    }

    std::vector<BYTE> hashObject;   
    hashObject.resize(hashObjectSize);

    DWORD hashSize = 0; 
    if (BCryptGetProperty(*hAlg.get(),
                          BCRYPT_HASH_LENGTH,
                          (PUCHAR)&hashSize,
                          sizeof(DWORD),
                          &result, 
                          0) != STATUS_SUCCESS)
    {
        std::cerr << "Failed to get key hash size" << std::endl;
        return false;
    }

    hash.resize(hashSize);

    if (BCryptCreateHash(*hAlg.get(),
                         hHash.get(),
                         (PUCHAR)hashObject.data(),
                         hashObjectSize,
                         nullptr,
                         0,
                         0) != 0)
    {
        std::cerr << "Failed to open hash object" << std::endl;
        return false;
    }

    if (BCryptHashData(*hHash.get(),
                       (PUCHAR)data.data(),
                       (ULONG)data.size(),
                       0) != 0)
    {
        std::cerr << "Failed to hash data" << std::endl;
        return false;
    }

    if (BCryptFinishHash(*hHash.get(),
                         (PUCHAR)hash.data(),
                         hashSize,
                         0) != 0)
    {
        std::cerr << "Failed to finish hash" << std::endl;
        return false;
    }

    return true;
}

bool EncryptData(const std::vector<BYTE>& plainText, std::vector<BYTE>& cipherText, const std::vector<BYTE>& key, const std::vector<BYTE>& iv)
{
    std::unique_ptr<BCRYPT_ALG_HANDLE, BCryptCloseAlgorithmProviderDeleter> hAlg(new BCRYPT_ALG_HANDLE(nullptr));
    std::unique_ptr<BCRYPT_KEY_HANDLE, BCryptDestroyKeyDeleter> hKey(new BCRYPT_KEY_HANDLE(nullptr));

    if (BCryptOpenAlgorithmProvider(hAlg.get(),
                                    BCRYPT_AES_ALGORITHM,
                                    nullptr,
                                    0) != 0)
    {
        std::cerr << "Failed to open algorithm provider for AES" << std::endl;
        return false;
    }

    DWORD keyObjectSize = 0;
    DWORD result = 0;
    if (BCryptGetProperty(*hAlg.get(),
                          BCRYPT_OBJECT_LENGTH,
                          (PUCHAR)&keyObjectSize,
                          sizeof(DWORD),
                          &result, 
                          0) != STATUS_SUCCESS)
    {
        std::cerr << "Failed to get key object size" << std::endl;
        return false;
    }

    if (BCryptSetProperty(*hAlg.get(),
                          BCRYPT_CHAINING_MODE,
                          (PUCHAR)BCRYPT_CHAIN_MODE_CBC,
                          sizeof(BCRYPT_CHAIN_MODE_CBC),
                          0) != STATUS_SUCCESS)
    {
        std::cerr << "Failed to set chaining mode" << std::endl;
        return false;
    }

    std::vector<BYTE> keyObject(keyObjectSize);
    if (BCryptGenerateSymmetricKey(*hAlg.get(),
                                   hKey.get(),
                                   keyObject.data(),
                                   keyObjectSize,
                                   (PUCHAR)key.data(),
                                   (ULONG)key.size(),
                                   0) != STATUS_SUCCESS)
    {
        std::cerr << "Failed to generate AES key" << std::endl;
        return false;
    }

    DWORD cipherTextSize = 0;
    if (BCryptEncrypt(*hKey.get(),
                       (PUCHAR)plainText.data(),
                       (ULONG)plainText.size(),
                       nullptr,
                       (PUCHAR)iv.data(),
                       (ULONG)iv.size(),
                       nullptr,
                       0,
                       &cipherTextSize,
                       BCRYPT_BLOCK_PADDING) != STATUS_SUCCESS)
    {
        std::cerr << "Failed to calculate cipherText size" << std::endl;
        return false;
    }

    cipherText.resize(cipherTextSize);

    DWORD dataSize = 0;
    if (BCryptEncrypt(*hKey.get(),
                      (PUCHAR)plainText.data(),
                      (ULONG)plainText.size(),
                      nullptr,
                      (PUCHAR)iv.data(),
                      (ULONG)iv.size(),
                      cipherText.data(),
                      cipherTextSize,
                      &dataSize,
                      BCRYPT_BLOCK_PADDING) != STATUS_SUCCESS)
    {
        std::cerr << "Encryption failed" << std::endl;
        return false;
    }

    cipherText.resize(dataSize);

    return true;
}

bool DecryptData(const std::vector<BYTE>& cipherText, std::vector<BYTE>& plainText, const std::vector<BYTE>& key, const std::vector<BYTE>& iv)
{
    std::unique_ptr<BCRYPT_ALG_HANDLE, BCryptCloseAlgorithmProviderDeleter> hAlg(new BCRYPT_ALG_HANDLE(nullptr));
    std::unique_ptr<BCRYPT_KEY_HANDLE, BCryptDestroyKeyDeleter> hKey(new BCRYPT_KEY_HANDLE(nullptr));
    
    if (BCryptOpenAlgorithmProvider(hAlg.get(),
                                    BCRYPT_AES_ALGORITHM,
                                    nullptr,
                                    0) != STATUS_SUCCESS)
    {
        std::cerr << "Failed to open algorithm provider for AES" << std::endl;
        return false;
    }

    DWORD keyObjectSize = 0;
    DWORD result = 0;
    if (BCryptGetProperty(*hAlg.get(),
                          BCRYPT_OBJECT_LENGTH,
                          (PUCHAR)&keyObjectSize,
                          sizeof(DWORD),
                          &result, 
                          0) != STATUS_SUCCESS)
    {
        std::cerr << "Failed to get key object size" << std::endl;
        return false;
    }

    if (BCryptSetProperty(*hAlg.get(),
                          BCRYPT_CHAINING_MODE,
                          (PUCHAR)BCRYPT_CHAIN_MODE_CBC,
                          sizeof(BCRYPT_CHAIN_MODE_CBC),
                          0) != STATUS_SUCCESS)
    {
        std::cerr << "Failed to set chaining mode" << std::endl;
        return false;
    }

    std::vector<BYTE> keyObject(keyObjectSize);
    if (BCryptGenerateSymmetricKey(*hAlg.get(),
                                   hKey.get(),
                                   keyObject.data(),
                                   keyObjectSize,
                                   (PUCHAR)key.data(),
                                   (ULONG)key.size(),
                                   0) != STATUS_SUCCESS)
    {
        std::cerr << "Failed to generate AES key" << std::endl;
        return false;
    }

    DWORD plainTextSize = 0;
    if (BCryptDecrypt(*hKey.get(),
                      (PUCHAR)cipherText.data(),
                      (ULONG)cipherText.size(),
                      nullptr,
                      (PUCHAR)iv.data(),
                      (ULONG)iv.size(),
                      nullptr,
                      0,
                      &plainTextSize,
                      BCRYPT_BLOCK_PADDING) != STATUS_SUCCESS)
    {
        std::cerr << "Failed to calculate plainText size" << std::endl;
        return false;
    }

    plainText.resize(plainTextSize);

    DWORD dataSize = 0;
    if (BCryptDecrypt(*hKey.get(),
                      (PUCHAR)cipherText.data(),
                      (ULONG)cipherText.size(),
                      nullptr,
                      (PUCHAR)iv.data(),
                      (ULONG)iv.size(),
                      plainText.data(),
                      plainTextSize,
                      &dataSize,
                      BCRYPT_BLOCK_PADDING) != STATUS_SUCCESS)
    {        
        std::cerr << "Decryption failed" << std::endl;
        return false;
    }

    plainText.resize(dataSize);

    return true;
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 3)
    {
        std::cerr << "Demo <plainText> <password>" << std::endl;
        return -1;
    }

    const std::string plainTextStr = argv[1];
    const std::string passwordStr = argv[2];

    // ComputeHash get SHA-256 data
    std::vector<BYTE> hash;
    std::vector<BYTE> password(passwordStr.begin(), passwordStr.end());
    if (!ComputeHash(password, hash))
    {
        std::cerr << "Failed to compute hash for password." << std::endl;
        return -1;
    }

    // AES supports 128, 192 and 256 bit.
    // So you can change hash size to chagne AES key length.
    // hash.resize(16);
    // hash.resize(24);

    // BCryptEncrypt function overrite initial vectors so we must copy initial vectors to use for decryption
    std::vector<BYTE> iv0 = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                              0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
    std::vector<BYTE> iv1(iv0.begin(), iv0.end());
    
    std::vector<BYTE> plainText(plainTextStr.begin(), plainTextStr.end());

    std::cout << "             Plain Text: " << plainTextStr << std::endl;
    std::cout << "       Plain Text (Hex): " << OutputBinaryAsHex(plainText) << std::endl;
    std::cout << "               Password: " << passwordStr << std::endl;
    std::cout << "      Password (Hashed): " << OutputBinaryAsHex(hash) << std::endl;
    std::cout << "                     IV: " << OutputBinaryAsHex(iv0) << std::endl;

    std::vector<BYTE> cipherText;
    if (!EncryptData(plainText, cipherText, hash, iv0))
    {
        std::cerr << "Encryption failed." << std::endl;
        return -1;
    }
    
    std::cout << "         Encrypted Text: " << OutputBinaryAsHex(cipherText) << std::endl;

    std::vector<BYTE> decryptedText;
    if (!DecryptData(cipherText, decryptedText, hash, iv1))
    {
        std::cerr << "Decryption failed." << std::endl;
        return -1;
    }

    std::cout << "         Decrypted Text: " << std::string(decryptedText.begin(), decryptedText.end()) << std::endl;
    std::cout << "   Decrypted Text (Hex): " << OutputBinaryAsHex(decryptedText) << std::endl;

    return 0;
}