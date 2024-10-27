#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/rand.h>

#define AES_BLOCK_SIZE 16

struct EVP_CIPHER_CTX_Deleter {
    void operator()(EVP_CIPHER_CTX* ctx) const {
        if (ctx) EVP_CIPHER_CTX_free(ctx);
    }
};

struct EVP_MD_CTX_Deleter {
    void operator()(EVP_MD_CTX* ctx) const {
        if (ctx) EVP_MD_CTX_free(ctx);
    }
};

std::string OutputBinaryAsHex(const std::vector<uint8_t>& data)
{
    std::ostringstream oss;
    for (const uint8_t& byte : data)
        oss << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(byte) << " ";
    return oss.str();
}

bool ComputeHash(const std::vector<uint8_t>& data, std::vector<uint8_t>& hash)
{
    const size_t hashSize = 32;
    const char* name = "SHA256";
    const EVP_MD* md = EVP_get_digestbyname(name);
    if (!md)
    {
        std::cerr << "Unknown message digest " << name << std::endl;
        return false;
    }

    std::unique_ptr<EVP_MD_CTX, EVP_MD_CTX_Deleter> ctx(EVP_MD_CTX_new());
    if (!ctx)
    {
        std::cerr << "Failed to create context." << std::endl;
        return false;
    }

    if (!EVP_DigestInit_ex(ctx.get(), md, nullptr))
    {
        std::cerr << "Message digest initialize failed." << std::endl;
        return false;
    }

    if (!EVP_DigestUpdate(ctx.get(), data.data(), data.size()))
    {
        std::cerr << "Message digest update failed." << std::endl;
        return false;
    }

    hash.resize(hashSize);
    uint32_t tmp;
    if (!EVP_DigestFinal_ex(ctx.get(), hash.data(), &tmp))
    {
        std::cerr << "Message digest finalization failed." << std::endl;
        return false;
    }

    return true;
}

bool EncryptData(const EVP_CIPHER* cipher,
                 const std::vector<uint8_t>& plainText,
                 const std::vector<uint8_t>& key,
                 const std::vector<uint8_t>& iv,
                 std::vector<uint8_t>& cipherText)
{
    std::unique_ptr<EVP_CIPHER_CTX, EVP_CIPHER_CTX_Deleter> ctx(EVP_CIPHER_CTX_new());
    if (!ctx)
    {
        std::cerr << "Failed to create context." << std::endl;
        return false;
    }

    if (EVP_EncryptInit_ex(ctx.get(), cipher, nullptr, key.data(), iv.data()) != 1)
    {
        std::cerr << "Failed to initialize encryption. Error: " << ERR_error_string(ERR_get_error(), nullptr) << std::endl;
        return false;
    }

    size_t cipherTextSize = plainText.size() + AES_BLOCK_SIZE;
    cipherText.resize(cipherTextSize);

    int len;
    if (EVP_EncryptUpdate(ctx.get(), cipherText.data(), &len, plainText.data(), (int)plainText.size()) != 1)
    {
        std::cerr << "Failed to encrypt data. Error: " << ERR_error_string(ERR_get_error(), nullptr) << std::endl;
        return false;
    }

    cipherTextSize = len;

    if (EVP_EncryptFinal_ex(ctx.get(), (uint8_t*)cipherText.data() + len, &len) != 1)
    {
        std::cerr << "Failed to finalize encryption. Error: " << ERR_error_string(ERR_get_error(), nullptr) << std::endl;
        return false;
    }

    cipherTextSize += len;
    cipherText.resize(cipherTextSize);

    return true;
}

bool DecryptData(const EVP_CIPHER* cipher,
                 const std::vector<uint8_t>& cipherText,
                 const std::vector<uint8_t>& key,
                 const std::vector<uint8_t>& iv,
                 std::vector<uint8_t>& plainText)
{
    std::unique_ptr<EVP_CIPHER_CTX, EVP_CIPHER_CTX_Deleter> ctx(EVP_CIPHER_CTX_new());
    if (!ctx)
    {
        std::cerr << "Failed to create context."<< std::endl;
        return false;
    }

    if (EVP_DecryptInit_ex(ctx.get(), cipher, nullptr, key.data(), iv.data()) != 1)
    {
        std::cerr << "Failed to initialize decryption. Error: " << ERR_error_string(ERR_get_error(), nullptr) << std::endl;
        return false;
    }

    plainText.resize(cipherText.size());

    int len;
    if (EVP_DecryptUpdate(ctx.get(), plainText.data(), &len, cipherText.data(), (int)cipherText.size()) != 1)
    {
        std::cerr << "Failed to decrypt data. Error: " << ERR_error_string(ERR_get_error(), nullptr) << std::endl;
        return false;
    }

    size_t plainTextSize = len;
    if (EVP_DecryptFinal_ex(ctx.get(), (uint8_t*)plainText.data() + len, &len) != 1)
    {
        std::cerr << "Failed to finalize decryption. Error: " << ERR_error_string(ERR_get_error(), nullptr) << std::endl;
        return false;
    }

    plainTextSize += len;
    plainText.resize(plainTextSize);

    return true;
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 4)
    {
        std::cerr << "Demo <plainText> <password> <aes length. 128, 192 or 256>" << std::endl;
        return -1;
    }

    const std::string plainTextStr = argv[1];
    const std::string passwordStr = argv[2];
    const uint32_t length = (uint32_t)atoi(argv[3]);

    const EVP_CIPHER* cipher = nullptr;
    switch (length)
    {
        case 128:
            cipher = EVP_aes_128_cbc();
            break;
        case 192:
            cipher = EVP_aes_192_cbc();
            break;
        case 256:
            cipher = EVP_aes_256_cbc();
            break;
    }

    if (!cipher)
    {
        std::cerr << length << "is not supported AES length" << std::endl;
        return -1;
    }

    // ComputeHash get SHA-256 data
    std::vector<uint8_t> key;
    std::vector<uint8_t> password(passwordStr.begin(), passwordStr.end());
    if (!ComputeHash(password, key))
    {
        std::cerr << "Failed to compute hash for password." << std::endl;
        return -1;
    }

    // resize to fit AES length
    key.resize(length / 8);

    std::vector<uint8_t> iv = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };

    std::vector<uint8_t> plainText(plainTextStr.begin(), plainTextStr.end());

    std::cout << "              Algorithm: " << "AES-" << length << std::endl;
    std::cout << "             Plain Text: " << plainTextStr << std::endl;
    std::cout << "       Plain Text (Hex): " << OutputBinaryAsHex(plainText) << std::endl;
    std::cout << "               Password: " << passwordStr << std::endl;
    std::cout << "      Password (Hashed): " << OutputBinaryAsHex(key) << std::endl;
    std::cout << "                     IV: " << OutputBinaryAsHex(iv) << std::endl;

    std::vector<uint8_t> cipherText;
    if (!EncryptData(cipher, plainText, key, iv, cipherText))
    {
        std::cerr << "Encryption failed." << std::endl;
        return -1;
    }

    std::cout << "         Encrypted Text: " << OutputBinaryAsHex(cipherText) << std::endl;

    std::vector<uint8_t> decryptedText;
    if (!DecryptData(cipher, cipherText, key, iv, decryptedText))
    {
        std::cerr << "Decryption failed." << std::endl;
        return -1;
    }

    std::cout << "         Decrypted Text: " << std::string(decryptedText.begin(), decryptedText.end()) << std::endl;
    std::cout << "   Decrypted Text (Hex): " << OutputBinaryAsHex(decryptedText) << std::endl;

    return 0;
}
