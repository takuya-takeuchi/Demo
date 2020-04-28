#include <string>
#include <iostream>
#include <fstream>

// override SODIUM_EXPORT if use libsodium as static
#define SODIUM_DLL_EXPORT
#include <sodium.h>

#define CHUNK_SIZE 4096

static int encrypt(std::string & target_file, std::string & source_file,
                   const unsigned char key[crypto_secretstream_xchacha20poly1305_KEYBYTES])
{
    unsigned char  buf_in[CHUNK_SIZE];
    unsigned char  buf_out[CHUNK_SIZE + crypto_secretstream_xchacha20poly1305_ABYTES];
    unsigned char  header[crypto_secretstream_xchacha20poly1305_HEADERBYTES];
    crypto_secretstream_xchacha20poly1305_state st;
    bool           eof;
    int            ret = -1;

    std::ofstream ofs;
    std:: ifstream ifs;
    ifs.open(source_file.c_str(), std::ios::in | std::ios::binary);
    if (!ofs.good())
        return ret;
    ofs.open(target_file.c_str(), std::ios::trunc | std::ios::binary);
    if (!ofs.good())
        return ret;

    crypto_secretstream_xchacha20poly1305_init_push(&st, header, key);
    ofs.write((char*)(&header[0]), sizeof header);
    do {
        auto rlen = ifs.read((char*)buf_in, sizeof buf_in).gcount();
        eof = ifs.eof();
        auto tag = eof ? crypto_secretstream_xchacha20poly1305_TAG_FINAL : 0;

        unsigned long long out_len;        
        if (crypto_secretstream_xchacha20poly1305_push(&st,
                                                   buf_out,
                                                   &out_len,
                                                   buf_in,
                                                   rlen,
                                                   NULL,
                                                   0,
                                                   tag) != 0) {
            goto ret;
        }

        ofs.write((char*)(&buf_out[0]), (size_t)out_len);
        ofs.flush();
    } while (!eof);

    ret = 0;
ret:
    ifs.close();
    ofs.close();

    return ret;
}

static int decrypt(std::string & target_file, std::string & source_file,
                   const unsigned char key[crypto_secretstream_xchacha20poly1305_KEYBYTES])
{
    unsigned char  buf_in[CHUNK_SIZE + crypto_secretstream_xchacha20poly1305_ABYTES];
    unsigned char  buf_out[CHUNK_SIZE];
    unsigned char  header[crypto_secretstream_xchacha20poly1305_HEADERBYTES];
    crypto_secretstream_xchacha20poly1305_state st;
    bool           eof;
    int            ret = -1;

    std::ofstream ofs;
    std:: ifstream ifs;
    ifs.open(source_file.c_str(), std::ios::in | std::ios::binary);
    if (!ofs.good())
        return ret;
    ofs.open(target_file.c_str(), std::ios::trunc | std::ios::binary);
    if (!ofs.good())
        return ret;

    ifs.read((char*)header, sizeof header);
    if (crypto_secretstream_xchacha20poly1305_init_pull(&st, header, key) != 0)
    {
        goto ret; /* incomplete header */
    }

    do {
        ifs.read((char*)buf_in, sizeof buf_in);
        auto rlen = ifs.gcount();
        eof = ifs.eof();

        unsigned char  tag;
        unsigned long long out_len;
        if (crypto_secretstream_xchacha20poly1305_pull(&st,
                                                       buf_out,
                                                       &out_len,
                                                       &tag,
                                                       buf_in,
                                                       rlen,
                                                       NULL,
                                                       0) != 0) {
            goto ret; /* corrupted chunk */
        }

        if (tag == crypto_secretstream_xchacha20poly1305_TAG_FINAL && ! eof) {
            goto ret; /* premature end (end of file reached before the end of the stream) */
        }

        ofs.write((char*)(&buf_out[0]), (size_t)out_len);
        ofs.flush();
    } while (! eof);

    ret = 0;
ret:
    ifs.close();
    ofs.close();
    return ret;
}

int main(int argc, char* argv[])
{
    if (argc != 5)
    {
        std::cout << "[Error] Argument must be <password> <source file> <encrypted file> <destination file>" << std::endl;
        return -1;
    }

    const char *password = argv[1];
    const char *fileToEncrypt = argv[2];
    const char *encryptedFile = argv[3];
    const char *outputFile = argv[4];

    if (strnlen_s(password, crypto_secretstream_xchacha20poly1305_KEYBYTES) != crypto_secretstream_xchacha20poly1305_KEYBYTES)
    {
        std::cout << "[Error] password length must be 32" << std::endl;
        return -1;
    }

    std::cout << "[Info] sodium_init" << std::endl;
    const int32_t sodiumInitReturnVal = sodium_init();
    if (sodiumInitReturnVal < 0)
    {
        std::cout << "[Error] libsodium couldn't be initialized with sodium_init()" << std::endl;
        return -1;
    }

    if (sodiumInitReturnVal == 1)
    {
        std::cout << "[Info] libsodium has been already initialized" << std::endl;
    }
    else
    {
        std::cout << "[Info] libsodium successfully initialized" << std::endl;
    }

    // https://libsodium.gitbook.io/doc/secret-key_cryptography/secretstream#file-encryption-example-code

    // 256bit = 32byte
    unsigned char key[crypto_secretstream_xchacha20poly1305_KEYBYTES];

    //crypto_secretstream_xchacha20poly1305_keygen(key);
    memcpy(&key[0], password, crypto_secretstream_xchacha20poly1305_KEYBYTES);

    if (encrypt(std::string(encryptedFile), std::string(fileToEncrypt), key) != 0)
    {
        std::cout << "[Error] Failed to encrypt" << std::endl;
        return 1;
    }

    std::cout << "[Info] succeeded to encrypt" << std::endl;

    if (decrypt(std::string(outputFile), std::string(encryptedFile), key) != 0)
    {
        std::cout << "[Error] Failed to decrypt" << std::endl;
        return 1;
    }

    std::cout << "[Info] succeeded to dencrypt" << std::endl;

    return 0;
}