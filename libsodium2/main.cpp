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
        goto ret;
    ofs.open(target_file.c_str(), std::ios::trunc | std::ios::binary);
    if (!ofs.good())
        goto ret;

    memset(&st, 0, sizeof st);
    memset(&header[0], 0, sizeof header);
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
    unsigned char* buffer = nullptr;

    std::ofstream ofs;
    std:: ifstream ifs;
    ifs.open(source_file.c_str(), std::ios::in | std::ios::binary);
    if (!ofs.good())
        goto ret;

    // get file size
    ifs.seekg( 0, std::ios_base::end );
    size_t filesize = ifs.tellg();
    ifs.seekg( 0, std::ios_base::beg );

    memset(&st, 0, sizeof st);
    memset(&header[0], 0, sizeof header);
    auto len = ifs.read((char*)header, sizeof header).gcount();
    if (crypto_secretstream_xchacha20poly1305_init_pull(&st, header, key) != 0)
    {
        goto ret; /* incomplete header */
    }

    size_t total = 0;
    buffer = (unsigned char*)calloc(filesize - sizeof header, sizeof(unsigned char));
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

        memcpy(&buffer[total], &buf_out[0], out_len);
        total += out_len;
    } while (! eof);

    ofs.open(target_file.c_str(), std::ios::trunc | std::ios::binary);
    if (!ofs.good())
        goto ret;
    
    ofs.write((char*)&buffer[0], total);

    ret = 0;
ret:
    ifs.close();
    ofs.close();
    if (buffer) free(buffer);
    return ret;
}

int main(int argc, char* argv[])
{
    if (argc != 5)
    {
        std::cout << "[Error] Argument must be <e/d> <password> <source file> <destination file>" << std::endl;
        return -1;
    }

    const char *mode = argv[1];
    const char *password = argv[2];
    const char *src = argv[3];
    const char *dst = argv[4];

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

    if (strcmp("e", mode) == 0)
    {
        if (encrypt(std::string(dst), std::string(src), key) != 0)
        {
            std::cout << "[Error] Failed to encrypt" << std::endl;
            return 1;
        }

        std::cout << "[Info] succeeded to encrypt" << std::endl;
        return 0;
    }
    else if (strcmp("d", mode) == 0)
    {
        if (decrypt(std::string(dst), std::string(src), key) != 0)
        {
            std::cout << "[Error] Failed to decrypt" << std::endl;
            return 1;
        }

        std::cout << "[Info] succeeded to dencrypt" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "[Error] Mode '" << mode << "' is invalid" << std::endl;
        return 0;
    }    
}