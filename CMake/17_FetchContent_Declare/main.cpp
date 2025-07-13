#include <iostream>
#include <vector>
#include <zlib.h>

int main()
{
    // Input data
    const char* src = "Hello, zlib world! This is a test of compression.";
    size_t src_len = strlen(src);

    // Prepare buffer for compressed data (zlib recommends: original size + 0.1% + 12 bytes)
    uLongf compressed_size = compressBound(src_len);
    std::vector<Bytef> compressed(compressed_size);

    // Compress
    int ret = compress(compressed.data(), &compressed_size, (const Bytef*)src, src_len);
    if (ret != Z_OK)
    {
        std::cerr << "compress failed: " << ret << std::endl;
        return 1;
    }
    std::cout << "Compressed size: " << compressed_size << " bytes" << std::endl;

    // Prepare buffer for uncompressed data (should be the same as the original size)
    std::vector<Bytef> uncompressed(src_len + 1); // +1 for null terminator
    uLongf uncompressed_size = src_len;

    // Uncompress
    ret = uncompress(uncompressed.data(), &uncompressed_size, compressed.data(), compressed_size);
    if (ret != Z_OK)
    {
        std::cerr << "uncompress failed: " << ret << std::endl;
        return 1;
    }

    uncompressed[uncompressed_size] = '\0'; // Null-terminate for string output
    std::cout << "Uncompressed: " << (char*)uncompressed.data() << std::endl;

    return 0;
}