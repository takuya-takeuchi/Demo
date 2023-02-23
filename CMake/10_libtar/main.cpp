#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <vector>

#include <libtar.h>

void readFile(const char* filename, std::vector<uint8_t>& vec)
{
    std::ifstream file(filename, std::ios::binary);
    file.unsetf(std::ios::skipws);

    std::streampos fileSize;
    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    vec.reserve(fileSize);
    vec.insert(vec.begin(),
               std::istream_iterator<uint8_t>(file),
               std::istream_iterator<uint8_t>());
}

int tar_content_write(TAR* tar,
                      const void* data,
                      const size_t len)
{
    uint8_t buf[T_BLOCKSIZE];
    uint8_t* current_pos = buf;

    const uint8_t* data_pos = (const uint8_t*)data;
    int cnt = 0;
    while (data_pos < (const uint8_t*)data + len)
    {
        size_t l = std::min((const uint8_t*)data + len - data_pos, buf + T_BLOCKSIZE - current_pos);
        memcpy(current_pos, data_pos, l);
        current_pos += l;
        data_pos += l;

        if (current_pos == buf + T_BLOCKSIZE)
        {
            const auto r = tar_block_write(tar, buf);
            current_pos = buf;
            if (!(r > 0))
                return r;
            cnt += r;
        }
    }

    if (current_pos == buf)
        return cnt;
    
    memset(current_pos, 0, buf + T_BLOCKSIZE - current_pos);
    auto r = tar_block_write(tar, buf);
    return (r >= 0)? cnt + r : r;
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 4)
    {
        std::cout << "Usage: test <path-to-tar-file> <path-to-input-file-path> <path-of-file-in-tar>" << std::endl;
        return -1;
    }

    const auto tar_file_path = argv[1];
    const auto input_file_path = argv[2];
    const auto path_of_file = argv[3];

    // create new tar file
    TAR* _tar;
    if (tar_open(&_tar,
                 tar_file_path,
                 nullptr,
                 O_WRONLY | O_CREAT | O_TRUNC,
                 S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
                 0) < 0)
    {
        throw std::runtime_error("tar_open() failed");
        std::cout << "" << std::endl;
        return -1;
    }

    // make it shared_ptr to be deleted automatically
    std::shared_ptr<TAR> tar(_tar, tar_close);

    // read as binary
    std::vector<uint8_t> content;
    readFile(input_file_path, content);

    // append the in-memory content to tar archive as a regular file
    // file metadata
    memset(&(tar->th_buf), 0, sizeof(tar->th_buf));
    th_set_path(tar.get(), path_of_file);
    th_set_size(tar.get(), content.size());
    th_set_type(tar.get(), S_IFREG);
    th_set_mode(tar.get(), S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);
    th_set_user(tar.get(), getuid());
    th_set_group(tar.get(), getgid());
    th_set_mtime(tar.get(), time(nullptr));
    if (th_write(tar.get()) < 0)
    {
        std::cout << "th_write() failed" << std::endl;
        return -1;
    }

    // file content following metadata
    if (tar_content_write(tar.get(), content.data(), content.size()) < 0)
    {
        std::cout << "tar_content_write() failed" << std::endl;
        return -1;
    }

    // Mark the end of tar file
    if (tar_append_eof(tar.get()) < 0)
    {
        std::cout << "tar_append_eof() failed" << std::endl;
        return -1;
    }

    std::cout << "succeeded to create tar file" << std::endl;

    return 0;
}