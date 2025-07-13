#include <iostream>
#include <vector>
#include <cstdlib>

#include <jpeglib.h>

void save_jpeg(const char* filename, const unsigned char* rgb_buffer, int width, int height, int quality = 90)
{
    FILE* outfile = fopen(filename, "wb");
    if (!outfile)
    {
        std::cerr << "Cannot open output file " << filename << std::endl;
        return;
    }

    jpeg_compress_struct cinfo;
    jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);

    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, outfile);

    cinfo.image_width      = width;
    cinfo.image_height     = height;
    cinfo.input_components = 3;
    cinfo.in_color_space   = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);

    jpeg_start_compress(&cinfo, TRUE);

    JSAMPROW row_pointer[1];
    int row_stride = width * 3;

    while (cinfo.next_scanline < cinfo.image_height)
    {
        row_pointer[0] = (unsigned char*)&rgb_buffer[cinfo.next_scanline * row_stride];
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    fclose(outfile);
}

int main()
{
    int width = 100, height = 100;
    std::vector<unsigned char> rgb(width * height * 3, 0);
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
        {
            rgb[(y * width + x) * 3 + 0] = 255; // R
            rgb[(y * width + x) * 3 + 1] = 0;   // G
            rgb[(y * width + x) * 3 + 2] = 0;   // B
        }

    save_jpeg("test.jpg", rgb.data(), width, height, 85);

    std::cout << "test.jpg has been output" << std::endl;

    return 0;
}