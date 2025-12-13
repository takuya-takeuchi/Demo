#include <iostream>
#include <memory>
#include <string>

#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 2)
    {
        std::cout << "[Error] Demo </path/to/image>" << std::endl;
        return -1;
    }

    const auto image_path = argv[1];
    Pix *image = pixRead(image_path);
    if (image == nullptr)
    {
        std::cerr << "Failed to read '" << image_path << "'" << std::endl;
        return -1;
    }

    std::unique_ptr<tesseract::TessBaseAPI> ocr = std::make_unique<tesseract::TessBaseAPI>();
    if (ocr->Init(nullptr, "eng"))
    {
        std::cerr << "Could not initialize tesseract." << std::endl;
        return -1;
    }

    ocr->SetImage(image);
    ocr->Recognize(0);

    tesseract::ResultIterator* ri = ocr->GetIterator();
    tesseract::PageIteratorLevel level = tesseract::RIL_SYMBOL;

    if (ri != 0)
    {
        do {
            const char* symbol = ri->GetUTF8Text(level);
            float conf = ri->Confidence(level);
            int x1, y1, x2, y2;
            ri->BoundingBox(level, &x1, &y1, &x2, &y2);            
            std::cout << "Symbol: " << symbol << " Confidence: " << conf << " BoundingBox: [" << x1 << ", " << y1 << ", " << x2 << ", " << y2 << "]" << std::endl;
            delete[] symbol;
        } while (ri->Next(level));
    }

    ocr->End();
    pixDestroy(&image);

    return 0;
}