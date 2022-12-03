// OpenCV2Lib.h

#define BYTE unsigned char

#pragma once

using namespace System;
using namespace System::Drawing;
using namespace System::Drawing::Imaging;

namespace OpenCV2Lib {

	public ref class Test
	{
		// TODO: このクラスの、ユーザーのメソッドをここに追加してください。
	public:
		static void Optimized(Bitmap^ bitmap)
		{
			PixelFormat^ format = bitmap->PixelFormat;
			Rectangle^ rectangle = gcnew Rectangle(Point::Empty, bitmap->Size);
			BitmapData^ bitmapData = bitmap->LockBits(*rectangle, ImageLockMode::ReadOnly, *format);
			
			BYTE* pBitmap = static_cast<BYTE*>((void*)bitmapData->Scan0);

			Bitmap^ gray = gcnew Bitmap(rectangle->Width / 2, rectangle->Height / 2, PixelFormat::Format8bppIndexed);
			{
				Rectangle^ rectangleGray = gcnew Rectangle(Point::Empty, gray->Size);
				BitmapData^ bitmapGrayData = gray->LockBits(*rectangleGray, ImageLockMode::ReadWrite, PixelFormat::Format8bppIndexed);
				BYTE* gBitmap = static_cast<BYTE*>((void*)bitmapGrayData->Scan0);

				Optimized(pBitmap, gBitmap, bitmapData->Width, bitmapData->Stride, bitmapData->Height, 3);

				gray->UnlockBits(bitmapGrayData);
				bitmap->UnlockBits(bitmapData);
			}
		}

	private:
		static void Optimized(BYTE* pBitmap, BYTE* oBitmap, int width, int stride, int height, int bitCount)
		{
			BYTE* ptr = pBitmap;

			BYTE histogram[256];
			double accumulated[256];
			BYTE* h = &histogram[0];
			double* a = &accumulated[0];
			{
				width /= 2;
				height /= 2;
				for (int y = 0; y < height; y++)
				{
					int currentY = y * 2 * stride;
					int currentOutputY = y * width;
					for (int x = 0; x < width; x++)
					{
						int currentX = x * 2 * bitCount;

						// グレイスケール化
						BYTE b = ptr[currentY + currentX + 0];
						BYTE g = ptr[currentY + currentX + 1];
						BYTE r = ptr[currentY + currentX + 2];

						BYTE value = (BYTE)(0.299 * r + 0.587 * g + 0.114 * b);
						oBitmap[currentOutputY + x] = value;

						// ヒストグラム
						h[value]++;
					}
				}

				// 累積値計算
				double total = 0;
				for (int i = 0; i < 256; i++)
				{
					total += h[i];
					a[i] = total;
				}

				for (int i = 0; i < 256; i++)
				{
					a[i] = (a[i] * 255) / total;
				}

				// 補正
				for (int y = 0; y < height; y++)
				{
					int currentY = y * width;
					for (int x = 0; x < width; x++)
					{
						BYTE value = oBitmap[currentY + x];
						oBitmap[currentY + x] = (BYTE)a[value];
					}
				}
			}
		}

	};
}
