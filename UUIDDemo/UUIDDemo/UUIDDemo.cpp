// UUIDDemo.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"

#include <boost/filesystem.hpp>
#include <boost/locale.hpp>

using namespace boost::filesystem;
using namespace std;

#pragma comment(lib, "rpcrt4.lib")

int main()
{
	directory_iterator end_itr;
	for (directory_iterator itr("."); itr != end_itr; ++itr) {
		if (is_regular_file(*itr))
		{
			auto filename = (*itr).path().filename().string();

			// iterate only file which has 36 length name
			// 00000000-0000-0000-0000-000000000000
			if (filename.size() != 36)
				continue;

			std::cout << "std::string: " << filename << ", size: " << filename.size() << std::endl;

			auto cstr = filename.c_str();
			std::cout << "c_str: " << cstr << ", size: " << strlen(cstr) << std::endl;

			UUID Uuid;
			auto status = UuidFromStringA(RPC_CSTR(cstr), &Uuid);
			printf_s("UuidFromStringA returns %d\n", status);
		}
	}

    return 0;
}

