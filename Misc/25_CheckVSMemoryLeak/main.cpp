#ifdef _DEBUG
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#endif
#include <cstdlib>
#include <iostream>

int main()
{
#ifdef _DEBUG
    std::cout << "Set flag to _CrtSetDbgFlag" << std::endl;
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF);

    // To report memory leak report into stdio
    // Otherwise, report does not show until using Visual Studio Debugger window
    _CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_WARN, _CRTDBG_FILE_STDOUT);

    _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ERROR, _CRTDBG_FILE_STDOUT);

    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDOUT);
#endif

	int* a = (int*)malloc(sizeof(int) * 40000);
    int* leakedMemory = new int[100];

    // free(a);
    // delete leakedMemory;

#ifdef _DEBUG
    std::cout << "Check for memory leaks in the output window after the program ends." << std::endl;

    // If specified _CRTDBG_LEAK_CHECK_DF in _CrtSetDbgFlag, need not to call _CrtDumpMemoryLeaks
    _CrtDumpMemoryLeaks();
#endif

    return 0;
}
