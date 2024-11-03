#include <iostream>

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>

const char* scriptFileName = "PythonMod.py";
const char* scriptAddFunc = "add";

int main()
{
    pybind11::scoped_interpreter guard{};

    try
	{
		std::cout << "[Info] import: " << scriptFileName << std::endl;
        pybind11::module_ pyModule = pybind11::module_::import(scriptFileName);

		std::cout << "[Info] invoke: " << scriptAddFunc << " funtion" << std::endl;
        pybind11::function add = pyModule.attr(scriptAddFunc);
        int result = add(3, 4).cast<int>();

		std::cout << "[Info] 2 + 5 = ";
		std::cout << result << std::endl;
    }
	catch (const std::exception &e)
	{
        std::cerr << "[Error] An error occurred: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
