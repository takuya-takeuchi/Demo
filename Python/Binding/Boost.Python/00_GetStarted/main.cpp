#include <iostream>

#define BOOST_PYTHON_STATIC_LIB
#include <boost/python.hpp>

const char* scriptFileName = "PythonMod.py";
const char* scriptAddFunc = "add";

int main()
{
	Py_Initialize();

	boost::python::object modle = boost::python::import("__main__");
	boost::python::object attr = modle.attr("__dict__");

	std::cout << "[Info] exec_file: " << scriptFileName << std::endl;
	boost::python::exec_file(scriptFileName, attr, attr);

	std::cout << "[Info] invoke: " << scriptAddFunc << " funtion" << std::endl;
	boost::python::object pythonFun = attr[scriptAddFunc];

	boost::python::object v = pythonFun(2, 5);

	std::cout << "[Info] 2 + 5 = ";
	std::cout << boost::python::extract<int>(v) << std::endl;
}