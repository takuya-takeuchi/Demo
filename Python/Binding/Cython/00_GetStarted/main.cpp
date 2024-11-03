#include <iostream>
#include <filesystem>

#include <Python.h>

// Do not add extension (*.py) into tail fo scriptFileName
const char* scriptFileName = "PythonMod";
const char* scriptAddFunc = "add";

int main(int argc, char* argv[])
{
    Py_Initialize();

    // To make enable loading *.pyd
    const std::filesystem::path exePath(argv[0]);
    const std::filesystem::path exeFullPath = std::filesystem::absolute(exePath);
    const std::filesystem::path exeDir = exeFullPath.parent_path();
	std::cout << "[Info] execution directory: " << exeDir << std::endl;
    const std::string pythonCode = "import sys; sys.path.append(r'" + exeDir.string() + "')";
    PyRun_SimpleString(pythonCode.c_str());

	std::cout << "[Info] python script: " << scriptFileName << std::endl;
    PyObject* pName = PyUnicode_DecodeFSDefault(scriptFileName);
    PyObject* pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule == nullptr)
    {
        PyErr_Print();
        std::cerr << "[Error] Failed to load: " << scriptFileName << std::endl;
        return -1;
    }

	std::cout << "[Info] invoke: " << scriptAddFunc << " funtion" << std::endl;
    PyObject* pFunc = PyObject_GetAttrString(pModule, scriptAddFunc);
    if (pFunc && PyCallable_Check(pFunc))
    {
        PyObject* pArgs = PyTuple_Pack(2, PyLong_FromLong(2), PyLong_FromLong(5));
        PyObject* pValue = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pArgs);

        if (pValue == nullptr)
        {
            Py_DECREF(pFunc);
            Py_DECREF(pModule);
            PyErr_Print();

            std::cerr << "[Error] Call failed" << std::endl;
            return -1;
        }

        const long value = PyLong_AsLong(pValue);
        Py_DECREF(pValue);

        std::cout << "[Info] 2 + 5 = " << value << std::endl;
    }
    else
    {
        if (PyErr_Occurred())
            PyErr_Print();

        std::cerr << "[Error] Failed to find function: " << scriptAddFunc << std::endl;
    }

    Py_XDECREF(pFunc);
    Py_DECREF(pModule);

    Py_Finalize();

    return 0;
}