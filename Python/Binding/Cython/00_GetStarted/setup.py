from setuptools import Extension, setup
from Cython.Build import cythonize

extensions = [
    Extension("PythonMod", ["PythonMod.pyx"])
]

setup(
    name="CythonDemo",
    ext_modules=cythonize(extensions),
    zip_safe=False
)