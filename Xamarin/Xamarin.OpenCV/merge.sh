xcrun -sdk iphoneos lipo -create ./x64/Release/libOpenCV.iOS.a \
                                 ./x86/Release/libOpenCV.iOS.a \
                                 ./ARM/Release/libOpenCV.iOS.a \
                                 ./ARM64/Release/libOpenCV.iOS.a \
                                 -output ./libopencv.a
file libopencv.a