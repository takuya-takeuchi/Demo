set CAFFE_ROOT=D:\Works\Lib\Microsoft\caffe
set CLASIFY=%CAFFE_ROOT%\Build\x64\Release\pycaffe\classify.py
set BUILD=%CAFFE_ROOT%\Build\x64\Release\caffe.exe
set CONVERT=%CAFFE_ROOT%\Build\x64\Release\convert_imageset.exe
set MEAN=%CAFFE%\Build\x64\Release\compute_image_mean.exe
set BACKEND=lmdb
set OUTPUT_CLASSIFIED_NPY=output.npy
set MINICONDA=C:\Program Files\Miniconda2\python.exe

rem must not specify '\' at end of value of ROOTFOLDER
set ROOTFOLDER=D:\Works\NISTSpecialDatabase4GrayScaleImagesofFIGS\sd04\png_txt
set OUTPUT_BASE=%ROOTFOLDER%\Caffe\
set OUTPUT_TRAIN=%OUTPUT_BASE%train
set OUTPUT_MEAN_TRAIN=%OUTPUT_BASE%train_mean
set OUTPUT_TEST=%OUTPUT_BASE%test
set OUTPUT_MEAN_TEST=%OUTPUT_BASE%test_mean

set WIDTH=256
set HEIGHT=256

set LISTFILE_TRAIN=%ROOTFOLDER%\train.txt
set LISTFILE_TEST=%ROOTFOLDER%\test.txt

set DEPLOY=%ROOTFOLDER%\Model\train_val.prototxt
set WEIGHT=%ROOTFOLDER%\Model\caffe_alexnet_train_iter_3500.caffemodel

set SOLVER=Model\solver.prototxt
set DEPLOY=Model\deploy.prototxt