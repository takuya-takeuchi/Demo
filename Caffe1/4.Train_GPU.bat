@echo off

@call 0.SetEnv.bat

rem You must go through Model\solver.prototxt. You must specify the same value of solver.prototxt.
@mkdir Model\caffe_alexnet_train\caffe_train

"%BUILD%" train --solver="%SOLVER%" -gpu all