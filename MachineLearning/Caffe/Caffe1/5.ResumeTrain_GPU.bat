@echo off

@call 0.SetEnv.bat

"%BUILD%" train --solver="%SOLVER%" --snapshot="Model\caffe_alexnet_train\caffe_train_iter_10000.solverstate" -gpu all