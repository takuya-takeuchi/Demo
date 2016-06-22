@echo off

@call 0.SetEnv.bat

"%MEAN%" "%OUTPUT_TRAIN%_%BACKEND%" "%OUTPUT_MEAN_TRAIN%_%BACKEND%.binaryproto"
"%MEAN%" "%OUTPUT_TEST%_%BACKEND%" "%OUTPUT_MEAN_TEST%_%BACKEND%.binaryproto"