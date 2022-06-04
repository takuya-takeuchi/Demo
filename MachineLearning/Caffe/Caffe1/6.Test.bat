@echo off

@call 0.SetEnv.bat

@mkdir %WEIGHT%

"%BUILD%" test --model="%DEPLOY%" --weights "%WEIGHT%"