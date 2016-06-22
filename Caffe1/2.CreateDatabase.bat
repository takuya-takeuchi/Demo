@echo off

@call 0.SetEnv.bat

rmdir "%OUTPUT_BASE%" /s /q
mkdir "%OUTPUT_BASE%"

"%CONVERT%" "%ROOTFOLDER%" "%LISTFILE_TRAIN%" "%OUTPUT_TRAIN%_%BACKEND%" -resize_height %HEIGHT% -resize_width %WIDTH% -backend "%BACKEND%" -gray true
"%CONVERT%" "%ROOTFOLDER%" "%LISTFILE_TEST%" "%OUTPUT_TEST%_%BACKEND%" -resize_height %HEIGHT% -resize_width %WIDTH% -backend "%BACKEND%" -gray true