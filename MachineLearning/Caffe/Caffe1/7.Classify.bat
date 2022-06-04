@echo off

@call 0.SetEnv.bat

IF EXIST "%OUTPUT_CLASSIFIED_NPY%" (
	@del "%OUTPUT_CLASSIFIED_NPY%"
)

"%MINICONDA%" "%CLASIFY%" "%1" "%OUTPUT_CLASSIFIED_NPY%" --model_def "%DEPLOY%" --pretrained_model "%WEIGHT%" --gpu --images_dim %HEIGHT%,%WIDTH% --mean_file "" --channel_swap "0"

IF EXIST "%OUTPUT_CLASSIFIED_NPY%" (
	"%MINICONDA%" ShowResult.py label.txt "%OUTPUT_CLASSIFIED_NPY%"
) ELSE (
	@echo "%OUTPUT_CLASSIFIED_NPY%" is not found!!
)
