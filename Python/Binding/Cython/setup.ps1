#***************************************
#Arguments
#%1: PythonPath (e.g. C:\Python\3.11\x64\python.exe)
#***************************************
Param
(   
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $PythonPath
)

$current = $PSScriptRoot

if (!(Test-Path($PythonPath)))
{
    Write-Host "[Error] ${PythonPath} is missing" -ForegroundColor Red
    exit
}

$virtualEnvDirName = ".venv"
$virtualEnvDir = Join-Path $current $virtualEnvDirName
if (Test-Path($virtualEnvDir))
{
    Remove-Item "${virtualEnvDir}" -Force -Recurse | Out-Null
}

& "${PythonPath}" -m venv .venv

$activateScript = Join-Path $virtualEnvDir Scripts | `
                  Join-Path -ChildPath Activate.ps1
& "${activateScript}"
python -m pip install --upgrade pip
python -m pip install cython setuptools