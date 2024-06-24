# Invoke as admiministrator
if (!([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole("Administrators"))
{
    Start-Process powershell.exe "-File `"$PSCommandPath`"" -Verb RunAs;
    exit
}

# This script is based on https://docs.ros.org/en/crystal/Installation/Windows-Install-Binary.html

$current = $PSScriptRoot

# Check Visual Studio is installed or not
$visualStudioShells = @(
    "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
    "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat"
    "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
    "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat"
    "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\VC\Auxiliary\Build\vcvars64.bat"
    "C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
    "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvars64.bat"
    "C:\Program Files (x86)\Microsoft Visual Studio\2017\Professional\VC\Auxiliary\Build\vcvars64.bat"
    "C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
    "C:\Program Files (x86)\Microsoft Visual Studio\2015\Community\VC\Auxiliary\Build\vcvars64.bat"
    "C:\Program Files (x86)\Microsoft Visual Studio\2015\Professional\VC\Auxiliary\Build\vcvars64.bat"
    "C:\Program Files (x86)\Microsoft Visual Studio\2015\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
)

$visualStudioShell = ""
foreach ($shell in $visualStudioShells)
{
    $visualStudioShell = $shell
    if (Test-Path($visualStudioShell))
    {
        break
    }
}

if (!(Test-Path($visualStudioShell)))
{
    Write-Host "[Error] Visual Studio is not installed or x64 Native Tools Command Prompt is missing" -ForegroundColor Red
    exit
}

Write-Host "[Info] Visual Studio x64 Native Tools Command Prompt: ${visualStudioShell}" -ForegroundColor Green

# Check Chocolatey is installed or not
try
{
    $commandInfo = Get-Command choco -ErrorAction Stop
    $choco = $commandInfo.Source
    Write-Host "[Info] Chocolatey: ${choco}" -ForegroundColor Green
}
catch
{
    Write-Host "[Error] Chocolatey is not installed" -ForegroundColor Red
    exit
}

# Check Chocolatey is installed or not
try
{
    $commandInfo = Get-Command cmake -ErrorAction Stop
    $cmake = $commandInfo.Source
    Write-Host "[Info] CMake: ${cmake}" -ForegroundColor Green
}
catch
{
    Write-Host "[Error] CMake is not installed" -ForegroundColor Red
    exit
}

# Install embed python
$python_major_version = "38"
$python_version = "3.8.3"
$python_url_filename = "python-${python_version}-embed-amd64.zip"
$url = "https://www.python.org/ftp/python/${python_version}/${python_url_filename}"
$python_filename = "python.exe"
$installPythonDir = "C:\Python38"
$python = Join-Path $installPythonDir "python.exe"
if (!(Test-Path(${python})))
{
    Write-Host "[Info] ${python} is missing" -ForegroundColor Yellow
    Invoke-WebRequest "${url}" -OutFile "${python_url_filename}"
    New-Item -Type Directory $installPythonDir -Force | Out-Null
    Expand-Archive -Path "${python_url_filename}" -DestinationPath $installPythonDir
    Remove-Item "${python_url_filename}" -Force | Out-Null

    # replace `#import site`
    $pth = Join-Path $installPythonDir "python${python_major_version}._pth"
    $content = Get-Content -Path $pth -Raw
    $updatedContent = $content -replace "#import site", "import site"
    Set-Content -Path $pth -Value $updatedContent

    # expand python3X.zip to Lib
    $zip = Join-Path $installPythonDir "python${python_major_version}.zip"
    $libDir = Join-Path $installPythonDir Lib
    Expand-Archive -Path "${zip}" -DestinationPath $libDir
    
    $scriptsDir = Join-Path $installPythonDir "Scripts"
    New-Item -Type Directory ${scriptsDir} -Force | Out-Null
    Invoke-WebRequest https://bootstrap.pypa.io/get-pip.py -OutFile "get-pip.py"
    & "${python}" "get-pip.py"
    Remove-Item "get-pip.py"

    # If this file is represent, https://github.com/ros2/ros2/issues/1416 occurs
    $pth = Join-Path $installPythonDir "python38._pth"
    Remove-Item "${pth}" -Force | Out-Null
}

# build CPython
$libDir = Join-Path $installPythonDir "libs"
if (!(Test-Path(${libDir})))
{
    # build cpython to get include and libs
    $cpython_version = "3.8"
    $installCPythonDir = Join-Path $current install | `
                         Join-Path -ChildPath windows | `
                         Join-Path -ChildPath cpython
    New-Item -Type Directory $installCPythonDir -Force | Out-Null
    $installCPythonDir = Join-Path $installCPythonDir $cpython_version 
    $buildCPythonDir = Join-Path $installCPythonDir "PCbuild"
    $bat = Join-Path $buildCPythonDir "build.bat"
    if (!(Test-Path $installCPythonDir))
    {
        git clone -b $cpython_version "https://github.com/python/cpython" $installCPythonDir
    }
    else
    {
        Write-Host "[Info] CPython is already cloned" -ForegroundColor Yellow
    }

    Push-Location $buildCPythonDir | Out-Null
    git checkout $cpython_version
    $args = "-e -p x64"
    Start-Process -FilePath "cmd.exe" -ArgumentList "/c", ${bat}, $args -Wait
    Start-Process -FilePath "cmd.exe" -ArgumentList "/c", ${bat}, $args -Wait
    Pop-Location

    # copy inlcude dir
    $includeDir = Join-Path $installCPythonDir include
    Copy-Item "${includeDir}" "${installPythonDir}" -Force -Recurse
    
    # copy libs
    New-Item -Type Directory $libDir -Force | Out-Null
    $cpythonLibs = @( "_tkinter.lib", "python3.lib", "python${python_major_version}.lib" )
    foreach ($lib in $cpythonLibs)
    {
        $src = Join-Path $buildCPythonDir "amd64" | Join-Path -ChildPath $lib
        $dst  = Join-Path $libDir $lib
        Copy-Item "${src}" "${dst}" -Force
    }
}

$chocoPackages = @{
    "asio 1.12.1"="https://github.com/ros2/choco-packages/releases/download/2022-03-15/asio.1.12.1.nupkg"
    "eigen 3.3.4"="https://github.com/ros2/choco-packages/releases/download/2022-03-15/eigen.3.3.4.nupkg"
    "log4cxx 0.10.0"="https://github.com/ros2/choco-packages/releases/download/2022-03-15/log4cxx.0.10.0.nupkg"
    "tinyxml2 6.0.0"="https://github.com/ros2/choco-packages/releases/download/2022-03-15/tinyxml2.6.0.0.nupkg"
    "tinyxml-usestl 2.6.2"="https://github.com/ros2/choco-packages/releases/download/2022-03-15/tinyxml-usestl.2.6.2.nupkg"
}
$installedPackages = & ${choco} list
foreach($entry in $chocoPackages.GetEnumerator())
{
    $key = $entry.Key;
    $url = $entry.Value;
    if (!(${installedPackages} -contains "${key}"))
    {
        Write-Host "[Warn] ${key} is not installed" -ForegroundColor Yellow
        $parts = $url -split '/'
        $path = Join-Path $current $parts[-1]
        Invoke-WebRequest ${url} -OutFile $path
        $parts = $key -split ' '
        & "${choco}" install $parts[0] -y -s $current
        Remove-Item $path
    }
}

# Check whether openssl is installed or not
if (!(${installedPackages} -contains "openssl 1.1.1.2100"))
{
    & "${choco}" install -y openssl --version=1.1.1.2100
    [Environment]::SetEnvironmentVariable("OPENSSL_CONF", "C:\Program Files\OpenSSL-Win64\bin\openssl.cfg", "Machine")
}

# Check whether cppcheck is installed or not
if (!(${installedPackages} -contains "cppcheck"))
{
    & "${choco}" install -y cppcheck
}

# Check whether opencv is installed or not
$opencv_url_filename = "opencv-3.4.1-vc15.VS2017.zip"
$url = "https://github.com/ros2/ros2/releases/download/opencv-archives/${opencv_url_filename}"
$installOpenCVDir = "C:\opencv\3.4.1"
if (!(Test-Path(${installOpenCVDir})))
{
    Write-Host "[Info] OpenCV is not installed" -ForegroundColor Yellow
    Invoke-WebRequest "${url}" -OutFile "${opencv_url_filename}"
    powershell Expand-Archive -Path "${opencv_url_filename}" -DestinationPath "${installOpenCVDir}"
    Remove-Item "${opencv_url_filename}"
    [Environment]::SetEnvironmentVariable("OpenCV_DIR", "${installOpenCVDir}", "Machine")
}

# Install python packages
$installedPackages = & "${python}" -m pip freeze
$pythonPackages = @{
	"catkin_pkg"="catkin-pkg"
	"empy"="empy"
	"lark-parser"="lark-parser"
	"opencv-python"="opencv-python"
	"pyparsing"="pyparsing"
	"pyyaml"="PyYAML"
	"setuptools"="setuptools==66.1.1" # https://stackoverflow.com/questions/76043689/pkg-resources-is-deprecated-as-an-api
	"pydot"="pydot"
	"PyQt5"="PyQt5"
	"lxml"="lxml"
	"packaging"="packaging"
    "colcon-common-extensions"="colcon-common-extensions"
    "flake8"="flake8"
    "flake8-blind-except"="flake8-blind-except"
    "flake8-builtins"="flake8-builtins"
    "flake8-class-newline"="flake8-class-newline"
    "flake8-comprehensions"="flake8-comprehensions"
    "flake8-deprecated"="flake8-deprecated"
    "flake8-docstrings"="flake8-docstrings"
    "flake8-import-order"="flake8-import-order"
    "flake8-quotes"="flake8-quotes"
    "pep8"="pep8"
    "pydocstyle"="pydocstyle"
    "pytest"="pytest"
    "coverage"="coverage"
    "mock"="mock"
}
foreach($entry in $pythonPackages.GetEnumerator())
{
    $pythonPackage = $entry.Value;
    if (!("${installedPackages}".Contains("${pythonPackage}")))
    {
        Write-Host "[Info] ${pythonPackage} is not installed" -ForegroundColor Yellow
        & "${python}" -m pip install -U $entry.Key
    }
}
# Setup ROS2 packages
$ros2Packages = @{
	"humble"="https://github.com/ros2/ros2/releases/download/release-humble-20240523/ros2-humble-20240523-windows-release-amd64.zip"
}
foreach($entry in $ros2Packages.GetEnumerator())
{
    $package = $entry.Key;
    $url = $entry.Value;
    $installedRos2Dir = Join-Path $current "ros2" | Join-Path -ChildPath $package
    if (!(Test-Path(${installedRos2Dir})))
    {
        Write-Host "[Info] ROS2 ${package} is not installed" -ForegroundColor Yellow
        $parts = $url -split '/'
        $path = Join-Path $current $parts[-1]
        if (!(Test-Path($path)))
        {
            Invoke-WebRequest "${url}" -OutFile "${path}"
        }
        powershell Expand-Archive -Path "${path}" -DestinationPath "${installedRos2Dir}"
        # Remove-Item "${path}"
    }
}