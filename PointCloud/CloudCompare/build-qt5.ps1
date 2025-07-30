#***************************************
#Arguments
#%1: Build Configuration (Release/Debug)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Configuration
)

$current = $PSScriptRoot

# get os name
if ($global:IsWindows)
{
    $os = "win"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
}

$target = "qt5"
$sharedFlag = "OFF"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

git submodule update --init --recursive .

if ($global:IsWindows)
{
    # Check Visual Studio is installed or not
    $visualStudioShells = @(
        "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat"
        "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvarsall.bat"
        "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvarsall.bat"
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

    function Call($batfile)
    {
        cmd.exe /c "call `"${batfile}`" && set > %temp%\vars.txt"
        Get-Content "${env:temp}\vars.txt" | Foreach-Object {
            if ($_ -match "^(.*?)=(.*)$") {
                Set-Content "env:\$($matches[1])" $matches[2]
            }
        }
    }
    
    $env:_ROOT="${sourceDir}"
    $env:PATH="$env:_ROOT\qtbase\bin;$env:_ROOT\gnuwin32\bin;$env:PATH"
    $env:PATH="$env:_ROOT\qtrepotools\bin;$env:PATH"

    # Uncomment the below line when building with OpenSSL enabled. If so, make sure the directory points
    # to the correct location (binaries for OpenSSL).
    # $env:PATH="C:\OpenSSL-Win32\bin;$env:PATH"

    # When compiling with ICU, uncomment the lines below and change <icupath> appropriately:
    # $env:INCLUDE="/path/to/icu/include;$env:INCLUDE"
    # $env:LIB="/path/to/icu/lib;$env:LIB"
    # $env:PATH="/path/to/icu/lib;$env:PATH"

    $env:_ROOT=""

    git -C "${sourceDir}" submodule update --init --recursive
    # cp patch/linux/qt5/qtbase/src/corelib/global/qglobal.h qt5/qtbase/src/corelib/global/qglobal.h
    
    Push-Location $buildDir
    $configure = Join-Path $sourceDir configure

    ## build ad lgpl (-confirm-license flags)
    if ($Configuration -eq "Release")
    {
        & "${configure}" -developer-build -opensource -nomake examples -nomake tests -confirm-license -prefix="${installDir}" -release
    }
    else
    {
        & "${configure}" -developer-build -opensource -nomake examples -nomake tests -confirm-license -prefix="${installDir}" 
    }

    nmake
    Pop-Location
}
elseif ($global:IsMacOS)
{
    git -C "${sourceDir}" submodule update --init --recursive
    # cp patch/linux/qt5/qtbase/src/corelib/global/qglobal.h qt5/qtbase/src/corelib/global/qglobal.h
    
    Push-Location $buildDir
    $configure = Join-Path $sourceDir configure

    ## build ad lgpl (-confirm-license flags)
    if ($Configuration -eq "Release")
    {
        & "${configure}" -developer-build -opensource -nomake examples -nomake tests -confirm-license -prefix="${installDir}" -release
    }
    else
    {
        & "${configure}" -developer-build -opensource -nomake examples -nomake tests -confirm-license -prefix="${installDir}" 
    }

    make -j$(nproc)
    make install
    Pop-Location
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          $sourceDir
}