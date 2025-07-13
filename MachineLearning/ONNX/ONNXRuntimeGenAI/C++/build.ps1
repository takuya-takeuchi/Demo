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

$target = "onnxruntime-genai"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $Configuration

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

# create python virtual env
$venvDir = Join-Path $current ".venv"
if (!(Test-Path("${venvDir}")))
{
    Write-Host "[Info] Create python venv"
    python3 -m venv "${venvDir}"
}

Write-Host "[Info] Activate python venv"
if ($global:IsWindows)
{
    . "${venvDir}\Scripts\activate.ps1"
    Write-Host "[Info] Activated"
}
elseif ($global:IsMacOS)
{
    . "${venvDir}/bin/Activate.ps1"
    Write-Host "[Info] Activated"
}
elseif ($global:IsLinux)
{
    . "${venvDir}/bin/Activate.ps1"
    Write-Host "[Info] Activated"
}

Write-Host "[Info] Check python executable location"
python3 -c "import sys; print(f'Python executable: {sys.executable}')"

python3 -m pip install pip --upgrade
python3 -m pip install requests

Push-Location $target

git submodule update --init --recursive .

if ($global:IsWindows)
{
    python3 build.py --config ${Configuration} `
                     --cmake_generator "Visual Studio 17 2022" `
                     --parallel `
                     --build_dir ${buildDir} `
                     --skip_tests `
                     --skip_wheel `
                     --cmake_extra_defines CMAKE_INSTALL_PREFIX=$installDir

    cmake --install "${buildDir}\${Configuration}"
    Copy-Item "${buildDir}\${Configuration}\*.dll" "${installDir}\lib" -Force
    Copy-Item "${buildDir}\${Configuration}\*.lib" "${installDir}\lib" -Force
}
elseif ($global:IsMacOS)
{
    python3 build.py --config ${Configuration} `
                     --parallel `
                     --build_dir ${buildDir} `
                     --skip_tests `
                     --skip_wheel `
                     --cmake_extra_defines CMAKE_INSTALL_PREFIX=$installDir

    cmake --install "${buildDir}\${Configuration}"
    Copy-Item "${buildDir}\${Configuration}\*.dll" "${installDir}\lib" -Force
    Copy-Item "${buildDir}\${Configuration}\*.lib" "${installDir}\lib" -Force
}
elseif ($global:IsLinux)
{
    python3 build.py --config ${Configuration} `
                     --parallel `
                     --build_dir ${buildDir} `
                     --skip_tests `
                     --skip_wheel `
                     --cmake_extra_defines CMAKE_INSTALL_PREFIX=$installDir

    cmake --install "${buildDir}\${Configuration}"
    Copy-Item "${buildDir}\${Configuration}\*.dll" "${installDir}\lib" -Force
    Copy-Item "${buildDir}\${Configuration}\*.lib" "${installDir}\lib" -Force
}
Pop-Location