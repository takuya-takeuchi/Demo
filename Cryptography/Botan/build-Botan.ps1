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

$target = "Botan"
$url = "https://github.com/randombit/botan"
$branch = "55952b2a8b9ec6ae4220404837bc6942ea03be04"

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

$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

New-Item -Type Directory $installDir -Force | Out-Null

if (!(Test-Path($target)))
{
    git clone $url
}

Push-Location $target
git checkout $branch

if ($global:IsWindows)
{
    function Call($batfile)
    {
        cmd.exe /c "call `"${batfile}`" && set > %temp%\vars.txt"
        Get-Content "${env:temp}\vars.txt" | Foreach-Object {
            if ($_ -match "^(.*?)=(.*)$") {
                Set-Content "env:\$($matches[1])" $matches[2]
            }
        }
    }

    Call("C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat")

    python configure.py --with-cmake-config --prefix="${installDir}"
    nmake
    nmake install
}
elseif ($global:IsMacOS)
{
    python3 configure.py --with-cmake-config --prefix="${installDir}"
    make
    make install
}
elseif ($global:IsLinux)
{
    python3 configure.py --with-cmake-config --prefix="${installDir}"
    make
    make install
}

Pop-Location