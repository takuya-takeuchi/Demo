#***************************************
#Arguments
#%1: Version  (e.g. 3.3.2)
#%2: Build Configuration (Release/Debug)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Version,

   [Parameter(
   Mandatory=$True,
   Position = 2
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

$target = "openssl"
$shared = "static"
$sharedFlag = "OFF"

# build
$sourceDir = Join-Path $current $target | `
             Join-Path -ChildPath $Version
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $Version | `
            Join-Path -ChildPath $shared
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $Version | `
              Join-Path -ChildPath $shared

$exist = Test-Path(${sourceDir})
if (!$exist)
{
    Write-Host "[Error] '${sourceDir}' is missing" -ForegroundColor Red
    exit
}

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

if ($global:IsWindows)
{
    # check build tool
    $perlDir = $env:PERLPATH
    if (!$perlDir)
    {
        Write-Host "[Error] Environmental Variable: PERLPATH is missing" -ForegroundColor Red
        exit
    }

    if (!(Test-Path("${perlDir}")))
    {
        Write-Host "[Error] '${perlDir}' is missing" -ForegroundColor Red
        exit
    }

    $nasmDir = $env:NASMPATH
    if (!$nasmDir)
    {
        Write-Host "[Error] Environmental Variable: NASMPATH is missing" -ForegroundColor Red
        exit
    }

    if (!(Test-Path("${nasmDir}")))
    {
        Write-Host "[Error] '${nasmDir}' is missing" -ForegroundColor Red
        exit
    }

    # Check Visual Studio is installed or not
    $visualStudioShells = @(
        "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
        "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat"
        "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
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

    $env:PATH="${perlDir};${nasmDir};${env:PATH}"

    Call("${visualStudioShell}")

    # shared: Build a shared object in addition to the static archive. You probably need a RPATH when enabling shared to ensure openssl uses the correct libssl and libcrypto after installation.
    # no-shared: Disables shared objects (only a static library is created)
    # no-asm: Disables assembly language routines (and uses C routines)
    Push-Location $sourceDir
    if ($Configuration -eq "Debug")
    {
        perl Configure VC-WIN64A --prefix="${installDir}" no-asm no-shared -d
    }
    else
    {
        perl Configure VC-WIN64A --prefix="${installDir}" no-asm no-shared
    }
    nmake
    nmake test
    nmake install
    Pop-Location
}
elseif ($global:IsMacOS)
{
    Push-Location $sourceDir
    $architecture = [System.Runtime.InteropServices.RuntimeInformation]::OSArchitecture
    if ($architecture -eq "arm64")
    {
        if ($Configuration -eq "Debug")
        {
            perl Configure darwin64-arm64-cc --prefix="${installDir}" no-asm no-shared -d
        }
        else
        {
            perl Configure darwin64-arm64-cc --prefix="${installDir}" no-asm no-shared
        }
    }
    else
    {
        if ($Configuration -eq "Debug")
        {
            perl Configure darwin64-x86_64-cc --prefix="${installDir}" no-asm no-shared -d
        }
        else
        {
            perl Configure darwin64-x86_64-cc --prefix="${installDir}" no-asm no-shared
        }
    }
    make
    make install
    Pop-Location
}
elseif ($global:IsLinux)
{
    Push-Location $sourceDir
    if ($Configuration -eq "Debug")
    {
        ./Configure --prefix="${installDir}" no-asm no-shared -d
    }
    else
    {
        ./Configure --prefix="${installDir}" no-asm no-shared
    }
    make
    make install
    Pop-Location
}