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

function Join-PathArray {
    [CmdletBinding()]
    param (
        [Parameter(Mandatory = $true, ValueFromPipeline = $true)]
        [string[]]$PathElements
    )

    process {
        if ($PathElements.Count -eq 0) { return }
        $result = $PathElements[0]
        for ($i = 1; $i -lt $PathElements.Count; $i++) { $result = Join-Path -Path $result -ChildPath $PathElements[$i] }
        return $result
    }
}


$current = $PSScriptRoot
$rootDir = Split-Path $current -Parent
$configPath = Join-Path $rootDir "build-config.json"
if (!(Test-Path($configPath)))
{
    Write-Host "${configPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $configPath | ConvertFrom-Json

# get os name
if ($global:IsWindows)
{
    $os = "win"
    $clang = "clang.exe"
    $clangcxx = "clang++.exe"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
    $clang = "clang"
    $clangcxx = "clang++"
}
elseif ($global:IsLinux)
{
    $os = "linux"
    $clang = "clang"
    $clangcxx = "clang++"
}

# build
$sourceDir = $current
$buildDir = Join-PathArray -PathElements @($current, "build", $os, "program", $Configuration)
$installDir = Join-PathArray -PathElements @($current, "install", $os)

$llvmVersion = $config.llvm.version
$LLVM_INSTALL_DIR = Join-PathArray -PathElements @($rootDir, "install", $os, "llvm", $llvmVersion)
$CMAKE_C_COMPILER = Get-ChildItem -Path "${LLVM_INSTALL_DIR}" -Filter "${clang}" -Recurse -File
$CMAKE_CXX_COMPILER = Get-ChildItem -Path "${LLVM_INSTALL_DIR}" -Filter "${clangcxx}" -Recurse -File

$paths = @{
    "CMAKE_C_COMPILER"  = "${CMAKE_C_COMPILER}"
    "CMAKE_CXX_COMPILER" = "${CMAKE_CXX_COMPILER}"
}

foreach ($key in $paths.Keys)
{
    $targetPath = $paths[$key]
    if (!(Test-Path(${targetPath})))
    {
        Write-Host "[Error] ${key}: ${targetPath} is missing" -ForegroundColor Red
        return
    }
}

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

$cmakeArgs = @()
if ($global:IsWindows)
{
    function CallVisualStudioDeveloperConsole()
    {
        $vsVersion = $config.windows.visualStudioVersion
        $vs = "C:\Program Files\Microsoft Visual Studio\${vsVersion}"
        $path = "${vs}\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
        if (!(Test-Path($path)))
        {
            $path = "${vs}\Professional\VC\Auxiliary\Build\vcvars64.bat"
        }
        if (!(Test-Path($path)))
        {
            $path = "${vs}\Community\VC\Auxiliary\Build\vcvars64.bat"
        }

        Write-Host "Use: ${path}" -ForegroundColor Green

        cmd.exe /c "call `"${path}`" && set > %temp%\vcvars.txt"
        Get-Content "${env:temp}\vcvars.txt" | Foreach-Object {
            if ($_ -match "^(.*?)=(.*)$") {
                Set-Content "env:\$($matches[1])" $matches[2]
            }
        }
    }
    CallVisualStudioDeveloperConsole
    chcp 65001

    if ($config.windows.msvcStaticRuntime)
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    }
    else
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL"
    }

    # CMAKE_EXPORT_COMPILE_COMMANDS is not supported by Microsoft Visual C++ Generator, so we use Ninja generator instead.
    # Need not to use clang and clang++
    # clang-tidy checks only source code and header files, so build artifacts are not required.
    $cmakeArgs += @(
        "-G", "Ninja"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
    )
}
elseif ($global:IsMacOS)
{
    # Need not to use clang and clang++
    # clang-tidy checks only source code and header files, so build artifacts are not required.
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
    )
}
elseif ($global:IsLinux)
{
    # Need not to use clang and clang++
    # clang-tidy checks only source code and header files, so build artifacts are not required.
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
    )
}

$cmakeArgs += @(
    "-D CMAKE_EXPORT_COMPILE_COMMANDS=ON"
)

$cmakeArgs += @(
    "-B ${buildDir}"
    "${sourceDir}"
)

$configLogFile = Join-PathArray -PathElements @($buildDir, "cmake-config.log")
$buildLogFile = Join-PathArray -PathElements @($buildDir, "cmake-build.log")

cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
$nproc = [Environment]::ProcessorCount
cmake --build "${buildDir}" --config ${Configuration} --target install --parallel $nproc 2>&1 | Tee-Object -FilePath $buildLogFile

$clangTidy = Join-PathArray -PathElements @($LLVM_INSTALL_DIR, "bin", "clang-tidy")
& "${clangTidy}" -checks='-*,modernize-*,google-*' -p "${buildDir}" main.cpp