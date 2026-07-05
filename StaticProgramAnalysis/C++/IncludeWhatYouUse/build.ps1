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
$configPath = Join-Path $current "build-config.json"
if (!(Test-Path($configPath)))
{
    Write-Host "${configPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $configPath | ConvertFrom-Json
$target = "include-what-you-use"
$version = $config."${target}".version
$version = $config."${target}".mapping."${version}"
if ($config."${target}".shared)
{
    $shared = "dynamic"
    $sharedFlag = "ON"
}
else
{
    $shared = "static"
    $sharedFlag = "OFF"
}

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
$sourceDir = Join-Path $current $target
$buildDir = Join-PathArray -PathElements @($current, "build", $os, $target, $version, $shared, $Configuration)
$installDir = Join-PathArray -PathElements @($current, "install", $os, $target, $version, $shared, $Configuration)

$llvmVersion = $config.llvm.version
$LLVM_INSTALL_DIR = Join-PathArray -PathElements @($current, "install", $os, "llvm", $llvmVersion)
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

# reset submodules
Push-Location $current
git submodule update --init --recursive .
Pop-Location

Push-Location $sourceDir
git fetch --all --prune
git checkout $version
git submodule update --init --recursive .
Pop-Location

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

    $vsVersion = $config.windows.visualStudioVersion
    $vsInternalVersion = $config.windows.visualStudioInternalVersion

    $cmakeArgs += @(
        "-G", "Visual Studio ${vsInternalVersion} ${vsVersion}", "-A", "x64", "-T", "host=x64"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_PREFIX_PATH=$LLVM_INSTALL_DIR"
        "-D CMAKE_C_COMPILER=${CMAKE_C_COMPILER}",
        "-D CMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
    )
}
elseif ($global:IsMacOS)
{
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_PREFIX_PATH=$LLVM_INSTALL_DIR"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D CMAKE_C_COMPILER=${CMAKE_C_COMPILER}",
        "-D CMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
    )
}
elseif ($global:IsLinux)
{
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_PREFIX_PATH=$LLVM_INSTALL_DIR"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D CMAKE_C_COMPILER=${CMAKE_C_COMPILER}",
        "-D CMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
    )
}

$cmakeArgs += @(
    "-B ${buildDir}"
    "${sourceDir}"
)

$configLogFile = Join-PathArray -PathElements @($buildDir, "cmake-config.log")
$buildLogFile = Join-PathArray -PathElements @($buildDir, "cmake-build.log")

cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile

# https://github.com/include-what-you-use/include-what-you-use/issues/684
# For windows, porject files ignore location of Visual Studio
if ($global:IsWindows)
{
    $vsVersion = $config.windows.visualStudioVersion
    $orgVs = "C:\Program Files\Microsoft Visual Studio\2022\Enterprise"
    $vs = "C:\Program Files\Microsoft Visual Studio\${vsVersion}\Enterprise"
    if (!(Test-Path($vs)))
    {
        $vs = "C:\Program Files\Microsoft Visual Studio\${vsVersionn}\Professional"
    }
    if (!(Test-Path($vs)))
    {
        $vs = "C:\Program Files\Microsoft Visual Studio\${vsVersion}\Community"
    }

    $projects = Get-ChildItem -Path "${buildDir}" -Filter "*.vcxproj" -Recurse -File
    foreach ($project in $projects)
    {
        if (Test-Path(${project}))
        {
            (Get-Content "${project}").Replace("${orgVs}",
                                               "${vs}") | Set-Content "${project}"
        }
    }
}

$nproc = [Environment]::ProcessorCount
cmake --build "${buildDir}" --config ${Configuration} --target install --parallel $nproc 2>&1 | Tee-Object -FilePath $buildLogFile