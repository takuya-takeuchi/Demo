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
}
elseif ($global:IsMacOS)
{
    $os = "osx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
}

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-PathArray -PathElements @($current, "build", $os, $target, $version, $shared, $Configuration)
$installDir = Join-PathArray -PathElements @($current, "install", $os, $target, $version, $shared, $Configuration)

$llvmVersion = $config.llvm.version
$CMAKE_C_COMPILER = Join-PathArray -PathElements @($current, "install", $os, "llvm", $llvmVersion, "bin", "clang")
$CMAKE_CXX_COMPILER = Join-PathArray -PathElements @($current, "install", $os, "llvm", $llvmVersion, "bin", "clang++")

$paths = @(
    "${CMAKE_C_COMPILER}"
    "${CMAKE_CXX_COMPILER}"
)
foreach ($path in $paths)
{
    if (!(Test-Path(${path})))
    {
        Write-Host "[Error] ${path} is missing" -ForegroundColor Red
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
        $vs = "C:\Program Files\Microsoft Visual Studio\2022"
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

    $cmakeArgs += @(
        "-G", "Visual Studio 17 2022", "-A", "x64", "-T", "host=x64"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
        "-D PostgreSQL_ROOT=$libpqInstallDir"
    )
}
elseif ($global:IsMacOS)
{
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D PostgreSQL_ROOT=$libpqInstallDir"
    )
}
elseif ($global:IsLinux)
{
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}",
        "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
    )
}

$cmakeArgs += @(
    "-B ${buildDir}"
    "${sourceDir}"
)

$configLogFile = Join-PathArray -PathElements @($buildDir, "cmake-config.log")
$buildLogFile = Join-PathArray -PathElements @($buildDir, "cmake-build.log")

# $env:PKG_CONFIG_PATH = "${pkgConfigPath}:/usr/local/lib/pkgconfig"
cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
$nproc = [Environment]::ProcessorCount
cmake --build "${buildDir}" --config ${Configuration} --target install --parallel $nproc 2>&1 | Tee-Object -FilePath $buildLogFile