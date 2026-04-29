$current = $PSScriptRoot

$configuration = "Release"

# get os name
if ($global:IsWindows)
{
    $os = "win"
}
else
{
    Write-Host "This platfor is not supported"
}

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

$targets = @(
    "add_compile_options"
    "CMAKE_CXX_FLAGS"
    "MSVC_RUNTIME_LIBRARY"
    "CMAKE_MSVC_RUNTIME_LIBRARY"
)

foreach ($target in $targets)
{
    $sourceDir = Join-Path $current $target
    $buildDir = Join-Path $current build | `
                Join-Path -ChildPath $os | `
                Join-Path -ChildPath program | `
                Join-Path -ChildPath $target
    $installDir = Join-Path $current install | `
                  Join-Path -ChildPath $os | `
                  Join-Path -ChildPath program

    New-Item -Type Directory $buildDir -Force | Out-Null
    New-Item -Type Directory $installDir -Force | Out-Null

    $cmakeArgs = @(
        "-G", "Visual Studio 17 2022", "-A", "x64", "-T", "host=x64"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
    )

    if ($target -eq "CMAKE_MSVC_RUNTIME_LIBRARY")
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>"
        # $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL"
        $cmakeArgs += @(
            "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
        )
    }

    $cmakeArgs += @(
        "-B ${buildDir}"
        "-S ${sourceDir}"
    )

    $configLogFile = Join-Path $buildDir cmake-config.log
    $buildLogFile = Join-Path $buildDir cmake-build.log

    cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
    cmake --build ${buildDir} --config ${Configuration} --target install 2>&1 | Tee-Object -FilePath $buildLogFile
}

foreach ($target in $targets)
{
    $buildDir = Join-Path $current build | `
                Join-Path -ChildPath $os | `
                Join-Path -ChildPath program | `
                Join-Path -ChildPath $target
    $vcxprojFiles = Get-ChildItem -Recurse -Include "Demo*.vcxproj" $buildDir
    foreach ($vcxproj in $vcxprojFiles)
    {
        Write-Host $vcxproj -ForegroundColor Blue
        $xml = Select-Xml -Path $vcxproj -XPath "//ItemDefinitionGroup/ClCompile/RuntimeLibrary"
        Select-String -Path $vcxproj -Pattern "<RuntimeLibrary>"
    }
}