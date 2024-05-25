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

$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
                Join-Path -ChildPath $os | `
                Join-Path -ChildPath program

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
cmake -G "Visual Studio 17 2022" -A x64 `
        -D CMAKE_INSTALL_PREFIX=${installDir} `
        $sourceDir
cmake --build . --config ${configuration} --target install
Pop-Location


$targets = @(
    "add_compile_options"
    "CMAKE_CXX_FLAGS"
    "MSVC_RUNTIME_LIBRARY"
)

foreach ($target in $targets)
{
    $buildTargetDir = Join-Path $buildDir $target
    $vcxprojFiles = Get-ChildItem -Recurse -Include "Demo*.vcxproj" $buildTargetDir
    foreach ($vcxproj in $vcxprojFiles)
    {
        Write-Host $vcxproj -ForegroundColor Blue
        $xml = Select-Xml -Path $vcxproj -XPath "//ItemDefinitionGroup/ClCompile/RuntimeLibrary"
        Select-String -Path $vcxproj -Pattern "<RuntimeLibrary>"
    }
}