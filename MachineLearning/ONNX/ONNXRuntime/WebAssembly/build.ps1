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

$target = "onnxruntime"
$configuration = "Release"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $sharedType
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $sharedType | `
              Join-Path -ChildPath $configuration
$buildLog = Join-Path $buildDir build.log

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

git submodule update --init --recursive .

Push-Location $sourceDir

Push-Location js
$jsDir = Get-Location
if ($global:IsWindows)
{
    $script = Join-Path $jsDir build_web.bat
}
else
{
    $script = Join-Path $jsDir build_web.sh
}

# & "${script}"
Pop-Location

# copy build artifacts
$types = @(
    "wasm"
    "wasm_SIMD"
    "wasm_SIMD_threaded"
    "wasm_threaded"
)

foreach ($type in $types)
{
    $src = Join-Path $sourceDir build | `
           Join-Path -ChildPath $type | `
           Join-Path -ChildPath Release | `
           Join-Path -ChildPath "*.a"
    $dst = Join-Path $installDir $type |
           Join-Path -ChildPath lib
    
    New-Item -Type Directory $dst -Force | Out-Null
    Copy-Item $src $dst -Force | Out-Null
}

$src = Join-Path $sourceDir js | `
       Join-Path -ChildPath web | `
       Join-Path -ChildPath dist
$dst = $installDir
New-Item -Type Directory $dst -Force | Out-Null
Copy-Item $src $dst -Force -Recurse | Out-Null

Pop-Location