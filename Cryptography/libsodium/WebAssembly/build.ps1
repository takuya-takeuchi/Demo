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

$target = "libsodium"
$sharedFlag = "OFF"

$emsdkVersions = @(
    "3.1.57"
)

git submodule update --init --recursive .

foreach ($emSdkVersion in $emsdkVersions)
{
    # build
    $sourceDir = Join-Path $current $target
    $buildDirName = "build"
    $buildDir = Join-Path $current ${buildDirName} | `
                Join-Path -ChildPath $emSdkVersion | `
                Join-Path -ChildPath $os | `
                Join-Path -ChildPath $target
    $installDirName = "install"
    $installDir = Join-Path $current ${installDirName} | `
                  Join-Path -ChildPath $emSdkVersion | `
                  Join-Path -ChildPath $os | `
                  Join-Path -ChildPath $target
    $targetDir = Join-Path $installDir $target | `
                 Join-Path -ChildPath lib | `
                 Join-Path -ChildPath cmake
    
    New-Item -Type Directory $buildDir -Force | Out-Null
    New-Item -Type Directory $installDir -Force | Out-Null 

    $dockerInstallDir = Join-Path ${installDirName} $emSdkVersion | `
                        Join-Path -ChildPath $os | `
                        Join-Path -ChildPath $target
    
    if ($global:IsWindows)
    {
        $dockerInstallDir = $dockerInstallDir.Replace("`\", "/")
    }
    
    # build
    docker build . -t dev-libsodium-emscripten:${emSdkVersion} --build-arg EMSDK_VERSION=${emSdkVersion}
    $buildCommand = "./configure && ./dist-build/emscripten.sh --standard"
    $copyCommand = "cp -Rf ${target}-js/* /project/${dockerInstallDir}"
    docker run --rm --workdir /project/${target} -v "${current}:/project" "dev-libsodium-emscripten:${emSdkVersion}" sh -c "${buildCommand} && ${copyCommand}"

    # # copy opencv.js
    # $dstDir = Join-Path $installDir bin
    # $srcDir = Join-Path $buildDir bin
    # New-Item -Type Directory $dstDir -Force | Out-Null
    # Copy-Item (Join-Path $srcDir "*") $dstDir -Force -Recurse | Out-Null
}