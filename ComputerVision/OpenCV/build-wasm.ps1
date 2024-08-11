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

$target = "opencv4"
$shared = "static"
$sharedFlag = "OFF"

# --memory-init-file is not supported 3.1.55 or later
# This issue is fixed by https://github.com/opencv/opencv/pull/25629
# $emSdkVersion = "3.1.64"

# cv.imread is not a function
# https://github.com/opencv/opencv/issues/25057
# https://github.com/opencv/opencv/issues/21580
# https://github.com/opencv/opencv/issues/24620
# $emSdkVersion = "3.1.51"

# cv.imread is not a function
# $emSdkVersion = "3.0.1"

# https://github.com/opencv/opencv/pull/25084
# $emSdkVersion = "3.1.54"

# https://docs.opencv.org/4.x/d4/da1/tutorial_js_setup.html
# Mac is not supported
# cv.imread is not a function
# $emSdkVersion = "2.0.10"

# cv.imread is not a function
# https://stackoverflow.com/questions/67190799/how-to-include-cv-imread-when-building-opencv-js
$emSdkVersion = "1.39.15" # OK
$emsdkVersions = @(
    "1.39.15" # OK (cv or Module, either is fine)
    "1.39.16" # OK (Use Module, like Module.imdread or use alias `const cv = Module;`)
    "1.39.17" # OK (Use Module, like Module.imdread or use alias `const cv = Module;`)
    "1.39.18" # OK (Use Module, like Module.imdread or use alias `const cv = Module;`)
    "1.39.19" # OK (Use Module, like Module.imdread or use alias `const cv = Module;`)
    "1.39.20" # OK (Use Module, like Module.imdread or use alias `const cv = Module;`)
    "1.40.1"  # OK (Use Module, like Module.imdread or use alias `const cv = Module;`)
    # "2.0.34"  # OK (Use Module, like Module.imdread or use alias `const cv = Module;`)
    # "3.0.1"
)

git submodule update --init --recursive .

foreach ($emSdkVersion in $emsdkVersions)
{
    # build
    $sourceDir = Join-Path $current $target
    $buildDirName = "build-wasm"
    $buildDir = Join-Path $current ${buildDirName} | `
                Join-Path -ChildPath $emSdkVersion | `
                Join-Path -ChildPath $os | `
                Join-Path -ChildPath $target | `
                Join-Path -ChildPath $shared
    $installDirName = "install-wasm"
    $installDir = Join-Path $current ${installDirName} | `
                  Join-Path -ChildPath $emSdkVersion | `
                  Join-Path -ChildPath $os | `
                  Join-Path -ChildPath $target | `
                  Join-Path -ChildPath $shared
    $targetDir = Join-Path $installDir $target | `
                 Join-Path -ChildPath lib | `
                 Join-Path -ChildPath cmake
    
    New-Item -Type Directory $buildDir -Force | Out-Null
    New-Item -Type Directory $installDir -Force | Out-Null
    
    $buildPythonScript = Join-Path $target platform | `
                         Join-Path -ChildPath js | `
                         Join-Path -ChildPath build_js.py    
        
    $dockerBuildDir = Join-Path ${buildDirName} $emSdkVersion | `
                      Join-Path -ChildPath $os | `
                      Join-Path -ChildPath $target | `
                      Join-Path -ChildPath $shared
    $dockerInstallDir = Join-Path ${installDirName} $emSdkVersion | `
                        Join-Path -ChildPath $os | `
                        Join-Path -ChildPath $target | `
                        Join-Path -ChildPath $shared
    
    if ($global:IsWindows)
    {
        $dockerBuildDir = $dockerBuildDir.Replace("`\", "/")
        $dockerInstallDir = $dockerInstallDir.Replace("`\", "/")
    }
    
    # build
    $buildCommand = "emcmake python3 ./platforms/js/build_js.py --build_perf /project/${dockerBuildDir}"
    $installCommand = "cd /project/${dockerBuildDir} && make -j install"
    $copyCommand = "cp -Rf /usr/local/* /project/${dockerInstallDir}"
    docker run --rm --workdir /project/${target} -v "${current}:/project" "emscripten/emsdk:${emSdkVersion}" sh -c "${buildCommand} && ${installCommand} && ${copyCommand}"

    # copy opencv.js
    $dstDir = Join-Path $installDir bin
    $srcDir = Join-Path $buildDir bin
    New-Item -Type Directory $dstDir -Force | Out-Null
    Copy-Item (Join-Path $srcDir "*") $dstDir -Force -Recurse | Out-Null
}