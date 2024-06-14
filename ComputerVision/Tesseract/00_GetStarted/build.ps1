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

$target = "tesseract"

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    if (!($env:VCPKG_ROOT_DIR))
    {
        Write-Host "VCPKG_ROOT_DIR environmental variable is missing" -ForegroundColor Red
        return
    }

    $library_type = "x64-windows-static"
    $vcpkg_base_directory = "${env:VCPKG_ROOT_DIR}\installed\${library_type}"

    $libraries = @(
        "archive.lib"
        "bz2.lib"
        "gif.lib"
        "jpeg.lib"
        "leptonica-1.82.0.lib"
        "libcrypto.lib"
        "libcurl.lib"
        "libpng16.lib"
        "libsharpyuv.lib"
        "libwebp.lib"
        "libwebpdecoder.lib"
        "libwebpdemux.lib"
        "libwebpmux.lib"
        "lz4.lib"
        "lzma.lib"
        "openjp2.lib"
        "tesseract52.lib"
        "tiff.lib"
        "turbojpeg.lib"
        "zlib.lib"
        "zstd.lib"
    )

    $Tesseract_LIBRARIES = "Crypt32.lib;"
    foreach ($library in $libraries)
    {
        $Tesseract_LIBRARIES += ";"
        $path = Join-Path $vcpkg_base_directory lib | Join-Path -ChildPath $library
        $Tesseract_LIBRARIES += "`"${path}`""
    }

    $Tesseract_INCLUDE_DIR = Join-Path $vcpkg_base_directory include

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D Tesseract_LIBRARIES=${Tesseract_LIBRARIES} `
          -D Tesseract_INCLUDE_DIR=${Tesseract_INCLUDE_DIR} `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D BUILD_SHARED_LIBS=${sharedFlag} `
          -D BUILD_opencv_world=OFF `
          -D BUILD_opencv_java=OFF `
          -D BUILD_opencv_python=OFF `
          -D BUILD_opencv_python2=OFF `
          -D BUILD_opencv_python3=OFF `
          -D BUILD_PERF_TESTS=OFF `
          -D BUILD_TESTS=OFF `
          -D BUILD_DOCS=OFF `
          -D WITH_CUDA=OFF `
          -D BUILD_PROTOBUF=ON `
          -D BUILD_JPEG=ON `
          -D BUILD_PACKAGE=ON `
          -D BUILD_PNG=ON `
          -D BUILD_TIFF=ON `
          -D BUILD_ZLIB=ON `
          -D WITH_JPEG=ON `
          -D WITH_PNG=ON `
          -D WITH_TIFF=ON `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D BUILD_SHARED_LIBS=${sharedFlag} `
          -D BUILD_opencv_world=OFF `
          -D BUILD_opencv_java=OFF `
          -D BUILD_opencv_python=OFF `
          -D BUILD_opencv_python2=OFF `
          -D BUILD_opencv_python3=OFF `
          -D BUILD_PERF_TESTS=OFF `
          -D BUILD_TESTS=OFF `
          -D BUILD_DOCS=OFF `
          -D WITH_CUDA=OFF `
          -D BUILD_PROTOBUF=ON `
          -D BUILD_JPEG=ON `
          -D BUILD_PACKAGE=ON `
          -D BUILD_PNG=ON `
          -D BUILD_TIFF=ON `
          -D BUILD_ZLIB=ON `
          -D WITH_JPEG=ON `
          -D WITH_PNG=ON `
          -D WITH_TIFF=ON `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location