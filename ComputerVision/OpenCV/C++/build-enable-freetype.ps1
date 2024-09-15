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
else
{
    Write-Host "[Error] This platform does not support CUDA" -ForegroundColor Red
    exit
}

$target = "opencv4"
$shared = "static"
$sharedFlag = "OFF"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build-enable-freetype | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $shared
$installDir = Join-Path $current install-enable-freetype | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $shared
$targetDir = Join-Path $installDir $target | `
             Join-Path -ChildPath lib | `
             Join-Path -ChildPath cmake
$contribDir = Join-Path $current opencv_contrib | `
              Join-Path -ChildPath modules

$freetype_installDir = Join-Path $current install | `
                       Join-Path -ChildPath $os | `
                       Join-Path -ChildPath freetype | `
                       Join-Path -ChildPath $shared
$freetype_include_dir = Join-Path $freetype_installDir include
$freetype_lib_dir = Join-Path $freetype_installDir lib

$harfbuzz_installDir = Join-Path $current install | `
                       Join-Path -ChildPath $os | `
                       Join-Path -ChildPath harfbuzz | `
                       Join-Path -ChildPath $shared
$harfbuzz_include_dir = Join-Path $harfbuzz_installDir include
$harfbuzz_lib_dir = Join-Path $harfbuzz_installDir lib

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    # CMAKE_PREFIX_PATH does not work for harfbuzz and freetype
    # So specify each argument
    cmake -D BUILD_SHARED_LIBS=OFF `
          -D BUILD_WITH_STATIC_CRT=ON `
          -D CMAKE_INSTALL_PREFIX="${installDir}" `
          -D HARFBUZZ_FOUND:BOOL=ON `
          -D HARFBUZZ_INCLUDE_DIRS:PATH="${harfbuzz_include_dir}/harfbuzz" `
          -D HARFBUZZ_LIBRARIES:PATH="${harfbuzz_lib_dir}/harfbuzz.lib" `
          -D HARFBUZZ_LIBRARY:FILEPATH="${harfbuzz_lib_dir}/harfbuzz.lib" `
          -D FREETYPE_FOUND:BOOL=ON `
          -D FREETYPE_INCLUDE_DIRS:PATH="${freetype_include_dir}/freetype2" `
          -D FREETYPE_LIBRARY_DEBUG:FILEPATH="${freetype_lib_dir}/freetyped.lib" `
          -D FREETYPE_LIBRARY_RELEASE:FILEPATH="${freetype_lib_dir}/freetype.lib" `
          -D OPENCV_EXTRA_MODULES_PATH="${contribDir}" `
          -D BUILD_opencv_world=OFF `
          -D BUILD_opencv_java=OFF `
          -D BUILD_opencv_python=OFF `
          -D BUILD_opencv_python2=OFF `
          -D BUILD_opencv_python3=OFF `
          -D BUILD_PERF_TESTS=OFF `
          -D BUILD_TESTS=OFF `
          -D BUILD_DOCS=OFF `
          -D BUILD_opencv_core=ON `
          -D BUILD_opencv_highgui=OFF `
          -D BUILD_opencv_imgcodecs=ON `
          -D BUILD_opencv_imgproc=ON `
          -D BUILD_opencv_calib3d=OFF `
          -D BUILD_opencv_features2d=OFF `
          -D BUILD_opencv_flann=OFF `
          -D BUILD_opencv_java_bindings_generator=OFF `
          -D BUILD_opencv_ml=OFF `
          -D BUILD_opencv_objdetect=OFF `
          -D BUILD_opencv_photo=OFF `
          -D BUILD_opencv_python_bindings_generator=OFF `
          -D BUILD_opencv_shape=OFF `
          -D BUILD_opencv_stitching=OFF `
          -D BUILD_opencv_superres=OFF `
          -D BUILD_opencv_video=OFF `
          -D BUILD_opencv_videoio=OFF `
          -D BUILD_opencv_videostab=OFF `
          -D BUILD_opencv_alphamat=OFF `
          -D BUILD_opencv_aruco=OFF `
          -D BUILD_opencv_barcode=OFF `
          -D BUILD_opencv_bgsegm=OFF `
          -D BUILD_opencv_bioinspired=OFF `
          -D BUILD_opencv_ccalib=OFF `
          -D BUILD_opencv_cnn_3dobj=OFF `
          -D BUILD_opencv_cudaarithm=OFF `
          -D BUILD_opencv_cudabgsegm=OFF `
          -D BUILD_opencv_cudacodec=OFF `
          -D BUILD_opencv_cudafeatures2d=OFF `
          -D BUILD_opencv_cudafilters=OFF `
          -D BUILD_opencv_cudaimgproc=OFF `
          -D BUILD_opencv_cudalegacy=OFF `
          -D BUILD_opencv_cudaobjdetect=OFF `
          -D BUILD_opencv_cudaoptflow=OFF `
          -D BUILD_opencv_cudastereo=OFF `
          -D BUILD_opencv_cudawarping=OFF `
          -D BUILD_opencv_cudev=OFF `
          -D BUILD_opencv_cvv=OFF `
          -D BUILD_opencv_datasets=OFF `
          -D BUILD_opencv_dnn_objdetect=OFF `
          -D BUILD_opencv_dnn_superres=OFF `
          -D BUILD_opencv_dnns_easily_fooled=OFF `
          -D BUILD_opencv_dpm=OFF `
          -D BUILD_opencv_face=OFF `
          -D BUILD_opencv_freetype=ON `
          -D BUILD_opencv_fuzzy=OFF `
          -D BUILD_opencv_hdf=OFF `
          -D BUILD_opencv_hfs=OFF `
          -D BUILD_opencv_img_hash=OFF `
          -D BUILD_opencv_intensity_transform=OFF `
          -D BUILD_opencv_julia=OFF `
          -D BUILD_opencv_line_descriptor=OFF `
          -D BUILD_opencv_matlab=OFF `
          -D BUILD_opencv_mcc=OFF `
          -D BUILD_opencv_optflow=OFF `
          -D BUILD_opencv_ovis=OFF `
          -D BUILD_opencv_phase_unwrapping=OFF `
          -D BUILD_opencv_plot=OFF `
          -D BUILD_opencv_quality=OFF `
          -D BUILD_opencv_rapid=OFF `
          -D BUILD_opencv_reg=OFF `
          -D BUILD_opencv_rgbd=OFF `
          -D BUILD_opencv_saliency=OFF `
          -D BUILD_opencv_sfm=OFF `
          -D BUILD_opencv_shape=OFF `
          -D BUILD_opencv_stereo=OFF `
          -D BUILD_opencv_structured_light=OFF `
          -D BUILD_opencv_superres=OFF `
          -D BUILD_opencv_surface_matching=OFF `
          -D BUILD_opencv_text=OFF `
          -D BUILD_opencv_tracking=OFF `
          -D BUILD_opencv_videostab=OFF `
          -D BUILD_opencv_viz=OFF `
          -D BUILD_opencv_wechat_qrcode=OFF `
          -D BUILD_opencv_xfeatures2d=OFF `
          -D BUILD_opencv_ximgproc=OFF `
          -D BUILD_opencv_xobjdetect=OFF `
          -D BUILD_opencv_xphoto=OFF `
          -D BUILD_JASPER=OFF `
          -D BUILD_OPENEXR=OFF `
          -D BUILD_OPENJPEG=OFF `
          -D BUILD_PROTOBUF=OFF `
          -D BUILD_TIFF=OFF `
          -D BUILD_WEBP=OFF `
          -D WITH_ADE=OFF `
          -D WITH_CUDA=OFF `
          -D WITH_EIGEN=OFF `
          -D WITH_FFMPEG=OFF `
          -D WITH_IPP=ON `
          -D WITH_JASPER=OFF `
          -D WITH_OPENEXR=OFF `
          -D WITH_OPENJPEG=OFF `
          -D WITH_PROTOBUF=OFF `
          -D WITH_QUIRC=OFF `
          -D WITH_TIFF=OFF `
          -D WITH_WEBP=OFF `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D BUILD_SHARED_LIBS=${sharedFlag} `
          -D CMAKE_INSTALL_PREFIX="${installDir}" `
          -D CMAKE_PREFIX_PATH="${freetype_installDir};${harfbuzz_installDir}" `
          -D OPENCV_EXTRA_MODULES_PATH="${contribDir}" `
          -D BUILD_opencv_world=OFF `
          -D BUILD_opencv_java=OFF `
          -D BUILD_opencv_python=OFF `
          -D BUILD_opencv_python2=OFF `
          -D BUILD_opencv_python3=OFF `
          -D BUILD_PERF_TESTS=OFF `
          -D BUILD_TESTS=OFF `
          -D BUILD_DOCS=OFF `
          -D BUILD_opencv_core=ON `
          -D BUILD_opencv_highgui=OFF `
          -D BUILD_opencv_imgcodecs=ON `
          -D BUILD_opencv_imgproc=ON `
          -D BUILD_opencv_calib3d=OFF `
          -D BUILD_opencv_features2d=OFF `
          -D BUILD_opencv_flann=OFF `
          -D BUILD_opencv_java_bindings_generator=OFF `
          -D BUILD_opencv_ml=OFF `
          -D BUILD_opencv_objdetect=OFF `
          -D BUILD_opencv_photo=OFF `
          -D BUILD_opencv_python_bindings_generator=OFF `
          -D BUILD_opencv_shape=OFF `
          -D BUILD_opencv_stitching=OFF `
          -D BUILD_opencv_superres=OFF `
          -D BUILD_opencv_video=OFF `
          -D BUILD_opencv_videoio=OFF `
          -D BUILD_opencv_videostab=OFF `
          -D BUILD_opencv_alphamat=OFF `
          -D BUILD_opencv_aruco=OFF `
          -D BUILD_opencv_barcode=OFF `
          -D BUILD_opencv_bgsegm=OFF `
          -D BUILD_opencv_bioinspired=OFF `
          -D BUILD_opencv_ccalib=OFF `
          -D BUILD_opencv_cnn_3dobj=OFF `
          -D BUILD_opencv_cudaarithm=OFF `
          -D BUILD_opencv_cudabgsegm=OFF `
          -D BUILD_opencv_cudacodec=OFF `
          -D BUILD_opencv_cudafeatures2d=OFF `
          -D BUILD_opencv_cudafilters=OFF `
          -D BUILD_opencv_cudaimgproc=OFF `
          -D BUILD_opencv_cudalegacy=OFF `
          -D BUILD_opencv_cudaobjdetect=OFF `
          -D BUILD_opencv_cudaoptflow=OFF `
          -D BUILD_opencv_cudastereo=OFF `
          -D BUILD_opencv_cudawarping=OFF `
          -D BUILD_opencv_cudev=OFF `
          -D BUILD_opencv_cvv=OFF `
          -D BUILD_opencv_datasets=OFF `
          -D BUILD_opencv_dnn_objdetect=OFF `
          -D BUILD_opencv_dnn_superres=OFF `
          -D BUILD_opencv_dnns_easily_fooled=OFF `
          -D BUILD_opencv_dpm=OFF `
          -D BUILD_opencv_face=OFF `
          -D BUILD_opencv_freetype=ON `
          -D BUILD_opencv_fuzzy=OFF `
          -D BUILD_opencv_hdf=OFF `
          -D BUILD_opencv_hfs=OFF `
          -D BUILD_opencv_img_hash=OFF `
          -D BUILD_opencv_intensity_transform=OFF `
          -D BUILD_opencv_julia=OFF `
          -D BUILD_opencv_line_descriptor=OFF `
          -D BUILD_opencv_matlab=OFF `
          -D BUILD_opencv_mcc=OFF `
          -D BUILD_opencv_optflow=OFF `
          -D BUILD_opencv_ovis=OFF `
          -D BUILD_opencv_phase_unwrapping=OFF `
          -D BUILD_opencv_plot=OFF `
          -D BUILD_opencv_quality=OFF `
          -D BUILD_opencv_rapid=OFF `
          -D BUILD_opencv_reg=OFF `
          -D BUILD_opencv_rgbd=OFF `
          -D BUILD_opencv_saliency=OFF `
          -D BUILD_opencv_sfm=OFF `
          -D BUILD_opencv_shape=OFF `
          -D BUILD_opencv_stereo=OFF `
          -D BUILD_opencv_structured_light=OFF `
          -D BUILD_opencv_superres=OFF `
          -D BUILD_opencv_surface_matching=OFF `
          -D BUILD_opencv_text=OFF `
          -D BUILD_opencv_tracking=OFF `
          -D BUILD_opencv_videostab=OFF `
          -D BUILD_opencv_viz=OFF `
          -D BUILD_opencv_wechat_qrcode=OFF `
          -D BUILD_opencv_xfeatures2d=OFF `
          -D BUILD_opencv_ximgproc=OFF `
          -D BUILD_opencv_xobjdetect=OFF `
          -D BUILD_opencv_xphoto=OFF `
          -D BUILD_JASPER=OFF `
          -D BUILD_OPENEXR=OFF `
          -D BUILD_OPENJPEG=OFF `
          -D BUILD_PROTOBUF=OFF `
          -D BUILD_TIFF=OFF `
          -D BUILD_WEBP=OFF `
          -D WITH_ADE=OFF `
          -D WITH_CUDA=OFF `
          -D WITH_EIGEN=OFF `
          -D WITH_FFMPEG=OFF `
          -D WITH_IPP=ON `
          -D WITH_JASPER=OFF `
          -D WITH_OPENEXR=OFF `
          -D WITH_OPENJPEG=OFF `
          -D WITH_PROTOBUF=OFF `
          -D WITH_QUIRC=OFF `
          -D WITH_TIFF=OFF `
          -D WITH_WEBP=OFF `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D BUILD_SHARED_LIBS=${sharedFlag} `
          -D CMAKE_INSTALL_PREFIX="${installDir}" `
          -D CMAKE_PREFIX_PATH="${freetype_installDir};${harfbuzz_installDir}" `
          -D OPENCV_EXTRA_MODULES_PATH="${contribDir}" `
          -D BUILD_opencv_world=OFF `
          -D BUILD_opencv_java=OFF `
          -D BUILD_opencv_python=OFF `
          -D BUILD_opencv_python2=OFF `
          -D BUILD_opencv_python3=OFF `
          -D BUILD_PERF_TESTS=OFF `
          -D BUILD_TESTS=OFF `
          -D BUILD_DOCS=OFF `
          -D BUILD_opencv_core=ON `
          -D BUILD_opencv_highgui=OFF `
          -D BUILD_opencv_imgcodecs=ON `
          -D BUILD_opencv_imgproc=ON `
          -D BUILD_opencv_calib3d=OFF `
          -D BUILD_opencv_features2d=OFF `
          -D BUILD_opencv_flann=OFF `
          -D BUILD_opencv_java_bindings_generator=OFF `
          -D BUILD_opencv_ml=OFF `
          -D BUILD_opencv_objdetect=OFF `
          -D BUILD_opencv_photo=OFF `
          -D BUILD_opencv_python_bindings_generator=OFF `
          -D BUILD_opencv_shape=OFF `
          -D BUILD_opencv_stitching=OFF `
          -D BUILD_opencv_superres=OFF `
          -D BUILD_opencv_video=OFF `
          -D BUILD_opencv_videoio=OFF `
          -D BUILD_opencv_videostab=OFF `
          -D BUILD_opencv_alphamat=OFF `
          -D BUILD_opencv_aruco=OFF `
          -D BUILD_opencv_barcode=OFF `
          -D BUILD_opencv_bgsegm=OFF `
          -D BUILD_opencv_bioinspired=OFF `
          -D BUILD_opencv_ccalib=OFF `
          -D BUILD_opencv_cnn_3dobj=OFF `
          -D BUILD_opencv_cudaarithm=OFF `
          -D BUILD_opencv_cudabgsegm=OFF `
          -D BUILD_opencv_cudacodec=OFF `
          -D BUILD_opencv_cudafeatures2d=OFF `
          -D BUILD_opencv_cudafilters=OFF `
          -D BUILD_opencv_cudaimgproc=OFF `
          -D BUILD_opencv_cudalegacy=OFF `
          -D BUILD_opencv_cudaobjdetect=OFF `
          -D BUILD_opencv_cudaoptflow=OFF `
          -D BUILD_opencv_cudastereo=OFF `
          -D BUILD_opencv_cudawarping=OFF `
          -D BUILD_opencv_cudev=OFF `
          -D BUILD_opencv_cvv=OFF `
          -D BUILD_opencv_datasets=OFF `
          -D BUILD_opencv_dnn_objdetect=OFF `
          -D BUILD_opencv_dnn_superres=OFF `
          -D BUILD_opencv_dnns_easily_fooled=OFF `
          -D BUILD_opencv_dpm=OFF `
          -D BUILD_opencv_face=OFF `
          -D BUILD_opencv_freetype=ON `
          -D BUILD_opencv_fuzzy=OFF `
          -D BUILD_opencv_hdf=OFF `
          -D BUILD_opencv_hfs=OFF `
          -D BUILD_opencv_img_hash=OFF `
          -D BUILD_opencv_intensity_transform=OFF `
          -D BUILD_opencv_julia=OFF `
          -D BUILD_opencv_line_descriptor=OFF `
          -D BUILD_opencv_matlab=OFF `
          -D BUILD_opencv_mcc=OFF `
          -D BUILD_opencv_optflow=OFF `
          -D BUILD_opencv_ovis=OFF `
          -D BUILD_opencv_phase_unwrapping=OFF `
          -D BUILD_opencv_plot=OFF `
          -D BUILD_opencv_quality=OFF `
          -D BUILD_opencv_rapid=OFF `
          -D BUILD_opencv_reg=OFF `
          -D BUILD_opencv_rgbd=OFF `
          -D BUILD_opencv_saliency=OFF `
          -D BUILD_opencv_sfm=OFF `
          -D BUILD_opencv_shape=OFF `
          -D BUILD_opencv_stereo=OFF `
          -D BUILD_opencv_structured_light=OFF `
          -D BUILD_opencv_superres=OFF `
          -D BUILD_opencv_surface_matching=OFF `
          -D BUILD_opencv_text=OFF `
          -D BUILD_opencv_tracking=OFF `
          -D BUILD_opencv_videostab=OFF `
          -D BUILD_opencv_viz=OFF `
          -D BUILD_opencv_wechat_qrcode=OFF `
          -D BUILD_opencv_xfeatures2d=OFF `
          -D BUILD_opencv_ximgproc=OFF `
          -D BUILD_opencv_xobjdetect=OFF `
          -D BUILD_opencv_xphoto=OFF `
          -D BUILD_JASPER=OFF `
          -D BUILD_OPENEXR=OFF `
          -D BUILD_OPENJPEG=OFF `
          -D BUILD_PROTOBUF=OFF `
          -D BUILD_TIFF=OFF `
          -D BUILD_WEBP=OFF `
          -D WITH_ADE=OFF `
          -D WITH_CUDA=OFF `
          -D WITH_EIGEN=OFF `
          -D WITH_FFMPEG=OFF `
          -D WITH_IPP=ON `
          -D WITH_JASPER=OFF `
          -D WITH_OPENEXR=OFF `
          -D WITH_OPENJPEG=OFF `
          -D WITH_PROTOBUF=OFF `
          -D WITH_QUIRC=OFF `
          -D WITH_TIFF=OFF `
          -D WITH_WEBP=OFF `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location