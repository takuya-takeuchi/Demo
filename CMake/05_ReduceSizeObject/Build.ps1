$current = $PSSCriptRoot
$opencvDir = Join-Path ${current} opencv
$buildDir = Join-Path ${current} build

git submodule update --init --recursive

if ($IsWindows)
{
      $buildDir = Join-Path ${buildDir} windows
      $installDir = Join-Path ${opencvDir} install | `
                    Join-Path -ChildPath windows
      New-Item -Type Directory ${buildDir} -Force | Out-Null
      Set-Location ${buildDir}
      $installOpenCVDir = Join-Path ${opencvDir} install | `
                          Join-Path -ChildPath windows | `
                          Join-Path -ChildPath x64 | `
                          Join-Path -ChildPath vc16 | `
                          Join-Path -ChildPath staticlib
      $env:OpenCV_DIR = $installOpenCVDir

      Write-Host "OpenCV_DIR: ${env:OpenCV_DIR}" -ForegroundColor Blue
      cmake -G "Visual Studio 16 2019" -A x64 -T host=x64 `
            -D OpenCV_DIR=`"${installOpenCVDir}`" `
            ${current}
      cmake --build . --config Release
}
elseif ($IsLinux)
{
      $buildDir = Join-Path ${buildDir} linux
      $installDir = Join-Path ${opencvDir} install | `
                    Join-Path -ChildPath linux
      New-Item -Type Directory ${buildDir} -Force | Out-Null
      Set-Location ${buildDir}

      cmake -D BUILD_SHARED_LIBS=OFF `
            -D CMAKE_INSTALL_PREFIX="${installDir}" `
            -D BUILD_opencv_world=ON `
            ${opencvDir}
      cmake --build . --config Release --target install
}
elseif ($IsMacOS)
{
      $buildDir = Join-Path ${buildDir} osx
      $installDir = Join-Path ${opencvDir} install | `
                    Join-Path -ChildPath osx
      New-Item -Type Directory ${buildDir} -Force | Out-Null
      Set-Location ${buildDir}

      cmake -D BUILD_SHARED_LIBS=OFF `
            -D CMAKE_INSTALL_PREFIX="${installDir}" `
            -D BUILD_opencv_world=ON `
            ${opencvDir}
      cmake --build . --config Release --target install
}

Set-Location $current