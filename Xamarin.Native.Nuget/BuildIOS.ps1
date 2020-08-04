$curret = Get-Location

Write-Host "Build iOS Native Library" -Foreground Green
$buildDir = Join-Path Native build_ios
New-Item -ItemType Directory -Force $buildDir | Out-Null
Set-Location $buildDir
cmake -G Xcode `
      -D CMAKE_TOOLCHAIN_FILE=../../ios-cmake/ios.toolchain.cmake `
      -D CMAKE_SYSTEM_NAME=iOS `
      -D CMAKE_OSX_ARCHITECTURES="arm64;x86_64" `
      -D CMAKE_IOS_INSTALL_COMBINED=OS64COMBINED `
      ..

Write-Host "Build iOS Managed Library" -Foreground Green
Set-Location $curret
Set-Location NativeSharp
dotnet build -c "Release_Xamarin.iOS"

Set-Location $curret