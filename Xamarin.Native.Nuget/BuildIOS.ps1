$curret = Get-Location

function Build()
{
   Param([string]$target)

   # $CMAKE_OSX_ARCHITECTURES="armv7;armv7s;arm64;i386;x86_64"
   # device
   Write-Host "Build iOS $target Device Library" -Foreground Green
   $buildDir = Join-Path $target build_ios_device
   $installDir = Join-Path $curret $target | `
               Join-Path -ChildPath install_device
   $lib = Join-Path $installDir lib | `
         Join-Path -ChildPath libNative.a
   New-Item -ItemType Directory -Force $buildDir | Out-Null
   Push-Location $buildDir
   $CMAKE_OSX_ARCHITECTURES="arm64;arm64e"
   $CMAKE_IOS_INSTALL_COMBINED="NO"
   $CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH="NO"
   cmake -G Xcode `
         -D CMAKE_SYSTEM_NAME=iOS `
         -D CMAKE_OSX_ARCHITECTURES=$CMAKE_OSX_ARCHITECTURES `
         -D CMAKE_OSX_DEPLOYMENT_TARGET=9.3 `
         -D CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH=$CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH `
         -D CMAKE_IOS_INSTALL_COMBINED=$CMAKE_IOS_INSTALL_COMBINED `
         -D CMAKE_INSTALL_PREFIX=${installDir} `
         ..
   cmake --build . --config Release --target install
   Pop-Location

   # simulator
   Write-Host "Build iOS $target Simulator Library" -Foreground Green
   $buildDir = Join-Path $target build_ios_simulator
   $installDir = Join-Path $curret $target | `
               Join-Path -ChildPath install_simulator
   $lib = Join-Path $installDir lib | `
         Join-Path -ChildPath libNative.a
   New-Item -ItemType Directory -Force $buildDir | Out-Null
   Push-Location $buildDir
   $CMAKE_OSX_ARCHITECTURES="x86_64;arm64"
   $CMAKE_IOS_INSTALL_COMBINED="YES"
   $CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH="YES"
   cmake -G Xcode `
         -D CMAKE_SYSTEM_NAME=iOS `
         -D CMAKE_OSX_ARCHITECTURES=$CMAKE_OSX_ARCHITECTURES `
         -D CMAKE_OSX_DEPLOYMENT_TARGET=9.3 `
         -D CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH=$CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH `
         -D CMAKE_IOS_INSTALL_COMBINED=$CMAKE_IOS_INSTALL_COMBINED `
         -D CMAKE_INSTALL_PREFIX=${installDir} `
         ..
   cmake --build . --config Release --target install
   Pop-Location
}

Build NativeAdd
Build NativeMul

Write-Host "Build iOS Managed Library" -Foreground Green
Set-Location $curret
Set-Location NativeSharp
dotnet build -c "Release_Xamarin.iOS"

Set-Location $curret