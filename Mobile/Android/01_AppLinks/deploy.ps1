
if ($global:IsWindows)
{
    Push-Location AppLinksSource
    Start-Process -Wait -FilePath gradlew.bat -ArgumentList "clean assembleRelease"
    Pop-Location
    
    Push-Location AppLinksTarget
    Start-Process -Wait -FilePath gradlew.bat -ArgumentList "clean assembleRelease"
    Pop-Location
}
else
{
    Push-Location AppLinksSource
    gradle clean
    gradle assembleRelease
    Pop-Location
    
    Push-Location AppLinksTarget
    gradle clean
    gradle assembleRelease
    Pop-Location
}

pwsh ../SignAPK.ps1 "AppLinksSource/app/build/outputs/apk/release/app-release-unsigned.apk"
pwsh ../SignAPK.ps1 "AppLinksTarget/app/build/outputs/apk/release/app-release-unsigned.apk"

if ($global:IsWindows)
{
    $adb = Join-Path $env:ANDROID_HOME "platform-tools" | `
           Join-Path -ChildPath "adb.exe"
}
else
{
    $adb = Join-Path $env:ANDROID_HOME "platform-tools" | `
           Join-Path -ChildPath "adb"
}

& "${adb}" install -r AppLinksSource/app/build/outputs/apk/release/app-release-signed.apk
& "${adb}" install -r AppLinksTarget/app/build/outputs/apk/release/app-release-signed.apk