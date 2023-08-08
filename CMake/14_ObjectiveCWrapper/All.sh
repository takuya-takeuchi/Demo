#!/bin/bash +x

pwsh BuildObjectiveLuhn.ps1 iphonesimulator x86_64 Release
pwsh BuildObjectiveLuhn.ps1 iphoneos arm64  Release
pwsh CreateXCFramework.ps1 ObjectiveLuhn Luhn

pwsh BuildObjectiveLuhnWrapper.ps1 iphonesimulator x86_64 Release
pwsh BuildObjectiveLuhnWrapper.ps1 iphoneos arm64  Release
pwsh CreateXCFramework.ps1 ObjectiveLuhnWrapper Luhnc