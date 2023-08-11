#!/bin/bash +x

pwsh BuildObjectiveLuhn.ps1 x86_64 Release
pwsh BuildObjectiveLuhn.ps1 arm64  Release

pwsh BuildObjectiveLuhnWrapper.ps1 x86_64 Release
pwsh BuildObjectiveLuhnWrapper.ps1 arm64  Release

pwsh Build.ps1 x86_64 Release
pwsh Build.ps1 arm64  Release