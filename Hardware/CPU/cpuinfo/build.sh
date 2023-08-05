#!/bin/bash

pwsh Build.ps1 iphoneos arm64 Release \
  && pwsh Build.ps1 iphonesimulator x86_64 Release \
  && pwsh Build.ps1 iphonesimulator arm64 Release