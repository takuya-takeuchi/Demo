#!/bin/sh
# So far this has worked with Microsoft's versioning
# Update the version # for the version you want to install
VERSION='7.1.1'
# Variables
TARBALL="powershell-$VERSION-linux-arm32.tar.gz"
BASEURI="https://github.com/PowerShell/PowerShell/releases/download/v$VERSION"
TARGETDIR='~/powershell'
URI="$BASEURI/$TARBALL"
# Update the Pi and ensure dependencies
sudo apt-get update
sudo apt-get install '^libssl1.0.[0-9]$' libunwind8 -y
# Download the tarball and extract
wget $URI
mkdir -p $TARGETDIR
tar -xvf ./$TARBALL -C $TARGETDIR
$TARGETDIR/pwsh