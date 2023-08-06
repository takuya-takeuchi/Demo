#!/bin/bash +x

os = "macos"
channel = "stable"
version = "3.10.6"

curl https://storage.googleapis.com/flutter_infra_release/releases/${channel}/${os}/flutter_${os}_${version}-${channel}.zip -o flutter_${os}_${version}-${channel}.zip
unzip flutter_${os}_${version}-${channel}.zip
mv flutter ~/
rm flutter_${os}_${version}-${channel}.zip
echo 'export PATH=$PATH:$HOME/flutter/bin' >> ~/.zprofile
source ~/.zprofile

flutter doctor
flutter --disable-telemetry