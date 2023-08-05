#!/bin/bash +x

curl https://storage.googleapis.com/flutter_infra_release/releases/stable/macos/flutter_macos_3.10.6-stable.zip -o flutter_macos_3.10.6-stable.zip
unzip flutter_macos_3.10.6-stable.zip
mv flutter ~/
rm flutter_macos_3.10.6-stable.zip
echo 'export PATH=$PATH:$HOME/flutter/bin' >> ~/.zprofile
source ~/.zprofile

flutter doctor
flutter --disable-telemetry