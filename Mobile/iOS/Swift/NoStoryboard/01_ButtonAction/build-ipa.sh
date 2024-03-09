#!/bin/bash +x

PROJECT=Demo
DEVELOPMENT_TEAM=U27JXXX492
WORKSPACE=

if [ -d ${PROJECT}.xcworkspace ]; then
    WORKSPACE="-workspace ${PROJECT}.xcworkspace"
    pod install
fi 

echo -e "\033[1;32mplease enter password...\033[0m"
security unlock-keychain login.keychain

mkdir -p build

echo -e "\033[1;32mclean build...\033[0m"
xcodebuild \
    -scheme ${PROJECT} \
    ${WORKSPACE} \
    -configuration Release \
    -derivedDataPath ./build  \
    -sdk iphoneos \
    clean build \
    DEVELOPMENT_TEAM=${DEVELOPMENT_TEAM} \
    CODE_SIGN_STYLE=Automatic

echo -e "\033[1;32marchive...\033[0m"
xcodebuild \
    -scheme ${PROJECT} \
    ${WORKSPACE} \
    -configuration Release archive \
    -archivePath ./build/${PROJECT}.xcarchive \
    -sdk iphoneos \
    DEVELOPMENT_TEAM=${DEVELOPMENT_TEAM} \
    CODE_SIGN_STYLE=Automatic
    
echo -e "\033[1;32mbuild ipa...\033[0m"
xcodebuild \
    -exportArchive \
    -archivePath  ./build/${PROJECT}.xcarchive  \
    -exportPath ./build/ipa \
    -exportOptionsPlist ./build/${PROJECT}.xcarchive/Info.plist