if ($global:IsWindows)
{
    if (!(Test-Path(${env:JAVA_HOME})))
    {
        Write-Host "JAVA_HOME: ${env:JAVA_HOME} is missing" -ForegroundColor Red
        exit
    }

    $keytool = Join-Path $env:JAVA_HOME jre | `
               Join-Path -ChildPath bin | `
               Join-Path -ChildPath keytool.exe
    if (!(Test-Path(${keytool})))
    {
        $keytool = Join-Path $env:JAVA_HOME bin | `
                   Join-Path -ChildPath keytool.exe
    }

    $destination = Join-Path $env:USERPROFILE ".android"
}
elseif ($global:IsMacOS)
{
    $keytool = (which keytool)
    $destination = Join-Path $env:HOME ".android"
}
elseif ($global:IsLinux)
{
    $keytool = (which keytool)
    $destination = Join-Path $env:HOME ".android"
}

if (!(Test-Path("${keytool}")))
{
    Write-Host "keytool: '${keytool}' is missing" -ForegroundColor Red
    exit
}

Write-Host "keytool: '${keytool}'" -ForegroundColor Green

$KEYSTORE="debug.keystore"
$ALIAS="androiddebugkey"
$NAME="CN=Android Debug, O=Android, C=US"
$STOREPASS="android"
$KEYPASS=$STOREPASS
$VALIDITY=10950

Write-Host "Delete Key..." -ForegroundColor Green
& "${keytool}" -delete `
               -keystore ${KEYSTORE} `
               -alias ${ALIAS} `
               -storepass ${STOREPASS}

Write-Host "Generate Key..." -ForegroundColor Green
& "${keytool}" -genkey `
               -v `
               -storetype pkcs12 `
               -keystore ${KEYSTORE} `
               -keyalg RSA `
               -validity ${VALIDITY} `
               -storepass ${STOREPASS} `
               -alias ${ALIAS} `
               -dname "${NAME}" `
               -keypass ${KEYPASS}

New-Item -Type Directory "${destination}" -Force | Out-Null
$destination = Join-Path $destination "${KEYSTORE}"
Write-Host "Move to ${destination} ..." -ForegroundColor Green
Move-Item "${KEYSTORE}" "${destination}" -Force

Write-Host "Show keystore list ..." -ForegroundColor Green
& "${keytool}" -list -v -keystore "${destination}"