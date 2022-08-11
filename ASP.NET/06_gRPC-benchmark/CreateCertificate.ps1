# https://stackoverflow.com/questions/37714558/how-to-enable-server-side-ssl-for-grpc
$current = $PSScriptRoot

$password = "changeit"
$days = "365"
# /C=US/ST=CA/L=Cupertino/O=YourCompany/OU=YourApp/CN=MyRootCA
$caPublisher = "/C=JP/S=TOKYO/L=MINATO-KU/O=Demo Corporation/CN=Demo Corporation"
# /C=US/ST=CA/L=Cupertino/O=YourCompany/OU=YourApp/CN=%COMPUTERNAME%
$serverPublisher = "/C=JP/S=TOKYO/L=MINATO-KU/O=Demo Corporation/CN=Demo Corporation"
# /C=US/ST=CA/L=Cupertino /O=YourCompany /OU=YourApp/CN=%CLIENT-COMPUTERNAME%
$clientPublisher = "/C=JP/S=TOKYO/L=MINATO-KU/O=Demo Corporation/CN=Demo Corporation"
$openSSLVersion = "1.0.2u"

$opensslRoot = Join-Path "${current}" "tools" | `
               Join-Path -ChildPath "openssl" | `
               Join-Path -ChildPath "${openSSLVersion}"
$openssl = Join-Path "${opensslRoot}" "openssl.exe"
$opensslConf = Join-Path "${current}" "tools" | `
               Join-Path -ChildPath "openssl" | `
               Join-Path -ChildPath "openssl.cnf"
if (!(Test-Path("${opensslConf}")))
{
    Write-Host "openssl.cnf: ${opensslConf} is missing" -ForegroundColor Red
    exit -1
}
if (!(Test-Path("${openssl}")))
{
    Write-Host "openssl.cnf: ${openssl} is missing" -ForegroundColor Red
    exit -1
}

$env:OPENSSL_CONF="${opensslConf}"

Write-Host "Generate CA key:" -ForegroundColor Green
& "${openssl}" genrsa -passout pass:"${password}" -des3 -out ca.key 4096

Write-Host "Generate CA certificate:" -ForegroundColor Green
& "${openssl}" req -passin pass:"${password}" -new -x509 -days $days -key ca.key -out ca.crt -subj "${caPublisher}"

Write-Host "Generate server key:" -ForegroundColor Green
& "${openssl}" genrsa -passout pass:"${password}" -des3 -out server.key 4096

Write-Host "Generate server signing request:" -ForegroundColor Green
& "${openssl}" req -passin pass:"${password}" -new -key server.key -out server.csr -subj "${serverPublisher}"

Write-Host "Self-sign server certificate:" -ForegroundColor Green
& "${openssl}" x509 -req -passin pass:"${password}" -days $days -in server.csr -CA ca.crt -CAkey ca.key -set_serial 01 -out server.crt

Write-Host "Remove passphrase from server key:" -ForegroundColor Green
& "${openssl}" rsa -passin pass:"${password}" -in server.key -out server.key

Write-Host "Generate client key" -ForegroundColor Green
& "${openssl}" genrsa -passout pass:"${password}" -des3 -out client.key 4096

Write-Host "Generate client signing request:" -ForegroundColor Green
& "${openssl}" req -passin pass:"${password}" -new -key client.key -out client.csr -subj "${clientPublisher}"

Write-Host "Self-sign client certificate:" -ForegroundColor Green
& "${openssl}" x509 -passin pass:"${password}" -req -days $days -in client.csr -CA ca.crt -CAkey ca.key -set_serial 01 -out client.crt

Write-Host "Remove passphrase from client key:" -ForegroundColor Green
& "${openssl}" rsa -passin pass:"${password}" -in client.key -out client.key

$clients = @(
    "Client",
    "LegacyClient"
)

$sourceRoot = Join-Path $current "sources"

foreach ($client in $clients)
{
    $outputDir = Join-Path $sourceRoot $client
    $outputCertificatesDir = Join-Path $outputDir "certificates"
    New-Item -Type Directory -Force "${outputCertificatesDir}" | Out-Null
    Write-Host "Output: ${outputCertificatesDir}" -ForegroundColor Green

    $certificates = @(
        "ca.crt",
        "client.key",
        "client.crt"
    )

    foreach ($certificate in $certificates)
    {
        $srcFile = Join-Path $current $certificate
        $dstFile = Join-Path $outputCertificatesDir $certificate
        Copy-Item $srcFile $dstFile -Force
    }
}

$servers = @(
    "Server"
)

foreach ($server in $servers)
{
    $outputDir = Join-Path $sourceRoot $server
    $outputCertificatesDir = Join-Path $outputDir "certificates"
    New-Item -Type Directory -Force "${outputCertificatesDir}" | Out-Null
    Write-Host "Output: ${outputCertificatesDir}" -ForegroundColor Green

    $certificates = @(
        "ca.crt",
        "server.key",
        "server.crt"
    )

    foreach ($certificate in $certificates)
    {
        $srcFile = Join-Path $current $certificate
        $dstFile = Join-Path $outputCertificatesDir $certificate
        Copy-Item $srcFile $dstFile -Force
    }
}