$version = "1.0.2u"

$current = $PSScriptRoot
if ($global:IsWindows)
{
   $openssl = Join-Path $current openssl | `
              Join-Path -ChildPath $version | `
              Join-Path -ChildPath windows | `
              Join-Path -ChildPath openssl.exe
   $opensslConfig = Join-Path $current openssl | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath openssl.cnf
}
elseif ($global:IsMacOS)
{
   $openssl = "/usr/bin/openssl"
   $opensslConfig = Join-Path $current openssl | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath openssl.cnf
}
elseif ($global:IsLinux)
{
   $openssl = "/usr/bin/openssl"
   $opensslConfig = Join-Path $current openssl | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath openssl.cnf
}

$caRoot = Join-Path $current pki | Join-Path -ChildPath CA
$rootCAConf = Join-Path $current root-ca.conf
$rootCAPrivateKey = Join-Path $caRoot private | Join-Path -ChildPath root-ca.key
$rootCACsr = Join-Path $caRoot root-ca.csr
$rootCACrt = Join-Path $caRoot root-ca.crt

Write-Host "Create private key and certificate signing request..." -ForegroundColor Blue
& "${openssl}" req -new -config "${rootCAConf}" -out "${rootCACsr}" -keyout "${rootCAPrivateKey}"
Write-Host "Do Self-Sign certificate signing request..." -ForegroundColor Blue
& "${openssl}" ca -selfsign -config "${rootCAConf}" -in "${rootCACsr}" -out "${rootCACrt}" -extensions req_ext