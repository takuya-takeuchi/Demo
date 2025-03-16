$version = "1.0.2u"

$C="JP"                   # Country Name
$ST="Osaka"               # State or Province Name
$L="Osaka-shi"            # Locality Name
$O="Contoso Asia"         # Organization Name
$OU="Docs,Contoso Asia"   # Organizational Unit Name

if ($global:IsWindows)
{
   $openssl = Join-Path $PSScriptRoot openssl | `
              Join-Path -ChildPath $version | `
              Join-Path -ChildPath windows | `
              Join-Path -ChildPath openssl.exe
   $opensslConfig = Join-Path $PSScriptRoot openssl | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath openssl.cnf
}
elseif ($global:IsMacOS)
{
   $openssl = "/usr/bin/openssl"
   $opensslConfig = Join-Path $PSScriptRoot openssl | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath openssl.cnf
}
elseif ($global:IsLinux)
{
   $openssl = "/usr/bin/openssl"
   $opensslConfig = Join-Path $PSScriptRoot openssl | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath openssl.cnf
}

Write-Host "Create client key:" -ForegroundColor Green
& "${openssl}" genrsa -des3 -out client.key 4096

Write-Host "Create a CSR:" -ForegroundColor Green
& "${openssl}" req -new `
                   -key client.key `
                   -subj "/C=${C}/ST=${ST}/L=${L}/O=${O}/OU=${OU}" `
                   -out client.csr

Write-Host "Sign the CSR:" -ForegroundColor Green
& "${openssl}" x509 -req `
                    -days 365 `
                    -in client.csr `
                    -CA ca.crt `
                    -CAkey ca.key `
                    -set_serial 01 `
                    -out client.crt

Write-Host "Decrypt client key:" -ForegroundColor Green
& "${openssl}" rsa -in client.key -out client.decrypted.key

Write-Host "Create pfx:" -ForegroundColor Green
& "${openssl}" pkcs12 -export -out client.pfx -inkey client.decrypted.key -in client.crt