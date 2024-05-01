$version = "3.3.0"

$current = $PSScriptRoot
if ($global:IsWindows)
{
   $openssl = Join-Path $current openssl | `
              Join-Path -ChildPath $version | `
              Join-Path -ChildPath windows | `
              Join-Path -ChildPath x64 | `
              Join-Path -ChildPath bin | `
              Join-Path -ChildPath openssl.exe
   $opensslConfig = Join-Path $current openssl | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath openssl.cnf
   $env:OPENSSL_CONF=$opensslConfig
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

$privateKey = "private-key.pem"
$certificateSigningRequest = "server.csr"
$sanFile = "request.conf"

$C="JP"                # Country Name
$ST="Tokyo"            # State or Province Name
$L="Minato-ku"         # Locality Name
$O="Contoso"           # Organization Name
$OU="Docs,Contoso"     # Organizational Unit Name
$CN="www.contoso.com"  # Organizational Unit Name

Write-Host "Create private key ..." -ForegroundColor Blue
& "${openssl}" genpkey -algorithm RSA -out "${privateKey}"

Copy-Item "${opensslConfig}" "${sanFile}"
echo "[SAN]" >> "${sanFile}"
echo "subjectAltName=@san_names" >> "${sanFile}"
echo "basicConstraints=CA:FALSE" >> "${sanFile}"
echo "keyUsage = nonRepudiation, digitalSignature, keyEncipherment" >> "${sanFile}"
echo "extendedKeyUsage = serverAuth, clientAuth" >> "${sanFile}"
echo "[san_names]" >> "${sanFile}"
echo "DNS.1=localhost" >> "${sanFile}"
echo "IP.1=127.0.0.1`")" >> "${sanFile}"
# echo "IP.1=${IpAddress}" >> "${sanFile}"
# echo "IP.2=127.0.0.1`")" >> "${sanFile}"

Write-Host "Create certificate signing request ..." -ForegroundColor Blue
& "${openssl}" req -new `
                   -key "${privateKey}" `
                   -out "${certificateSigningRequest}" `
                   -config "${opensslConfig}" `
                   -subj "/C=${C}/ST=${ST}/L=${L}/CN=${CN}/OU=${OU}/O=${O}"
Remove-Item "${sanFile}"