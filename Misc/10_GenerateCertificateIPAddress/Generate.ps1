Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $IpAddress
)

$C="JP"              # Country Name
$ST="Tokyo"          # State or Province Name
$L="Minato-ku"       # Locality Name
$O="Contoso"         # Organization Name
$OU="Docs,Contoso"   # Organizational Unit Name
$CACN="Contoso Insecure Certificate Authority"   # CN for CA

$version = "1.0.2u"

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

$sanFile = "san.txt"

# remove old files
$files = @(
   "ca.crt",
   "ca.key",
   "ca.srl",
   "server.crt",
   "server.csr",
   "server.key",
   "${sanFile}"
)

foreach ($file in $files)
{
   if (Test-Path("${file}"))
   {
      Remove-Item "${file}"
   }
}

Write-Host "Create your CA crt and key:" -ForegroundColor Green
& "${openssl}" req -new `
                   -newkey rsa:4096 `
                   -days 3650 `
                   -nodes `
                   -x509 `
                   -sha256 `
                   -subj "/C=${C}/ST=${ST}/L=${L}/O=${O}/OU=${OU}/CN=${CACN}" `
                   -extensions v3_ca `
                   -config "${opensslConfig}" `
                   -keyout ca.key `
                   -out ca.crt

Copy-Item "${opensslConfig}" "{sanFile}"
echo "[SAN]" >> "{sanFile}"
echo "subjectAltName=@san_names" >> "{sanFile}"
echo "basicConstraints=CA:FALSE" >> "{sanFile}"
echo "keyUsage = nonRepudiation, digitalSignature, keyEncipherment" >> "{sanFile}"
echo "[san_names]" >> "{sanFile}"
echo "DNS.1=localhost" >> "{sanFile}"
echo "IP.1=${IpAddress}" >> "{sanFile}"
echo "IP.2=127.0.0.1`")" >> "{sanFile}"

Write-Host "Create a CSR:" -ForegroundColor Green
& "${openssl}" req -newkey rsa:2048 `
                   -nodes `
                   -sha256 `
                   -keyout server.key `
                   -out server.csr `
                   -config "${opensslConfig}" `
                   -subj "/C=${C}/ST=${ST}/L=${L}/O=${O}/OU=${OU}/CN=${IpAddress}"

Write-Host "Sign the CSR, resulting in CRT and add the v3 SAN extension:" -ForegroundColor Green
& "${openssl}" x509 -req `
                    -in server.csr `
                    -out server.crt `
                    -CA ca.crt `
                    -CAkey ca.key `
                    -CAcreateserial `
                    -sha256 `
                    -days 3650 `
                    -extensions SAN `
                    -extfile "{sanFile}"

if (Test-Path("{sanFile}"))
{
   Remove-Item "{sanFile}"
}

Write-Host "Check contents of CSR:" -ForegroundColor Green
& "${openssl}" req -text -noout -in server.csr 
Write-Host "Check contents of CRT" -ForegroundColor Green
& "${openssl}" x509 -text -noout -in server.crt

# remove files
$files = @(
   ".rnd",
   "ca.srl",
   "${sanFile}"
)

foreach ($file in $files)
{
   if (Test-Path("${file}"))
   {
      Remove-Item "${file}"
   }
}