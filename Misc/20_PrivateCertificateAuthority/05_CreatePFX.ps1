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

if (!(Test-Path("server.pem")))
{
   Write-Host "'server.pem' is missing" -ForegroundColor Red
   exit
}

$password = Read-Host "Enter Password" -AsSecureString
$confirmPassword = Read-Host "Confirm Password" -AsSecureString

function SecureString2PlainString($secure){
    $bstr = [System.Runtime.InteropServices.Marshal]::SecureStringToBSTR($secure)
    $plain = [System.Runtime.InteropServices.Marshal]::PtrToStringBSTR($bstr)
    [System.Runtime.InteropServices.Marshal]::ZeroFreeBSTR($bstr)
    return $plain
}

$p1 = SecureString2PlainString $password
$p2 = SecureString2PlainString $confirmPassword
if (!($p1 -eq $p2))
{
   Write-Host "password is incorrect" -ForegroundColor Red
   exit
}

Write-Host "Create personal information exchange..." -ForegroundColor Blue
& "${openssl}" pkcs12 -export -in server.crt -inkey server.pem -out server.pfx -password pass:${p1}