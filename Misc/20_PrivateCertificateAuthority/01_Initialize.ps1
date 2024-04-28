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
New-Item -Type Directory -Force $caRoot | Out-Null
Push-Location $caRoot
New-Item -Type Directory -Force certs | Out-Null
New-Item -Type Directory -Force db | Out-Null
New-Item -Type Directory -Force private | Out-Null
# chmod 700 private
$index = Join-Path db index
New-Item -Type File -Force $index | Out-Null
$serial = Join-Path db serial
& "${openssl}" rand -hex 16 | Out-File -FilePath "${serial}"
Pop-Location