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


$settings = @()
$settings += New-Object PSObject -Property @{Target = "Root CA";         RootDirectory="CA";     Name = "rca";    }
$settings += New-Object PSObject -Property @{Target = "Intermediate CA"; RootDirectory="ICA";    Name = "ica";    }
$settings += New-Object PSObject -Property @{Target = "Server";          RootDirectory="Server"; Name = "server"; }

foreach ($setting in $settings)
{
   $name = $setting.Name
   $target = $setting.Target
   Write-Host "`n[${target}]" -ForegroundColor Blue

   Write-Host "Create personal information exchange..." -ForegroundColor Blue
   $root = Join-Path $current pki | Join-Path -ChildPath $setting.RootDirectory
   $privateKey = Join-Path $root key.pem
   $certificate = Join-Path $root "${name}.crt"
   $pfx = Join-Path $root "${name}.pfx"
   & "${openssl}" pkcs12 -export -in "${certificate}" -inkey "${privateKey}" -out "${pfx}" -password pass:${p1}
}