#***************************************
#Arguments
#%1: Browser (chrome/firefox/iexplore/safari)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Browser
)

$program = "helloworld"
$current = $PSScriptRoot

# get os name
if ($global:IsWindows)
{
    $os = "win"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
}

# build
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

Push-Location $installDir
if ($global:IsWindows)
{
    emrun --browser ${Browser} "${program}.html"
}
elseif ($global:IsMacOS)
{
    emrun --browser ${Browser} "${program}.html"
}
elseif ($global:IsLinux)
{
    emrun --browser ${Browser} "${program}.html"
}
Pop-Location