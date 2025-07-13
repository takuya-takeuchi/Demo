$current = $PSScriptRoot
$venvDir = Join-Path $current ".venv"
if (!(Test-Path("${venvDir}")))
{
    Write-Host "[Info] Create python venv"
    python -m venv "${venvDir}"
}

Write-Host "[Info] Activate python venv"
if ($global:IsWindows)
{
    . "${venvDir}\Scripts\activate.ps1"
    Write-Host "[Info] Activated"
}
elseif ($global:IsMacOS)
{
    . "${venvDir}\Scripts\activate.ps1"
    Write-Host "[Info] Activated"
}
elseif ($global:IsLinux)
{
    . "${venvDir}\Scripts\activate.ps1"
    Write-Host "[Info] Activated"
}

Write-Host "[Info] Check python executable location"
python -c "import sys; print(f'Python executable: {sys.executable}')"

python -m pip install pip --upgrade
python -m pip install requests