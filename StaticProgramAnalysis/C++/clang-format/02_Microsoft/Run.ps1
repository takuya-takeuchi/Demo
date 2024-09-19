try
{
    $commandInfo = Get-Command clang-format -ErrorAction Stop
    $clangFormat = $commandInfo.Source
    Write-Host "[Info] clang-format: ${clangFormat}" -ForegroundColor Green
}
catch
{
    Write-Host "[Error] clang-format is not installed" -ForegroundColor Red
    exit
}

& "${clangFormat}" sample.cpp > sample_formatted.cpp