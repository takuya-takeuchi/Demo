$directories = @(
    "sources/Demo/bin/Debug/Logs",
    "sources/Demo/bin/Release/Logs"
)

$today = Get-Date
foreach ($directory in $directories) {
    $targetDirectory  = Join-Path $PSScriptRoot $directory
    New-Item -Type Directory -Force $targetDirectory | Out-Null
    Push-Location $targetDirectory
    for ($i=1; $i -le 7; $i++) {
         $date = $today.AddDays(-($i + 1))
         $year = $date.Year
         $month = $date.Month.ToString("00")
         $day = $date.Day.ToString("00")
         $time = "${year}/${month}/${day} 12:00:00"
         $file = "Application.${year}${month}${day}.log"
         New-Item -Type File $file | Out-Null
         Set-ItemProperty $file -name CreationTime -value "${time}"
         Set-ItemProperty $file -name LastWriteTime -value "${time}"
    }
    
    $date = $today.AddDays(-1)
    $year = $date.Year
    $month = $date.Month.ToString("00")
    $day = $date.Day.ToString("00")
    $time = "${year}/${month}/${day} 12:00:00"
    $file = "Application.log"
    New-Item -Type File $file | Out-Null
    Set-ItemProperty $file -name CreationTime -value "${time}"
    Set-ItemProperty $file -name LastWriteTime -value "${time}"
    Pop-Location
}