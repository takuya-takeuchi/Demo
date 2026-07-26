dotnet build -c Release
$exe = Resolve-Path ".\bin\Release\net10.0\Demo.exe"

1..10 | ForEach-Object {
    Start-Process `
        -FilePath $exe `
        -ArgumentList "--mode", "Independent"
}