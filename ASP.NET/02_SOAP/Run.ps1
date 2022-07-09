$demoDirectory = Join-Path sources Demo
Push-Location ${demoDirectory}
dotnet run -c Release
Pop-Location