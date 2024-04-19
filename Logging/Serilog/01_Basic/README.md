# Serilog Tutorial

## Abstracts

* Output log to console and file

## Requirements

* .NET 6.0 SDK

## Dependencies

* [Serilog.AspNetCore](https://github.com/serilog/serilog-aspnetcore)
  * 8.0.1
  * Apache-2.0 license
* [Serilog.Settings.Configuration](https://github.com/serilog/serilog-settings-configuration)
  * 8.0.0
  * Apache-2.0 license
* [Swashbuckle.AspNetCore](https://github.com/domaindrivendev/Swashbuckle.AspNetCore)
  * 6.5.0
  * MIT License

## How to build?

````bat
$ dotnet run -c Release
ビルドしています...
2024-04-19T23:27:50.6392889+09:00 [INF] Starting web application
2024-04-19T23:27:50.7957618+09:00 [DBG] Registered model binder providers, in the following order: ["Microsoft.AspNetCore.Mvc.ModelBinding.Binders.BinderTypeModelBinderProvider", "Microsoft.AspNetCore.Mvc.ModelBinding.Binders.ServicesModelBinderProvider", "Microsoft.AspNetCore.Mvc.ModelBinding.Binders.BodyModelBinderProvider", "Microsoft.AspNetCore.Mvc.ModelBinding.Binders.HeaderModelBinderProvider", "Microsoft.AspNetCore.Mvc.ModelBinding.Binders.FloatingPointTypeModelBinderProvider", "Microsoft.AspNetCore.Mvc.ModelBinding.Binders.EnumTypeModelBinderProvider", "Microsoft.AspNetCore.Mvc.ModelBinding.Binders.DateTimeModelBinderProvider", "Microsoft.AspNetCore.Mvc.ModelBinding.Binders.SimpleTypeModelBinderProvider", "Microsoft.AspNetCore.Mvc.ModelBinding.Binders.CancellationTokenModelBinderProvider", "Microsoft.AspNetCore.Mvc.ModelBinding.Binders.ByteArrayModelBinderProvider", "Microsoft.AspNetCore.Mvc.ModelBinding.Binders.FormFileModelBinderProvider", "Microsoft.AspNetCore.Mvc.ModelBinding.Binders.FormCollectionModelBinderProvider", "Microsoft.AspNetCore.Mvc.ModelBinding.Binders.KeyValuePairModelBinderProvider", "Microsoft.AspNetCore.Mvc.ModelBinding.Binders.DictionaryModelBinderProvider", "Microsoft.AspNetCore.Mvc.ModelBinding.Binders.ArrayModelBinderProvider", "Microsoft.AspNetCore.Mvc.ModelBinding.Binders.CollectionModelBinderProvider", "Microsoft.AspNetCore.Mvc.ModelBinding.Binders.ComplexObjectModelBinderProvider"]
2024-04-19T23:27:50.8306104+09:00 [DBG] Hosting starting
2024-04-19T23:27:50.8908417+09:00 [DBG] Using development certificate: "CN=localhost" (Thumbprint: "6C9210AA1581B881CEE434EC5F9CD2479E2525BA")
2024-04-19T23:27:50.9248050+09:00 [INF] Now listening on: "https://localhost:5001"
2024-04-19T23:27:50.9267755+09:00 [INF] Now listening on: "http://localhost:5000"
2024-04-19T23:27:50.9296009+09:00 [DBG] Loaded hosting startup assembly "Demo"
2024-04-19T23:27:50.9322685+09:00 [INF] Application started. Press Ctrl+C to shut down.
2024-04-19T23:27:50.9339751+09:00 [INF] Hosting environment: "Development"
2024-04-19T23:27:50.9348863+09:00 [INF] Content root path: "E:\\Works\\OpenSource\\Demo2\\Logging\\Serilog\\01_Basic\\sources\\Demo\\"
2024-04-19T23:27:50.9363211+09:00 [DBG] Hosting started
````