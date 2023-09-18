# Integration Test for Pigeon example

## Abstracts

* Add integration test and unit test to [01_Pigeon](../01_Pigeon)

## Dependencies

* [pigeon](https://pub.dev/packages/pigeon)
  * BSD-3-Clause license

## Unit Tests with coverage

#### iOS

````sh
$ xcrun xctrace list devices                                                                                           
== Devices ==
Mac mini (DFC7EACC-26D5-57E4-A92B-AAAAAAAAAAAA)
iPhone SE (16.6.1) (00008110-AAAAAAAAAAAAAAAA)

== Simulators ==
iPad (10th generation) Simulator (16.1) (4BC60086-72F6-4333-8496-0EDA07DD62B8)
iPad (10th generation) Simulator (16.4) (FBC68DB7-499B-40CC-9F6B-E766541A1193)
...
iPhone SE (3rd generation) Simulator (16.4) (DA8BBD45-ED0E-4C24-9726-719CFE9C8418)
iPod touch (7th generation) Simulator (15.5) (B36DACEA-7CD3-44F7-A35B-02530E8C278E)

$ xcodebuild test -workspace Runner.xcworkspace -scheme Runner -destination '<UUID of simulator or device>' -enableCodeCoverage YES -derivedDataPath DerivedData/ 
````

$ xcrun xccov view --report DerivedData/Logs/Test/Test-Runner-2023.09.18_20-12-56-+0900.xcresult/ --only-targets

$ xcrun xccov view --report DerivedData/Logs/Test/Test-Runner-2023.09.18_20-35-19-+0900.xcresult/ --only-targets

##### Note

If you see `Could not find included file 'Generated.xcconfig' in search paths (in target 'Runner')` when do test, you must do `flutter clean && flutter build`.

## Conclusion

* We can NOT complete unit test because `Unit tests` and `Widget tests` are running in memory rather than physical device
  * `pigeon` using `Methoc Channel` is not tested on the contrary test is freeze
* `integration_test` can do coverage test but it also does not get coverage data about native code
* `integration_test` can support only debug mode
  * `flutter drive` can support profile mode but id does not support coverage test
