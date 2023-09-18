# Integration Test for Pigeon example

## Abstracts

* Add integration test and unit test to [01_Pigeon](../01_Pigeon)

## Dependencies

* [pigeon](https://pub.dev/packages/pigeon)
  * BSD-3-Clause license

## Conclusion

* We can NOT complete unit test because `Unit tests` and `Widget tests` are running in memory rather than physical device
  * `pigeon` using `Methoc Channel` is not tested on the contrary test is freeze
* `integration_test` can do coverage test but it also does not get coverage data about native code
* `integration_test` can support only debug mode
  * `flutter drive` can support profile mode but id does not support coverage test
