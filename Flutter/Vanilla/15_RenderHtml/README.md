# Dart Documentation

## Abstracts

* Generate dart documentation
  * Exlucde `index.dart` by `@nodoc` annotation.
  * Use `@category` annotation to group classes.

## Note

`private` classes and functions are not generated.
This is design of `dart doc`.
See [Tracking issue for private documentation (#2154)](https://github.com/dart-lang/dartdoc/issues/3096).

## How to use?

````bat
$ dart doc .
Documenting demo...
Discovering libraries...
Linking elements...
Precaching local docs for 614427 elements...
  warning: deprecated dartdoc usage: The '--nodoc' option is deprecated, and will soon be removed.
    from package-demo: file:///D:/Works/OpenSource/Demo3/Flutter/Vanilla/14_DartDocmentation
Initialized dartdoc with 752 libraries
Generating docs for library app.dart from package:demo/app.dart...
Generating docs for library main.dart from package:demo/main.dart...
Generating docs for library presentations\pages\about_page\about_page.dart from package:demo/presentations/pages/about_page/about_page.dart...
Generating docs for library presentations\pages\home_page\home_page.dart from package:demo/presentations/pages/home_page/home_page.dart...
Found 1 warning and 0 errors.
Documented 4 public libraries in 43.3 seconds
Success! Docs generated into d:\works\opensource\demo3\flutter\vanilla\14_dartdocmentation\doc\api
````