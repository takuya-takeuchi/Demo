# Select Preferred Language for iOS

## Abstracts

* How to show 'Preferred Language' setting in application setting page

## Dependencies

* [app_settings](https://github.com/spencerccf/app_settings)
  * MIT license

## Detailed description 

Basically, `Prefered Language` of application setting is not present.

<img src="./images/ios-empty-setting.png" width="300" />

It will appear after a following conditions is met

* Added support multiple localizations to application and added preferred languages to iOS

At first, add some localizations supported by application.
For example, adding Japanese

<img src="./images/xcode-add-localizations.png" width="600" />

<img src="./images/xcode-add-localizations2.png" width="600" />

<img src="./images/xcode-add-localizations3.png" width="600" />

<img src="./images/xcode-add-localizations4.png" width="600" />

Then, add some preferred languages supported by system.
For example, added English

<img src="./images/ios-preferred-language.png" width="300" />

After this, application setting will be shown.

<img src="./images/ios-present-application-setting.png" width="300" />

And it contains `Preferred Language`.

<img src="./images/ios-app-setting.png" width="300" />
<img src="./images/ios-app-preferred-language.png" width="300" />

In conclusion, if developper hope to have user change application language from application setting, developer must add multiple localizations languagae in xcode.
In other words, application setting page will not be present even if user add preferred languages onln in iOS.

## Screenshots

<img src="./images/ios.gif" width="320" />