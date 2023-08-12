import os

from appium.webdriver import Remote
from appium_flutter_finder.flutter_finder import FlutterElement, FlutterFinder

driver = Remote('http://localhost:4723', dict(
    platformName='Android',
    automationName='flutter',
    platformVersion='11.0',
		deviceName="HQ618G0E28",
		appPackage="jp.taktak.flutter.android.demo",
    app='{}/../demo/build/app/outputs/flutter-apk/app-debug.apk'.format(
      os.path.dirname(os.path.realpath(__file__)))
))

finder = FlutterFinder()

text_finder = finder.by_text('You have pushed the button this many times:')
text_element = FlutterElement(driver, text_finder)
print(text_element.text)

button_increment_finder = finder.by_value_key("button_increment")
button_increment_element = FlutterElement(driver, button_increment_finder)
button_increment_element.click()

counter_finder = finder.by_value_key("message_count")
counter_element = FlutterElement(driver, counter_finder)
print(counter_element.text)

driver.save_screenshot("screenshot-device.png")

driver.quit()