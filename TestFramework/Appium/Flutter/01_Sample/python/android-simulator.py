import os

from appium.webdriver import Remote
from appium_flutter_finder.flutter_finder import FlutterElement, FlutterFinder


# "C:\Program Files (x86)\Android\android-sdk\platform-tools\adb.exe" devices
# List of devices attached
# emulator-5554   device

driver = Remote('http://localhost:4725', dict(
    platformName='Android',
    automationName='flutter',
    platformVersion='13.0',
    # deviceName='Pixel 5 - API 33',
		deviceName="emulator-5554",
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

driver.save_screenshot("screenshot-simulator.png")

driver.quit()