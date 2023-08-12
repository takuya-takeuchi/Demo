import os

from appium.webdriver import Remote
from appium_flutter_finder.flutter_finder import FlutterElement, FlutterFinder

driver = Remote('http://localhost:4723', dict(
    platformName='iOS',
    automationName='flutter',
    platformVersion='15.7',
    deviceName='iPhone (2)',
    udid='9a1e9bf15489282f50f795bd0768752f28d62604',
    # appid='jp.taktak.flutter.ios.demo'
    # https://github.com/appium-userland/appium-flutter-driver/issues/115
    # This issud may restrict us to launch debug app so use profile app
    # And appid does not work so use app!!
    app='{}/../demo/build/ios/Profile-iphoneos/Runner.app'.format(
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