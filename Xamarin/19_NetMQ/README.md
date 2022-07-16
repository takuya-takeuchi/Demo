# NetMQ Pub/Sub Demo

## Abstacts

* How to use NetMQ (ZeroMQ)

## Requirements

* Visual Studio 2022
* Xamarin

## Dependencies

* [Font Awesome Free](https://fontawesome.com/)
  * Fonts
    * SIL Open Font License
* [NetMQ ](https://github.com/zeromq/netmq)
  * GNU Lesser General Public License Version 3
* [NLog](https://github.com/NLog/NLog)
  * BSD-3-Clause License
* [Prism.Unity.Forms](https://github.com/PrismLibrary/Prism)
  * MIT License
* [Xamarin.Essentials.Interfaces](https://github.com/rdavisau/essential-interfaces)
  * MIT License
* [Xamarin.Forms](https://github.com/xamarin/Xamarin.Forms)
  * MIT License

## How to use?

### 1. Launch Pub server

````bat
19_NetMQ>dotnet run -c Release --project sources\Demo.Server
2022-07-17 02:02:42.5195 [INFO ] Publisher socket binding...
2022-07-17 02:02:42.5541 [INFO ] Sending message : TopicA msg-0
2022-07-17 02:02:43.0759 [INFO ] Sending message : TopicB msg-1
2022-07-17 02:02:43.5874 [INFO ] Sending message : TopicA msg-2
2022-07-17 02:02:44.0994 [INFO ] Sending message : TopicB msg-3
````

And you have to expose port `12345` by configure firewall.

### 2. Launch Mobile App

Input Pub server endpoint.
Port number must be 12345.

For example, `tcp://192.168.11.21:12345`

Click `Connect` button and then App start to subscribe message from server.

### Android

<img src="images/android-main.png?raw=true" width="40%" height="auto" title="Main Page"/>

### iOS

<img src="images/ios-main.png?raw=true" width="40%" height="auto" title="Main Page"/>
