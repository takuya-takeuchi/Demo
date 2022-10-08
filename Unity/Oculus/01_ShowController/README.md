# Show Controller

## Abstracts

* How to show oculus controller
* How to show line objects like lazer

## Hardware Requirements

* Oculus Quest 2

## Software Requirements

* Unity
  * 2021.3.5f1
* Oculus Integration
  * 41.0

## How to build

1. Run `git sumobule update --init --recursive .` to Restore Oculus Integration
2. Open project
3. Open Assets\ShowController.unity
4. Go to Build Settings
   * Change Texture Compression to ASTC
   * Switch Platform
5. Go to Project Settings
   * Install XR Plug-in Management and enable Oculus in Android tab
6. Build And Run

## Screen shots

[![result](./images/image.jpg "result")](./images/image.jpg)