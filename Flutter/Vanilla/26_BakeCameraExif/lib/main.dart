import 'dart:ffi';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import 'package:camera/camera.dart';
import 'package:image/image.dart' as img;
import 'package:exif/exif.dart';

Future<void> main() async {
  runApp(const MainPage());
}

class MainPage extends StatefulWidget {
  const MainPage({Key? key}) : super(key: key);

  @override
  State<MainPage> createState() => _MainPageState();
}

class _MainPageState extends State<MainPage> {
  late List<CameraDescription> _cameras;
  late Future<CameraController?> _cameraController;
  CameraController? _currentCameraController;
  CameraDescription? _currentCamera;
  Image? _picture;
  double? _pictureWidth;
  double? _pictureHeight;
  int _capturePictureWidth = 0;
  int _capturePictureHeight = 0;
  bool _skipScanning = false;
  int _currentCameraIndex = 0;

  @override
  void initState() {
    super.initState();
    _cameraController = _setup();
  }

  @override
  void dispose() {
    _currentCameraController?.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    const double barHeight = 44;
    const double padding = 8;

    return MaterialApp(
      home: Scaffold(
        body: Center(
          child: Stack(
            children: [
              Container(
                constraints: const BoxConstraints.expand(),
                child: FutureBuilder(
                  future: _cameraController,
                  builder: (BuildContext context, AsyncSnapshot<CameraController?> snapshot) {
                    if (snapshot.connectionState == ConnectionState.done && snapshot.data != null && _currentCameraController != null) {
                      return CameraPreview(snapshot.data!);
                    }
                    return const Spacer();
                  },
                ),
              ),
              Align(
                alignment: Alignment.bottomCenter,
                child: Container(
                  width: double.infinity,
                  color: const Color.fromRGBO(0, 0, 0, 0.5),
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                    children: [
                      GestureDetector(
                        onTap: () async => await _onSwitchCamera(),
                        child: const Padding(
                          padding: EdgeInsets.all(padding),
                          child: Icon(Icons.switch_camera, size: barHeight, color: Colors.white),
                        ),
                      ),
                      Text("$_capturePictureWidth x $_capturePictureHeight", style: const TextStyle(color: Colors.white)),
                      GestureDetector(
                        onTap: () async => await _onTakePicture(),
                        child: const Padding(
                          padding: EdgeInsets.all(padding),
                          child: Icon(Icons.camera, size: barHeight, color: Colors.white),
                        ),
                      ),
                    ],
                  ),
                ),
              ),
              Align(
                alignment: Alignment.topRight,
                child: Visibility(
                  visible: _picture != null,
                  child: Padding(
                    padding: const EdgeInsets.all(20),
                    child: Container(
                      width: _pictureWidth,
                      height: _pictureHeight,
                      decoration: BoxDecoration(
                        color: Colors.white,
                        borderRadius: BorderRadius.circular(15),
                        border: Border.all(color: Colors.white, width: 2),
                      ),
                      child: ClipRRect(
                        borderRadius: const BorderRadius.all(Radius.circular(15)),
                        child: SizedBox(width: _pictureWidth, height: _pictureHeight, child: _picture ?? const Spacer()),
                      ),
                    ),
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Future<CameraController?> _initialize(int cameraIndex, [ResolutionPreset resolutionPreset = ResolutionPreset.max, bool enableAudio = false]) async {
    _currentCameraController = null;
    final cameraController = CameraController(_cameras[cameraIndex], resolutionPreset, enableAudio: enableAudio);

    // await _cameraController.initialize();
    await cameraController.initialize().catchError((Object e) {
      if (e is CameraException) {
        switch (e.code) {
          case 'CameraAccessDenied':
            // ignore: avoid_print
            print('User denied camera access.');
            break;
          default:
            // ignore: avoid_print
            print('Handle other errors.');
            break;
        }
      }
    });

    if (!mounted) {
      return null;
    }

    _currentCameraController = cameraController;
    setState(() {
      _cameraController = Future.value(cameraController);
      _currentCamera = cameraController.description;
    });

    await cameraController.startImageStream(_processImage);
    return cameraController;
  }

  int _deviceOrientationToDegrees(DeviceOrientation o) {
    switch (o) {
      case DeviceOrientation.portraitUp:
        return 0;
      case DeviceOrientation.landscapeLeft:
        return 90;
      case DeviceOrientation.portraitDown:
        return 180;
      case DeviceOrientation.landscapeRight:
        return 270;
    }
  }

  int _computeJpegRotationDegrees(CameraController controller, CameraDescription desc) {
    final deviceDeg = _deviceOrientationToDegrees(controller.value.deviceOrientation);
    final sensorDeg = desc.sensorOrientation; // 90 or 270 が多い

    // CameraX/Camera2 の一般的な補正式
    if (desc.lensDirection == CameraLensDirection.front) {
      return (sensorDeg + deviceDeg) % 360;
    } else {
      return (sensorDeg - deviceDeg + 360) % 360;
    }
  }

  Future<void> _onTakePicture() async {
    try {
      _currentCameraController?.setFlashMode(FlashMode.off);
      final picture = await _currentCameraController?.takePicture();
      if (picture == null) {
        return;
      }

      final bytes = await picture.readAsBytes();
      final image = img.decodeImage(bytes);
      if (image == null) {
        throw StateError('decode failed: ${picture.path}');
      }

      final angle = _computeJpegRotationDegrees(_currentCameraController!, _currentCamera!);
      final fixed = (angle == 0) ? image : img.copyRotate(image, angle: angle);

      final Uint8List imageBinary = Uint8List.fromList(
        img.encodePng(image), // or encodeJpg
      );

      var ratio = fixed.width / fixed.height;
      var isLandscape = ratio > 1;

      const double base = 125;
      setState(() {
        if (isLandscape) {
          _pictureHeight = base;
          _pictureWidth = base / ratio;
        } else {
          _pictureHeight = base * ratio;
          _pictureWidth = base;
        }

        _picture = Image.memory(imageBinary);
      });
    } catch (e) {
      // If an error occurs, log the error to the console.
      // ignore: avoid_print
      print(e);
    }
  }

  Future<void> _onSwitchCamera() async {
    _currentCameraIndex = _currentCameraIndex == 0 ? 1 : 0;
    _currentCameraController?.stopImageStream();
    _initialize(_currentCameraIndex, ResolutionPreset.max, false);
  }

  Future<void> _processImage(CameraImage availableImage) async {
    if (!mounted || _skipScanning) return;
    setState(() {
      _skipScanning = true;
    });

    setState(() {
      _skipScanning = false;
      _capturePictureWidth = availableImage.width;
      _capturePictureHeight = availableImage.height;
    });
  }

  Future<CameraController?> _setup() async {
    _cameras = await availableCameras();
    return await _initialize(0, ResolutionPreset.max, false);
  }
}
