import 'package:flutter/material.dart';

import 'package:camera/camera.dart';
import 'package:native_device_orientation/native_device_orientation.dart';

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
  late CameraController _cameraController;
  bool _isReady = false;
  bool _skipScanning = false;
  int _currentCamera = 0;

  @override
  void initState() {
    super.initState();
    _setup();
  }

  @override
  void dispose() {
    _cameraController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final size = MediaQuery.of(context).size;
    final deviceRatio = size.width / size.height;

    return MaterialApp(
      home: Scaffold(
        body: Center(
          child: Stack(children: [
            CameraPreview(_cameraController),
            Align(
              alignment: Alignment.bottomCenter,
              child: Container(
                width: double.infinity,
                color: const Color.fromRGBO(255, 255, 255, 0.5),
                child: GestureDetector(
                  onTap: () async => await _onSwitchCamera(),
                  child: _autoOrientedWidget(
                    const Icon(Icons.switch_camera, size: 44),
                  ),
                ),
              ),
            )
          ]),
        ),
      ),
    );
  }

  Widget _autoOrientedWidget(Widget widget) {
    return NativeDeviceOrientedWidget(
        portraitUp: (context) => _orientedWidget(widget, 0),
        landscapeLeft: (context) => _orientedWidget(widget, 1),
        landscapeRight: (context) => _orientedWidget(widget, 3),
        portraitDown: (context) => _orientedWidget(widget, 2),
        fallback: (context) => _orientedWidget(widget, 0),
        useSensor: true);
  }

  Widget _orientedWidget(Widget widget, int quarterTurns) {
    return RotatedBox(quarterTurns: 0, child: widget);
  }

  Future<void> _onSwitchCamera() async {
    _cameraController.stopImageStream();

    _currentCamera = _currentCamera == 0 ? 1 : 0;
    final cameraController = CameraController(
      _cameras[_currentCamera],
      ResolutionPreset.max,
      enableAudio: false,
    );

    await cameraController.initialize().catchError((Object e) {
      if (e is CameraException) {
        switch (e.code) {
          case 'CameraAccessDenied':
            print('User denied camera access.');
            break;
          default:
            print('Handle other errors.');
            break;
        }
      }
    });

    await cameraController.startImageStream(_processImage);

    // rebuild UI
    setState(() {
      _cameraController = cameraController;
    });
  }

  Future<void> _processImage(CameraImage availableImage) async {
    if (!mounted || _skipScanning) return;
    setState(() {
      _skipScanning = true;
    });

    // final inputImage = convert(
    //   camera: cameras[0],
    //   cameraImage: availableImage,
    // );

    // _recognizedText = await _textRecognizer.processImage(inputImage);
    // if (!mounted) return;
    // setState(() {
    //   _skipScanning = false;
    // });
    // if (_recognizedText != null && _recognizedText!.text.isNotEmpty) {
    //   _controller.stopImageStream();
    //   setState(() {
    //     isScanned = true;
    //   });
    // }
  }

  Future<void> _setup() async {
    _cameras = await availableCameras();
    _cameraController = CameraController(
      _cameras[0],
      ResolutionPreset.max,
      enableAudio: false,
    );

    // await _cameraController.initialize();
    await _cameraController.initialize().catchError((Object e) {
      if (e is CameraException) {
        switch (e.code) {
          case 'CameraAccessDenied':
            print('User denied camera access.');
            break;
          default:
            print('Handle other errors.');
            break;
        }
      }
    });

    if (!mounted) {
      return;
    }

    setState(() {
      _isReady = true;
    });

    await _cameraController.startImageStream(_processImage);
  }
}
