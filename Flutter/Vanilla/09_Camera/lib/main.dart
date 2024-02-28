import 'dart:io';

import 'package:flutter/material.dart';

import 'package:camera/camera.dart';

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
  String? _picturePath;
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
    // final size = MediaQuery.of(context).size;
    // final deviceRatio = size.width / size.height;

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
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                    children: [
                      GestureDetector(
                          onTap: () async => await _onSwitchCamera(),
                          child: const Icon(Icons.switch_camera, size: 44)),
                      GestureDetector(
                          onTap: () async => await _onTakePicture(),
                          child: const Icon(Icons.camera, size: 44)),
                    ],
                  )),
            ),
            Align(
              alignment: Alignment.topRight,
              child: Visibility(
                visible: _picturePath != null,
                child: Padding(
                  padding: const EdgeInsets.all(20),
                  child: Container(
                  width: 100,
                  height: 100,
                  decoration: BoxDecoration(
                      color: Colors.white,
                      borderRadius: BorderRadius.circular(15),
                      border: Border.all(color: Colors.white, width: 2)),
                  child: ClipRRect(
                    borderRadius: BorderRadius.all(Radius.circular(15)),
                    child: SizedBox(
                      width: 300,
                      height: 150,
                      child: _picturePath != null ? Image.file(File(_picturePath!)) : const Spacer(),
                    ),
                  ),),
                ),
              ),
            ),
          ]),
        ),
      ),
    );
  }

  Future<void> _onTakePicture() async {
    try {
      final image = await _cameraController.takePicture();
      setState(() {
        _picturePath = image.path;
      });
    } catch (e) {
      // If an error occurs, log the error to the console.
      print(e);
    }
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
