import 'dart:io';

import 'package:flutter/material.dart';
import 'package:image_size_getter/image_size_getter.dart';
import 'package:image_size_getter/file_input.dart';
import 'package:native_device_orientation/native_device_orientation.dart';

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
  Image? _picture;
  double? _pictureWidth;
  double? _pictureHeight;
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
                visible: _picture != null,
                child: Padding(
                  padding: const EdgeInsets.all(20),
                  child: Container(
                    width: _pictureWidth,
                    height: _pictureHeight,
                    decoration: BoxDecoration(
                        color: Colors.white,
                        borderRadius: BorderRadius.circular(15),
                        border: Border.all(color: Colors.white, width: 2)),
                    child: ClipRRect(
                      borderRadius: const BorderRadius.all(Radius.circular(15)),
                      child: SizedBox(
                        width: _pictureWidth,
                        height: _pictureHeight,
                        child: _picture ?? const Spacer(),
                      ),
                    ),
                  ),
                ),
              ),
            ),
          ]),
        ),
      ),
    );
  }

  Future<void> _initialize(
    int cameraIndex, [
    ResolutionPreset resolutionPreset = ResolutionPreset.max,
    bool enableAudio = false,
  ]) async {
    final cameraController = CameraController(
      _cameras[cameraIndex],
      resolutionPreset,
      enableAudio: enableAudio,
    );

    // await _cameraController.initialize();
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

    if (!mounted) {
      return;
    }

    setState(() {
      _isReady = true;
      _cameraController = cameraController;
    });

    await cameraController.startImageStream(_processImage);
  }

  Future<void> _onTakePicture() async {
    try {
      final picture = await _cameraController.takePicture();

      // Image.File does not retrieve width and height
      final path = File(picture.path);
      final size = ImageSizeGetter.getSize(FileInput(path));
      final ratio = size.width / size.height;
      final image = Image.file(File(picture.path));
      const double base = 125;
      final orientation = await NativeDeviceOrientationCommunicator()
          .orientation(useSensor: false);
      setState(() {
        switch (orientation) {
          case NativeDeviceOrientation.portraitUp:
            _pictureWidth = base;
            _pictureHeight = base * ratio;
            break;
          case NativeDeviceOrientation.portraitDown:
            _pictureWidth = base;
            _pictureHeight = base * ratio;
            break;
          case NativeDeviceOrientation.landscapeLeft:
            _pictureWidth = base * ratio;
            _pictureHeight = base;
            break;
          case NativeDeviceOrientation.landscapeRight:
            _pictureWidth = base * ratio;
            _pictureHeight = base;
            break;
          case NativeDeviceOrientation.unknown:
            _pictureWidth = base * ratio;
            _pictureHeight = base;
            break;
        }

        _picture = image;
      });
    } catch (e) {
      // If an error occurs, log the error to the console.
      print(e);
    }
  }

  Future<void> _onSwitchCamera() async {
    _currentCamera = _currentCamera == 0 ? 1 : 0;
    _cameraController.stopImageStream();
    _initialize(_currentCamera, ResolutionPreset.max, false);
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
    _initialize(0, ResolutionPreset.max, false);
  }
}
