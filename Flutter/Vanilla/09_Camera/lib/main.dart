import 'dart:io';

import 'package:camera/camera.dart';
import 'package:exif/exif.dart';
import 'package:flutter/material.dart';
import 'package:image_size_getter/image_size_getter.dart';
import 'package:image_size_getter/file_input.dart';

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
  Image? _picture;
  double? _pictureWidth;
  double? _pictureHeight;
  int _capturePictureWidth = 0;
  int _capturePictureHeight = 0;
  bool _skipScanning = false;
  int _currentCamera = 0;

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
          child: Stack(children: [
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
                      Text(
                        "$_capturePictureWidth x $_capturePictureHeight",
                        style: const TextStyle(color: Colors.white),
                      ),
                      GestureDetector(
                        onTap: () async => await _onTakePicture(),
                        child: const Padding(
                          padding: EdgeInsets.all(padding),
                          child: Icon(
                            Icons.camera,
                            size: barHeight,
                            color: Colors.white,
                          ),
                        ),
                      ),
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
                    decoration: BoxDecoration(color: Colors.white, borderRadius: BorderRadius.circular(15), border: Border.all(color: Colors.white, width: 2)),
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

  Future<CameraController?> _initialize(
    int cameraIndex, [
    ResolutionPreset resolutionPreset = ResolutionPreset.max,
    bool enableAudio = false,
  ]) async {
    _currentCameraController = null;
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
    });

    await cameraController.startImageStream(_processImage);
    return cameraController;
  }

  Future<void> _onTakePicture() async {
    try {
      _currentCameraController?.setFlashMode(FlashMode.off);
      final picture = await _currentCameraController?.takePicture();
      if (picture?.path == null) {
        return;
      }

      // Image.File does not retrieve width and height
      final path = File(picture!.path);
      final size = ImageSizeGetter.getSize(FileInput(path));
      var ratio = size.width / size.height;
      var isLandscape = ratio > 1;
      final file = File(picture.path);
      final image = Image.file(File(picture.path));

      final exifData = await readExifFromFile(file);
      final imageOrientation = exifData['Image Orientation'];
      // 1 = Horizontal (normal) : 標準
      // 2 = Mirror horizontal   : 水平方向に反転
      // 3 = Rotate 180          : 180°回転
      // 4 = Mirror vertical     : 垂直方向に反転
      // 5 = Mirror horizontal and rotate 270 CW :反時計回りに90°回転および垂直方向に反転
      // 6 = Rotate 90 CW        : 反時計回りに90°回転
      // 7 = Mirror horizontal and rotate 90 CW  : 時計回りに90°回転および垂直方向に反転
      // 8 = Rotate 270 CW       :方向: 時計回りに90°回転
      if (imageOrientation != null && imageOrientation.values.length == 1) {
        final value = imageOrientation.values.firstAsInt();
        if (value >= 5) {
          ratio = size.height / size.width;
          isLandscape = !isLandscape;
        }
      }

      const double base = 125;
      setState(() {
        if (isLandscape) {
          _pictureWidth = base;
          _pictureHeight = base / ratio;
        } else {
          _pictureWidth = base * ratio;
          _pictureHeight = base;
        }

        _picture = image;
      });
    } catch (e) {
      // If an error occurs, log the error to the console.
      // ignore: avoid_print
      print(e);
    }
  }

  Future<void> _onSwitchCamera() async {
    _currentCamera = _currentCamera == 0 ? 1 : 0;
    _currentCameraController?.stopImageStream();
    _initialize(_currentCamera, ResolutionPreset.max, false);
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
