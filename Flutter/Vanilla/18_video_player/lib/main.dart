import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import 'package:video_player/video_player.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  SystemChrome.setPreferredOrientations([
    DeviceOrientation.portraitUp,
    DeviceOrientation.portraitDown,
    DeviceOrientation.landscapeLeft,
    DeviceOrientation.landscapeRight,
  ]).then((_) {
    runApp(const MainPage());
  });
}

class MainPage extends StatefulWidget {
  const MainPage({Key? key}) : super(key: key);

  @override
  State<MainPage> createState() => _MainPageState();
}

class _MainPageState extends State<MainPage> {
  late VideoPlayerController _controller;
  bool _isDefaultVideoEnded = false;

  @override
  void initState() {
    super.initState();

    // https://github.com/flutter/flutter/issues/41156
    // https://github.com/flutter/flutter/issues/72878
    // Use listner to loop vidoe instead of setLooping
    _controller = VideoPlayerController.asset("assets/852352-hd_1920_1080_30fps.mp4")
      ..initialize().then((_) {
        setState(() {});
      })
      ..setLooping(false)
      ..addListener(_onVideoEndListener)
      ..play();
  }

  @override
  void dispose() {
    _controller.removeListener(_onVideoEndListener);
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Video Demo',
      home: Scaffold(
        body: Center(
          child: _controller.value.isInitialized
              ? AspectRatio(
                  aspectRatio: _controller.value.aspectRatio,
                  child: VideoPlayer(_controller),
                )
              : Container(),
        ),
      ),
    );
  }

  Future<void> _onVideoEndListener() async {
    if (_controller.value.isInitialized &&
        !_controller.value.isBuffering &&
        !_controller.value.isPlaying &&
        _controller.value.duration <= _controller.value.position &&
        !_isDefaultVideoEnded) {
      _isDefaultVideoEnded = true;
      await _controller.pause();
      await _controller.seekTo(Duration.zero);
      await _controller.play();
      _isDefaultVideoEnded = false;
    }
  }
}
