import 'package:flutter/material.dart';
import 'package:video_player/video_player.dart';

Future<void> main() async {
  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: MyHomePage(),
    );
  }
}

class MyHomePage extends StatefulWidget {
  @override
  _MyHomePageState createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  late VideoPlayerController _controller;
  bool isDefaultVideoEnded = false;

  @override
  void initState() {
    super.initState();
    // _controller = VideoPlayerController.asset("assets/Nippou_Line_815_series_20210401.apng")
    // _controller = VideoPlayerController.asset("assets/Nippou_Line_815_series_20210401.webm")
    _controller = VideoPlayerController.asset("assets/big-buck-bunny_trailer.mp4")
      ..initialize().then((_) {
        // Ensure the first frame is shown after the video is initialized, even before the play button has been pressed.
        setState(() {});
      })
      ..setLooping(false)
      ..addListener(_onVideoEndListener)
      ..play();
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        body: SizedBox.expand(
          child: FittedBox(
            fit: BoxFit.cover,
            child: SizedBox(
              width: _controller.value.size.width,
              height: _controller.value.size.height,
              child: _controller.value.isInitialized
                  ? AspectRatio(
                      aspectRatio: _controller.value.aspectRatio,
                      child: VideoPlayer(_controller),
                    )
                  : Container(),
            ),
          ),
        ),
        floatingActionButton: FloatingActionButton(
          onPressed: () {
            setState(() {
              _controller.value.isPlaying ? _controller.pause() : _controller.play();
            });
          },
          child: Icon(
            _controller.value.isPlaying ? Icons.pause : Icons.play_arrow,
          ),
        ),
      ),
    );
  }

  @override
  void dispose() {
    _controller.removeListener(_onVideoEndListener);
    _controller.dispose();
    super.dispose();
  }

  Future<void> _onVideoEndListener() async {
    if (_controller.value.isInitialized &&
        !_controller.value.isBuffering &&
        !_controller.value.isPlaying &&
        _controller.value.duration <= _controller.value.position &&
        !isDefaultVideoEnded) {
      // https://github.com/flutter/flutter/issues/41156
      // Refer: https://zenn.dev/fbd_tech/scraps/7b400670215ee0
      // Do NOT use setLooping!!
      isDefaultVideoEnded = true;
      await _controller.pause();
      await _controller.seekTo(Duration.zero);
      await _controller.play();
      isDefaultVideoEnded = false;
    }
  }
}
