import 'package:flutter/material.dart';
import 'package:flutter_vlc_player/flutter_vlc_player.dart';

Future<void> main() async {
  runApp(const MainPage());
}

class MainPage extends StatefulWidget {
  const MainPage({Key? key}) : super(key: key);

  @override
  State<MainPage> createState() => _MainPageState();
}

class _MainPageState extends State<MainPage> {
  late Future<VlcPlayerController?> _videoPlayerController;

  @override
  void initState() {
    super.initState();
    _videoPlayerController = _setup();
  }

  @override
  void dispose() async {
    final controller = await _videoPlayerController;
    await controller?.stopRendererScanning();
    await controller?.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        body: Center(
          child: Stack(children: [
            Container(
              constraints: const BoxConstraints.expand(),
              child: FutureBuilder(
                future: _videoPlayerController,
                builder: (BuildContext context, AsyncSnapshot<VlcPlayerController?> snapshot) {
                  if (snapshot.connectionState == ConnectionState.done && snapshot.data != null) {
                    return VlcPlayer(
                      controller: snapshot.data!,
                      aspectRatio: 16 / 9,
                      placeholder: const Center(child: CircularProgressIndicator()),
                    );
                  }
                  return const Spacer();
                },
              ),
            ),
          ]),
        ),
      ),
    );
  }

  Future<VlcPlayerController?> _setup() async {
    return VlcPlayerController.network(
      'https://media.w3.org/2010/05/sintel/trailer.mp4',
      // 'http://root:password@192.168.111.111/axis-cgi/mjpg/video.cgi',
      // 'rtsp://root:password@192.168.111.111/axis-media/media.amp?videocodec=h264',
      hwAcc: HwAcc.full,
      autoPlay: true,
      options: VlcPlayerOptions(),
    );
  }
}
