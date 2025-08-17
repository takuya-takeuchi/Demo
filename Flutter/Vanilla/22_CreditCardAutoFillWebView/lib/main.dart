import 'package:flutter/material.dart';
import 'package:webview_flutter/webview_flutter.dart';
import 'package:url_launcher/url_launcher.dart';

void main() => runApp(const MyApp());

class MyApp extends StatelessWidget {
  const MyApp({super.key});
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Credit Card Autofill Test',
      theme: ThemeData(useMaterial3: true),
      home: const HomePage(),
    );
  }
}

class HomePage extends StatefulWidget {
  const HomePage({super.key});
  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  // あなたのテストページ（例： https://example.com/autofill/ ）
  final _urlCtrl = TextEditingController(text: 'https://webapp.taktak.uk/cc-autofill/');

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Credit Card Autofill Test')),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            const SizedBox(height: 12),
            Row(
              children: [
                Expanded(
                  child: FilledButton(
                    onPressed: () {
                      final uri = Uri.tryParse(_urlCtrl.text);
                      if (uri == null) return;
                      Navigator.of(context).push(MaterialPageRoute(
                        builder: (_) => WebViewPage(initialUri: uri),
                      ));
                    },
                    child: const Text('Open in WebView'),
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: OutlinedButton(
                    onPressed: () async {
                      final uri = Uri.tryParse(_urlCtrl.text);
                      if (uri == null) return;
                      // iOS: SFSafariViewController / Android: Chrome Custom Tabs
                      await launchUrl(
                        uri,
                        mode: LaunchMode.inAppBrowserView,
                        webOnlyWindowName: '_blank',
                      );
                    },
                    child: const Text('Open in In-App Browser (Safari/Chrome Tab)'),
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}

class WebViewPage extends StatefulWidget {
  const WebViewPage({super.key, required this.initialUri});
  final Uri initialUri;

  @override
  State<WebViewPage> createState() => _WebViewPageState();
}

class _WebViewPageState extends State<WebViewPage> {
  late final WebViewController _controller;

  @override
  void initState() {
    super.initState();
    _controller = WebViewController()
      ..setJavaScriptMode(JavaScriptMode.unrestricted)
      ..setBackgroundColor(const Color(0x00000000))
      ..setNavigationDelegate(NavigationDelegate(
        onNavigationRequest: (req) => NavigationDecision.navigate,
      ))
      ..loadRequest(widget.initialUri);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('WebView')),
      body: SafeArea(child: WebViewWidget(controller: _controller)),
    );
  }
}
