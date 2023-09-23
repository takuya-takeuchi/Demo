package jp.taktak.demo

import androidx.annotation.NonNull
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine

import jp.taktak.demo.NativeApiImplementation

class MainActivity: FlutterActivity() {
  override fun configureFlutterEngine(@NonNull flutterEngine: FlutterEngine) {
    super.configureFlutterEngine(flutterEngine)

    val api = NativeApiImplementation(flutterEngine.dartExecutor.binaryMessenger)
    NativeApi.setUp(flutterEngine.dartExecutor.binaryMessenger, api);
  }
}
