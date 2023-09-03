package jp.taktak.demo

import NativeApi
import MessageData
import MessageFlutterApi
import FlutterError

import androidx.annotation.NonNull
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.embedding.engine.plugins.FlutterPlugin

// #docregion kotlin-class
private class PigeonApiImplementation: NativeApi {
  override fun getPlatformVersion(): String {
    return "Android ${android.os.Build.VERSION.RELEASE}"
  }
  override fun getPlatformVersionAsync(callback: (Result<String>) -> Unit) {
    callback(Result.success(this.getPlatformVersion()))
  }
}
// #enddocregion kotlin-class

class MainActivity: FlutterActivity() {
  override fun configureFlutterEngine(@NonNull flutterEngine: FlutterEngine) {
    super.configureFlutterEngine(flutterEngine)

    val api = PigeonApiImplementation()
    NativeApi.setUp(flutterEngine.dartExecutor.binaryMessenger, api);
  }
}
