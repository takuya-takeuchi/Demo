package jp.taktak.demo

import NativeApi
import FlutterError

import androidx.annotation.NonNull
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.embedding.engine.plugins.FlutterPlugin

private class NativeApiImplementation: NativeApi {
  override fun getPlatformVersion(): String {
    return "Android ${android.os.Build.VERSION.RELEASE}"
  }
  override fun getPlatformVersionAsync(callback: (Result<String>) -> Unit) {
    callback(Result.success(this.getPlatformVersion()))
  }
}