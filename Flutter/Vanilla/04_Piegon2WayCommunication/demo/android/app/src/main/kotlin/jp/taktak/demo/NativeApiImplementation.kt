package jp.taktak.demo

import FlutterApi
import NativeApi
import FlutterError

import androidx.annotation.NonNull
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.embedding.engine.plugins.FlutterPlugin
import io.flutter.plugin.common.BinaryMessenger

class NativeApiImplementation(binaryMessenger: BinaryMessenger): NativeApi {
  val _FlutterApi: FlutterApi;
  init {
    _FlutterApi = FlutterApi(binaryMessenger)
  }

  override fun startAsync(callback: (Result<Unit>) -> Unit) {
    callback(Result.success(Unit))
  }
}