package jp.taktak.demo

import FlutterApi
import NativeApi
import FlutterError
import ProgressRequest

import androidx.annotation.NonNull
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.embedding.engine.plugins.FlutterPlugin
import io.flutter.plugin.common.BinaryMessenger

class NativeApiImplementation(binaryMessenger: BinaryMessenger): NativeApi {
  private val _flutterApi: FlutterApi
  private var _thread : Thread? = null
  private var _isRunning : Boolean = false

  init {
    _flutterApi = FlutterApi(binaryMessenger)
  }

  override fun startAsync(callback: (Result<Unit>) -> Unit) {
    if (!this._isRunning) {
      this._thread = Thread {
        this._isRunning = true
        for(i in 0..100){
          Thread.sleep(50) // 50 ms

          var requestArg = ProgressRequest((i + 1).toLong(), false)
          // this._flutterApi.sendProgressAsync(requestArg, () -> {})
        }
        this._isRunning = false
      }

      this._thread?.start()
    }

    callback(Result.success(Unit))
  }
}