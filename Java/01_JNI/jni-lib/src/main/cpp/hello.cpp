#include <iostream>
#include <jni.h>

JNIEXPORT jstring JNICALL Java_jni_Tools_hello(JNIEnv *env, jobject thisObject)
{
  std::string message("hello, world!!");
  return env->NewStringUTF(message.c_str());
}