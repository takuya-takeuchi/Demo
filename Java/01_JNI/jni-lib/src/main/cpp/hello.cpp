#include <iostream>
#include "jni_Hello.h"

JNIEXPORT jstring JNICALL Java_jni_Hello_message(JNIEnv *env, jobject thisObject)
{
  std::string str("hello, world!!");
  return env->NewStringUTF(str.c_str());
}