#include <jni.h>
#include <string>

extern "C" JNIEXPORT jstring JNICALL Java_com_example_hello_Hello_message(JNIEnv* env, jobject /* this */)
{
  std::string str("hello, world!!");
  return env->NewStringUTF(str.c_str());
}