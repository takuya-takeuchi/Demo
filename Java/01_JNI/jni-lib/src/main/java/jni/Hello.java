package jni;

public class Hello {
    static {
        if (!LibraryLoader.load(Hello.class, "hello"))
            System.loadLibrary("hello");
    }

    public native String message();
}