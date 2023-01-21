package jni;

public class Runner {
    public static void main(String[] args) {
        Hello hello = new Hello();
        System.out.println(hello.message());
    }
}