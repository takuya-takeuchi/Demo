package jni;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class HelloTest {

    @Test
    void testHello() {
        Hello hello = new Hello();
        assertEquals(hello.message(), "hello, world!!");
    }
}