package jni;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class ToolsTest {

    @Test
    void testFooBar() {
        Hello hello = new Hello();
        assertEquals(hello.hello(), "hello, world!!");
    }
}