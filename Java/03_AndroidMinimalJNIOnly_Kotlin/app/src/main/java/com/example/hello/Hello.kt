package com.example.hello

class Hello
{

    external fun message(): String?

    companion object
    {
        init {
            System.loadLibrary("hello")
        }
    }
}

