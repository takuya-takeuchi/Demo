package com.example.hello

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import com.example.hello.databinding.ActivityHelloBinding

class Hello : AppCompatActivity()
{

    override fun onCreate(savedInstanceState: Bundle?)
    {
        super.onCreate(savedInstanceState)
        val binding = ActivityHelloBinding.inflate(layoutInflater)
        setContentView(binding.root)
        binding.helloTextview.text = message()
    }

    external fun message(): String?

    companion object
    {
        init {
            System.loadLibrary("hello")
        }
    }
}

