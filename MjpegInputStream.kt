package com.example.shuttlebot

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import java.io.BufferedInputStream
import java.io.InputStream

class MjpegInputStream(private val inputStream: InputStream) {
    private val reader = BufferedInputStream(inputStream)
    private var lastByte = 0

    fun readFrame(): Bitmap? {
        try {
            while (true) {
                val cur = reader.read()
                if (cur < 0) return null
                if (lastByte == 0xFF && cur == 0xD8) break
                lastByte = cur
            }

            val baos = java.io.ByteArrayOutputStream()
            baos.write(0xFF)
            baos.write(0xD8)

            while (true) {
                val b = reader.read()
                if (b < 0) break
                baos.write(b)
                if (lastByte == 0xFF && b == 0xD9) break
                lastByte = b
            }

            return BitmapFactory.decodeByteArray(baos.toByteArray(), 0, baos.size())
        } catch (e: Exception) {
            return null
        }
    }
}
