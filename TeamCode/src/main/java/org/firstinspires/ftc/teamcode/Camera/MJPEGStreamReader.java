package org.firstinspires.ftc.teamcode.Camera;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;

public class MJPEGStreamReader {
    private InputStream stream;//to read raw bytes

    public MJPEGStreamReader(String urlStr) throws IOException {
        URL url = new URL(urlStr);
        stream = url.openConnection().getInputStream();//reading raw bytes from wifi connection
    }

    public Bitmap readFrame() throws IOException {//decodes one JPEG file
        // Search for JPEG end (0xFFD9)
        ByteArrayOutputStream jpegBuffer = new ByteArrayOutputStream();
        int prev = 0;
        int curr = 0;
        while ((curr = stream.read()) != -1) {
            jpegBuffer.write(curr);
            if (prev == 0xFF && curr == 0xD9) break; // End of JPEG
            prev = curr;
        }

        byte[] jpegData = jpegBuffer.toByteArray();
        return BitmapFactory.decodeByteArray(jpegData, 0, jpegData.length);//convert jpeg byte array to bitmap
    }
}