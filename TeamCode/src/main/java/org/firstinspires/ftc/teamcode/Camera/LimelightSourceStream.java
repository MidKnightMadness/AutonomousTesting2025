package org.firstinspires.ftc.teamcode.Camera;

import android.graphics.Bitmap;

import androidx.core.view.WindowInsetsAnimationCompat;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;


import java.io.IOException;

import javax.xml.transform.Source;

@Deprecated
public class LimelightSourceStream implements CameraStreamSource {

    private MJPEGStreamReader streamReader;
    private final String url = "http://172.29.0.2:5800/stream.mjpg";
    public LimelightSourceStream() throws IOException {
        streamReader = new MJPEGStreamReader(url);
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        try {
            Bitmap frame = streamReader.readFrame();
            continuation.dispatch(c -> c.accept(frame));
        } catch (IOException e) {
            // Handle failed frame fetch
            continuation.dispatch(c -> c.accept(null));
        }
    }

}


