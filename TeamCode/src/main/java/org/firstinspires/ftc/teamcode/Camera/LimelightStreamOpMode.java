package org.firstinspires.ftc.teamcode.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

import java.io.IOException;

@TeleOp(group = "Vision", name = "Limelight Stream")
public class LimelightStreamOpMode extends OpMode {
    boolean streamStarted = false;

    @Override
    public void init() {
        try{
            CameraStreamSource streamSource = new LimelightSourceStream();
            FtcDashboard.getInstance().startCameraStream(streamSource, 15);
            streamStarted = true;
        } catch (IOException e) {
            telemetry.addData("Error Message", e.toString());
        }
    }

    @Override
    public void loop() {
       if(streamStarted){
           telemetry.addLine("Stream Started");
       }
       else{
           telemetry.addLine("Stream Not Started");
       }
    }
}
