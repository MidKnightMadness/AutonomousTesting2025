package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Camera.Vision;
import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;
import org.firstinspires.ftc.teamcode.Components.Sample;

@Config
@TeleOp(name = "StationaryVisionTest")
public class StationaryVisionTest extends OpMode {
    public static double BUFFER_TIME = 0.5;//Seconds
    public Pose2d pose;
    public static double poseX;
    public static double poseY;
    public static double poseHeading;
    Vision vision;
    @Override
    public void init() {
        vision = new Vision(hardwareMap, telemetry, true);
        pose = new Pose2d(poseX, poseY, poseHeading);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            vision.update(pose.position.x, pose.position.y, pose.heading.toDouble());
//            //Get closest sample of fov
            Sample sampleDetected = vision.getClosestSample(SampleColors.YELLOW, BUFFER_TIME);

            telemetry.addData("Closest Sample Color:", sampleDetected.getColor());
            telemetry.addData("Closest Sample Confidence", sampleDetected.getConfidence());
            telemetry.addData("Closest Sample X", sampleDetected.getRelativeX());
            telemetry.addData("Closest Sample Y", sampleDetected.getRelativeY());
            telemetry.addData("Closest Sample Rot", sampleDetected.getSampleRotation());

        }

    }
}
