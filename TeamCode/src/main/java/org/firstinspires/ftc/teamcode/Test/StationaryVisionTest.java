package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Camera.Vision;
import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;
import org.firstinspires.ftc.teamcode.Components.Sample;

@Config
@TeleOp(group = "Vision", name = "StationaryVisionTest")
public class StationaryVisionTest extends OpMode {
    public static double BUFFER_TIME = 0.5;//Seconds
    public Pose2d robotPose;
    public static double poseX = 0;
    public static double poseY = 0;
    public static double poseHeading = 0;//Degrees
    Vision vision;
    @Override
    public void init() {
        vision = new Vision(hardwareMap, telemetry, true);
        robotPose = new Pose2d(poseX, poseY, poseHeading/180 * Math.PI);
    }

    static double bufferTime = 0.3;//sec to search for closest sample
    SampleColors searchColor = SampleColors.YELLOW;
    @Override
    public void loop() {
            int prioritySample = 1;
            vision.update(new Pose2d(poseX, poseY, poseHeading));

            Sample closestSample = vision.getClosestSample(searchColor, bufferTime);
            if(closestSample != null) {
                telemetry.addLine("-------------------------------------------");
                telemetry.addData("Priority(Starting From 1):", prioritySample);

                telemetry.addLine("Color: " + closestSample.getColor() + ", Confidence: " + closestSample.getConfidence());
                telemetry.addLine("(RelX, RelY): " + closestSample.getRelativeX() + ", " + closestSample.getRelativeY());
                telemetry.addLine("(FieldX, FieldY): " + closestSample.getWorldX() + ", " + closestSample.getWorldY());
            }


            for (Sample sampleDetected : vision.getSortedSamples()) {
                telemetry.addLine("-------------------------------------------");
                telemetry.addData("Priority(Starting From 1):", prioritySample);

                telemetry.addLine("Color: " + sampleDetected.getColor() + ", Confidence: " + sampleDetected.getConfidence());
                telemetry.addLine("(RelX, RelY): " +  sampleDetected.getRelativeX() + ", " + sampleDetected.getRelativeY());
                telemetry.addLine("(FieldX, FieldY): " + sampleDetected.getWorldX() + ", " + sampleDetected.getWorldY());
                prioritySample += 1;
            }


    }
}
