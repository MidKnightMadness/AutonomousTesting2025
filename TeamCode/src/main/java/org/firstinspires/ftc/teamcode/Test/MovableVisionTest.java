package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Camera.Vision;
import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;
import org.firstinspires.ftc.teamcode.Components.Sample;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RunOptions;

import java.util.List;

@Config
@TeleOp(group = "Vision", name = "MovableVisionTest")
public class MovableVisionTest extends OpMode {
    public static double BUFFER_TIME = 0.5;//Seconds
    public static double poseX = 0;
    public static double poseY = 0;
    public static double poseHeading = 0;//Degrees
    Vision vision;
    MecanumDrive mecanumDrive;
    List<LynxModule> allHubs;
    Timer timer;
    public Pose2d initialPose = new Pose2d(poseX, poseY, Math.toRadians(poseHeading));
    public Pose2d currentRobotPose = new Pose2d(poseX, poseY, Math.toRadians(poseHeading));
    double drivingPower = 0.5;
    public static double rotationFactor = 0.5;
    public static double STRAFE_ROTATION_FACTOR = 0.1; // add rotation while strafing to counteract uneven rotation

    SampleColors searchColor = SampleColors.YELLOW;
    //Rising edge detector
    boolean previousGamepad1A;
    @Override
    public void init() {
        timer = new Timer();

        mecanumDrive = new MecanumDrive(hardwareMap, initialPose);
        vision = new Vision(hardwareMap, telemetry, true);

        MecanumDrive.PARAMS.maxWheelVel = 60;

        if (RunOptions.useBulkReads) {
            allHubs = hardwareMap.getAll(LynxModule.class);

            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        }
    }

    public static double bufferTime = 0.2;//time(sec) allowed to determine closest sample from samples list
    @Override
    public void loop() {
        if (RunOptions.useBulkReads) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
        }
        timer.updateTime();
        telemetry.addData("Update Rate (Hz)", 1 / timer.getDeltaTime());


        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_x * drivingPower,
                        -gamepad1.left_stick_y * drivingPower
                ),
                (-gamepad1.right_stick_x * rotationFactor + gamepad1.left_stick_x * STRAFE_ROTATION_FACTOR) * drivingPower
        ));

        currentRobotPose = mecanumDrive.localizer.getPose();

        telemetry.addLine("CurrentRobotPose: (" + currentRobotPose.position.x + ", " + currentRobotPose.position.y + "), " +currentRobotPose.heading);

        telemetry.addLine("-------------------------------------------");
        if(gamepad1.a && !previousGamepad1A) {
            int prioritySample = 1;
            vision.update(mecanumDrive.localizer.getPose());

            Sample closestSample = vision.getClosestSample(searchColor, bufferTime);
            if(closestSample != null) {
                telemetry.addLine("-------------------------------------------");
                telemetry.addData("Priority(Starting From 1):", prioritySample);

                telemetry.addLine("Color: " + closestSample.getColor() + ", Confidence: " + closestSample.getConfidence());
                telemetry.addLine("(RelX, RelY): " + closestSample.getRelativeX() + ", " + closestSample.getRelativeY());
//                telemetry.addLine("(FieldX, FieldY): " + closestSample.getWorldX() + ", " + closestSample.getWorldY());
                telemetry.addData("Distance", closestSample.getSampleDistance());
                telemetry.addData("Sample Angle", closestSample.getSampleTheta() * 180/ Math.PI);

            }
            for (Sample sampleDetected : vision.getSortedSamples()) {
                telemetry.addLine("-------------------------------------------");
                telemetry.addData("Priority(Starting From 1):", prioritySample);

                telemetry.addLine("Color: " + sampleDetected.getColor() + ", Confidence: " + sampleDetected.getConfidence());
                telemetry.addLine("(RelX, RelY): " +  sampleDetected.getRelativeX() + ", " + sampleDetected.getRelativeY());
//                telemetry.addLine("(FieldX, FieldY): " + sampleDetected.getWorldX() + ", " + sampleDetected.getWorldY());
                telemetry.addData("Distance", closestSample.getSampleDistance());
                telemetry.addData("Sample Angle", closestSample.getSampleTheta() * 180/ Math.PI);
                prioritySample += 1;
            }
            previousGamepad1A = true;
        }

        if(!gamepad1.a){
            previousGamepad1A = false;
        }

        telemetry.update();
    }
}
