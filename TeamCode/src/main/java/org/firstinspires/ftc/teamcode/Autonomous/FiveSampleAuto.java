package org.firstinspires.ftc.teamcode.Autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Components.ButtonToggle;
import org.firstinspires.ftc.teamcode.Kinematics.InverseKinematics;
import org.firstinspires.ftc.teamcode.Kinematics.InverseKinematics.IKResult;
import org.firstinspires.ftc.teamcode.Kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Components.Timer;

import java.util.ArrayList;
import java.util.List;


//Start at the left of the 2nd tile
@Config
@Autonomous(name = "FiveSampleAuto")
public class FiveSampleAuto extends FourSampleAuto {

    List<SamplePose> samplePositions;

    Timer timer;
    RevColorSensorV3 clawColorSensor;

    public static class SamplePose {
        public double x;
        public double y;
        public double heading;

        public SamplePose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        public SamplePose() { }

        public String positionString() {
            return String.format("(%.2f, %.2f)", x, y);
        }

        public void incrementX(double x) { this.x += x; }
        public void incrementY(double y) { this.y += y; }
        public void incrementHeading(double h) {this.heading += h; }
        public Pose2d toPose2d() {return new Pose2d(x, y, heading); }
    }

    ButtonToggle leftBump;
    ButtonToggle rightBump;

    // sample 27.5” x 42.75”

    public static double coordinateInputSpeed = 1;
    public static double headingInputSpeed = 0.1;

    @Override
    public void init() {
        super.init();
        samplePositions = new ArrayList<>();
        samplePositions.add(new SamplePose(0, 0, Math.toRadians(-90)));
        clawColorSensor = hardwareMap.get(RevColorSensorV3.class, "Claw Color Sensor");
        timer = new Timer();
    }

    int index = 0;

    Vector2d sampleOffset = new Vector2d(55, -32);
    List<IKResult> results = new ArrayList<>();

    @SuppressLint("DefaultLocale")
    @Override
    public void init_loop() {
        timer.updateTime();

        leftBump = new ButtonToggle();
        rightBump = new ButtonToggle();

        if (gamepad1.x) mecanumDrive.otos.calibrateImu();
        if (gamepad1.y) mecanumDrive.otos.resetTracking();

        telemetry.addData("OTOS heading", mecanumDrive.otos.getPosition().h);

        if (leftBump.update(gamepad1.left_bumper)) {
            index++;
            try { Thread.sleep(200); } catch (InterruptedException e) { }
        }

        if (rightBump.update(gamepad1.right_bumper)) {
            if (index > 0) index--;
            try { Thread.sleep(200); } catch (InterruptedException e) { }
        }

        if (index > samplePositions.size() - 1) {
            samplePositions.add(new SamplePose());
        }

        SamplePose activeSample = samplePositions.get(index);

        activeSample.x += -gamepad1.left_stick_y * timer.getDeltaTime() * coordinateInputSpeed;
        activeSample.y += -gamepad1.left_stick_x * timer.getDeltaTime() * coordinateInputSpeed;
        activeSample.heading += -gamepad1.right_stick_y * timer.getDeltaTime() * headingInputSpeed;

        samplePositions.set(index, new SamplePose(activeSample.x, activeSample.y, activeSample.heading));
        telemetry.addData("Active sample", index + 1);
        telemetry.addLine();

        for (int i = 0; i < samplePositions.size(); i++) {
            telemetry.addLine("Sample #" + (i + 1));
            telemetry.addData("Sample position", samplePositions.get(i).positionString());
            telemetry.addData("Sample heading", Math.toDegrees(samplePositions.get(i).heading));
        }

        if (gamepad1.b) {
            telemetry.addData("Sample field coordinates", String.format("(%f, %f)", activeSample.x + sampleOffset.x, activeSample.y + sampleOffset.y));
            InverseKinematics.IKResult result = InverseKinematics.solve(new Pose2d(activeSample.x + sampleOffset.x, activeSample.y + sampleOffset.y, activeSample.heading));
            telemetry.addData("Is Position Reachable", result.isReachable);

            if (result.isReachable) {
                telemetry.addData("Robot position", result.robotPose.position);
                telemetry.addData("Robot heading", Math.toDegrees(result.robotPose.heading.toDouble()));
                telemetry.addData("Turntable position", result.turntablePosition);
                telemetry.addData("Turntable rotation", Math.toDegrees(Kinematics.turnTablePositionToOrientation(result.turntablePosition)));
            } else {
                telemetry.addLine(result.message);
            }
        }

        telemetry.update();
    }

    void getPickupResults() {
        for (SamplePose samplePose : samplePositions) {
            IKResult result = InverseKinematics.solve(new Pose2d(samplePose.x + sampleOffset.x, samplePose.y + sampleOffset.y, samplePose.heading));
            if (result.isReachable) results.add(result);
        }
    }

    public void subPickup() {
        IKResult firstSample = results.get(0);

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        resetAfterScoring(),
                        slides.bringDown(1),
                        mecanumDrive.actionBuilder()
                                .splineTo(submersibleIntermediatePose.position, submersibleIntermediatePose.heading)
                                .splineTo(firstSample.robotPose.position, firstSample.robotPose.heading)
                                .build(),
                        new SequentialAction(
                                new SleepAction(0.5),
                                arm.setPositionSmooth(Arm.SCANNING_POSITION),
                                turnTable.setPosition(firstSample.turntablePosition)
                        )
                ),
                manipulatorPickUp(),
                mecanumDrive.actionBuilder(firstSample.robotPose)
                        .splineTo(submersibleIntermediatePose.position, submersibleIntermediatePose.heading)
                        .build()
        ));
    }

    @Override
    public void start() {
        getPickupResults();

        Actions.runBlocking(
                scoreInBasket(0, 0)
        );

        firstLineSample();
        secondLineSample();
        thirdLineSample();

        subPickup();
    }

    @Override
    public void loop() { }
}

