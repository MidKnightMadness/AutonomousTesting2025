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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ColorSensor.ColorClassifier;
import org.firstinspires.ftc.teamcode.ColorSensor.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.ColorSensor.RGB;
import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;
import org.firstinspires.ftc.teamcode.Components.ButtonToggle;
import org.firstinspires.ftc.teamcode.Kinematics.InverseKinematics;
import org.firstinspires.ftc.teamcode.Kinematics.InverseKinematics.IKResult;
import org.firstinspires.ftc.teamcode.Kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.Mechanisms.TurnTable;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;

import java.util.ArrayList;
import java.util.List;


//Start at the left of the 2nd tile
@Config
@Autonomous(name = "FiveSampleAuto")
public class FiveSampleAuto extends FourSampleAuto {

    List<SamplePose> samplePositions;

    Timer timer;

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

    SampleColors allianceColor = SampleColors.BLUE;

    // sample 27.5” x 42.75”

    public static double coordinateInputSpeed = 1;
    public static double headingInputSpeed = 0.1;

    ColorSensorWrapper colorSensorWrapper;
    @Override
    public void init() {
        super.init();
        samplePositions = new ArrayList<>();
        samplePositions.add(new SamplePose(0, 3, Math.toRadians(-90)));
        colorSensorWrapper = new ColorSensorWrapper(hardwareMap, allianceColor);
        timer = new Timer();
    }

    int inputIndex = 0;

    public static Vector2d sampleOffset = new Vector2d(64.5, -32);
    List<IKResult> results = new ArrayList<>();

    @SuppressLint("DefaultLocale")
    @Override
    public void init_loop() {
        timer.updateTime();

        leftBump = new ButtonToggle();
        rightBump = new ButtonToggle();

        if (gamepad1.left_bumper) allianceColor = SampleColors.BLUE;
        if (gamepad1.right_bumper) allianceColor = SampleColors.RED;

        telemetry.addData("Current Alliance Color: ", allianceColor);

        if (gamepad1.x) mecanumDrive.otos.calibrateImu();
        if (gamepad1.y) mecanumDrive.otos.resetTracking();

        telemetry.addData("OTOS heading", mecanumDrive.otos.getPosition().h);

        if (leftBump.update(gamepad1.left_bumper)) {
            inputIndex++;
            try { Thread.sleep(200); } catch (InterruptedException e) { }
        }

        if (rightBump.update(gamepad1.right_bumper)) {
            if (inputIndex > 0) inputIndex--;
            try { Thread.sleep(200); } catch (InterruptedException e) { }
        }

        if (inputIndex > samplePositions.size() - 1) {
            samplePositions.add(new SamplePose());
        }

        SamplePose activeSample = samplePositions.get(inputIndex);

        activeSample.x += -gamepad1.left_stick_y * timer.getDeltaTime() * coordinateInputSpeed;
        activeSample.y += -gamepad1.left_stick_x * timer.getDeltaTime() * coordinateInputSpeed;
        activeSample.heading += -gamepad1.right_stick_y * timer.getDeltaTime() * headingInputSpeed;

        samplePositions.set(inputIndex, new SamplePose(activeSample.x, activeSample.y, activeSample.heading));
        telemetry.addData("Active sample", inputIndex + 1);
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

    int currentIndex = 0;
    
    public void subPickup() {
        IKResult ikResult = results.get(currentIndex);

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        resetAfterScoring(),
                        slides.bringDown(1),
                        mecanumDrive.actionBuilder()
                                .strafeToLinearHeading(submersibleIntermediatePose.position, submersibleIntermediatePose.heading)
                                .strafeToLinearHeading(ikResult.robotPose.position, ikResult.robotPose.heading)
                                .build(),
                        new SequentialAction(
                                new SleepAction(0.5),
                                arm.setPositionSmooth(Arm.SCANNING_POSITION),
                                turnTable.setPosition(ikResult.turntablePosition)
                        )
                )
        ));

        boolean pickedUp = pickUpAndCheck();

        if (pickedUp) {
            Actions.runBlocking(new ParallelAction(
                    mecanumDrive.actionBuilder()
                            .strafeToSplineHeading(submersibleIntermediatePose.position, scoringPose.heading)
                            .strafeToSplineHeading(new Vector2d(scoringPose.position.x, scoringPose.position.y), scoringPose.heading)
                            .build(),

                    new SequentialAction(
                            // change the below to run when the position passes the intermediate position instead of a hardcoded wait time
                            new SleepAction(1),
                            new ParallelAction(
                                    arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION),
                                    slides.liftUp(1),
                                    wrist.setPosition(Wrist.BASKET_POSITION),
                                    turnTable.setPositionSmooth(TurnTable.NEUTRAL_POS, 0.5)
                            ),
                            arm.setPositionSmooth(Arm.BASKET_POSITION),
                            sampleClaw.releaseAction(0.2)
                    )
            ));
            parkFromBasket();
        }
        else {
            sampleClaw.release();
            subZonePark();
        }
    }

    public boolean pickUpAndCheck() {
        Actions.runBlocking(
                new SequentialAction(
                    new ParallelAction(
                            arm.setPositionSmooth(Arm.SAMPLE_INTAKE),
                            sampleClaw.releaseAction(0.5)
                    ),
                    new SleepAction(0.5),
                    sampleClaw.grabAction(0.5),
                    arm.setPositionSmooth(Arm.PERPENDICULAR),
                    new SleepAction(0.5)
                )
        );

        return colorSensorWrapper.isSamplePickedUp(5);
    }

    @Override
    public void start() {
        getPickupResults();
        telemetry.setAutoClear(false);
        timer.restart();

        Actions.runBlocking(
                scoreInBasket(0, 0)
        );

        firstLineSample();
        secondLineSample();
        thirdLineSample();

        sampleClaw.release();

        subPickup();
    }

    public void subZonePark() {
        mecanumDrive.actionBuilder().build();
        Actions.runBlocking(new SequentialAction(
                arm.setPositionSmooth(Arm.PERPENDICULAR),
                new ParallelAction(
                        mecanumDrive.actionBuilder()
                                .strafeToLinearHeading(submersibleIntermediatePose.position, submersibleIntermediatePose.heading)
                                .strafeToLinearHeading(parkingPose.position, parkingPose.heading)
                                .build(),
                        new SequentialAction(
                                new SleepAction(1),
                                arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION)
                        )
                ),
                arm.setPositionSmooth(Arm.ARM_TO_BAR)
        ));

        arm.leftServo.getController().pwmDisable();
        arm.rightServo.getController().pwmDisable();
    }


    @Override
    public void loop() { }
}

