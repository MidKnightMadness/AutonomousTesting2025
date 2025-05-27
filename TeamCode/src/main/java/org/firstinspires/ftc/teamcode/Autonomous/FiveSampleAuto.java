package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;
import org.firstinspires.ftc.teamcode.Components.ButtonToggle;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.Mechanisms.VerticalSlides;

import java.util.ArrayList;
import java.util.List;

@Autonomous
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

    @Override
    public void init() {
        super.init();
        samplePositions = new ArrayList<>();
        samplePositions.add(new SamplePose(0, 3, Math.toRadians(-90)));
        timer = new Timer();
    }

    int inputIndex = 0;

    @Override
    public void init_loop() {
        timer.updateTime();

        leftBump = new ButtonToggle();
        rightBump = new ButtonToggle();

        if (gamepad1.left_bumper) allianceColor = SampleColors.BLUE;
        if (gamepad1.right_bumper) allianceColor = SampleColors.RED;

        telemetry.addData("Current Alliance Color: ", allianceColor);

        if (leftBump.update(gamepad1.left_bumper)) {
            inputIndex++;
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
            }
        }

        if (rightBump.update(gamepad1.right_bumper)) {
            if (inputIndex > 0) inputIndex--;
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
            }
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

        telemetry.update();

    }

    @Override
    public void start() {
        mecanumDrive.localizer.setPose(startingPose);
        verticalSlides.resetEncoders();
        lastPose = new EditablePose(startingPose.position.x, startingPose.position.y, Math.toDegrees(startingPose.heading.toDouble()));
        Actions.runBlocking(
                new SequentialAction(
                        score(new Vector2d(0, 0)),
                        resetAfterCycle(rightSamplePickup),
                        pickUp(RIGHT.pivExtension),
                        score(new Vector2d(0, 0)),
                        resetAfterCycle(middleSamplePickup),
                        pickUp(MIDDLE.pivExtension),
                        score(new Vector2d(0, 0)),
                        resetAfterCycle(leftSamplePickup),
                        pickUp(LEFT.pivExtension),
                        score(new Vector2d(1, 1)),
                        resetAfterCycle(submersibleIntermediate),
                        park()
                ));
    }

    public Action park() {
        return new SequentialAction(
                mecanumDrive.actionBuilder(lastPose.pose2d).strafeToLinearHeading(parkPose.vector2d, parkPose.heading).build(),
                new InstantAction(() -> arm.setAngle(70)),
                new SleepAction(0.5),
                new InstantAction(() -> {
                    arm.leftServo.getController().pwmDisable();
                    arm.rightServo.getController().pwmDisable();
                })
        );
    }

    public Action resetAfterCycle(EditablePose targetPose) {
        EditablePose oldPose = lastPose;
        lastPose = targetPose;

        return new SequentialAction(
                wrist.setDegreesAction(0),
                pivotingSlides.setExtensionAction(RESET.pivExtension),
                new ParallelAction(
                        arm.setAngleSmooth(CYCLE.armDeg),
                        verticalSlides.bringDown(RESET.downPower),
                        new SequentialAction(
                                mecanumDrive.actionBuilder(oldPose.pose2d).strafeToLinearHeading(targetPose.vector2d, targetPose.heading).build()
                        )
                )
        );
    }

    public Action pickUp(double pivExtension) {
        return new ParallelAction(
                new SequentialAction(
                        new ParallelAction(
                                wrist.setDegreesAction(PICKUP.wristDeg),
                                pivotingSlides.setExtensionAction(pivExtension),
                                arm.setAngleSmooth(PICKUP.armPrepDeg)
                        ),
                        new SleepAction(0.5),
                        new ParallelAction(
                                spintake.intake(),
                                arm.setAngleSmooth(PICKUP.armPickupDeg)
                        )
                )
        );
    }

    public Action score(Vector2d offset) {
        EditablePose oldPose = lastPose;
        lastPose = basketScoring;

        return new SequentialAction(
                new ParallelAction(
                        pivotingSlides.setExtensionAction(30),
                        arm.setAngleSmooth(CYCLE.armDeg)
                ),
                new ParallelAction(
                        mecanumDrive.actionBuilder(oldPose.pose2d).strafeToLinearHeading(basketScoring.vector2d.plus(offset), basketScoring.heading).build(),
                        verticalSlides.setPosition(VerticalSlides.BASKET_BACKWARDS_SCORING, 1)
                ),
                pivotingSlides.setExtensionAction(CYCLE.pivExtension),
                new SleepAction(0.2),
                wrist.setDegreesAction(CYCLE.wristScoreDeg),
                new SleepAction(0.3),
                spintake.outtake()
        );
    }


}
