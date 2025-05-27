package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.CRArm;
import org.firstinspires.ftc.teamcode.Mechanisms.PivotingSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.Spintake;
import org.firstinspires.ftc.teamcode.Mechanisms.VerticalSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;

@Autonomous
@Config
@Configurable
public class FourSampleAuto extends OpMode {

    public static EditablePose preloadScoring = new EditablePose(-54, 54, 135);
    public static EditablePose basketScoring = new EditablePose(-54, 54, -45);
    public static EditablePose rightSamplePickup = new EditablePose(-53, 53, -7);
    public static EditablePose middleSamplePickup = new EditablePose(-54, 58, 5);
    public static EditablePose leftSamplePickup = new EditablePose(-49, 55, 30);
    public static EditablePose submersibleIntermediate = new EditablePose(-9.5, 32, -90);
    public static EditablePose parkPose = new EditablePose(-9.5, 29, -90);
    public static class Reset {
        public double downPower = 0.8;
        public double pivExtension = 25;
        public double armDeg = 51;
    }

    public static class Pickup {
        public double wristDeg = 95;
        public double pivExtension = 85;
        public double armPrepDeg = 135;
        public double armPickupDeg = 143;
    }

    public static class LeftPickup {
        public double pivExtension = 110;
    }

    public static class MiddlePickup {
        public double pivExtension = 135;
    }

    public static class RightPickup {
        public double pivExtension = 135;
    }

    public static class CycleScoring {
        public double armDeg = 13;
        public double wristScoreDeg = -39.9;
        public double pivExtension = 240;
    }

    public static Reset RESET = new Reset();
    public static Pickup PICKUP = new Pickup();
    public static CycleScoring CYCLE = new CycleScoring();
    public static LeftPickup LEFT = new LeftPickup();
    public static RightPickup RIGHT = new RightPickup();
    public static MiddlePickup MIDDLE = new MiddlePickup();


    Arm arm;
    MecanumDrive mecanumDrive;
    VerticalSlides verticalSlides;
    PivotingSlides pivotingSlides;
    Wrist wrist;
    Spintake spintake;

    EditablePose lastPose;
    Timer timer;
    final Pose2d startingPose = new Pose2d(-63.5, 41, Math.toRadians(0));

    @Override
    public void init() {
        arm = new Arm(hardwareMap);
        mecanumDrive = new MecanumDrive(hardwareMap, startingPose);
        verticalSlides = new VerticalSlides(hardwareMap);
        pivotingSlides = new PivotingSlides(hardwareMap);
        wrist = new Wrist(hardwareMap);
        spintake = new Spintake(hardwareMap);
        timer = new Timer();

        wrist.setInitPosition();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("start pos", mecanumDrive.localizer.getPose().position);
        telemetry.addData("start heading", mecanumDrive.localizer.getPose().heading);
    }

    @Override
    public void init_loop() {

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
                        arm.setAngleSmooth(RESET.armDeg),
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

    @Override
    public void loop() {

    }

    public void runActionsWithUpdate(Action action) {
        Actions.runBlocking(new ParallelAction(
                action,
                new ContinuousAction(() -> {
                    timer.updateTime();

                    telemetry.addData("Update rate", (1 / timer.getDeltaTime()));
                    telemetry.update();
                })
        ));
    }

    public static class ContinuousAction implements Action {
        private final Runnable runnable;

        public ContinuousAction(Runnable runnable) {
            this.runnable = runnable;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            runnable.run();
            return true;
        }
    }
}