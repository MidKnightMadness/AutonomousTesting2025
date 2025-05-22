package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
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
    public static EditablePose rightSamplePickup = new EditablePose(-53, 53, -8);
    public static EditablePose middleSamplePickup = new EditablePose(-54, 58, 5);
    public static EditablePose leftSamplePickup = new EditablePose(-49, 55, 28);

    public static class PreloadScoring {
        public double wristDeg = 90;
        public double pivExtension = 240;
        public double armDeg = 58;
    }

    public static class Reset {
        public double downPower = 0.8;
        public double pivExtension = 25;
        public double armDeg = 45;
    }

    public static class Pickup {
        public double wristDeg = 95;
        public double pivExtension = 85;
        public double armPrepDeg = 120;
        public double armPickupDeg = 134;
    }

    public static class CycleScoring {
        public double armDeg = 8;
        public double wristScoreDeg = -39.9;
        public double pivExtension = 240;
    }

    public static PreloadScoring PRELOAD = new PreloadScoring();
    public static Reset RESET = new Reset();
    public static Pickup PICKUP = new Pickup();
    public static CycleScoring CYCLE = new CycleScoring();

    Arm arm;
    MecanumDrive mecanumDrive;
    VerticalSlides verticalSlides;
    PivotingSlides pivotingSlides;
    Wrist wrist;
    Spintake spintake;

    EditablePose lastPose;
    Timer timer;

    private final Pose2d startingPose = new Pose2d(-64.5, 32, Math.toRadians(90));

    @Override
    public void init() {
        arm = new Arm(hardwareMap, telemetry);
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
        if (gamepad1.a) {
            arm.homeEncoders();
        }

        telemetry.addData("Arm current angle", arm.getCurrentAngleDegrees());
        telemetry.addData("Left zero voltage", arm.leftEncoder.zeroVoltage);
        telemetry.addData("Right zero voltage", arm.rightEncoder.zeroVoltage);
        telemetry.addData("Arm target", arm.getTargetAngle());
    }

    @Override
    public void start() {
        mecanumDrive.localizer.setPose(startingPose);
//        arm.homeEncoders();
        verticalSlides.resetEncoders();

        runActionsWithUpdate(
            new SequentialAction(
                scorePreload(),
                resetAfterPreload(leftSamplePickup),
                pickUp(),
                score(),
                resetAfterCycle(middleSamplePickup),
                pickUp(),
                score(),
                resetAfterCycle(rightSamplePickup),
                pickUp(),
                score()
        ));
    }

    public Action scorePreload() {
        lastPose = preloadScoring;
        return new ParallelAction(
            arm.setAngle(PRELOAD.armDeg),
            verticalSlides.liftUp(1),
            pivotingSlides.setExtensionAction(PRELOAD.pivExtension),
            wrist.setDegreesAction(PRELOAD.wristDeg),
            new SequentialAction(
                mecanumDrive.actionBuilder(startingPose).strafeToLinearHeading(preloadScoring.vector2d, preloadScoring.heading).build(),
                spintake.outtake()
            )
        );
    }

    public Action resetAfterPreload(EditablePose targetPose) {
        EditablePose oldPose = lastPose;
        lastPose = targetPose;

        return new SequentialAction(
            arm.setAngle(RESET.armDeg),
            wrist.setDegreesAction(0),
            new ParallelAction(
                verticalSlides.bringDown(RESET.downPower),
                pivotingSlides.setExtensionAction(RESET.pivExtension),
                new SequentialAction(
                    mecanumDrive.actionBuilder(oldPose.pose2d).strafeToLinearHeading(targetPose.vector2d, targetPose.heading).build()
                )
            )
        );
    }

    public Action resetAfterCycle(EditablePose targetPose) {
        EditablePose oldPose = lastPose;
        lastPose = targetPose;

        return new SequentialAction(
                wrist.setDegreesAction(0),
                pivotingSlides.setExtensionAction(RESET.pivExtension),
                new SleepAction(0.5),
                new ParallelAction(
                        arm.setAngle(RESET.armDeg),
                        verticalSlides.bringDown(RESET.downPower),
                        new SequentialAction(
                                mecanumDrive.actionBuilder(oldPose.pose2d).strafeToLinearHeading(targetPose.vector2d, targetPose.heading).build()
                        )
                )
        );
    }

    public Action pickUp() {
        return new ParallelAction(
            new SequentialAction(
                new ParallelAction(
                        wrist.setDegreesAction(PICKUP.wristDeg),
                        pivotingSlides.setExtensionAction(PICKUP.pivExtension),
                        arm.setAngle(PICKUP.armPrepDeg)
                ),
                new SleepAction(0.5),
                new ParallelAction(
                    spintake.intake(),
                    arm.setAngle(PICKUP.armPickupDeg)
                )
            )
        );
    }

    public Action score() {
        EditablePose oldPose = lastPose;
        lastPose = basketScoring;
        return new SequentialAction(
            new ParallelAction(
                pivotingSlides.setExtensionAction(30),
                arm.setAngle(CYCLE.armDeg)
            ),
            new ParallelAction(
                mecanumDrive.actionBuilder(oldPose.pose2d).strafeToLinearHeading(basketScoring.vector2d, basketScoring.heading).build(),
                verticalSlides.setPosition(VerticalSlides.BASKET_BACKWARDS_SCORING, 1)
            ),
            pivotingSlides.setExtensionAction(CYCLE.pivExtension),
            new SleepAction(0.2),
            wrist.setDegreesAction(CYCLE.wristScoreDeg),
            new SleepAction(0.2),
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
                    if (!Arm.inAction) {
                        arm.update(arm.getTargetAngle());
                    }

                    if (gamepad1.a) {
                        arm.homeEncoders();
                    }


                    telemetry.addData("Update rate", (1 / timer.getDeltaTime()));
                    telemetry.addData("Arm angle", arm.getCurrentAngleDegrees());

                    telemetry.addData("Left voltage", arm.leftEncoder.getVoltage());
                    telemetry.addData("Right voltage", arm.rightEncoder.getVoltage());

                    telemetry.addData("Left skips", arm.leftEncoder.numRejections);
                    telemetry.addData("Right skips", arm.rightEncoder.numRejections);

                    telemetry.addData("Left angle", arm.leftEncoder.getAbsolutePositionDegrees());
                    telemetry.addData("Right angle", arm.rightEncoder.getAbsolutePositionDegrees());

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