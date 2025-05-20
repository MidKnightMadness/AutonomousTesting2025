package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.PivotingSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.Spintake;
import org.firstinspires.ftc.teamcode.Mechanisms.VerticalSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;

@Autonomous
@Config
public class FourSampleAuto extends OpMode {

    public static Vector2d basketScoring = new Vector2d(-54, 54);
    public static Vector2d firstSamplePickup = new Vector2d(-54, 54);
    public static Vector2d secondSamplePickup = new Vector2d(-54, 54);
    public static Vector2d thirdSamplePickup = new Vector2d(-54, 54);

    public static double preloadScoringRot = 135;
    public static double basketScoringRot = -45;
    public static double firstSampleRot = 15;
    public static double secondSampleRot = 0;
    public static double thirdSampleRot = 0;

    public static double wristDeg = 90;
    public static double pivExtension = 240;
    public static double armAngle = 58;

    Arm arm;
    MecanumDrive mecanumDrive;
    VerticalSlides verticalSlides;
    PivotingSlides pivotingSlides;
    Wrist wrist;
    Spintake spintake;

    private final Pose2d startingPose = new Pose2d(-64.5, 32, Math.toRadians(90));
    private final Pose2d preloadScoringPose = new Pose2d(basketScoring, Math.toRadians(preloadScoringRot));
    private final Pose2d basketScoringPose = new Pose2d(basketScoring, Math.toRadians(basketScoringRot));
    private final Pose2d firstSamplePose = new Pose2d(firstSamplePickup, Math.toRadians(firstSampleRot));
    private final Pose2d secondSamplePose = new Pose2d(secondSamplePickup, Math.toRadians(secondSampleRot));
    private final Pose2d thirdSamplePose = new Pose2d(thirdSamplePickup, Math.toRadians(thirdSampleRot));

    @Override
    public void init() {
        arm = new Arm(hardwareMap, telemetry);
        mecanumDrive = new MecanumDrive(hardwareMap, startingPose);
        verticalSlides = new VerticalSlides(hardwareMap);
        pivotingSlides = new PivotingSlides(hardwareMap);
        wrist = new Wrist(hardwareMap);
        spintake = new Spintake(hardwareMap);

        wrist.setInitPosition();

        telemetry.addData("start pos", mecanumDrive.localizer.getPose().position);
        telemetry.addData("start heading", mecanumDrive.localizer.getPose().heading);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Arm current angle", arm.getCurrentAngleDegrees());
        telemetry.addData("Arm target", arm.getTargetAngle());
    }

    @Override
    public void start() {
        mecanumDrive.localizer.setPose(startingPose);
        arm.homeEncoders();
        verticalSlides.resetEncoders();

        runActionsWithUpdate(
            new SequentialAction(
                scorePreload(),
                new SleepAction(1),
                resetAfterScoring()
        ));
    }

    public Action scorePreload() {
        return new ParallelAction(
            arm.setAngle(armAngle),
            verticalSlides.liftUp(1),
            pivotingSlides.setExtensionAction(pivExtension),
            wrist.setDegreesAction(wristDeg),
            new SequentialAction(
                mecanumDrive.actionBuilder(startingPose).strafeToLinearHeading(basketScoring, Math.toRadians(preloadScoringRot)).build(),
                spintake.outtake()
            )
        );
    }

    public Action resetAfterScoring() {
        return new SequentialAction(
            arm.setAngle(45),
            wrist.setDegreesAction(0),
            new ParallelAction(
                verticalSlides.bringDown(0.8),
                pivotingSlides.setExtensionAction(25),
                new SequentialAction(
                    mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose()).strafeToLinearHeading(firstSamplePickup, Math.toRadians(firstSampleRot)).build()
                )
            )
        );
    }

    @Override
    public void loop() {

    }

    public void runActionsWithUpdate(Action action) {
        Actions.runBlocking(new ParallelAction(
                action,
                new ContinuousAction(() -> {
                    arm.update(armAngle);
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