package org.firstinspires.ftc.teamcode.Actions;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp
public class ActionsTestOpMode extends OpMode {

    SampleClaw sampleClaw;
    TurnTable turnTable;
    VerticalSlides slides;
    MecanumDrive drive;
    Pose2d initialPose;
    Wrist wrist;
    Arm arm;
    SpecimenClaw specimenClaw;
    @Override
    public void init() {
        initialPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, initialPose);
        sampleClaw = new SampleClaw(hardwareMap);
        specimenClaw = new SpecimenClaw(hardwareMap);
        wrist = new Wrist(hardwareMap);
        arm = new Arm(hardwareMap);
        turnTable = new TurnTable(hardwareMap);

        slides = new VerticalSlides(hardwareMap);
        arm.setInitPosition();
        wrist.setInitPosition();
        sampleClaw.release();
        turnTable.setPosition(TurnTable.NEUTRAL_POS);

        telemetry.setAutoClear(false);
    }

    @Override
    public void start() {
        drive.localizer.setPose(new Pose2d(new Vector2d(0, 0), Math.toRadians(-90)));

        Actions.runBlocking(

        new SequentialAction(
                arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION),
                new SleepAction(0.5),
                arm.setPositionSmooth(Arm.INIT_AUTO_POS),
                new SleepAction(0.5),
                arm.setPositionSmooth(Arm.SAMPLE_INTAKE),
                new SleepAction(0.5)
//                new ParallelAction(
//
////                        wrist.setPosition(Wrist.STRAIGHT_POSITION),
////                        turnTable.setPosition(TurnTable.NEUTRAL_POS),
////                        drive.actionBuilderNoCorrection().strafeToLinearHeading(TwoPlayerTeleOp.basketTrajectoryIntermediate, Math.toRadians(-90)).build()
//                ),
//
//                new ParallelAction(
//                        drive.actionBuilderNoCorrection(new Pose2d(TwoPlayerTeleOp.basketTrajectoryIntermediate, Math.toRadians(-90))).strafeToLinearHeading(TwoPlayerTeleOp.basketTrajectoryPosition, Math.toRadians(-225)).build()
//                )
        ));
    }

    @Override
    public void loop() {

    }
}
