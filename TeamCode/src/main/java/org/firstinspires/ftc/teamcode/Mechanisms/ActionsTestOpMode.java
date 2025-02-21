package org.firstinspires.ftc.teamcode.Mechanisms;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.FourSampleAuto;
import org.firstinspires.ftc.teamcode.Kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp
public class ActionsTestOpMode extends OpMode {

    SampleClaw sampleClaw;
    TurnTable turnTable;
    VerticalSlides slides;
    MecanumDrive mecanumDrive;
    Pose2d initialPose;
    Wrist wrist;
    Arm arm;
    SpecimenClaw specimenClaw;
    Timer timer;
    @Override
    public void init() {
        initialPose = new Pose2d(0, 0, 0);
        mecanumDrive = new MecanumDrive(hardwareMap, initialPose, telemetry);
        sampleClaw = new SampleClaw(hardwareMap);
        specimenClaw = new SpecimenClaw(hardwareMap);
        wrist = new Wrist(hardwareMap);
        arm = new Arm(hardwareMap);
        turnTable = new TurnTable(hardwareMap);
        timer = new Timer();

        slides = new VerticalSlides(hardwareMap);
        arm.setPositionDirect(Arm.STRAIGHT_UP_POSITION);
        wrist.setInitPosition();
        sampleClaw.release();
        turnTable.setPosition(TurnTable.NEUTRAL_POS);
        slides.resetEncoders();

        telemetry.setAutoClear(true);
    }

    @Override
    public void start() {
        mecanumDrive.localizer.setPose(FourSampleAuto.startingPose);
        Actions.runBlocking(
                new ParallelAction(
                        arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION),
                        mecanumDrive.actionBuilder().strafeToLinearHeading(FourSampleAuto.scoringPose.position, FourSampleAuto.scoringPose.heading).build()
                )
        );
    }



    Pose2d sampleCoordinate = new Pose2d(10, 10, 0);

    double endEffectorOrientation = Math.toRadians(-90);


    Action pickUpSample(Pose2d samplePose) {
        return new SequentialAction();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
//        timer.updateTime();
//
//        double armPos = arm.leftServo.getPosition() + gamepad1.left_stick_y * timer.getDeltaTime() * 0.5;
//        double wristPos = calculateWristPosition(armPos, endEffectorOrientation);
//
//        endEffectorOrientation += gamepad1.right_stick_y * timer.getDeltaTime() * 0.5;
//
//        drive.updatePoseEstimate();
//        arm.setPositionDirect(armPos);
//        wrist.setPositionDirect(wristPos);
//
//        telemetry.addData("ArmPos", armPos);
//        telemetry.addData("Arm orientation", Math.toDegrees(Kinematics.armPositionToOrientation(armPos)));
//        telemetry.addData("WristPos", wristPos);
//        telemetry.addData("Wrist orientation", Math.toDegrees(Kinematics.wristPositionToOrientation(wristPos)));
//
//        telemetry.addData("End effector orientation", Math.toDegrees(endEffectorOrientation));
//        Kinematics.updatePosition(slides, arm, wrist, turnTable);
//        Pose2d endEffectorPose = Kinematics.endEffectorPosition;
//
//        telemetry.addData("End effector pose", String.format("(%f, %f)", endEffectorPose.position.x, endEffectorPose.position.y));
//        telemetry.addData("End effector rotation", Math.toDegrees(endEffectorPose.heading.toDouble()));
//
//        telemetry.update();
    }

    public double calculateWristPosition(double armPos, double targetEndEffectorOrientation) {
        return Kinematics.wristOrientationToPosition(targetEndEffectorOrientation - Kinematics.armPositionToOrientation(armPos));
    }
}
