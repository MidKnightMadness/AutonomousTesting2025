package org.firstinspires.ftc.teamcode.Test;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.FourSampleAuto;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.PivotingSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.SampleClaw;
import org.firstinspires.ftc.teamcode.Mechanisms.SpecimenClaw;
import org.firstinspires.ftc.teamcode.Mechanisms.TurnTable;
import org.firstinspires.ftc.teamcode.Mechanisms.VerticalSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;


@TeleOp(group="Test")
public class ActionsTestOpMode extends OpMode {
    RevColorSensorV3 outsideColorSensor;
    RevColorSensorV3 insideColorSensor;
    Servo leftSpintake;
    Servo rightSpintake;

    SampleClaw sampleClaw;
    TurnTable turnTable;
    VerticalSlides slides;
    MecanumDrive mecanumDrive;
    Pose2d initialPose;
//    Wrist wrist;
    Arm arm;
    SpecimenClaw specimenClaw;
    PivotingSlides pivotingSlides;
    Timer timer;
    @Override
    public void init() {
//        initialPose = new Pose2d(0, 0, 0);
//        mecanumDrive = new MecanumDrive(hardwareMap, initialPose);
//        sampleClaw = new SampleClaw(hardwareMap);
//        specimenClaw = new SpecimenClaw(hardwareMap);
//        wrist = new Wrist(hardwareMap);
        arm = new Arm(hardwareMap);
//        turnTable = new TurnTable(hardwareMap);
        timer = new Timer();

//        slides = new VerticalSlides(hardwareMap);
        arm.setPositionDirect(Arm.STRAIGHT_UP_POSITION);
        pivotingSlides = new PivotingSlides(hardwareMap);
//        wrist.setInitPosition();
//        sampleClaw.release();
//        turnTable.setPosition(TurnTable.NEUTRAL_POS);
//        slides.resetEncoders();

        leftSpintake = hardwareMap.get(Servo.class, "Left Spintake");
        rightSpintake = hardwareMap.get(Servo.class, "Right Spintake");
        rightSpintake.setDirection(Servo.Direction.REVERSE);

//        outsideColorSensor = hardwareMap.get(RevColorSensorV3.class, "outsideColorSensor");
        insideColorSensor = hardwareMap.get(RevColorSensorV3.class, "insideColorSensor");

        telemetry.setAutoClear(true);
    }

//    @Override
//    public void start() {
//        mecanumDrive.localizer.setPose(FourSampleAuto.startingPose);
//    }



    Pose2d sampleCoordinate = new Pose2d(10, 10, 0);

    double endEffectorOrientation = Math.toRadians(-90);

    Action pickUpSample(Pose2d samplePose) {
        return new SequentialAction();
    }

//    @SuppressLint("DefaultLocale")
    double extensionLength = 9;
    double speed = 1;
    final double INTAKE_THRESHOLD  = 0.6;
    final double OUTTAKE_THRESHOLD = 2;

    boolean intaking;
    boolean outtaking;
    @Override
    public void loop() {

        timer.updateTime();
        double deltaTime = timer.getDeltaTime();
//

        double armPos = arm.leftServo.getPosition() + gamepad1.left_stick_y * deltaTime * 0.5;
//        double wristPos = calculateWristPosition(armPos, endEffectorOrientation);
//
//        endEffectorOrientation += gamepad1.right_stick_y * timer.getDeltaTime() * 0.5;
//
//        drive.updatePoseEstimate();
        if(gamepad1.left_bumper) {
            arm.setPositionDirect(armPos);
        }
        else if(gamepad1.x){
            Actions.runBlocking(arm.setPositionSmooth(Arm.SCANNING_POSITION));
        }
        else if(gamepad1.y){
            Actions.runBlocking(arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION));
        }

        if(gamepad1.dpad_left){
            intaking = true;
        }
        if(gamepad1.dpad_right){
            outtaking = true;
        }


        if (intaking) {
            leftSpintake.setPosition(1);
            rightSpintake.setPosition(1);
            intaking = (insideColorSensor.getDistance(DistanceUnit.INCH) > INTAKE_THRESHOLD);
        }
        else if (outtaking) {
            leftSpintake.setPosition(0);
            rightSpintake.setPosition(0);
            outtaking = (insideColorSensor.getDistance(DistanceUnit.INCH) < OUTTAKE_THRESHOLD);
        }
        else {
            leftSpintake.setPosition(0.5);
            rightSpintake.setPosition(0.5);
        }




//        wrist.setPositionDirect(wristPos);
//

        extensionLength += gamepad1.dpad_up ? deltaTime * speed : 0;
        extensionLength -= gamepad1.dpad_down ? deltaTime * speed : 0;

        if (gamepad1.a) pivotingSlides.setExtensionLength(extensionLength);
        if (gamepad1.b) pivotingSlides.setExtensionLength(0);




        telemetry.addData("ArmPos", armPos);
        telemetry.addData("Extension Length", extensionLength);
        
        telemetry.addData("Left Servo Pos", leftSpintake.getPosition());
        telemetry.addData("Right Servo Pos", rightSpintake.getPosition());
        telemetry.addData("Outside color sensor distance", outsideColorSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Inside color sensor distance", insideColorSensor.getDistance(DistanceUnit.INCH));

        telemetry.addData("Inside color sensor optical", insideColorSensor.getRawLightDetected());
        telemetry.addData("Outside color sensor optical", outsideColorSensor.getRawLightDetected());

        telemetry.addData("Inside color sensor raw", insideColorSensor.rawOptical());
        telemetry.addData("outside color sensor raw", outsideColorSensor.rawOptical());

        telemetry.addData("Time", timer.updateTime());

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
        telemetry.update();
    }
}
