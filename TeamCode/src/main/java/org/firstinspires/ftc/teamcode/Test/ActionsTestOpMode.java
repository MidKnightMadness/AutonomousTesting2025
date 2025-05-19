package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OutdatedPrograms.OldArm;
import org.firstinspires.ftc.teamcode.Mechanisms.PivotingSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.VerticalSlides;


@TeleOp(group="Test")
public class ActionsTestOpMode extends OpMode {
    RevColorSensorV3 insideColorSensor;
    Servo leftSpintake;
    Servo rightSpintake;

    VerticalSlides slides;
    MecanumDrive mecanumDrive;
    Pose2d initialPose;
//    Wrist wrist;
    PivotingSlides pivotingSlides;
    Timer timer;
    @Override
    public void init() {
        initialPose = new Pose2d(0, 0, 0);
        mecanumDrive = new MecanumDrive(hardwareMap, initialPose);
        timer = new Timer();

        pivotingSlides = new PivotingSlides(hardwareMap);

        rightSpintake.setDirection(Servo.Direction.REVERSE);

        insideColorSensor = hardwareMap.get(RevColorSensorV3.class, "Inside Color");

        telemetry.setAutoClear(true);
    }

    @Override
    public void loop() {
        Actions.runBlocking(
                mecanumDrive.actionBuilder().strafeTo(new Vector2d(20, 20)).build()
        );


        telemetry.update();
    }
}
