package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.SampleClaw;
import org.firstinspires.ftc.teamcode.Mechanisms.VerticalSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Specimen Auto")
@Config
public class SpecimenAuto extends OpMode {

    public static Pose2d scoringPose = new Pose2d(new Vector2d( 28, 16.5),0);
    public static Vector2d firstSpecimenPosition = new Vector2d(45, -24);
    public static double firstSpecimenHeading =  Math.toRadians(180);
    public static Pose2d thirdSamplePose = new Pose2d(new Vector2d(24.5, 24.5), Math.toRadians(45));

    SampleClaw sampleClaw;
    Arm arm;
    VerticalSlides slides;
    Wrist wrist;

    MecanumDrive mecanumDrive;

    Pose2d startingPose = new Pose2d(0, 0, 0);

    @Override
    public void init() {
        sampleClaw = new SampleClaw(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        slides = new VerticalSlides(hardwareMap);

        mecanumDrive = new MecanumDrive(hardwareMap, startingPose);
    }

    @Override
    public void start() {

        Actions.runBlocking(
                new SequentialAction(
                        mecanumDrive.actionBuilder()
                                .strafeTo(scoringPose.position)
                                .build(),
                        new SleepAction(1),
                        mecanumDrive.actionBuilder()
                                .splineTo(firstSpecimenPosition, firstSpecimenHeading)
                                .build()

                )
        );
    }

    @Override
    public void loop() {

    }
}
