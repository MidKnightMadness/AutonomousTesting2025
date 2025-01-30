package org.firstinspires.ftc.teamcode.Actions;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp
public class ActionsTestOpMode extends OpMode {

    SampleClaw sampleClaw;
    VerticalSlides verticalSlides;
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
        verticalSlides = new VerticalSlides(hardwareMap);
    }

    @Override
    public void start() {
        Actions.runBlocking(new SequentialAction(
                specimenClaw.setPosition(SpecimenClaw.GRAB_POSITION),
                drive.actionBuilder().waitSeconds(2).build(),
                specimenClaw.setPosition(SpecimenClaw.RELEASE_POSITION)
        ));
    }

    @Override
    public void loop() {

    }



}
