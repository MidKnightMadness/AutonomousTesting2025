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
    TurnTable turnTable;
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
        turnTable = new TurnTable(hardwareMap);

        verticalSlides = new VerticalSlides(hardwareMap);
        arm.setInitPosition();
        wrist.setInitPosition();
        sampleClaw.release();
        turnTable.setPosition(TurnTable.NEUTRAL_POS);
    }

    @Override
    public void start() {
        Actions.runBlocking(new SequentialAction(new ParallelAction(
                arm.setPositionSmooth(Arm.SAMPLE_INTAKE_AUTO, 1),
                wrist.setPositionSmooth(Wrist.SAMPLE_LINE_POSITION_AUTO, 0.7)
        ),
                sampleClaw.setPosition(SampleClaw.GRAB_POSITION),
                drive.actionBuilder().waitSeconds(0.2).build(),
                arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION, 2)));


    }

    @Override
    public void loop() {

    }




}
