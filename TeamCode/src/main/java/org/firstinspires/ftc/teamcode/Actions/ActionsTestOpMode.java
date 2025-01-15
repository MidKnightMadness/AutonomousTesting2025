package org.firstinspires.ftc.teamcode.Actions;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;


public class ActionsTestOpMode extends OpMode {

    Claw claw;
    VerticalSlides verticalSlides;
    MecanumDrive drive;
    Pose2d initialPose;
    @Override
    public void init() {
        initialPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, initialPose);
        claw = new Claw(hardwareMap);
        verticalSlides = new VerticalSlides(hardwareMap);
    }

    @Override
    public void start() {
        Actions.runBlocking(new SequentialAction(
            claw.open(0),
            verticalSlides.liftUp(),
            drive.actionBuilder(initialPose)
                    .splineTo(new Vector2d(30, 30), Math.PI / 2)
                    .splineTo(new Vector2d(0, 60), Math.PI)
                    .build()
        ));
    }

    @Override
    public void loop() {

    }
}
