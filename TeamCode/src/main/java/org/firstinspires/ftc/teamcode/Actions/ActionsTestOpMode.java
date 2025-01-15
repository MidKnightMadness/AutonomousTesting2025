package org.firstinspires.ftc.teamcode.Actions;

import com.acmerobotics.roadrunner.ParallelAction;
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
    Wrist wrist;
    Arm arm;
    @Override
    public void init() {
        initialPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, initialPose);
        claw = new Claw(hardwareMap);
        wrist = new Wrist(hardwareMap);
        arm = new Arm(hardwareMap);
        verticalSlides = new VerticalSlides(hardwareMap);
    }

    @Override
    public void start() {
        Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                new SequentialAction(
                                        drive.actionBuilder(initialPose)
                                                .lineToX(2)
                                                .turn(Math.toRadians(90))
                                                .lineToX(10)
                                                .build()
                                ),
                                verticalSlides.liftUp(),
                                arm.setBasketPosition(0),
                                wrist.setBasketPos(0)
                        ),
                        claw.release(0.2),
                        drive.actionBuilder(initialPose)
                                .waitSeconds(1)
                                .build()
                )
        );


    }

    @Override
    public void loop() {

    }



}
