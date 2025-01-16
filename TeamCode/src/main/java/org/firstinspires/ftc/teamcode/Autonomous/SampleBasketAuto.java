package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Actions.Arm;
import org.firstinspires.ftc.teamcode.Actions.Claw;
import org.firstinspires.ftc.teamcode.Actions.VerticalSlides;
import org.firstinspires.ftc.teamcode.Actions.Wrist;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;


//Start at the left of the 2nd tile

@Autonomous(name = "SampleBasketAuto")
public class SampleBasketAuto extends OpMode {

    Claw claw;
    Arm arm;
    VerticalSlides slides;
    Wrist wrist;
    IMU imu;

    MecanumDrive mecanumDrive;



    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    Pose2d startingPose = new Pose2d(0, 0, 0);


    @Override
    public void init() {
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        slides = new VerticalSlides(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imuExpansion");

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        mecanumDrive = new MecanumDrive(hardwareMap, startingPose);
        claw.grab(0);
    }


    @Override
    public void start() {


        //Starting Action: Output first sample into top basket

        Actions.runBlocking(new SequentialAction(
                mecanumDrive.actionBuilder(startingPose)
                        .splineTo(new Vector2d(5,24), 135)
                        .build(),
                slides.liftUp(),
                arm.setBasketPosition(1),
                wrist.setBasketPos(0),
                claw.release(1)
        ));


        //Trajectory 1: Pickup first sample and Drive
        TrajectoryActionBuilder tab1 = mecanumDrive.actionBuilder(startingPose)
                .splineTo(new Vector2d(20, 0), -90)
                ;


        //Trajectory 2: Pickup second sample and Drive
        TrajectoryActionBuilder tab2 = mecanumDrive.actionBuilder(startingPose)
                .splineTo(new Vector2d(20, 0), -90)
                ;

        //Trajectory 3: Pickup third sample and Drive

        TrajectoryActionBuilder tab3 = mecanumDrive.actionBuilder(startingPose)
                .splineTo(new Vector2d(20, 0), -90)
                ;

        //Follow trajectory 1 and reset end effector positions, return back to
//        Actions.runBlocking(
//                new SequentialAction(
//                    new ParallelAction(
//                        slides.bringDown(),
//                        arm.setSamplePosition(0),
//                        wrist.setSamplePos(0),
//                        tab1.build()
//                    ),
//                    claw.grab(1),
//                    new ParallelAction(
//                            new SequentialAction(
//                            mecanumDrive.actionBuilder(startingPose)
//                                .turn(Math.toRadians(-35))
//                                .build(),
//                            slides.liftUp(
//                                    )),
//                        arm.setBasketPosition(0),
//                        wrist.setBasketPos(0)
//                    ),
//                    claw.release(1)
//                )
//        );


//        //Follow trajectory 2
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                slides.bringDown(),
//                                arm.setSamplePosition(0),
//                                wrist.setSamplePos(0),
//                                tab2.build()
//                        ),
//                        claw.grab(1),
//                        new ParallelAction(
//                                mecanumDrive.actionBuilder(startingPose)
//                                        .turn(Math.toRadians(-35))
//                                        .build(),
//                                slides.liftUp(),
//                                arm.setBasketPosition(0),
//                                wrist.setBasketPos(0)
//                        ),
//                        claw.release(1)
//                )
//        );
//
//        //Follow trajectory 3
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                slides.bringDown(),
//                                arm.setSamplePosition(0),
//                                wrist.setSamplePos(0),
//                                tab3.build()
//                        ),
//                        claw.grab(1),
//                        new ParallelAction(
//                                mecanumDrive.actionBuilder(startingPose)
//                                        .turn(Math.toRadians(-35))
//                                        .build(),
//                                slides.liftUp(),
//                                arm.setBasketPosition(0),
//                                wrist.setBasketPos(0)
//                        ),
//                        claw.release(1)
//                )
//        );





    }


    @Override
    public void loop() {

    }


}
