package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.VelConstraint;
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

@Config
@Autonomous(name = "SampleBasketAuto")
public class SampleBasketAuto extends OpMode {
    public static double wristInitialSamplePosition = 0;
    public static double toBasketY = 23;
    public static double initialXMove = 5;
    public static double initialTurn = 130;

    public static Vector2d firstSample = new Vector2d(18, 18);
    public static Vector2d secondSample = new Vector2d(20, 28);
    public static Vector2d thirdSample = new Vector2d(18, 30);

    public static double thirdSampleOrientation = 30;

    //Y1 is closest to side wall, Yellow Lines Position
    public final Vector2d Y1 = new Vector2d(-70, -25.75);
    public final Vector2d Y2 = new Vector2d(-60, -25.75);
    public final Vector2d Y3 = new Vector2d(-50, -25.75);


    //C3 is closest to side wall, Colored Line Position
    public final Vector2d C1 = new Vector2d(70, -25.75);
    public final Vector2d C2 = new Vector2d(60, -25.75);
    public final Vector2d C3 = new Vector2d(50, -25.75);


    Claw claw;
    Arm arm;
    VerticalSlides slides;
    Wrist wrist;
    IMU imu;

    MecanumDrive mecanumDrive;

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    Pose2d startingPose = new Pose2d(0, 0,  Math.toRadians(90));


    @Override
    public void init() {
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        slides = new VerticalSlides(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imuExpansion");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        mecanumDrive = new MecanumDrive(hardwareMap, startingPose);

        claw.grab();
        arm.setInitPosition();
        wrist.setInitPosition();

    }


    @Override
    public void start() {
//
//        Starting Action: Outake first sample into top basket
        Pose2d targetPose = new Pose2d(new Vector2d(initialXMove, toBasketY), initialTurn);

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                    mecanumDrive.actionBuilder(startingPose)
                        .strafeToLinearHeading(new Vector2d(initialXMove, toBasketY), initialTurn)
                        .build(),
                    slides.liftUp(),
                    arm.setBasketPositionAction(0),
                    wrist.setBasketPositionAction(0)
                    ),
                claw.releaseAction(0)
        ));


        firstLineSample();

        secondLineSample();
//
//        thirdLineSample();

    }
    private void firstLineSample() {
        Actions.runBlocking(new SequentialAction(
                mecanumDrive.actionBuilder(startingPose)
                        .turnTo(0).build(),
                new ParallelAction(
                    mecanumDrive.actionBuilder(startingPose)
                            .strafeToConstantHeading((firstSample))
                            .build(),
                    slides.bringDown()
                ),
                arm.setSamplePositionAction(0),
                wrist.setSampleLinePos(0),
                mecanumDrive.actionBuilder(startingPose)
                                .waitSeconds(0.5).build(),

                claw.grabAction(0),
                mecanumDrive.actionBuilder(startingPose)
                        .waitSeconds(0.5).build(),

                arm.setBasketPositionAction(0),
                wrist.setBasketPos(0),

                new ParallelAction(
                    slides.liftUp(),
                    mecanumDrive.actionBuilder(startingPose)
                        .strafeToLinearHeading(new Vector2d(initialXMove, toBasketY), initialTurn)
                        .build()
                ),
                claw.releaseAction(0)
        ));
    }

    public void secondLineSample(){
        Actions.runBlocking(new SequentialAction(
                mecanumDrive.actionBuilder(startingPose)
                        .turnTo(0).build(),

                new ParallelAction(
                        mecanumDrive.actionBuilder(startingPose)
                                .strafeToConstantHeading((secondSample))
                                .build(),
                        slides.bringDown()
                ),
                arm.setSamplePositionAction(0),
                wrist.setSampleLinePos(0),

                mecanumDrive.actionBuilder(startingPose)
                        .waitSeconds(0.5).build(),
                claw.grabAction(0),

                mecanumDrive.actionBuilder(startingPose)
                        .waitSeconds(0.5).build(),

                arm.setBasketPositionAction(0),
                wrist.setBasketPos(0),

                new ParallelAction(
                        slides.liftUp(),
                        mecanumDrive.actionBuilder(startingPose)
                                .turnTo(Math.toRadians(initialTurn))
                                .strafeTo(new Vector2d(initialXMove, toBasketY))
                                .build()
                ),
                claw.releaseAction(0)
        ));
    }

    public void thirdLineSample(){
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        mecanumDrive.actionBuilder(startingPose)
                                .strafeTo(thirdSample)
                                .turnTo(thirdSampleOrientation)
                                .build(),
                        slides.bringDown()
                ),

                arm.setSamplePositionAction(0),
                wrist.setSampleLinePos(0),

                mecanumDrive.actionBuilder(startingPose)
                        .waitSeconds(0.5).build(),

                claw.grabAction(0),

                mecanumDrive.actionBuilder(startingPose)
                        .waitSeconds(0.5).build(),

                arm.setBasketPositionAction(0),
                wrist.setBasketPos(0),
                new ParallelAction(
                        slides.liftUp(),
                        mecanumDrive.actionBuilder(startingPose)
                                .turnTo(Math.toRadians(initialTurn))
                                .strafeTo(new Vector2d(initialXMove, toBasketY))
                                .build()
                ),
                claw.releaseAction(0)
        ));
    }

    @Override
    public void loop() {

    }


}
