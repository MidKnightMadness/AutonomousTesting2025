package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.OutdatedPrograms.OldArm;
import org.firstinspires.ftc.teamcode.Mechanisms.VerticalSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;


//Start at the left of the 2nd tile
@Config
@Autonomous(name = "FourSampleAuto")
public class FourSampleAuto extends OpMode {
    public static Pose2d scoringPose = new Pose2d(new Vector2d(10.5, 22), Math.toRadians(130));
    public static Pose2d firstSamplePose = new Pose2d(new Vector2d( 19, 16.5),0);
    public static Pose2d secondSamplePose = new Pose2d(new Vector2d(19, 26.5), 0);
    public static Pose2d thirdSamplePose = new Pose2d(new Vector2d(24.5, 24.5), Math.toRadians(45));
    public static double thirdSampleIntermediateOffset = 3;
    public static Pose2d submersibleIntermediatePose = new Pose2d(55, 17, Math.toRadians(-90));
    public static Pose2d parkingPose = new Pose2d(new Vector2d(55, 0), Math.toRadians(-90));

    OldArm arm;
    VerticalSlides slides;
    Wrist wrist;
    MecanumDrive mecanumDrive;

    public static Pose2d startingPose = new Pose2d(0, 0,  Math.toRadians(90));

    @Override
    public void init() {
        arm = new OldArm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        slides = new VerticalSlides(hardwareMap);
        mecanumDrive = new MecanumDrive(hardwareMap, startingPose);

        arm.setInitPosition();
        wrist.setInitPosition();
        slides.resetEncoders();
    }

    @Override
    public void init_loop() {

        telemetry.update();
    }

    @Override
    public void start() {

        Actions.runBlocking(
                scoreInBasket(0, 0)
        );

        firstLineSample();
        secondLineSample();
        thirdLineSample();
        parkFromBasket();
    }


    public Action scoreInBasket(double xOffset, double yOffset) {
        return new SequentialAction(
                new ParallelAction(
                        arm.setPositionSmooth(OldArm.STRAIGHT_UP_POSITION),
                        mecanumDrive.actionBuilder().strafeToSplineHeading(new Vector2d(scoringPose.position.x + xOffset, scoringPose.position.y + yOffset), scoringPose.heading).build(),
                        slides.liftUp(1),
                        wrist.setPosition(Wrist.BASKET_POSITION)
                ),
                arm.setPositionSmooth(OldArm.BASKET_POSITION)
        );
    }

    public Action manipulatorPickUp() {
        return new SequentialAction(
                new ParallelAction(
                        arm.setPositionSmooth(OldArm.SAMPLE_INTAKE)
                ),
                new SleepAction(0.2)
        );
    }

    public Action goToPickUpPosition(Pose2d pose) {
        return new ParallelAction(
                slides.bringDown(1),
                mecanumDrive.actionBuilder()
                        .strafeToLinearHeading(pose.position, pose.heading)
                        .build(),
                new SequentialAction(
                        new SleepAction(0.2),
                        arm.setPositionSmooth(OldArm.SCANNING_POSITION)
                )
        );
    }

    public Action resetAfterScoring() {
        return new ParallelAction(
                arm.setPositionSmooth(OldArm.STRAIGHT_UP_POSITION),
                wrist.setPosition(Wrist.SAMPLE_PICKUP_POSITION)
        );
    }

    public void firstLineSample() {
        Actions.runBlocking(new SequentialAction(
                resetAfterScoring(),
                goToPickUpPosition(firstSamplePose),
                manipulatorPickUp(),
                scoreInBasket(0, 0)
        ));
    }

    public void secondLineSample() {
        Actions.runBlocking(new SequentialAction(
                resetAfterScoring(),
                goToPickUpPosition(secondSamplePose),
                manipulatorPickUp(),

                new ParallelAction(
                        arm.setPositionSmooth(OldArm.STRAIGHT_UP_POSITION),
                        new SequentialAction(
                                new SleepAction(0.5),
                                scoreInBasket(0, 0)
                        )
                )
        ));
    }

    public void thirdLineSample() {
        Pose2d intermediatePose = new Pose2d(thirdSamplePose.position.x - thirdSampleIntermediateOffset, thirdSamplePose.position.y - thirdSampleIntermediateOffset, thirdSamplePose.heading.toDouble());

        Actions.runBlocking(new SequentialAction(
                resetAfterScoring(),
                new ParallelAction(
                        goToPickUpPosition(intermediatePose),
                        wrist.setPosition(Wrist.SAMPLE_PICKUP_POSITION)
                ),

                // open before picking up
                new ParallelAction(
                        mecanumDrive.actionBuilder(intermediatePose)
                                .strafeToLinearHeading(thirdSamplePose.position, thirdSamplePose.heading)
                                .build()
                ),

                manipulatorPickUp(),
                new SleepAction(0.2),

                new ParallelAction(
                        mecanumDrive.actionBuilder(thirdSamplePose)
                                .strafeToLinearHeading(intermediatePose.position, intermediatePose.heading)
                                .build(),
                        arm.setPositionSmooth(OldArm.STRAIGHT_UP_POSITION)
                ),

                wrist.setPosition(Wrist.BASKET_POSITION),

                scoreInBasket(0, 0)
        ));
    }

    public void parkFromBasket() {
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        resetAfterScoring(),
                        slides.bringDown(1),
                        mecanumDrive.actionBuilder()
                                .strafeToLinearHeading(submersibleIntermediatePose.position, submersibleIntermediatePose.heading)
                                .strafeToLinearHeading(parkingPose.position, parkingPose.heading)
                                .build()
                ),
                arm.setPositionSmooth(Kinematics.armOrientationToPosition(Math.toRadians(80)))
        ));

        arm.leftServo.getController().pwmDisable();
        arm.rightServo.getController().pwmDisable();
    }


    @Override
    public void loop() { }
}


