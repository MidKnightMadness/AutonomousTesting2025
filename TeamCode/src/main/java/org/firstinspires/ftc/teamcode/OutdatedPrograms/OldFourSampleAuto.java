package org.firstinspires.ftc.teamcode.OutdatedPrograms;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.VerticalSlides;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;


//Start at the left of the 2nd tile

@Autonomous(name = "OldFourSampleAuto")
@Deprecated
public class OldFourSampleAuto extends OpMode {
    public static Pose2d scoringPose = new Pose2d(new Vector2d(10.5, 22), Math.toRadians(130));
    public static Pose2d firstSamplePose = new Pose2d(new Vector2d( 19, 16.5),0);
    public static Pose2d secondSamplePose = new Pose2d(new Vector2d(19, 26.5), 0);
    public static Pose2d thirdSamplePose = new Pose2d(new Vector2d(24.5, 24.5), Math.toRadians(45));
    public static double thirdSampleIntermediateOffset = 3;
    public static Pose2d submersibleIntermediatePose = new Pose2d(55, 17, Math.toRadians(-90));
    public static Pose2d parkingPose = new Pose2d(new Vector2d(55, 0), Math.toRadians(-90));

    VerticalSlides slides;
    MecanumDrive mecanumDrive;

    public static Pose2d startingPose = new Pose2d(0, 0,  Math.toRadians(90));

    @Override
    public void init() {
        slides = new VerticalSlides(hardwareMap);
        mecanumDrive = new MecanumDrive(hardwareMap, startingPose);

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
                        mecanumDrive.actionBuilder().strafeToSplineHeading(new Vector2d(scoringPose.position.x + xOffset, scoringPose.position.y + yOffset), scoringPose.heading).build(),
                        slides.liftUp(1)
                )
        );
    }

    public Action manipulatorPickUp() {
        return new SequentialAction(
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
                        new SleepAction(0.2)
                )
        );
    }

    public Action resetAfterScoring() {
        return new ParallelAction(

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
                        goToPickUpPosition(intermediatePose)
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
                                .build()
                ),


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
                )
        ));
    }


    @Override
    public void loop() { }
}


