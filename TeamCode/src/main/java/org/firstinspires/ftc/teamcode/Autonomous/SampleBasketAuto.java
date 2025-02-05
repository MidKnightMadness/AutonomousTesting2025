package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Actions.Arm;
import org.firstinspires.ftc.teamcode.Actions.SampleClaw;
import org.firstinspires.ftc.teamcode.Actions.TurnTable;
import org.firstinspires.ftc.teamcode.Actions.VerticalSlides;
import org.firstinspires.ftc.teamcode.Actions.Wrist;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;


//Start at the left of the 2nd tile
@Config
@Autonomous(name = "SampleBasketAuto")
public class SampleBasketAuto extends OpMode {

    public static Pose2d scoringPose = new Pose2d(new Vector2d(8.5, 25), Math.toRadians(130));

    public static Pose2d firstSamplePose = new Pose2d(new Vector2d( 18, 19.5),0);
    public static Pose2d secondSamplePose = new Pose2d(new Vector2d(17, 31), 0);
    public static Pose2d thirdSampleIntermediate = new Pose2d(new Vector2d(20, 26), Math.toRadians(45));
    public static Pose2d thirdSamplePose = new Pose2d(new Vector2d(21, 27.5), Math.toRadians(45));

    public static Pose2d parkingPose = new Pose2d(new Vector2d(60, -8), Math.toRadians(90));

    public static double initSlidesUpPos = 10;
    public static double firstSlidesUpPos = 100;
    public static double secondSlidesUpPos = 100;


    SampleClaw sampleClaw;
    Arm arm;
    VerticalSlides slides;
    Wrist wrist;
    TurnTable turnTable;
    MecanumDrive mecanumDrive;

    Pose2d startingPose = new Pose2d(0, 0,  Math.toRadians(90));

    @Override
    public void init() {
        sampleClaw = new SampleClaw(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        turnTable = new TurnTable(hardwareMap);
        slides = new VerticalSlides(hardwareMap);
        mecanumDrive = new MecanumDrive(hardwareMap, startingPose);

        sampleClaw.grab();
        arm.setInitPosition();
        wrist.setInitPosition();
        turnTable.setInitPosition();
    }

    public Action scoreInBasket(double xOffset, double yOffset) {//TODO: make sure it doesnt hit side wall when outaking sample
        return new SequentialAction(
                arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION, 0.7),
                new ParallelAction(
                        mecanumDrive.actionBuilder().strafeToSplineHeading(new Vector2d(scoringPose.position.x + xOffset, scoringPose.position.y + yOffset), scoringPose.heading).build(),
                        slides.liftUp(),
                        wrist.setPositionSmooth(Wrist.BASKET_POSITION_AUTO, 0.7),
                        turnTable.setPositionSmooth(TurnTable.NEUTRAL_POS, 0.5)
                ),
                arm.setPositionSmooth(Arm.BASKET_POSITION, 0.3),
                new SleepAction(0.4),
                sampleClaw.setPosition(SampleClaw.RELEASE_POSITION),
                new SleepAction(0.5)
        );
    }

    public Action manipulatorPickUp() {
        return new SequentialAction(
                new ParallelAction(
                        wrist.setPositionSmooth(Wrist.SAMPLE_LINE_POSITION_AUTO, 0.7),
                        arm.setPositionSmooth(Arm.SAMPLE_INTAKE_AUTO, 0.7)
                ),
                new SleepAction(0.5),
                sampleClaw.grabAction(0),
                new SleepAction(0.2)
        );
    }

    @Override
    public void start() {
        Actions.runBlocking(
                scoreInBasket(0, 0)
        );

        firstLineSample();
        secondLineSample();
        thirdLineSample();
    }

    private void firstLineSample() {
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        //set arm backwards to not interfere because the slides + drive sometimes slides go down faster
                        //slides might go down faster before drives out so arm to initial position

                        arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION, 0.5),
                        wrist.setPositionSmooth(Wrist.SAMPLE_LINE_POSITION_AUTO, 0.7)
                ),
                new ParallelAction(
                slides.bringDown(0.6),
                mecanumDrive.actionBuilder()
                        .strafeToLinearHeading(firstSamplePose.position, firstSamplePose.heading)
                        .build()
                        ),
                new SleepAction(0.3),
                manipulatorPickUp(),
                scoreInBasket(0, 0)
        ));
    }

    public void secondLineSample(){
        Actions.runBlocking(new SequentialAction(

                new ParallelAction(
                        //set arm backwards to not interfere because the slides + drive sometimes slides go down faster
                        //slides might go down faster before drives out so arm to initial position
                        arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION, 0.5),
                        wrist.setPositionSmooth(Wrist.SAMPLE_LINE_POSITION_AUTO, 0.7)
                ),
                new ParallelAction(
                        slides.bringDown(0.6),
                        mecanumDrive.actionBuilder()
                                .strafeToLinearHeading(secondSamplePose.position, secondSamplePose.heading)
                                .build()
                ),
                new SleepAction(0.3),
                manipulatorPickUp(),
                scoreInBasket(0, 0)
        ));
    }

    public void thirdLineSample(){
        Actions.runBlocking(new SequentialAction(

                new ParallelAction(
                        //set arm backwards to not interfere because the slides + drive sometimes slides go down faster
                        //slides might go down faster before drives out so arm to initial position
                        wrist.setPositionSmooth(Wrist.THIRD_SAMPLE, 0.5),
                        arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION, 0.5)
                ),
                new ParallelAction(
                        slides.bringDown(0.6),
                        mecanumDrive.actionBuilder()
                                .strafeToLinearHeading(thirdSamplePose.position, thirdSamplePose.heading)
                                .build(),
                        turnTable.setPositionSmooth(TurnTable.THIRD_SAMPLE_POS, 0.5),
                        wrist.setPositionSmooth(Wrist.SAMPLE_LINE_POSITION_AUTO, 0.5),

                        new SequentialAction(
                                new SleepAction(0.5),
                                arm.setPositionSmooth(Arm.SAMPLE_INTAKE_AUTO, 1.5)
                        )
                ),

                new SleepAction(1000),
                manipulatorPickUp(),


                scoreInBasket(0, 2)
        ));
    }


    public void park(){
        Actions.runBlocking(new SequentialAction(
        new ParallelAction(
            mecanumDrive.actionBuilder().splineTo(parkingPose.position, parkingPose.heading).build(),
            arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION, 0.5),
            new SequentialAction(
                    new SleepAction(0.25), slides.bringDown(0.7)
            )
        ),
        arm.setPositionSmooth(Arm.ARM_TO_BAR, 0.5)
        ));
    }

    @Override
    public void loop() {

    }


}

