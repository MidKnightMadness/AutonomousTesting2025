package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Actions.Arm;
import org.firstinspires.ftc.teamcode.Actions.SampleClaw;
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
@Autonomous(name = "UpdatedSampleBasketAuto")
public class SampleBasketAuto extends OpMode {

    public static Pose2d scoringPose = new Pose2d(new Vector2d(8.5, 25), Math.toRadians(130));

    public static Pose2d firstSamplePose = new Pose2d(new Vector2d( 19, 18),0);
    public static Pose2d secondSamplePose = new Pose2d(new Vector2d(19, 28), 0);
    public static Pose2d thirdSamplePose = new Pose2d(new Vector2d(17, 30), Math.toRadians(25));

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

    public Action scoreInBasket() {//TODO: make sure it doesnt hit side wall when outaking sample
        return new SequentialAction(
                arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION, 0.7),
                new ParallelAction(
                        mecanumDrive.actionBuilder().strafeToSplineHeading(scoringPose.position, scoringPose.heading).build(),
                        slides.liftUp(),
                        wrist.setPositionSmooth(Wrist.BASKET_POSITION_AUTO, 0.5),
                        turnTable.setPositionSmooth(TurnTable.NEUTRAL_POS, 0.5)
                ),
                arm.setPositionSmooth(Arm.BASKET_POSITION_AUTO, 0.3),
                waitSeconds(0.2),
                sampleClaw.setPosition(SampleClaw.RELEASE_POSITION)
        );
    }

    public Action manipulatorPickUp() {
        return new SequentialAction(
                new ParallelAction(
                    wrist.setPositionSmooth(Wrist.SAMPLE_LINE_POSITION_AUTO, 0.3),
                    arm.setPositionSmooth(Arm.SAMPLE_INTAKE_AUTO, 0.5)
                ),
                sampleClaw.grabAction(0.5),
                waitSeconds(0.2)
        );
    }

    public Action waitSeconds(double seconds) {
        return mecanumDrive.actionBuilder().waitSeconds(seconds).build();
    }

    @Override
    public void start() {
        Actions.runBlocking(
                scoreInBasket()
        );

        firstLineSample();
        secondLineSample();
//        thirdLineSample();
    }

    private void firstLineSample() {
        Actions.runBlocking(new SequentialAction(

                new ParallelAction(
                        //set arm backwards to not interfere because the slides + drive sometimes slides go down faster
                        //slides might go down faster before drives out so arm to initial position

                        arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION, 0.5),
                        wrist.setPositionSmooth(Wrist.SAMPLE_LINE_POSITION_AUTO, 0.5)
                ),
                new ParallelAction(
                slides.bringDown(0.6),
                mecanumDrive.actionBuilder()
                        .strafeToLinearHeading(firstSamplePose.position, firstSamplePose.heading)
                        .build()
                        ),
                manipulatorPickUp(),
                scoreInBasket()
        ));
    }

    public void secondLineSample(){
        Actions.runBlocking(new SequentialAction(

                new ParallelAction(
                        //set arm backwards to not interfere because the slides + drive sometimes slides go down faster
                        //slides might go down faster before drives out so arm to initial position

                        arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION, 0.5),
                        wrist.setPositionSmooth(Wrist.SAMPLE_LINE_POSITION_AUTO, 0.5)
                ),
                new ParallelAction(
                        slides.bringDown(0.6),
                        mecanumDrive.actionBuilder()
                                .strafeToLinearHeading(secondSamplePose.position, secondSamplePose.heading)
                                .build()
                ),
                manipulatorPickUp(),
                scoreInBasket()
        ));
    }

    public void thirdLineSample(){
        Actions.runBlocking(new SequentialAction(
                // turn first to avoid collision with basket
                //Removed b/c turning heading to 0 goes counter clockwise to 0, should go clockwise
                //check
                //mecanumDrive.actionBuilder(startingPose)
                //                        .turnTo(firstSamplePose.heading).build(),


                new ParallelAction(
                        //set arm backwards to not interfere because the slides + drive sometimes slides go down faster
                        //slides might go down faster before drives out so arm to initial position
                        arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION, 2),
                        waitSeconds(0.5),
                        mecanumDrive.actionBuilder(startingPose)
                                .strafeToLinearHeading(secondSamplePose.position, secondSamplePose.heading)
                                .build(),
                        turnTable.setPositionSmooth(TurnTable.THIRD_SAMPLE_ANGLE, 0.5),
                        slides.bringDown(0.6)
                ),

                manipulatorPickUp(),
                scoreInBasket()

        ));
    }

    @Override
    public void loop() {

    }


}

