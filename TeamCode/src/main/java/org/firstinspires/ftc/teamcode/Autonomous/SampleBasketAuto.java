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

    public static Pose2d firstSamplePose = new Pose2d(new Vector2d( 19.75, 18.5),0);
    public static Pose2d secondSamplePose = new Pose2d(new Vector2d(18.75, 30), 0);
    public static Pose2d thirdSampleIntermediate = new Pose2d(new Vector2d(22, 26), Math.toRadians(45));
    public static Pose2d thirdSamplePose = new Pose2d(new Vector2d(22, 28.25), Math.toRadians(45));

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

    double armSpeedMultiplier = 1;

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
                arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION),
                new ParallelAction(
                        mecanumDrive.actionBuilder().strafeToSplineHeading(new Vector2d(scoringPose.position.x + xOffset, scoringPose.position.y + yOffset), scoringPose.heading).build(),
                        slides.liftUp(),
                        wrist.setPositionSmooth(Wrist.BASKET_POSITION_AUTO, 0.7),
                        turnTable.setPositionSmooth(TurnTable.NEUTRAL_POS, 0.5)
                ),
                arm.setPositionSmooth(Arm.BASKET_POSITION),
                new SleepAction(0.4),
                sampleClaw.setPosition(SampleClaw.RELEASE_POSITION),
                new SleepAction(0.5)
        );
    }

    public Action manipulatorPickUp() {
        return new SequentialAction(
                new ParallelAction(
//                        wrist.setPositionSmooth(Wrist.SAMPLE_LINE_POSITION_AUTO, 0.7),
                        arm.setPositionSmooth(Arm.SAMPLE_INTAKE),
                        sampleClaw.releaseAction(0)
                ),
                new SleepAction(0.5),
                sampleClaw.grabAction(0),
                new SleepAction(0.2)
        );
    }

    @Override
    public void start() {
        Actions.runBlocking(
                scoreInBasket(0, -1)
        );

        firstLineSample();
        secondLineSample();
        thirdLineSample();
//        park();
    }

    public Action resetAfterScoring() {
        return new ParallelAction(
                //set arm backwards to not interfere because the slides + drive sometimes slides go down faster
                //slides might go down faster before drives out so arm to initial position
                arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION),
                wrist.setPositionSmooth(Wrist.SAMPLE_LINE_POSITION_AUTO, 0.7)
        );
    }

    private void firstLineSample() {
        Actions.runBlocking(new SequentialAction(
                resetAfterScoring(),
                new ParallelAction(
                slides.bringDown(0.6),
                mecanumDrive.actionBuilder()
                        .strafeToLinearHeading(firstSamplePose.position, firstSamplePose.heading)
                        .build()
                        ),
                new SleepAction(0.3),
                manipulatorPickUp(),
                scoreInBasket(1, 0)
        ));
    }

    public void secondLineSample(){
        Actions.runBlocking(new SequentialAction(
                resetAfterScoring(),
                new ParallelAction(
                        slides.bringDown(0.6),
                        mecanumDrive.actionBuilder()
                                .strafeToLinearHeading(secondSamplePose.position, secondSamplePose.heading)
                                .build()
                ),
                new SleepAction(0.3),
                manipulatorPickUp(),
                scoreInBasket(2, 2)
        ));
    }

    public void thirdLineSample() {
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        //set arm backwards to not interfere because the slides + drive sometimes slides go down faster
                        //slides might go down faster before drives out so arm to initial position
                        wrist.setPositionSmooth(Wrist.THIRD_SAMPLE, 0.5),
                        arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION)
                ),
                new ParallelAction(
                        slides.bringDown(0.6),
                        mecanumDrive.actionBuilder()
                                .strafeToLinearHeading(thirdSamplePose.position, thirdSamplePose.heading)
                                .waitSeconds(0.5)
                                .build(),
                        turnTable.setPositionSmooth(TurnTable.THIRD_SAMPLE_POS, 0.5),
                        wrist.setPositionSmooth(Wrist.SAMPLE_LINE_POSITION_AUTO, 0.5),
                        sampleClaw.setPosition(SampleClaw.GRAB_POSITION),

                        new SequentialAction(
                                new SleepAction(0.2),
                                arm.setPositionSmooth(Arm.SAMPLE_INTAKE - 0.03)
                        )
                ),

                sampleClaw.releaseAction(0),
                new SleepAction(0.5),
                wrist.setPosition(0.5),

                manipulatorPickUp(),
                wrist.setPosition(Wrist.BASKET_POSITION_AUTO),

                scoreInBasket(-1, 2)
        ));
    }


    public void park(){
        Actions.runBlocking(new SequentialAction(
        new ParallelAction(
            mecanumDrive.actionBuilder().splineTo(parkingPose.position, parkingPose.heading).build(),
            arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION),
            new SequentialAction(
                    new SleepAction(0.25), slides.bringDown(0.7)
            )
        ),
        arm.setPositionSmooth(Arm.ARM_TO_BAR)
        ));
    }

    @Override
    public void loop() {

    }


}

