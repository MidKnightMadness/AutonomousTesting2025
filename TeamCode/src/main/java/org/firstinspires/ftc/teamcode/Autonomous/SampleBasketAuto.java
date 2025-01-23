package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Actions.Arm;
import org.firstinspires.ftc.teamcode.Actions.Claw;
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

    public static Pose2d scoringPose = new Pose2d(new Vector2d(5, 23), Math.toRadians(130));

    public static Pose2d firstSamplePose = new Pose2d(new Vector2d(18, 18),0);
    public static Pose2d secondSamplePose = new Pose2d(new Vector2d(20, 28), 0);
    public static Pose2d thirdSamplePose = new Pose2d(new Vector2d(18, 30), Math.toRadians(25));

    Claw claw;
    Arm arm;
    VerticalSlides slides;
    Wrist wrist;

    MecanumDrive mecanumDrive;

    Pose2d startingPose = new Pose2d(0, 0,  Math.toRadians(90));

    @Override
    public void init() {
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        slides = new VerticalSlides(hardwareMap);
        mecanumDrive = new MecanumDrive(hardwareMap, startingPose);

        claw.grab();
        arm.setInitPosition();
        wrist.setInitPosition();
    }

    public Action scoreInBasket() {
        return new SequentialAction(
                new ParallelAction(
                        mecanumDrive.actionBuilder().strafeToSplineHeading(scoringPose.position, scoringPose.heading).build(),
                        arm.setStraightUp(0),
                        slides.liftUp(),
                        wrist.setBasketPositionAction(0)
                ),
                arm.setBasketPositionAction(0),
                waitSeconds(0.5),

                claw.releaseAction(0)
        );
    }

    public Action manipulatorPickUp() {
        return new SequentialAction(
                arm.setSampleIntermediate(0),
                wrist.setSampleLinePos(0),
                waitSeconds(0.5),
                arm.setSamplePositionAction(0),
                waitSeconds(0.5),
                claw.grabAction(0),
                waitSeconds(0.5)
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

                // turn first to avoid collision with basket
                //Removed b/c turning heading to 0 goes counter clockwise to 0, should go clockwise
                //check
                //mecanumDrive.actionBuilder(startingPose)
                //                        .turnTo(firstSamplePose.heading).build(),

                new ParallelAction(
                        //set arm backwards to not interfere because the slides + drive sometimes slides go down faster
                        //slides might go down faster before drives out so arm to initial position

                        arm.setInitPositionAction(0),
                        mecanumDrive.actionBuilder()
                                .strafeToLinearHeading(firstSamplePose.position, firstSamplePose.heading)
                                .build(),
                        slides.bringDown()
                ),

                manipulatorPickUp(),
                scoreInBasket()
        ));
    }

    public void secondLineSample(){
        Actions.runBlocking(new SequentialAction(
                // turn first to avoid collision with basket
                //Removed b/c turning heading to 0 goes counter clockwise to 0, should go clockwise
                //check
                //mecanumDrive.actionBuilder(startingPose)
                //                        .turnTo(firstSamplePose.heading).build(),


                new ParallelAction(
                        //set arm backwards to not interfere because the slides + drive sometimes slides go down faster
                        //slides might go down faster before drives out so arm to initial position
                        arm.setInitPositionAction(0),
                        mecanumDrive.actionBuilder(startingPose)
                                .strafeToLinearHeading(secondSamplePose.position, secondSamplePose.heading)
                                .build(),
                        slides.bringDown()
                ),

                //TODO: after manipulator picks up, going to basket position for second one is off
                //because the sample line pos 2 is more to the left, so it turns more to get to the position at the end of the path(closer to basket)
                //so it hits the right side of the top basket
                //need to fix

                manipulatorPickUp(),
                scoreInBasket()

        ));
    }

    public void thirdLineSample(){
        Actions.runBlocking(new SequentialAction(
//                new ParallelAction(
//                        mecanumDrive.actionBuilder(startingPose)
//                                .strafeTo(thirdSample)
//                                .turnTo(thirdSampleOrientation)
//                                .build(),
//                        slides.bringDown()
//                ),

                arm.setSamplePositionAction(0),
                wrist.setSampleLinePos(0),

                mecanumDrive.actionBuilder(startingPose)
                        .waitSeconds(0.5).build(),

                claw.grabAction(0),

                mecanumDrive.actionBuilder(startingPose)
                        .waitSeconds(0.5).build(),

                arm.setBasketPositionAction(0)
//                wrist.setBasketPos(0),
//                new ParallelAction(
//                        slides.liftUp(),
//                        mecanumDrive.actionBuilder(startingPose)
//                                .turnTo(Math.toRadians(initialTurn))
//                                .strafeTo(new Vector2d(initialXMove, toBasketY))
//                                .build()
//                ),
//                claw.releaseAction(0)
        ));
    }

    @Override
    public void loop() {

    }


}

