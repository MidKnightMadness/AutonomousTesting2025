package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.SampleClaw;
import org.firstinspires.ftc.teamcode.Mechanisms.TurnTable;
import org.firstinspires.ftc.teamcode.Mechanisms.VerticalSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.Components.Kinematics;
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

    public static Pose2d firstSamplePose = new Pose2d(new Vector2d( 19.75, 17),0);
    public static Pose2d secondSamplePose = new Pose2d(new Vector2d(19.75, 27), 0);
    public static Pose2d thirdSamplePose = new Pose2d(new Vector2d(23.5, 24.75), Math.toRadians(45));

    public static Pose2d parkingPose = new Pose2d(new Vector2d(60, -8), Math.toRadians(90));

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
        slides.resetEncoders();
    }

    public Action scoreInBasket(double xOffset, double yOffset) {
        return new SequentialAction(
                arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION),
                new ParallelAction(
                        mecanumDrive.actionBuilder().strafeToSplineHeading(new Vector2d(scoringPose.position.x + xOffset, scoringPose.position.y + yOffset), scoringPose.heading).build(),
                        slides.liftUp(0.8),
                        wrist.setPosition(Wrist.BASKET_POSITION),
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
//                        wrist.setPosition(Wrist.SAMPLE_LINE_POSITION_AUTO),
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
                scoreInBasket(0, 0)
        );

        firstLineSample();
        secondLineSample();
        thirdLineSample();
    }

    public Action pickUpWithFeedback() {
        return new SequentialAction(
                new InstantAction(() -> Kinematics.updatePosition(slides, arm, wrist, turnTable))
        );
    }

    public Action resetAfterScoring() {
        return new ParallelAction(
                //set arm backwards to not interfere because the slides + drive sometimes slides go down faster
                //slides might go down faster before drives out so arm to initial position
                arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION),
                wrist.setPosition(Wrist.SAMPLE_PICKUP_POSITION)
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
                scoreInBasket(0, 0)
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
                scoreInBasket(0, 0)
        ));
    }

    public void thirdLineSample() {
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        //set arm backwards to not interfere because the slides + drive sometimes slides go down faster
                        //slides might go down faster before drives out so arm to initial position
                        wrist.setPosition(Wrist.THIRD_SAMPLE),
                        arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION)
                ),
                new ParallelAction(
                        slides.bringDown(0.6),
                        mecanumDrive.actionBuilder()
                                .strafeToLinearHeading(thirdSamplePose.position, thirdSamplePose.heading)
                                .waitSeconds(0.5)
                                .build(),
                        turnTable.setPositionSmooth(TurnTable.THIRD_SAMPLE_POS, 0.5),
                        wrist.setPosition(Wrist.SAMPLE_PICKUP_POSITION),
                        sampleClaw.setPosition(SampleClaw.GRAB_POSITION),

                        new SequentialAction(
                                new SleepAction(0.2),
                                arm.setPositionSmooth(Arm.SAMPLE_INTAKE - 0.04)
                        )
                ),

                sampleClaw.releaseAction(0),
                new SleepAction(0.5),

                manipulatorPickUp(),
                wrist.setPosition(Wrist.BASKET_POSITION),

                scoreInBasket(0, 0)
        ));
    }

    @Override
    public void loop() {

    }


}

