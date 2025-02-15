package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Twist2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.SampleClaw;
import org.firstinspires.ftc.teamcode.Mechanisms.TurnTable;
import org.firstinspires.ftc.teamcode.Mechanisms.VerticalSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.Components.Area;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;


//Start at the left of the 2nd tile
@Config
@Autonomous(name = "FiveSampleAuto")
public class FiveSampleAuto extends OpMode {

    public static Pose2d scoringPose = new Pose2d(new Vector2d(8.5, 25), Math.toRadians(130));

    public static Pose2d firstSamplePose = new Pose2d(new Vector2d( 19.75, 17.5),Math.toRadians(0));
    public static Pose2d secondSamplePose = new Pose2d(new Vector2d(18.75, 27.25), Math.toRadians(0));
    public static Pose2d thirdSamplePose = new Pose2d(new Vector2d(23.5, 24.75), Math.toRadians(45));

    public static Pose2d parkingPose = new Pose2d(new Vector2d(60, -8), Math.toRadians(90));

    //Sub Gamepad Initialization Poses
    public static Pose2d subFirstSamplePose = new Pose2d(new Vector2d(0,0), Math.toRadians(0));
    public static Pose2d subSecondSamplePose = new Pose2d(new Vector2d(0,0), Math.toRadians(0));

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

    Timer timer;

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

        timer = new Timer();
    }
    double settingSubSampleNumber = 1;

    double gamepadInterval = 0.1;
    String mode = "Sample Pos";

    Area areaOne;
    Area areaTwo;

    double xTolerence = 0;
    double yTolerence = 0;
    @Override
    public void init_loop(){
        timer.updateTime();

        if(gamepad1.left_bumper){
            if(mode.equals("Sample Pos")){
                mode = "Rectangular coordinates";
            }
            else{
                mode = "Sample Pos";
            }
        }

        if(gamepad1.left_bumper){//set 1st sub sample
            settingSubSampleNumber = 1;
        }

        else if(gamepad1.right_bumper){//set 2nd sub sample
            settingSubSampleNumber = 2;
        }


        double xChange = 0;
        double yChange = 0;
        double xToleranceChange = 0;
        double yToleranceChange = 0;
        double headingChange = 0;

        if(settingSubSampleNumber == 1 || settingSubSampleNumber == 2){


            if (gamepad1.dpad_up) {
                yChange += gamepadInterval;
            } else if (gamepad1.dpad_right) {
                xChange += gamepadInterval;
            } else if (gamepad1.dpad_left) {
                xChange -= gamepadInterval;
            } else if (gamepad1.dpad_down) {
                yChange -= gamepadInterval;
            }
            //TODO: Add heading change based off of gamepad


            if(gamepad1.left_stick_y > 0.2){
                xToleranceChange += gamepad1.left_stick_y * gamepadInterval * timer.getDeltaTime();
            }
            else if(gamepad1.right_stick_y > 0.2){
                yToleranceChange += gamepad1.right_stick_y * gamepadInterval * timer.getDeltaTime();
            }



            if(mode.equals("Sample Pos")){//Change samplePosition based on dpad values
                changeSamplePos(xChange, yChange, headingChange, settingSubSampleNumber);
                if(settingSubSampleNumber == 1){
                    areaOne = new Area(subFirstSamplePose, xTolerence + xToleranceChange, yTolerence + yToleranceChange);
                }
                else if(settingSubSampleNumber == 2){
                    areaTwo = new Area(subSecondSamplePose, xTolerence + xToleranceChange, yTolerence + yToleranceChange);
                }
            }


        }

        telemetry.addLine("-------------------------------------------------");
        telemetry.addData("Currently Editing Sample Pos", settingSubSampleNumber);
        telemetry.addData("Mode", mode);
        telemetry.addData("SubFirstSamplePose", subFirstSamplePose.toString());
        telemetry.addData("SubSecondSamplePose", subSecondSamplePose.toString());
        telemetry.addLine("-------------------------------------------------");
        telemetry.addData("XChange", xChange);
        telemetry.addData("YChange", yChange);
        telemetry.addData("HeadingChange", headingChange);
        telemetry.addData("XTolerenceChange", xToleranceChange);
        telemetry.addData("YTolerenceChange", yToleranceChange);
        telemetry.addLine("-------------------------------------------------");
        telemetry.addData("Area One", areaOne.getAreaCoordinates().toString());
        telemetry.addData("Area Two", areaTwo.getAreaCoordinates().toString());

    }


    public boolean changeRectangularPos(double xChange, double yChange, double headingChange, double sampleNumber, double rectCoordinateNumber) {//rectCoorNumber = FL, FR, BL, BR

        Pose2d currentSubSampleNumber = null;

        if(sampleNumber == 1) {
            currentSubSampleNumber = subFirstSamplePose;
        }
        else if(sampleNumber == 2) {
            currentSubSampleNumber = subSecondSamplePose;
        }
        else {
            return false;
        }




        currentSubSampleNumber.plus(new Twist2d(new Vector2d(xChange, yChange), headingChange));

        if(sampleNumber == 1) {
            subFirstSamplePose.copy(currentSubSampleNumber.position, currentSubSampleNumber.heading);
        }
        else{
            subSecondSamplePose.copy(currentSubSampleNumber.position, currentSubSampleNumber.heading);
        }
        return true;
    }

    public boolean changeSamplePos(double xChange, double yChange, double headingChange, double sampleNumber) {
        Pose2d currentSubSampleNumber = null;

        if(sampleNumber == 1) {
            currentSubSampleNumber = subFirstSamplePose;
        }
        else if(sampleNumber == 2) {
            currentSubSampleNumber = subSecondSamplePose;
        }
        else {
            return false;
        }



        currentSubSampleNumber.plus(new Twist2d(new Vector2d( xChange, yChange), headingChange));

        if(sampleNumber == 1) {
            subFirstSamplePose.copy(currentSubSampleNumber.position, currentSubSampleNumber.heading);
        }
        else{
            subSecondSamplePose.copy(currentSubSampleNumber.position, currentSubSampleNumber.heading);
        }
        return true;
    }



    public Action scoreInBasket(double xOffset, double yOffset) {//TODO: make sure it doesnt hit side wall when outaking sample
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
                wrist.setPosition(Wrist.SAMPLE_LINE_POSITION_AUTO)
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
                scoreInBasket(1, -1)
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
                scoreInBasket(2, -3)
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
                        wrist.setPosition(Wrist.SAMPLE_LINE_POSITION_AUTO),
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
                wrist.setPosition(Wrist.BASKET_POSITION),

                scoreInBasket(0, 0)
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

