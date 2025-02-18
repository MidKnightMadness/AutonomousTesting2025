package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Twist2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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
import com.qualcomm.robotcore.hardware.ColorSensor;


//Start at the left of the 2nd tile
@Config
@Autonomous(name = "FiveSampleAuto")
public class FiveSampleAuto extends FourSampleAuto {

    public static Pose2d scoringPose = new Pose2d(new Vector2d(8.5, 25), Math.toRadians(130));

    public static Pose2d firstSamplePose = new Pose2d(new Vector2d( 19.75, 17.5),Math.toRadians(0));
    public static Pose2d secondSamplePose = new Pose2d(new Vector2d(18.75, 27.25), Math.toRadians(0));
    public static Pose2d thirdSamplePose = new Pose2d(new Vector2d(23.5, 24.75), Math.toRadians(45));

    public static Pose2d parkingPose = new Pose2d(new Vector2d(60, -8), Math.toRadians(90));

    //Sub Gamepad Initialization Poses
    public static Pose2d subFirstSamplePose = new Pose2d(new Vector2d(0,0), Math.toRadians(0));
    public static Pose2d subSecondSamplePose = new Pose2d(new Vector2d(0,0), Math.toRadians(0));


    public double firstSubHeading = 0;
    public double secondSubHeading = 0;

    public boolean subOneFinalized;
    public boolean subTwoFinalized;


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
    double currentTime;
    RevColorSensorV3 clawColorSensor;


    @Override
    public void init() {
        sampleClaw = new SampleClaw(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        turnTable = new TurnTable(hardwareMap);
        slides = new VerticalSlides(hardwareMap);
        mecanumDrive = new MecanumDrive(hardwareMap, startingPose, telemetry);

        sampleClaw.grab();
        arm.setInitPosition();
        wrist.setInitPosition();
        turnTable.setInitPosition();
        slides.resetEncoders();

        clawColorSensor = hardwareMap.get(RevColorSensorV3.class, "Claw Color Sensor");

        timer = new Timer();
        currentTime = timer.updateTime();
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
//        if(gamepad1.left_bumper){
//            if(mode.equals("Sample Pos")){
//                mode = "Rectangular coordinates";
//            }
//            else{
//                mode = "Sample Pos";
//            }
//        }

        if (gamepad1.left_bumper){//set 1st sub sample
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

            if(gamepad1.b){
                headingChange += gamepadInterval;
            }
            else if(gamepad1.x){
                headingChange -= gamepadInterval;
            }
            if(gamepad1.left_stick_y > 0.2){
                xToleranceChange += gamepad1.left_stick_y * gamepadInterval * timer.getDeltaTime();
            }
            else if(gamepad1.right_stick_y > 0.2){
                yToleranceChange += gamepad1.right_stick_y * gamepadInterval * timer.getDeltaTime();
            }


            if(gamepad1.left_trigger > 0.5){
                subOneFinalized = true;
            }

            if(gamepad1.right_trigger > 0.5){
                subTwoFinalized = true;
            }

            if(mode.equals("Sample Pos")){//Change samplePosition based on dpad values
                changeSamplePos(xChange, yChange, headingChange, settingSubSampleNumber);
                if(settingSubSampleNumber == 1 && subOneFinalized == false){
                    firstSubHeading += headingChange;
                    areaOne = new Area(subFirstSamplePose, xTolerence + xToleranceChange, yTolerence + yToleranceChange);
                }
                else if(settingSubSampleNumber == 2 && subTwoFinalized == false){
                    secondSubHeading += headingChange;
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
        telemetry.addLine("-------------------------------------------------");
        telemetry.addData("Sub One Finalized", subOneFinalized);
        telemetry.addData("Sub One Finalized", subTwoFinalized);


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
        if(sampleNumber != 1 || sampleNumber != 2) {
            return false;
        }

        if(sampleNumber == 1) {
            subFirstSamplePose.plus(new Twist2d(new Vector2d( xChange, yChange), headingChange));
        }
        else{
            subSecondSamplePose.plus(new Twist2d(new Vector2d( xChange, yChange), headingChange));
        }
        return true;
    }

    public Action subSampleSearch(Area area, double heading, double timeSearch){
        //TODO: Create method and way to search the area with a given heading for the sample and a given amount of time allowed to search
        return null;
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

