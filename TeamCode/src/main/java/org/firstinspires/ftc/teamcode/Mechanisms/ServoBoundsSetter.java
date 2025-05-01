package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class ServoBoundsSetter extends OpMode {

    Servo leftArm;
    Servo rightArm;
    Servo leftPivotingSlides;
    Servo rightPivotingSlides;
    Servo leftWrist;
    Servo rightWrist;
    Servo turnTable;
    Servo leftSpintake;
    Servo rightSpintake;
    Servo specimenClaw;
    Timer timer;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


//        leftArm = hardwareMap.get(Servo.class, "Arm Left");
//        rightArm = hardwareMap.get(Servo.class, "Arm Right");
        leftPivotingSlides = hardwareMap.get(Servo.class, "PivotingSlides Left");
        rightPivotingSlides = hardwareMap.get(Servo.class, "PivotingSlides Right");
//        leftWrist = hardwareMap.get(Servo.class, "Wrist Left");
//        rightWrist = hardwareMap.get(Servo.class, "Wrist Right");
//        turnTable = hardwareMap.get(Servo.class, "Turn Table");
//        leftSpintake = hardwareMap.get(Servo.class, "Spintake Left");
//        rightSpintake = hardwareMap.get(Servo.class, "Spintake Right");
//        specimenClaw = hardwareMap.get(Servo.class, "Specimen Claw");


//        rightSpintake.setDirection(Servo.Direction.REVERSE);
        leftPivotingSlides.setDirection(Servo.Direction.REVERSE);
        timer = new Timer();
    }

    boolean manualControl = false;
    String activeServo = "Dual Pivoting Slides";
    double servoPosition = 0.5;


    @Override
    public void loop() {
        telemetry.addLine("Gamepad 1");
        telemetry.addLine("A - Left Arm");
        telemetry.addLine("X - Right Arm");
        telemetry.addLine("Y - Dual Arm");
        telemetry.addLine("B - Right Pivoting Slides");

        //DPADS - Pivoting Slides and Wrist - Individual
        telemetry.addLine("Dpad Left - Left Wrist");
        telemetry.addLine("Dpad Right - Right Wrist");


        telemetry.addLine("Left Bumper - Turn Table");
        telemetry.addLine("Left Trigger - Specimen Claw");
        telemetry.addLine("Right Bumper - Dual Spintake");
        telemetry.addLine("Right Trigger - Left Pivoting Slides");
        telemetry.addLine("Gamepad 2.A - Dual Pivoting Slides");
        telemetry.addLine("Back - Left Spintake");
        telemetry.addLine("Start - Right Spintake");

        telemetry.addLine("Dpad up - Increase");
        telemetry.addLine("Dpad down - Decrease");


        //Assign active servo
        if (gamepad1.a) {
            activeServo = "Left Arm";
        }
        if (gamepad1.x) {
            activeServo = "Right Arm";
        }
        if (gamepad1.y) {
            activeServo = "Dual Arm";
        }
        if(gamepad1.b){
            activeServo = "Right Pivoting Slides";
        }
        if(gamepad2.a){
            activeServo = "Dual Pivoting Slides";
        }

        if(gamepad1.dpad_left){
            activeServo = "Left Wrist";
        }
        if(gamepad1.dpad_right){
            activeServo = "Right Wrist";
        }


        if(gamepad1.left_bumper){
            activeServo = "Turn Table";
        }
        if(gamepad1.left_trigger > 0.5){
            activeServo = "Specimen Claw";
        }
        if(gamepad1.right_bumper){
            activeServo = "Dual Spintake";
        }
        if(gamepad1.right_trigger > 0.5){
            activeServo = "Left Pivoting Slides";
        }

        if(gamepad1.back){
            activeServo = "Left Spintake";

        }
        else if(gamepad1.start){
            activeServo = "Right Spintake";
        }

        //Set values to servo posiiton
        if(activeServo.equals("Left Arm")){
            leftArm.setPosition(servoPosition);
        }


        else if(activeServo.equals("Right Arm")){
            rightArm.setPosition(servoPosition);
        }
        else if(activeServo.equals("Dual Arm")){
            leftArm.setPosition(servoPosition);
            rightArm.setPosition(servoPosition);
        }

        else if(activeServo.equals("Right Pivoting Slides")){
            rightPivotingSlides.setPosition(servoPosition);
        }
        else if(activeServo.equals("Left Pivoting Slides")){
            leftPivotingSlides.setPosition(servoPosition);
        }
        else if(activeServo.equals("Dual Pivoting Slides")){
            leftPivotingSlides.setPosition(servoPosition);
            rightPivotingSlides.setPosition(servoPosition);
        }

        else if(activeServo.equals("Left Wrist")){
            leftWrist.setPosition(servoPosition);
        }
        else if(activeServo.equals("Right Wrist")){
            rightWrist.setPosition(servoPosition);
        }
        else if(activeServo.equals("Turn Table")){
            turnTable.setPosition(servoPosition);
        }
        else if(activeServo.equals("Specimen Claw")){
            specimenClaw.setPosition(servoPosition);
        }
        else if(activeServo.equals("Dual Spintake")){
            leftSpintake.setPosition(servoPosition);
            rightSpintake.setPosition(servoPosition);
        }
        else if(activeServo.equals("Left Spintake")){
            leftSpintake.setPosition(servoPosition);
        }
        else if(activeServo.equals("Right Spintake")){
            rightSpintake.setPosition(servoPosition);
        }



        timer.updateTime();
        if(gamepad1.dpad_up){
            servoPosition += 0.25 * timer.getDeltaTime();
        }

        if(gamepad1.dpad_down){
            servoPosition -= 0.25 * timer.getDeltaTime();
        }





        telemetry.addData("Dpad Right", gamepad1.dpad_right);
        telemetry.addData("Dpad Up", gamepad1.dpad_up);
        telemetry.addData("Active servo", activeServo);
        telemetry.addData("Servo position", servoPosition);
        telemetry.update();
    }
}
