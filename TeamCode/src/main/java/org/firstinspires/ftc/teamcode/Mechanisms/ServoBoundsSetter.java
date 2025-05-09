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
    Servo leftSpintake;
    Servo rightSpintake;
    Timer timer;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        leftArm = hardwareMap.get(Servo.class, "Left Arm");
        rightArm = hardwareMap.get(Servo.class, "Right Arm");
        leftPivotingSlides = hardwareMap.get(Servo.class, "Left Pivoting Slides");
        rightPivotingSlides = hardwareMap.get(Servo.class, "Right Pivoting Slides");
        leftWrist = hardwareMap.get(Servo.class, "Left Wrist");
        rightWrist = hardwareMap.get(Servo.class, "Right Wrist");
        leftSpintake = hardwareMap.get(Servo.class, "Left Spintake");
        rightSpintake = hardwareMap.get(Servo.class, "Right Spintake");

        leftSpintake.setDirection(Servo.Direction.REVERSE);
        leftPivotingSlides.setDirection(Servo.Direction.REVERSE);
        leftWrist.setDirection(Servo.Direction.REVERSE);
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


        telemetry.addLine("Left Bumper - Dual Spintake");
        telemetry.addLine("Left Trigger - Left Spintake");
        telemetry.addLine("Right Bumper - Right Spintake");
        telemetry.addLine("Right Trigger - Left Pivoting Slides");
        telemetry.addLine("Back - Dual Pivoting Slides");

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
        if(gamepad2.back){
            activeServo = "Dual Pivoting Slides";
        }

        if(gamepad1.dpad_left){
            activeServo = "Left Wrist";
        }
        if(gamepad1.dpad_right){
            activeServo = "Right Wrist";
        }


        if(gamepad1.left_bumper){
            activeServo = "Dual Spintake";
        }
        if(gamepad1.left_trigger > 0.5){
            activeServo = "Left Spintake";
        }

        if(gamepad1.right_bumper){
            activeServo = " Right Spintake";
        }
        if(gamepad1.right_trigger > 0.5){
            activeServo = "Left Pivoting Slides";
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
