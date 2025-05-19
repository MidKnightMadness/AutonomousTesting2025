package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class ServoBoundsSetter extends OpMode {

    ServoImplEx leftPivotingSlides;
    ServoImplEx rightPivotingSlides;
    ServoImplEx leftWrist;
    ServoImplEx rightWrist;
    CRServo leftSpintake;
    CRServo rightSpintake;
    Timer timer;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftPivotingSlides = hardwareMap.get(ServoImplEx.class, "Left Pivoting Slides");
        rightPivotingSlides = hardwareMap.get(ServoImplEx.class, "Right Pivoting Slides");
        leftWrist = hardwareMap.get(ServoImplEx.class, "Left Wrist");
        rightWrist = hardwareMap.get(ServoImplEx.class, "Right Wrist");
        leftSpintake = hardwareMap.get(CRServo.class, "Left Spintake");
        rightSpintake = hardwareMap.get(CRServo.class, "Right Spintake");

        leftSpintake.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftPivotingSlides.setDirection(Servo.Direction.REVERSE);
        rightPivotingSlides.setDirection(Servo.Direction.REVERSE);

        leftWrist.setDirection(Servo.Direction.REVERSE);
        timer = new Timer();
    }

    boolean manualControl = false;
    String activeServo = "";
    double servoPosition = 0.5;

    public void disableAllPowers() {
        leftPivotingSlides.setPwmDisable();
        rightPivotingSlides.setPwmDisable();
        leftWrist.setPwmDisable();
        rightWrist.setPwmDisable();
//        leftSpintake.getController().pwmDisable();
//        rightSpintake.getController().pwmDisable();
    }

    @Override
    public void loop() {
        telemetry.addLine("Gamepad 1");
        telemetry.addLine("B - Right Pivoting Slides");

        //DPADS - Pivoting Slides and Wrist - Individual
        telemetry.addLine("X - Left Wrist");
        telemetry.addLine("Y - Right Wrist");
        telemetry.addLine("A - Dual Wrist");
        telemetry.addLine("Left Bumper - Dual Spintake");
        telemetry.addLine("Left Trigger - Left Spintake");
        telemetry.addLine("Right Bumper - Right Spintake");
        telemetry.addLine("Right Trigger - Left Pivoting Slides");
        telemetry.addLine("Back - Dual Pivoting Slides");

        telemetry.addLine("Dpad up - Increase");
        telemetry.addLine("Dpad down - Decrease");


        //Assign active servo
        if(gamepad1.b){
            activeServo = "Right Pivoting Slides";
        }
        if(gamepad1.back){
            activeServo = "Dual Pivoting Slides";
        }

        if(gamepad1.x){
            activeServo = "Left Wrist";
        }
        if(gamepad1.y){
            activeServo = "Right Wrist";
        }
        if(gamepad1.a){
            activeServo = "Dual Wrist";
        }

        if(gamepad1.right_stick_y > 0.9){
            activeServo = "";
        }

        if(gamepad1.left_bumper){
            activeServo = "Dual Spintake";
        }
        if(gamepad1.left_trigger > 0.5){
            activeServo = "Left Spintake";
        }

        if(gamepad1.right_bumper){
            activeServo = "Right Spintake";
        }
        if(gamepad1.right_trigger > 0.5){
            activeServo = "Left Pivoting Slides";
        }


        if (activeServo.isEmpty()) {
            disableAllPowers();
        }
        else if(activeServo.equals("Right Pivoting Slides")){
            rightPivotingSlides.setPwmEnable();
            rightPivotingSlides.setPosition(servoPosition);
        }
        else if(activeServo.equals("Left Pivoting Slides")){
            leftPivotingSlides.setPwmEnable();
            leftPivotingSlides.setPosition(servoPosition);
        }
        else if(activeServo.equals("Dual Pivoting Slides")){
            leftPivotingSlides.setPwmEnable();
            rightPivotingSlides.setPwmEnable();

            leftPivotingSlides.setPosition(servoPosition);
            rightPivotingSlides.setPosition(servoPosition);
        }
        else if (activeServo.equals("Dual Wrist")) {
            rightWrist.setPwmEnable();
            leftWrist.setPwmEnable();
            rightWrist.setPosition(servoPosition);
            leftWrist.setPosition(servoPosition);
        }
        else if(activeServo.equals("Left Wrist")){
            leftWrist.setPwmEnable();
            leftWrist.setPosition(servoPosition);
        }
        else if(activeServo.equals("Right Wrist")){
            rightWrist.setPwmEnable();
            rightWrist.setPosition(servoPosition);
        }
        else if(activeServo.equals("Dual Spintake")){
            leftSpintake.setPower(servoPosition - 0.5);
            rightSpintake.setPower(servoPosition - 0.5);
        }
        else if(activeServo.equals("Left Spintake")){
            leftSpintake.setPower(servoPosition - 0.5);
        }
        else if(activeServo.equals("Right Spintake")){
            rightSpintake.setPower(servoPosition - 0.5);
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
