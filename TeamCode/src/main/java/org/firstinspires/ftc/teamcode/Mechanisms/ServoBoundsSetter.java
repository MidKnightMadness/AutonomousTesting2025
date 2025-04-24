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

    Servo leftArmServo;
    Servo rightArmServo;
    Servo sampleClaw;
    Servo specimenClaw;
    Servo wristServo;
    Servo turnTable;

    Timer timer;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        sampleClaw = hardwareMap.get(Servo.class, "Sample Claw");
        specimenClaw = hardwareMap.get(Servo.class, "Specimen Claw");
        leftArmServo = hardwareMap.get(Servo.class, "Arm Left");
        rightArmServo = hardwareMap.get(Servo.class, "Arm Right");
        wristServo  = hardwareMap.get(Servo.class, "Wrist");
        turnTable = hardwareMap.get(Servo.class, "Turn Table");

        timer = new Timer();
    }

    boolean manualControl = false;
    String activeServo = "Wrist";
    double servoPosition = 0.5;


    @Override
    public void loop() {
        telemetry.addLine("A - Left Arm");
        telemetry.addLine("X - Right Arm");
        telemetry.addLine("Y - Sample Claw");
        telemetry.addLine("B - Wrist");
        telemetry.addLine("Right Bumper - Dual Arm");
        telemetry.addLine("Left Bumper - Turn Table");
        telemetry.addLine("Left Trigger - Specimen Claw");

        if (gamepad1.a) {
            activeServo = "Left arm";
        }
        if (gamepad1.x) {
            activeServo = "Right arm";
        }
        if (gamepad1.y) {
            activeServo = "Sample Claw";
        }
        if(gamepad1.b){
            activeServo = "Wrist";
        }
        if(gamepad1.right_bumper){
            activeServo = "Dual Arm";
        }
        if(gamepad1.left_bumper){
            activeServo = "Turn Table";
        }
        if(gamepad1.left_trigger > 0.5){
            activeServo = "Specimen Claw";
        }

        timer.updateTime();

        if (gamepad1.dpad_up) {
            manualControl = false;
            servoPosition += 0.25 * timer.getDeltaTime();
        }

        else if (gamepad1.dpad_down) {
            manualControl = false;
            servoPosition -= 0.25 * timer.getDeltaTime();
        }

        else if(gamepad1.dpad_left){
            manualControl = true;
            leftArmServo.setPosition(0);
            rightArmServo.setPosition(0);
        }

        else if(gamepad1.dpad_right){
            manualControl = true;
            leftArmServo.setPosition(0.6);
            rightArmServo.setPosition(0.6);
        }

        if(!manualControl) {
            if (activeServo.equals("Left arm")) {
                leftArmServo.setPosition((servoPosition));
            } else if (activeServo.equals("Right arm")) {
                rightArmServo.setPosition(servoPosition);
            } else if (activeServo.equals("Wrist")) {
                wristServo.setPosition(servoPosition);
            } else if (activeServo.equals("Sample Claw")) {
                sampleClaw.setPosition(servoPosition);
            } else if(activeServo.equals("Specimen Claw")){
                specimenClaw.setPosition(servoPosition);
            }
            else if(activeServo.equals("Turn Table")){
                turnTable.setPosition(servoPosition);
            }
            else if(activeServo.equals("Dual Arm")) {
                leftArmServo.setPosition(servoPosition);
                rightArmServo.setPosition(servoPosition);
            }
        }

        telemetry.addData("Dpad Right", gamepad1.dpad_right);
        telemetry.addData("Dpad Up", gamepad1.dpad_up);
        telemetry.addData("Dpad Left", gamepad1.dpad_left);
        telemetry.addData("Active servo", activeServo);
        telemetry.addData("Servo position", servoPosition);
        telemetry.update();
    }
}
