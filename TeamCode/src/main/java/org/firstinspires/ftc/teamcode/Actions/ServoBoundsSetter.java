package org.firstinspires.ftc.teamcode.Actions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp
public class ServoBoundsSetter extends OpMode {

    Servo leftArmServo;
    Servo rightArmServo;
    Servo clawGrabber;
    Servo wristServo;
    Servo turnTable;

    MecanumDrive drive;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        clawGrabber = hardwareMap.get(Servo.class, "Sample Claw");
        leftArmServo = hardwareMap.get(Servo.class, "Arm Left");
        rightArmServo = hardwareMap.get(Servo.class, "Arm Right");
        wristServo  = hardwareMap.get(Servo.class, "Wrist");
        turnTable = hardwareMap.get(Servo.class, "Turn Table");


    }

    boolean manualControl = false;
    String activeServo = "Turn Table";
    double servoPosition = 0.5;
    double currentPos = 0.3;

    @Override
    public void loop() {

//        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
//            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//
//                drive.setDrivePowers(new PoseVelocity2d(
//                        new Vector2d(
//                                -gamepad1.left_stick_x,
//                                -gamepad1.left_stick_y
//                        ),
//                        -gamepad1.right_stick_x
//                ));
//
//                drive.updatePoseEstimate();
//
//                Pose2d pose = drive.localizer.getPose();
//                telemetry.addData("x", pose.position.x);
//                telemetry.addData("y", pose.position.y);
//                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
//                telemetry.addData("Right stick x", gamepad1.right_stick_x);
//
//
//                TelemetryPacket packet = new TelemetryPacket();
//                packet.fieldOverlay().setStroke("#3F51B5");
//                Drawing.drawRobot(packet.fieldOverlay(), pose);
//                FtcDashboard.getInstance().sendTelemetryPacket(packet);
//            }


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


        if (gamepad1.dpad_up) {
            manualControl = false;
            servoPosition += 0.005;
        }

        else if (gamepad1.dpad_down) {
            manualControl = false;
            servoPosition -= 0.005;
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

        if(manualControl == false) {
            if (activeServo.equals("Left arm")) {
                leftArmServo.setPosition((servoPosition));
            } else if (activeServo.equals("Right arm")) {
                rightArmServo.setPosition(servoPosition);
            } else if (activeServo.equals("Wrist")) {
                wristServo.setPosition(servoPosition);
            } else if (activeServo.equals("Sample Claw")) {
                clawGrabber.setPosition(servoPosition);
            } else if(activeServo.equals("Turn Table")){
                turnTable.setPosition(servoPosition);
            }
            else {
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
