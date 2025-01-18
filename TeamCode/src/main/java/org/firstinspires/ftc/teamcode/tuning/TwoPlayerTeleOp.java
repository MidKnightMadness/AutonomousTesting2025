package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Actions.Arm;
import org.firstinspires.ftc.teamcode.Actions.Claw;
import org.firstinspires.ftc.teamcode.Actions.VerticalSlides;
import org.firstinspires.ftc.teamcode.Actions.Wrist;
import org.firstinspires.ftc.teamcode.ButtonToggle;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.Timer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

@TeleOp(name="TeleOp - Two Player")
public class TwoPlayerTeleOp extends LinearOpMode {
    VerticalSlides lift;
    Claw claw;
    Arm arm;
    Wrist wrist;
    ButtonToggle right_2Bumper = new ButtonToggle();
    boolean armManualPosition = false;
    boolean wristManualPosition = false;

    double previousTime;
    Timer timer;

    boolean clawClosed = false;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lift = new VerticalSlides(hardwareMap);
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);

        timer = new Timer();
        clawClosed = true;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        double currentTime = timer.updateTime();


        waitForStart();

        double power = 1;


        while (opModeIsActive()) {
            timer.updateTime();

            if (gamepad1.right_bumper) {
                power = 0.5;
            }
            else {
                power = 1;
            }
            //Gamepad 1 controls
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_x * power,
                            -gamepad1.left_stick_y * power
                    ),
                    -gamepad1.right_stick_x * power
            ));


            lift.getLeftMotor().setPower(gamepad1.left_trigger * (gamepad1.left_bumper ? -1 : 1));
            lift.getRightMotor().setPower(gamepad1.left_trigger * (gamepad1.left_bumper ? -1 : 1));

            if (gamepad1.left_bumper && gamepad1.y && gamepad1.left_trigger != 0) {
                while (true) {
                    lift.getRightMotor().setPower(-gamepad1.left_trigger);
                    lift.getRightMotor().setPower(-gamepad1.left_trigger);
                }
            }

            //Gamepad 2 controls
            //Claw
            if(gamepad2.right_bumper){
                claw.release();
            }
            else if (gamepad2.right_trigger > 0.5) {
                claw.grab();
            }


            armManualPosition = false;
            //Arm
            if (gamepad2.y) {
                arm.setBasketPositionTeleOp();
                armManualPosition = true;
            }
            else if (gamepad2.a) {
                arm.setSamplePositionTeleOp();
                armManualPosition = true;
            }
            else if(gamepad2.b){
                arm.setSpecimenOutakePosition();
                armManualPosition = true;
            }
            else if(gamepad2.x){
                arm.setSpecimenIntakePosition();
                armManualPosition = true;
            }

            // -1 to 1 -> 0 to 1
            if(!armManualPosition){
//                arm.setPosition((gamepad2.right_stick_x / 2) + 0.5, 0);
                arm.leftServo.setPosition(arm.leftServo.getPosition() + gamepad2.right_stick_y * timer.getDeltaTime() * 0.5);
                if (gamepad1.right_bumper) {
                    arm.leftServo.setPosition(arm.leftServo.getPosition() + gamepad1.right_trigger * timer.getDeltaTime() * 0.5);
                }
            }

            //Wrist

            wristManualPosition = false;
            if(gamepad2.left_bumper){
                wrist.setSampleSubPos();
                wristManualPosition = true;
            }
            else if(gamepad2.left_trigger > 0.5){
                wrist.servo.setPosition(Wrist.SPECIMEN_INTAKE_POSITION);
                wristManualPosition = true;
            }
            if(!wristManualPosition){
                wrist.servo.setPosition(wrist.servo.getPosition() + gamepad2.left_stick_y * timer.getDeltaTime() * 0.5);
            }


            previousTime = currentTime;


            Pose2d pose = drive.localizer.getPose();

            telemetry.addData("Gamepad 2 left stick x", gamepad2.left_stick_x);

            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("Right stick x", gamepad1.right_stick_x);
            telemetry.addData("Right motor pos", lift.getRightMotor().getCurrentPosition());
            telemetry.addData("Left motor pos", lift.getLeftMotor().getCurrentPosition());

            telemetry.addData("Arm Left Pos", arm.leftServo.getPosition());
            telemetry.addData("Wrist Pos", wrist.servo.getPosition());
            telemetry.addData("Claw Pos", claw.servo.getPosition());

            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
