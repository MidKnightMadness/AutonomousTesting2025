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
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

@TeleOp(name="LocalizationTest2")
public class LocalizationTest extends LinearOpMode {
    VerticalSlides lift;
    Claw claw;
    Arm arm;
    Wrist wrist;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lift = new VerticalSlides(hardwareMap);
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_x,
                            -gamepad1.left_stick_y
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            lift.getLeftMotor().setPower(gamepad1.left_trigger * (gamepad1.left_bumper ? -1 : 1));
            lift.getRightMotor().setPower(gamepad1.left_trigger * (gamepad1.left_bumper ? -1 : 1));

            if (gamepad1.y) {
                arm.setBasketPosition();
            }
            else if (gamepad1.a) {
                arm.setSamplePosition();
            }

            if (gamepad1.x) {
                claw.servo.setPosition(Claw.GRAB_POSITION);
            }
            else if (gamepad1.b) {
                claw.servo.setPosition(Claw.RELEASE_POSITION);
            }


            wrist.servo.setPosition(gamepad1.right_trigger);

            Pose2d pose = drive.localizer.getPose();

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
