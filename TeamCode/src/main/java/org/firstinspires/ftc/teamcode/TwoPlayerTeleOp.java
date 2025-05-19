package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Util;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.PivotingSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.Spintake;
import org.firstinspires.ftc.teamcode.Mechanisms.VerticalSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.Components.Timer;

import java.util.List;

@TeleOp(name="TeleOp - Two Player", group="A")
@Config
public class TwoPlayerTeleOp extends OpMode {

    public static double STRAFE_ROTATION_FACTOR = 0.1; // rotation while strafing to counteract uneven rotation
    public static double rotationFactor = 0.5;

    VerticalSlides slides;

    PivotingSlides pivotingSlides;
    Arm arm;
    Wrist wrist;
    Spintake spintake;

    boolean inSlideAction;
    boolean inArmAction;
    boolean inSpintakeAction;

    Action slideAction;
    Action spintakeAction;
    Action armAction;

    Timer timer;

    boolean clawClosed = false;

    MecanumDrive drive;


    List<LynxModule> allHubs;
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slides = new VerticalSlides(hardwareMap);
        pivotingSlides = new PivotingSlides(hardwareMap);
        arm = new Arm(hardwareMap, telemetry);
        wrist = new Wrist(hardwareMap);
        spintake = new Spintake(hardwareMap);

        timer = new Timer();
        clawClosed = true;

        MecanumDrive.PARAMS.maxWheelVel = 60;

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        wrist.setPositionDirect(Wrist.INTAKE_POSITION);
        slides.enableFeedforward();

        if (RunOptions.useBulkReads) {
            allHubs = hardwareMap.getAll(LynxModule.class);

            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        }
    }

    @Override
    public void start() {
        timer.updateTime();
    }

    double drivingPower = 1;

    @Override
    public void loop() {
            if (RunOptions.useBulkReads) {
                for (LynxModule module : allHubs) {
                    module.clearBulkCache();
                }
            }

            timer.updateTime();

            gamepad1Controls();

            telemetry.addData("Update Rate (Hz)", 1 / timer.getDeltaTime());
            telemetry.addData("pivoting slides", pivotingSlidesExtension);
            telemetry.addData("Arm target", armTargetAngle);

            if (gamepad1.right_bumper) {
                inSpintakeAction = true;
                spintakeAction = spintake.outtake();
            }
            else if (gamepad1.right_trigger > 0.5) {
                inSpintakeAction = true;
                spintakeAction = spintake.intake();
            }

            runPivotingSlides();
            runWristControls();
            runArmControls();

            runSpintakeControls();

            if (RunOptions.enableTelemetry) {
                PoseVelocity2d vel = drive.updatePoseEstimate();
                Pose2d pose = drive.localizer.getPose();

                telemetry.addData("Slide mode", slideMode);

                telemetry.addLine("-----------Robot Values -----------");
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.addData("velocity (translational)", Math.sqrt(vel.linearVel.x * vel.linearVel.x + vel.linearVel.y * vel.linearVel.y));

                telemetry.addLine("-----------Servo Positions -----------");
                telemetry.addData("Is Arm Running", inArmAction);
                telemetry.addData("Right motor pos", slides.getRightMotor().getCurrentPosition());
                telemetry.addData("Left motor pos", slides.getLeftMotor().getCurrentPosition());

                telemetry.update();
            }
    }

    int slideMode = 0; // 0: sample, 1: specimen


    public void runSpintakeControls() {
        if (inSpintakeAction) {
            inSpintakeAction = spintakeAction.run(packet);

            if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left) {
                inSlideAction = false;
            }

            telemetry.addLine("Running automation");
            telemetry.update();

            return;
        }

    }

    public void runSlideControls() {
        if (inSlideAction) {
            inSlideAction = slideAction.run(packet);

            if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left) {
                inSlideAction = false;
            }

            telemetry.addLine("Running automation");
            telemetry.update();

            return;
        }

        double slidesPower = gamepad1.left_trigger * (gamepad1.left_bumper ? -1 : 1);

        if (slidesPower >= 0 && slidesPower < VerticalSlides.feedforwardPower) {
            slides.getLeftMotor().setPower(VerticalSlides.feedforwardPower);
            slides.getRightMotor().setPower(VerticalSlides.feedforwardPower);
        }
        else {
            slides.getLeftMotor().setPower(slidesPower);
            slides.getRightMotor().setPower(slidesPower);
        }

        telemetry.addData("Vert. slides power", slidesPower);
        // hang
//        if (gamepad1.left_bumper && gamepad1.y && gamepad1.left_trigger != 0) {
//            while (true) {
//                slides.getLeftMotor().setPower(-1);
//                slides.getRightMotor().setPower(-1);
//
//                if (gamepad1.y && gamepad1.a) {
//                    while (true) {
//                        slides.getLeftMotor().setPower(-0.1);
//                        slides.getRightMotor().setPower(-0.1);
//                    }
//                }
//            }
//        }

    }

    public void gamepad1Controls(){
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_x * drivingPower,
                        -gamepad1.left_stick_y * drivingPower
                ),
                (-gamepad1.right_stick_x * rotationFactor + gamepad1.left_stick_x * STRAFE_ROTATION_FACTOR) * drivingPower
        ));

        runSlideControls();

        if (gamepad1.a) {
            slideAction = slidesDown();
            inSlideAction = true;
        }
        else if (gamepad1.y) {
            slideAction = slidesUp();
            inSlideAction = true;
        }
    }

    public Action slidesDown() {
        if (slideMode == 0) {
            return new SequentialAction(
                    slides.bringDown(0.8)
            );
        }

        return new SequentialAction(
                slides.setPosition(VerticalSlides.SPECIMEN_INTAKE, 0.8)
        );
    }

    public Action slidesUp() {
        if (slideMode == 0) {
            return new ParallelAction(
                    slides.liftUp(0.8)
            );
        }

        return new SequentialAction(
                slides.setPosition(VerticalSlides.SPECIMEN_OUTTAKE, 0.8)
        );
    }

    double armTargetAngle;
    double pivotingSlidesExtension = 0;
    public static double armSpeed = 150; // degrees / sec
    public static double pivotingSlidesSpeed = 300;  // mm / s

    public void runWristControls() {
        if (gamepad1.x) {
            wrist.setPositionDirect(0.1863);
        }
        else if (gamepad1.b) {
            wrist.setPositionDirect(Wrist.SERVO_MAX);
        }

        double sign = gamepad2.right_bumper ? 1 : -1;

        if (Math.abs(gamepad2.left_stick_y) > 0.05){
            wrist.setPosition(wrist.leftServo.getPosition() + sign * gamepad2.right_trigger * timer.getDeltaTime() * 0.5);
        }
    }

    public void runPivotingSlides() {
        if (gamepad2.left_trigger > 0.5) {
            pivotingSlidesExtension = PivotingSlides.MAX_EXTENSION_LENGTH;
        }
        else if (gamepad2.left_bumper) {
            pivotingSlidesExtension = 30;
        }

        pivotingSlidesExtension -= gamepad2.left_stick_y * timer.getDeltaTime() * pivotingSlidesSpeed;
        pivotingSlidesExtension = Util.clamp(pivotingSlidesExtension, 0, PivotingSlides.MAX_EXTENSION_LENGTH);
        pivotingSlides.setExtensionLength(pivotingSlidesExtension);


        telemetry.addData("pivoting slides servo", pivotingSlides.leftServo.getPosition());
    }

    public void runArmControls() {
        if (gamepad2.x) {

            armTargetAngle = 0;
            inArmAction = true;
        }
        else if (gamepad2.y) {
            armTargetAngle = 51;
            inArmAction = true;
        }
        else if(gamepad2.b){
            armTargetAngle = 141;
            inArmAction = true;
        }

        arm.setPowerWithFF(-gamepad2.right_stick_y);
    }
}

