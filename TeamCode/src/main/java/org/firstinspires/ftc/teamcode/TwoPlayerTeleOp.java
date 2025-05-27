package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Autonomous.FourSampleAuto.CYCLE;
import static org.firstinspires.ftc.teamcode.Autonomous.FourSampleAuto.RESET;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Util;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.PivotingSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.Spintake;
import org.firstinspires.ftc.teamcode.Mechanisms.VerticalSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.Components.Timer;

@TeleOp(name="TeleOp - Two Player", group="A")
@Config
@Configurable
public class TwoPlayerTeleOp extends OpMode {

    public static double STRAFE_ROTATION_FACTOR = 0.15; // rotation while strafing to counteract uneven rotation
    public static double rotationFactor = 0.5;

    VerticalSlides slides;
    PivotingSlides pivotingSlides;
    Arm arm;
    Wrist wrist;
    Spintake spintake;

    boolean inPresetAction;
    boolean inArmAction;
    boolean inSpintakeAction;

    Action presetAction;
    Action spintakeAction;
    Action armAction;

    Timer timer;

    MecanumDrive drive;

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slides = new VerticalSlides(hardwareMap);
        pivotingSlides = new PivotingSlides(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        spintake = new Spintake(hardwareMap);

        timer = new Timer();

        MecanumDrive.PARAMS.maxWheelVel = 60;

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        wrist.setPosition(Wrist.INTAKE_POSITION);
        slides.enableFeedforward();
    }

    @Override
    public void start() {
        timer.updateTime();
    }

    double drivingPower = 1;

    @Override
    public void loop() {
            timer.updateTime();

            gamepad1Controls();

            telemetry.addData("Update Rate (Hz)", 1 / timer.getDeltaTime());
            telemetry.addData("pivoting slides", pivotingSlidesExtension);

            runPivotingSlides();
            runWristControls();
            runArmControls();

            runSpintakeControls();

            if (RunOptions.enableTelemetry) {
                PoseVelocity2d vel = drive.updatePoseEstimate();
                Pose2d pose = drive.localizer.getPose();

                telemetry.addLine("-----------Robot Values -----------");
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.addData("velocity (translational)", Math.sqrt(vel.linearVel.x * vel.linearVel.x + vel.linearVel.y * vel.linearVel.y));

                telemetry.addLine("-----------Servo Positions -----------");
                telemetry.addData("Is Arm Action", inArmAction);
                telemetry.addData("Right motor pos", slides.getRightMotor().getCurrentPosition());
                telemetry.addData("Left motor pos", slides.getLeftMotor().getCurrentPosition());

                telemetry.update();
            }
    }

    public void runSpintakeControls() {
        if (inSpintakeAction) {
            inSpintakeAction = spintakeAction.run(packet);

            if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left) {
                inSpintakeAction = false;
            }

            telemetry.addLine("Running spintake");
            telemetry.update();

            return;
        }

        if (gamepad1.right_bumper) {
            inSpintakeAction = true;
            spintakeAction = spintake.outtake();
        }
        else if (gamepad1.right_trigger > 0.5) {
            inSpintakeAction = true;
            spintakeAction = spintake.intake();
        }
    }

    public void runSlideControls() {
        if (inPresetAction) {
            inPresetAction = presetAction.run(packet);

            if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left) {
                inPresetAction = false;
            }

            telemetry.addLine("Running slides action");
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
            presetAction = slidesDown();
            inPresetAction = true;
        }
        else if (gamepad1.y) {
            presetAction = slidesUp();
            inPresetAction = true;
        }
    }

    public Action slidesDown() {
        return new SequentialAction(
            wrist.setDegreesAction(90),
            new InstantAction(() -> {
                pivotingSlidesExtension = RESET.pivExtension;
            }),
            new ParallelAction(
                    slides.bringDown(RESET.downPower)
            ));
    }

    public Action slidesUp() {
        return new ParallelAction(
            new SequentialAction(
                slides.setPosition(VerticalSlides.BASKET_BACKWARDS_SCORING, 1),
                wrist.setDegreesAction(CYCLE.wristScoreDeg)
            ),
            arm.setAngleSmooth(14),
            new InstantAction(() -> {
                pivotingSlidesExtension = CYCLE.pivExtension;
            })
        );
    }

    double pivotingSlidesExtension = 0;
    public static double armPower = 1.2;
    public static double pivotingSlidesSpeed = 300;  // mm / s

    public void runWristControls() {
        if (gamepad1.x) {
            wrist.setPosition(0.1863);
        }
        else if (gamepad1.b) {
            wrist.setPosition(Wrist.SERVO_MAX);
        }

        double sign = gamepad2.right_bumper ? 1 : -1;

        if (Math.abs(gamepad2.left_stick_y) > 0.05){
            wrist.setPositionAction(wrist.leftServo.getPosition() + sign * gamepad2.right_trigger * timer.getDeltaTime() * 0.5);
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
        pivotingSlides.setExtension(pivotingSlidesExtension);


        telemetry.addData("Pivoting slides servo", pivotingSlides.leftServo.getPosition());
    }

    public void runArmControls() {
        if (inPresetAction) return;

        if (inArmAction) {
            inArmAction = armAction.run(packet);
            return;
        }

        if (gamepad2.y) {
            armAction = arm.setAngleSmooth(51);
            inArmAction = true;
        }
        else if (gamepad2.b) {
            armAction = arm.setAngleSmooth(130);
            inArmAction = true;
        }
        else if(gamepad2.x){
            armAction = arm.setAngleSmooth(0);
            inArmAction = true;
        }
        else if (gamepad2.a) {
            armAction = arm.setAngleSmooth(143);
            inArmAction = true;
        }

        //Arm
        if(!inArmAction) {
            if (Math.abs(gamepad2.right_stick_y) > 0.05) {
                arm.setPosition(arm.leftServo.getPosition() - gamepad2.right_stick_y * timer.getDeltaTime() * armPower);
            }
        }
    }
}

