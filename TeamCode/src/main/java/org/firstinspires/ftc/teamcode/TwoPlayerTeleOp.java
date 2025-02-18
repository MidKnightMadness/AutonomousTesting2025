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

import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.SampleClaw;
import org.firstinspires.ftc.teamcode.Mechanisms.SpecimenClaw;
import org.firstinspires.ftc.teamcode.Mechanisms.TurnTable;
import org.firstinspires.ftc.teamcode.Mechanisms.VerticalSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.Kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.Components.Timer;

import java.util.List;

@TeleOp(name="TeleOp - Two Player", group="A")
@Config
public class TwoPlayerTeleOp extends OpMode {
    public static double STRAFE_ROTATION_FACTOR = 0.1; // add rotation while strafing to counteract uneven rotation

    public static Vector2d basketTrajectoryIntermediate = new Vector2d(0, 20);
    public static Vector2d basketTrajectoryPosition = new Vector2d(-40, 28);

    public static double rotationFactor = 0.5;

    VerticalSlides slides;
    SampleClaw sampleClaw;
    SpecimenClaw specimenClaw;

    Arm arm;
    Wrist wrist;
    TurnTable turnTable;

    boolean isInActionCommand = false;

    Action activeAction;

    Timer timer;

    boolean clawClosed = false;
    FtcDashboard dash = FtcDashboard.getInstance();

    MecanumDrive drive;

    Action armAction;

    List<LynxModule> allHubs;
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slides = new VerticalSlides(hardwareMap);
        sampleClaw = new SampleClaw(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        specimenClaw = new SpecimenClaw(hardwareMap);
        turnTable = new TurnTable(hardwareMap);

        timer = new Timer();
        clawClosed = true;
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), telemetry);

        if (RunOptions.useBulkReads) {
            allHubs = hardwareMap.getAll(LynxModule.class);

            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        }
    }

    @Override
    public void start() {
        turnTable.servo.setPosition(TurnTable.NEUTRAL_POS);
        arm.leftServo.setPosition(Arm.STRAIGHT_UP_POSITION);
        arm.rightServo.setPosition(Arm.STRAIGHT_UP_POSITION);
        wrist.servo.setPosition(Wrist.BASKET_POSITION);
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

            gamepad1Controls();
            gamepad2Controls();

            PoseVelocity2d vel = drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            telemetry.addData("Update Rate (Hz)", 1 / timer.getDeltaTime());

            if (RunOptions.enableTelemetry) {
                Kinematics.updatePosition(slides, arm, wrist, turnTable);
                Pose2d endEffectorPose = Kinematics.endEffectorPosition;

                telemetry.addData("End effector pose", String.format("(%f, %f)", endEffectorPose.position.x, endEffectorPose.position.y));
                telemetry.addData("End effector rotation", Math.toDegrees(endEffectorPose.heading.toDouble()));

                telemetry.addLine("-----------Robot Values -----------");

                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.addData("velocity (translational)", Math.sqrt(vel.linearVel.x * vel.linearVel.x + vel.linearVel.y * vel.linearVel.y));

                telemetry.addLine("-----------Servo Positions -----------");
                telemetry.addData("Arm Left Pos", arm.leftServo.getPosition());
                telemetry.addData("Arm Right Pos", arm.rightServo.getPosition());
                telemetry.addData("Wrist Pos", wrist.servo.getPosition());
                telemetry.addData("Sample Claw Pos", sampleClaw.servo.getPosition());
                telemetry.addData("Specimen Claw Pos", specimenClaw.servo.getPosition());
                telemetry.addData("Turntable Wrist Pos", turnTable.servo.getPosition());

                telemetry.addData("Is Arm Running", inArmAction);
                telemetry.addData("Right motor pos", slides.getRightMotor().getCurrentPosition());
                telemetry.addData("Left motor pos", slides.getLeftMotor().getCurrentPosition());

                telemetry.update();
            }
    }

    public void runSlideControls() {
        if (isInActionCommand) {
            isInActionCommand = activeAction.run(packet);

            // stop running
            if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left) {
                isInActionCommand = false;
            }

            telemetry.addLine("Running automation");

            telemetry.update();

            return;
        }

        slides.getLeftMotor().setPower(gamepad1.left_trigger * (gamepad1.left_bumper ? -1 : 1));
        slides.getRightMotor().setPower(gamepad1.left_trigger * (gamepad1.left_bumper ? -1 : 1));


        if (gamepad1.left_bumper && gamepad1.y && gamepad1.left_trigger != 0) {
            while (true) {
                slides.getLeftMotor().setPower(-gamepad1.left_trigger);
                slides.getRightMotor().setPower(-gamepad1.left_trigger);

                if (gamepad1.y && gamepad1.a) {
                    while (true) {
                        slides.getLeftMotor().setPower(-0.1);
                        slides.getRightMotor().setPower(-0.1);
                    }

                }
            }
        }

    }

    public void gamepad1Controls(){

        if (gamepad1.x) {
            drivingPower = 0.5;
        }
        else {
            drivingPower = 1;
        }

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_x * drivingPower,
                        -gamepad1.left_stick_y * drivingPower
                ),
                (-gamepad1.right_stick_x * rotationFactor + gamepad1.left_stick_x * STRAFE_ROTATION_FACTOR) * drivingPower
        ));

        runSlideControls();

        double turnTableDirection = 1;
        if (gamepad1.right_bumper) {
            turnTableDirection = -1;
        }

        if(gamepad1.right_trigger > 0.05){
            turnTable.servo.setPosition(turnTable.servo.getPosition() + turnTableDirection * gamepad1.right_trigger * timer.getDeltaTime() * 1);
        }

        if (gamepad1.a) {
            activeAction = slidesDown();
            isInActionCommand = true;
        }
        else if (gamepad1.b) {
            activeAction = slidesUp();
            isInActionCommand = true;
        }
    }

    public Action scoreInBasket() {
        drive.localizer.setPose(new Pose2d(new Vector2d(0, 0), Math.toRadians(-90)));

        return new SequentialAction(
            new ParallelAction(
                    wrist.setPosition(Wrist.STRAIGHT_POSITION),
                    drive.actionBuilderNoCorrection().strafeToLinearHeading(basketTrajectoryIntermediate, Math.toRadians(-90)).build(),
                    turnTable.setPosition(TurnTable.NEUTRAL_POS)
            ),

            new ParallelAction(
                    arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION, 1),
                    drive.actionBuilderNoCorrection(new Pose2d(basketTrajectoryIntermediate, Math.toRadians(-225))).strafeToLinearHeading(basketTrajectoryPosition, Math.toRadians(-225)).build(),
                    wrist.setPosition(Wrist.BASKET_POSITION),
                    slides.liftUp(0.8)
            )
        );
    }

    public Action goToSub(){
        drive.localizer.setPose(new Pose2d(new Vector2d(0, 0), Math.toRadians(-225)));

        return new SequentialAction(
                new ParallelAction(
                        arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION, 1),
                        drive.actionBuilderNoCorrection().strafeToLinearHeading(basketTrajectoryIntermediate, Math.toRadians(-225)).build(),
                        slides.bringDown(0.6),
                        turnTable.setPosition(TurnTable.NEUTRAL_POS),
                        wrist.setPosition(Wrist.SAMPLE_PICKUP_POSITION)
                ),
                arm.setPositionSmooth(Arm.SAMPLE_INTAKE, 0.5),
                new ParallelAction(
                        drive.actionBuilderNoCorrection().strafeToLinearHeading(basketTrajectoryPosition, Math.toRadians(-90)).build()
                )
        );
    }

    public Action slidesDown() {
        return new SequentialAction(
                slides.bringDown(0.5)
        );
    }

    public Action slidesUp() {
        return new ParallelAction(
                slides.liftUp(0.8),
                turnTable.setPosition(TurnTable.NEUTRAL_POS)
        );
    }


    String activeClaw = "Sample Claw";
    boolean inArmAction;

    public void gamepad2Controls(){
        //Change Claws
        if(gamepad2.dpad_right) {
           activeClaw = "Specimen Claw";
        }
        if(gamepad2.dpad_left) {
            activeClaw = "Sample Claw";
        }

        //Claw
        if(activeClaw.equals("Sample Claw")) {
            if (gamepad2.right_bumper) {
                sampleClaw.release();
            } else if (gamepad2.right_trigger > 0.5) {
                sampleClaw.grab();;
            }
        }

        else{
            if (gamepad2.right_bumper) {
                specimenClaw.release();
            } else if (gamepad2.right_trigger > 0.5) {
                specimenClaw.grab();
            }
        }

        runWristControls();
        runArmControls();
    }

    public void runWristControls() {
        if (gamepad2.left_bumper) {
            wrist.servo.setPosition(Wrist.BASKET_POSITION);
            return;
        }

        if(Math.abs(gamepad2.left_stick_y) > 0.05){
            wrist.servo.setPosition(wrist.servo.getPosition() + gamepad2.left_stick_y * timer.getDeltaTime() * 0.5);
        }
    }

    public void runArmControls() {
        if (inArmAction) {
            inArmAction = armAction.run(packet);
            return;
        }

        if (gamepad2.y) {
            armAction = arm.setPositionSmooth(Arm.BASKET_POSITION);
            inArmAction = true;
        }
        else if (gamepad2.a) {
            armAction = arm.setPositionSmooth(Arm.SAMPLE_INTAKE);
            inArmAction = true;
        }
        else if(gamepad2.x){
            armAction = arm.setPositionSmooth(Arm.INIT_AUTO_POS);
            inArmAction = true;
        }
        else if(gamepad2.b){
            armAction = arm.setPositionSmooth(Arm.STRAIGHT_UP_POSITION);
            inArmAction = true;
        }

        //Arm
        if (Math.abs(gamepad2.right_stick_y) > 0.05){
            arm.leftServo.setPosition(arm.leftServo.getPosition() - gamepad2.right_stick_y * timer.getDeltaTime() * 0.5);
            arm.rightServo.setPosition(arm.leftServo.getPosition() - gamepad2.right_stick_y * timer.getDeltaTime() * 0.5);
        }
    }
}

