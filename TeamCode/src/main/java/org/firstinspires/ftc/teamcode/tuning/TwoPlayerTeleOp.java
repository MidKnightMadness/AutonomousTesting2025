package org.firstinspires.ftc.teamcode.tuning;

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
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Actions.Arm;
import org.firstinspires.ftc.teamcode.Actions.SampleClaw;
import org.firstinspires.ftc.teamcode.Actions.SpecimenClaw;
import org.firstinspires.ftc.teamcode.Actions.TurnTable;
import org.firstinspires.ftc.teamcode.Actions.VerticalSlides;
import org.firstinspires.ftc.teamcode.Actions.Wrist;
import org.firstinspires.ftc.teamcode.ButtonToggle;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.Timer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="TeleOp - Two Player")
@Config
public class TwoPlayerTeleOp extends OpMode {
    public static double STRAFE_ROTATION_FACTOR = 0.1; // add rotation while strafing to counteract uneven rotation

    VerticalSlides lift;
    SampleClaw sampleClaw;
    SpecimenClaw specimenClaw;
    Arm arm;
    Wrist wrist;
    TurnTable turnTable;
    boolean armManualPosition = false;
    boolean wristManualPosition = false;
    Timer timer;

    boolean clawClosed = false;
    FtcDashboard dash = FtcDashboard.getInstance();

    enum Action_Types {
        WRIST,
        ARM,
        CLAW,
        TURNTABLE
    }

    MecanumDrive drive;
    double currentTime;
    double previousTime;
    double startTime;
    double updateRate = 0;

    List<Action> runningActions = new ArrayList<>();
    List<Action_Types> runningActionTypes = new ArrayList<>();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lift = new VerticalSlides(hardwareMap);
        sampleClaw = new SampleClaw(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        specimenClaw = new SpecimenClaw(hardwareMap);
        turnTable = new TurnTable(hardwareMap);

        timer = new Timer();
        clawClosed = true;
        runningActionTypes = new ArrayList<>();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        turnTable.servo.setPosition(TurnTable.NEUTRAL_POS);
        arm.leftServo.setPosition(Arm.STRAIGHT_UP_POSITION);
        arm.rightServo.setPosition(Arm.STRAIGHT_UP_POSITION);

        startTime = timer.updateTime();
        previousTime = 0;

    }
    double power = 0.8;
    @Override
    public void loop(){
            TelemetryPacket packet = new TelemetryPacket();

            gamepad1Controls();
            gamepad2Controls();


            previousTime = currentTime;
            currentTime = timer.updateTime();
            updateRate = 1/ (currentTime - previousTime);


            Pose2d pose = drive.localizer.getPose();

            telemetry.addData("Update Rate", updateRate);
            telemetry.addData("Current Time", currentTime);


            telemetry.addLine("-----------Robot Values -----------");

            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            telemetry.addLine("-----------Servo Positions -----------");
            telemetry.addData("Arm Left Pos", arm.leftServo.getPosition());
            telemetry.addData("Arm Right Pos", arm.rightServo.getPosition());
            telemetry.addData("Wrist Pos", wrist.servo.getPosition());
            telemetry.addData("Sample Claw Pos", sampleClaw.servo.getPosition());
            telemetry.addData("Specimen Claw Pos", specimenClaw.servo.getPosition());
            telemetry.addData("Turntable Wrist Pos", turnTable.servo.getPosition());

            telemetry.addData("Arm Manual", armManualPosition);
            telemetry.addData("Is Arm Running", isArmRunning);
            telemetry.addData("Is Wrist Running", isWristRunning);
            telemetry.addLine("-----------Gamepad Values -----------");
            telemetry.addData("Gamepad 2 left stick x", gamepad2.left_stick_x);
            telemetry.addData("Gamepad 2 Right Stick y", gamepad2.right_stick_y);
            telemetry.addData("Right stick x", gamepad1.right_stick_x);
            telemetry.addData("Right motor pos", lift.getRightMotor().getCurrentPosition());
            telemetry.addData("Left motor pos", lift.getLeftMotor().getCurrentPosition());

            telemetry.update();


            List<Action> newActions = new ArrayList<>();
            List<Action_Types> newActionTypes = new ArrayList<>();

            for (int i = 0; i < runningActions.size(); i++){
                Action action = runningActions.get(i);
                Action_Types type = runningActionTypes.get(i);

                action.preview(packet.fieldOverlay());
                boolean stillRunning = action.run(packet);

                if(!stillRunning){
                    if(type == Action_Types.ARM){
                        isArmRunning = false;
                    }
                    if(type == Action_Types.WRIST){
                        isWristRunning = false;
                    }
                }


                if (stillRunning){
                    newActions.add(action);
                    newActionTypes.add(type);
                }
            }

            runningActions = newActions;
            runningActionTypes = newActionTypes;

            packet.addLine("Running Actions: " + runningActions);
            packet.addLine("Running Actions Types: " + runningActionTypes);
            dash.sendTelemetryPacket(packet);
    }

    public void gamepad1Controls(){

        if (gamepad1.x) {
            power = 0.5;
        }
        else {
            power = 1;
        }

        //TODO: Fix strafing within submersible
        //Gamepad 1 controls
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_x * power,
                        -gamepad1.left_stick_y * power
                ),
                (-gamepad1.right_stick_x + gamepad1.left_stick_x * STRAFE_ROTATION_FACTOR) * power
        ));


        lift.getLeftMotor().setPower(gamepad1.left_trigger * (gamepad1.left_bumper ? -1 : 1));
        lift.getRightMotor().setPower(gamepad1.left_trigger * (gamepad1.left_bumper ? -1 : 1));

        if (gamepad1.left_bumper && gamepad1.y && gamepad1.left_trigger != 0) {
            lift.getLeftMotor().setPower(-gamepad1.left_trigger);
            lift.getRightMotor().setPower(-gamepad1.left_trigger);
        }

        double turnTableDirection = 1;
        if (gamepad1.right_bumper) {
            turnTableDirection = -1;
        }

        if(gamepad1.right_trigger > 0.05){
            turnTable.servo.setPosition(turnTable.servo.getPosition() + turnTableDirection * gamepad1.right_trigger * timer.getDeltaTime() * 0.5);
        }

        if (gamepad1.a) {

        }
    }


    public Action scoreInBasket() {
        drive.localizer.setPose(new Pose2d(new Vector2d(0, 0), 0));

        return new SequentialAction(
            new ParallelAction(
                    wrist.setPosition(Wrist.STRAIGHT_POSITION)

            )
        );
    }


    String activeClaw = "Sample Claw";
    boolean isArmRunning;
    boolean isWristRunning;

    public void gamepad2Controls(){

        double deltaTime = 0;

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
                runningActionTypes.add(Action_Types.CLAW);
            }
        }

        else{
            if (gamepad2.right_bumper) {
                specimenClaw.release();
            } else if (gamepad2.right_trigger > 0.5) {
                specimenClaw.grab();
            }
        }


        armManualPosition = false;
        //Arm
        if (gamepad2.y && !isArmRunning) {
            Actions.runBlocking(arm.setPositionSmooth(Arm.BASKET_POSITION_MAIN, 0.5));
            runningActionTypes.add(Action_Types.ARM);
            armManualPosition = true;
        }
        else if (gamepad2.a && !isArmRunning) {
            Actions.runBlocking(arm.setPositionSmooth(Arm.SAMPLE_INTAKE_POSITION_MAIN, 1));
            runningActionTypes.add(Action_Types.ARM);
            armManualPosition = true;
        }
        else if(gamepad2.x && !isArmRunning){
            Actions.runBlocking(arm.setPositionSmooth(Arm.INIT_AUTO_POS, 1));
            runningActionTypes.add(Action_Types.ARM);
            armManualPosition = true;
        }
        else if(gamepad2.b && !isArmRunning){
            Actions.runBlocking(arm.setPositionSmooth(Arm.SPECIMEN_INTAKE_POSITION_MAIN, 0.5));
            runningActionTypes.add(Action_Types.ARM);
            armManualPosition = true;
        }


        //Arm
        if(!isArmRunning && Math.abs(gamepad2.right_stick_y) > 0.05){
            arm.leftServo.setPosition(arm.leftServo.getPosition() - gamepad2.right_stick_y * timer.getDeltaTime() * 0.5);
            arm.rightServo.setPosition(arm.leftServo.getPosition() - gamepad2.right_stick_y * timer.getDeltaTime() * 0.5);
//            if (gamepad1.right_bumper) {
//                arm.setPosition(arm.leftServo.getPosition() + gamepad1.right_trigger * timer.getDeltaTime() * 0.5);
//            }
        }

        if(Math.abs(gamepad2.left_stick_y) > 0.05){
            wrist.servo.setPosition(wrist.servo.getPosition() + gamepad2.left_stick_y * timer.getDeltaTime() * 0.5);
        }

        telemetry.addData("Delta Time", deltaTime);
    }
}

