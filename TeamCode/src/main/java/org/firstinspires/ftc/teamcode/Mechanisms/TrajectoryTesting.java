package org.firstinspires.ftc.teamcode.Mechanisms;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.FourSampleAuto;
import org.firstinspires.ftc.teamcode.Kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp
@Config
public class TrajectoryTesting extends OpMode {

    MecanumDrive mecanumDrive;
    Pose2d initialPose;

    public static double tangentDegrees = 90;
    public static double dX = 10;
    public static double dY = 10;

    TelemetryPacket telemetryPacket;

    @Override
    public void init() {
        initialPose = new Pose2d(0, 0, 0);
        mecanumDrive = new MecanumDrive(hardwareMap, initialPose);

        telemetryPacket = new TelemetryPacket();

        telemetry.setAutoClear(true);
    }

    @Override
    public void start() {
        mecanumDrive.localizer.setPose(FourSampleAuto.startingPose);
    }

    Action currentAction;
    boolean isInAction;
    String actionLabel;

    @Override
    public void loop() {
        mecanumDrive.updatePoseEstimate();
        double tangent = Math.toRadians(tangentDegrees);

        if (isInAction) {
            isInAction = currentAction.run(telemetryPacket);
            telemetry.addData("Current Action", actionLabel);
            return;
        }

        if (gamepad1.a) {
            try { Thread.sleep(500); } catch (InterruptedException e) { }

            isInAction = true;
            actionLabel = "Both X and Y";
            currentAction = mecanumDrive.actionBuilder().setTangent(tangent).lineToX(dX).lineToY(dY).build();
        }
        else if (gamepad1.x) {
            try { Thread.sleep(500); } catch (InterruptedException e) { }

            isInAction = true;
            actionLabel = "Only X";
            currentAction = mecanumDrive.actionBuilder().setTangent(tangent).lineToX(dX).build();
        }
        else if (gamepad1.y) {
            try { Thread.sleep(500); } catch (InterruptedException e) { }

            isInAction = true;
            actionLabel = "Only Y";
            currentAction = mecanumDrive.actionBuilder().setTangent(tangent).lineToY(dY).build();
        }
    }
}
