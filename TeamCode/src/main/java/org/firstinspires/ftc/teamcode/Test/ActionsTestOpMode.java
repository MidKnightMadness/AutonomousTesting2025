package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.FourSampleAuto;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.CRArm;
import org.firstinspires.ftc.teamcode.Mechanisms.PivotingSlides;


@TeleOp(group="Test")
public class ActionsTestOpMode extends OpMode {
    Arm arm;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = new Arm(hardwareMap);
        arm.setInitPosition();
    }

    @Override
    public void start() {

        Actions.runBlocking(
                new SequentialAction(
                        arm.setAngleSmooth(51),
                        new SleepAction(1),
                        arm.setAngleSmooth(141),
                        new SleepAction(1),
                        arm.setAngleSmooth(0)
                )
        );
    }

    @Override
    public void loop() {

        telemetry.update();
    }
}
