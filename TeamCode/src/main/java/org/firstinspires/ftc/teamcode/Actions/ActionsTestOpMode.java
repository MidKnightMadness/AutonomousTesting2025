package org.firstinspires.ftc.teamcode.Actions;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp
public class ActionsTestOpMode extends OpMode {

    Claw claw;
    VerticalSlides verticalSlides;
    MecanumDrive drive;
    Pose2d initialPose;
    Wrist wrist;
    Arm arm;
    @Override
    public void init() {
        initialPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, initialPose);
        claw = new Claw(hardwareMap);
        wrist = new Wrist(hardwareMap);
        arm = new Arm(hardwareMap);
        verticalSlides = new VerticalSlides(hardwareMap);
    }

    @Override
    public void start() {
        Actions.runBlocking(new SequentialAction(
                arm.setPositionSmooth(Arm.Left.STRAIGHT_UP_POSITION_LEFT, 2),
                arm.setPositionSmooth(Arm.Left.FACING_DOWN_POSITION_AUTO_LEFT, 2)
        ));
    }

    @Override
    public void loop() {

    }



}
