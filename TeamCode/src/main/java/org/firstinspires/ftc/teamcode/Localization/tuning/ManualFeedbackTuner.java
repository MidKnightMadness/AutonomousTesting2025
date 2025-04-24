package org.firstinspires.ftc.teamcode.Localization.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Localization.InternalIMU.ThreeDeadWheelIMULocalizer;
import org.firstinspires.ftc.teamcode.Localization.InternalIMU.TwoDeadWheelLocalizer;

@Config
public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 50;

    public static int MODE = 1; // 0: axial, 1 : lateral, 2: heading
    public static Vector2d tuningOffset = new Vector2d(20, 10);
    public static double headingDifference = Math.toRadians(90);

    MecanumDrive drive;
    Pose2d startingPose;

    @Override
    public void runOpMode()  {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            startingPose = new Pose2d(-60, -60, 0);
            drive = new MecanumDrive(hardwareMap, startingPose);
            
            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (drive.localizer instanceof ThreeDeadWheelIMULocalizer) {
                if (ThreeDeadWheelIMULocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelIMULocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelIMULocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
            waitForStart();



            while (opModeIsActive()) {
                Actions.runBlocking(
                        tuneAction()
                );
            }
        }
    }

    public Action tuneAction() {
        if (MODE == 0) return axial();
        if (MODE == 1) return lateral();
        if (MODE == 2) return turn();

        return combine();
    }


    public Action combine() {
        return drive.actionBuilder(startingPose)
                .strafeToLinearHeading(new Vector2d(startingPose.position.x + tuningOffset.x, startingPose.position.y + tuningOffset.y), startingPose.heading.toDouble() + headingDifference)
                .strafeToLinearHeading(startingPose.position, startingPose.heading)
                .build();
    }

    public Action axial() {
        return drive.actionBuilder(startingPose)
                .lineToX(startingPose.position.x + DISTANCE)
                .lineToX(startingPose.position.x)
                .build();
    }

    public Action lateral() {
        return drive.actionBuilder(startingPose)
                .strafeTo(new Vector2d(startingPose.position.x, startingPose.position.y + DISTANCE))
                .strafeTo(startingPose.position)
                .build();
    }

    public Action turn() {
        return drive.actionBuilder(startingPose)
                .turnTo(Math.toRadians(180))
                .turnTo(0)
                .build();
    }
}
