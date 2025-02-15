package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.messages.Drawing;
import org.firstinspires.ftc.teamcode.Localization.Localizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Localization.OldThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.Localization.ThreeDeadWheelIMULocalizer;
import org.firstinspires.ftc.teamcode.Localization.ThreeDeadWheelLocalizer;

@TeleOp(name="LocalizationTest")
@Config
public class LocalizationTest extends LinearOpMode {

    BNO055IMU imuExpansion;
    Localizer firstLocalizer;
    Localizer secondLocalizer;
    Localizer twoWheel;

    Timer timer;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startingPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose);
        timer = new Timer();

        HeadingFusion headingFusion = new HeadingFusion(0.05, 5, 0.05);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imuExpansion = hardwareMap.get(BNO055IMU.class, "imuExpansion");
        imuExpansion.initialize(parameters);

        firstLocalizer = new ThreeDeadWheelIMULocalizer(hardwareMap, imuExpansion, MecanumDrive.PARAMS.inPerTick, startingPose);
        secondLocalizer = new OldThreeWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick, startingPose);
        twoWheel = new ThreeDeadWheelLocalizer(hardwareMap, imuExpansion, MecanumDrive.PARAMS.inPerTick, startingPose, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_x,
                            -gamepad1.left_stick_y
                    ),
                    -gamepad1.right_stick_x
            ));

            PoseVelocity2d vel = firstLocalizer.update();
            secondLocalizer.update();
            twoWheel.update();
            timer.updateTime();

            Pose2d pose = firstLocalizer.getPose();

            telemetry.addData("Deltatime", timer.getDeltaTime());
            telemetry.addLine();

            telemetry.addLine("\nIMU Three wheel Localizer");
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d pose2 = secondLocalizer.getPose();

            telemetry.addLine("\nOld Three Wheel Localizer");
            telemetry.addData("x", pose2.position.x);
            telemetry.addData("y", pose2.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose2.heading.toDouble()));

            telemetry.addLine("\nRaw IMU Values");
            telemetry.addData("heading (expansion)", imuExpansion.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

            Pose2d pose3 = twoWheel.getPose();

            telemetry.addLine("\nFiltered 3 wheel Localizer");
            telemetry.addData("x", pose3.position.x);
            telemetry.addData("y", pose3.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose3.heading.toDouble()));

            telemetry.addLine("\nGain scheduled heading");
            telemetry.addData("heading", Math.toDegrees(headingFusion.update(pose.heading.toDouble(), pose2.heading.toDouble(), timer.getDeltaTime())));
            telemetry.addData("IMUGain", headingFusion.getIMUGain());
            telemetry.addData("integratedError", headingFusion.getIntegratedError());

            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
