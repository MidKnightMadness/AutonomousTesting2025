package org.firstinspires.ftc.teamcode.Localization.GoBildaPinpoint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Localization.Localizer;

public class PinpointOdometryLocalizer implements Localizer {

    public GoBildaPinpointDriver odo;

    Pose2d pose;

    public PinpointOdometryLocalizer(HardwareMap hardwareMap, Pose2d startingPose) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(138.874, 33); // using left odometry pod
        // odo.setOffsets(-139.1662678093, 33);  // using right odometry pod

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.recalibrateIMU();
        odo.resetPosAndIMU();
        setPose(startingPose);

    }

    @Override
    public void setPose(Pose2d pose) {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, pose.position.x, pose.position.y, AngleUnit.RADIANS, pose.heading.toDouble()));
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public PoseVelocity2d update() {
        odo.update();

        Pose2D vel = odo.getVelocity();
        Pose2D pos = odo.getPosition();

        this.pose = new Pose2d(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.RADIANS));
        return new PoseVelocity2d(new Vector2d(vel.getX(DistanceUnit.INCH), vel.getY(DistanceUnit.INCH)), vel.getHeading(AngleUnit.RADIANS));
    }

    public static double normalizeAngle(double angle) {
        return mod((angle + Math.PI), 2 * Math.PI) - Math.PI;
    }

    public static double mod(double num, double divisor) {
        return num - Math.floor(num / divisor) * divisor;
    }

}
