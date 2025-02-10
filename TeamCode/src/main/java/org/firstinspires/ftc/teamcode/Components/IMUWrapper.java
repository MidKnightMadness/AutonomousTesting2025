package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class IMUWrapper
{
    IMU imuControlHub;
    BNO055IMU imuExpansionHub;


    public IMUWrapper(HardwareMap hardwareMap) {
        BNO055IMU.Parameters expansionIMUParameters = new BNO055IMU.Parameters();
        expansionIMUParameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        expansionIMUParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        expansionIMUParameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample OpMode
        expansionIMUParameters.loggingEnabled      = true;
        expansionIMUParameters.loggingTag          = "IMU";
        expansionIMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imuControlHub = hardwareMap.get(IMU.class, "imuControl");
        imuExpansionHub = hardwareMap.get(BNO055IMU.class, "imuExpansion");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imuControlHub.initialize(new IMU.Parameters(orientationOnRobot));
        imuExpansionHub.initialize(expansionIMUParameters);
    }

    public static double normalizeAngle(double angle) {
        return mod((angle + Math.PI), 2 * Math.PI) - Math.PI;
    }

    public static double mod(double num, double divisor) {
        return num - Math.floor(num / divisor) * divisor;
    }

    public double getControlHubYaw() {
        return imuControlHub.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    public double getYawVelocity() {
        return imuExpansionHub.getAngularVelocity().zRotationRate;
    }
    public double getExpansionHubYaw() {
        return imuExpansionHub.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }


}
