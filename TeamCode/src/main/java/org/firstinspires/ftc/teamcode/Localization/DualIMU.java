package org.firstinspires.ftc.teamcode.Localization;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

public class DualIMU {
    public IMU imuControl;
    public IMU imuExpansion;

    private static DualIMU instance;

    public static DualIMU getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new DualIMU();

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

            BNO055IMUNew.Parameters expansionParameters = new BNO055IMUNew.Parameters(orientationOnRobot);
            expansionParameters.calibrationDataFile = "AdafruitIMUCalibration.json";

            instance.imuControl = hardwareMap.get(IMU.class, "imuControl");
            instance.imuExpansion = hardwareMap.get(IMU.class, "imuExpansion");

            instance.imuControl.initialize(new IMU.Parameters(orientationOnRobot));
            instance.imuExpansion.initialize(expansionParameters);
        }

        return instance;
    }
}
