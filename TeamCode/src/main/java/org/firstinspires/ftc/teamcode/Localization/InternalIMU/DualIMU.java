package org.firstinspires.ftc.teamcode.Localization.InternalIMU;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

@Deprecated
public class DualIMU {
    public IMU imuControl;
    public IMU imuExpansion;

    private static DualIMU instance;

    public static  RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

    public static IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
    public static DualIMU getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new DualIMU();



            BNO055IMUNew.Parameters expansionParameters = new BNO055IMUNew.Parameters(orientationOnRobot);
            expansionParameters.calibrationDataFile = "AdafruitIMUCalibration.json";

            instance.imuControl = hardwareMap.get(IMU.class, "imuControl");
            instance.imuExpansion = hardwareMap.get(IMU.class, "imuExpansion");

            instance.imuControl.initialize(parameters);
            instance.imuExpansion.initialize(expansionParameters);
        }

        return instance;
    }
}
