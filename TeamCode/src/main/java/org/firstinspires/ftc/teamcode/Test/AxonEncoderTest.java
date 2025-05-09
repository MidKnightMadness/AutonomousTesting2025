package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Components.ArmController;
import org.firstinspires.ftc.teamcode.Components.AxonEncoder;
import org.firstinspires.ftc.teamcode.Components.Timer;

@TeleOp
@Config
public class AxonEncoderTest extends OpMode {

    public CRServo axon1;
    public CRServo axon2;
    Timer timer;

    ArmController controller;

    AxonEncoder axon1Tracker;
    AxonEncoder axon2Tracker;

    @Override
    public void init() {
        axon1 = hardwareMap.get(CRServo.class, "Left Arm");
        axon2 = hardwareMap.get(CRServo.class, "Right Arm");

        axon1Tracker = new AxonEncoder(hardwareMap, "Arm Left Encoder", 0);
        axon2Tracker = new AxonEncoder(hardwareMap, "Arm Right Encoder", 0);

        controller = new ArmController(hardwareMap,0, 0, 0, 0, 0, 0, telemetry);

        timer = new Timer();
    }

    public static double axon1Value;
    public static double axon2Value;

    @Override
    public void loop() {
        timer.updateTime();
/*

        axon1.setPower(axon1Value);
        axon2.setPower(axon2Value);
*/
        controller.setIndividualPositions(axon1Value, axon2Value);

        double axon1Voltage = axon1Tracker.getVoltage();
        double axon2Voltage = axon2Tracker.getVoltage();

        axon1Tracker.update();
        axon2Tracker.update();


        telemetry.addData("Axon 1 Set Power", axon1Value);
        telemetry.addData("Axon 2 Set Power", axon2Value);

        telemetry.addLine("Encoder readings");
        telemetry.addData("Axon 1 Voltage", controller.leftEncoder.getVoltage());
        telemetry.addData("Axon 2 Voltage", controller.rightEncoder.getVoltage());

        telemetry.addData("Axon 1 Position", controller.leftEncoder.getAbsolutePositionDegrees());
        telemetry.addData("Axon 2 Position", controller.rightEncoder.getAbsolutePositionDegrees());

        telemetry.addData("Axon 1 Error", controller.leftController.previousError);
        telemetry.addData("Axon 2 Error", controller.rightController.previousError);
        telemetry.update();
    }
}
