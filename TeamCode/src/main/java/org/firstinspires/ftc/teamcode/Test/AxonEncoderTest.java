package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Components.Timer;

@TeleOp
@Config
public class AxonEncoderTest extends OpMode {

    Timer timer;
    Arm controller;

    @Override
    public void init() {
        controller = new Arm(hardwareMap, telemetry);

        timer = new Timer();
    }

    public static double axon1Value;
    public static double axon2Value;

    public static double armSetRotation;

    public boolean controlIndividual = false;

    public static double axon1Power = 0;
    public static double axon2Power = 0;

    @Override
    public void loop() {
        timer.updateTime();

        if (controlIndividual) {
            if (axon1Power != 0 || axon2Power != 0) {
                controller.leftServo.setPower(axon1Power);
                controller.rightServo.setPower(axon2Power);
            }
            else {
                controller.setIndividualPositions(axon1Value, axon2Value);
                telemetry.addData("Axon 1 Set Value", axon1Value);
                telemetry.addData("Axon 2 Set Value", axon2Value);
            }
        }
        else {
            controller.update(armSetRotation);
            telemetry.addData("Arm set rotation", armSetRotation);
        }


        telemetry.addLine("\nEncoder readings");

        telemetry.addData("Axon 1 Zero", controller.leftEncoder.zeroVoltage);
        telemetry.addData("Axon 2 Zero", controller.rightEncoder.zeroVoltage);

        telemetry.addData("Axon 1 Voltage", controller.leftEncoder.getVoltage());
        telemetry.addData("Axon 2 Voltage", controller.rightEncoder.getVoltage());

        telemetry.addData("Axon 1 Position", controller.leftEncoder.getAbsolutePositionDegrees());
        telemetry.addData("Axon 2 Position", controller.rightEncoder.getAbsolutePositionDegrees());
        telemetry.update();
    }
}
