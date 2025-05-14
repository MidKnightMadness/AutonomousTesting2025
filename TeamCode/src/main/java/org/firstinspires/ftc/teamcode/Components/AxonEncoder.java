package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
public class AxonEncoder {
    private static final double MAX_VOLTAGE = 3.3;

    private double previousVoltage = -1;
    private double totalRotations = 0.0;
    public double zeroVoltage = 0.0;

    public AnalogInput analogInput;
    Direction direction;

    public AxonEncoder(HardwareMap hardwareMap, String analogDeviceName) {
        analogInput = hardwareMap.get(AnalogInput.class, analogDeviceName);
        direction = Direction.FORWARD;

        home();
    }

    public void home() {
        zeroVoltage = analogInput.getVoltage();
        previousVoltage = zeroVoltage;
        totalRotations = 0.0;
    }

    public void update() {
        double currentVoltage = analogInput.getVoltage();

        if (previousVoltage < 0) {
            previousVoltage = currentVoltage;
            return;
        }

        double voltageDifference = currentVoltage - previousVoltage;

        // Detect wraparound
        if (voltageDifference > MAX_VOLTAGE / 2) {
            totalRotations -= 1.0;
        } else if (voltageDifference < -MAX_VOLTAGE / 2) {
            totalRotations += 1.0;
        }

        previousVoltage = currentVoltage;
    }

    public double getAbsolutePositionRadians() {
        double fractionalRotation = (previousVoltage - zeroVoltage + MAX_VOLTAGE) % MAX_VOLTAGE / MAX_VOLTAGE;
        double sign = direction == Direction.FORWARD ? 1: -1;
        return sign * (totalRotations + fractionalRotation) * 2 * Math.PI;
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public double getAbsolutePositionDegrees() {
        return Math.toDegrees(getAbsolutePositionRadians());
    }

    public void reset() {
        previousVoltage = -1;
        totalRotations = 0.0;
        zeroVoltage = 0.0;
    }

    public double getVoltage() {
        return analogInput.getVoltage();
    }
}

