package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
public class AxonEncoder {
    private static final double MAX_VOLTAGE = 3.3;

    private double previousVoltage = -1;
    private double totalRotations = 0.0;

    public double degreeOffset;

    public AnalogInput analogInput;

    Direction direction;

    public AxonEncoder(HardwareMap hardwareMap, String analogDeviceName, double degreeOffset) {
        analogInput = hardwareMap.get(AnalogInput.class, analogDeviceName);
        this.degreeOffset = degreeOffset;
        direction = Direction.FORWARD;
    }

    public void setOffset(double offset) {
        this.degreeOffset = offset;
    }


    public void update() {
        double currentVoltage = analogInput.getVoltage();

        if (previousVoltage < 0) {
            // First reading initialization
            previousVoltage = currentVoltage;
            return;
        }

        double voltageDifference = currentVoltage - previousVoltage;

        // Detect wraparound
        if (voltageDifference > MAX_VOLTAGE / 2) {
            // Wrapped from high to low voltage (e.g., 3.2V to 0.1V)
            totalRotations -= 1.0;
        } else if (voltageDifference < -MAX_VOLTAGE / 2) {
            // Wrapped from low to high voltage (e.g., 0.1V to 3.2V)
            totalRotations += 1.0;
        }

        previousVoltage = currentVoltage;
    }

    public double getAbsolutePositionRadians() {
        double fractionalRotation = previousVoltage / MAX_VOLTAGE;
        double sign = direction == Direction.FORWARD ? 1: -1;
        return sign * (totalRotations + fractionalRotation) * 2 * Math.PI + Math.toRadians(degreeOffset);
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public double getAbsolutePositionDegrees() {
        return Math.toDegrees(getAbsolutePositionRadians());
    }

    /**
     * Resets the tracker to zero position.
     */
    public void reset() {
        previousVoltage = -1;
        totalRotations = 0.0;
    }

    public double getVoltage() {
        return analogInput.getVoltage();
    }
}

