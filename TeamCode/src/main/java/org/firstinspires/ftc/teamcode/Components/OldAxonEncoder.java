package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@Config
public class OldAxonEncoder {
    private static final double MAX_VOLTAGE = 3.3;
    public static double DV_THRESHOLD = 0.4;

    private double previousVoltage = -1;
    private double totalRotations = 0.0;
    public double zeroVoltage = 0.0;

    public AnalogInput analogInput;
    Direction direction;

    public OldAxonEncoder(HardwareMap hardwareMap, String analogDeviceName) {
        analogInput = hardwareMap.get(AnalogInput.class, analogDeviceName);
        direction = Direction.FORWARD;

        home();
    }

    public void home() {
        setHome(analogInput.getVoltage());
    }

    public void setHome(double homeVoltage) {
        zeroVoltage = homeVoltage;
        previousVoltage = zeroVoltage;
        totalRotations = 0.0;
    }

    public double circularDelta(double curr, double prev) {
        // shift so that 0 wrap is in the middle
        double half   = MAX_VOLTAGE / 2.0;
        double raw    = curr - prev + half;
        // bring into [0, MAX_V)
        raw = raw - Math.floor(raw / MAX_VOLTAGE) * MAX_VOLTAGE;
        // shift back into [−half, +half)
        return raw - half;
    }

    public void update() {
        double currentVoltage = analogInput.getVoltage();

        if (previousVoltage < 0) {
            previousVoltage = currentVoltage;
            return;
        }

        double voltageDifference = currentVoltage - previousVoltage;
        double circularDifference = circularDelta(currentVoltage, previousVoltage);

        // filter out large voltage spikes
        if (Math.abs(circularDifference) > DV_THRESHOLD) return;

        // Detect wraparound
        if (voltageDifference > MAX_VOLTAGE / 2) {
            totalRotations -= 1.0;
        } else if (voltageDifference < -MAX_VOLTAGE / 2) {
            totalRotations += 1.0;
        }

        previousVoltage = currentVoltage;
    }

    public double getAbsolutePositionRadians() {
        double fractionalRotation = (previousVoltage - zeroVoltage) / MAX_VOLTAGE;
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

