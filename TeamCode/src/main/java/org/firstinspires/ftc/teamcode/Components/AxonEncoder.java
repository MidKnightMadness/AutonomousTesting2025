package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

/**
 * AxonEncoder is a utility class for interfacing with analog encoders found in Axon servos.
 * It tracks the absolute and relative orientation of the servo shaft by reading analog voltage values,
 * handling wraparounds, and filtering out invalid spikes. The class provides methods to home, reset,
 * and retrieve the current position in both radians and degrees.
 *
 * This class is designed for use in FTC robotics applications where precise angular tracking of Axon servos is required.
 */

@Config
public class AxonEncoder {
    private static final double MAX_VOLTAGE = 3.3;
    public static double DV_THRESHOLD = 3.3;

    private double previousVoltage = -1;
    private double totalRotations = 0.0;
    public double zeroVoltage = 0.0;
    public double homeVoltage;

    public double numRejections = 0;

    public AnalogInput analogInput;
    Direction direction;

    static LinearInterpolator leftInterpolator = LinearInterpolator.leftAxonVoltageInterpolator();
    static LinearInterpolator rightInterpolator = LinearInterpolator.rightAxonVoltageInterpolator();

    public AxonEncoder(HardwareMap hardwareMap, String analogDeviceName) {
        analogInput = hardwareMap.get(AnalogInput.class, analogDeviceName);
        direction = Direction.FORWARD;

        home();
    }

    public void home() {
        setHome(analogInput.getVoltage());
    }

    public void setHome(double homeVoltage) {
        zeroVoltage = homeVoltage;
        this.homeVoltage = homeVoltage;
        previousVoltage = -1;
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
        homeVoltage = zeroVoltage;
        double currentVoltage = analogInput.getVoltage();

        if (previousVoltage < 0) {
            previousVoltage = currentVoltage;
            return;
        }

        double voltageDifference = currentVoltage - previousVoltage;
        double circularDifference = circularDelta(currentVoltage, previousVoltage);

        // filter out large voltage spikes
        if (Math.abs(circularDifference) > DV_THRESHOLD) {
            numRejections++;
            return;
        }

        // Detect wraparound
        if (voltageDifference > MAX_VOLTAGE / 2) {
            totalRotations -= 1.0;
        } else if (voltageDifference < -MAX_VOLTAGE / 2) {
            totalRotations += 1.0;
        }

        previousVoltage = currentVoltage;
    }

    public void update(double power, boolean left) {
        double currentVoltage = analogInput.getVoltage();

        homeVoltage = zeroVoltage + (left ? leftInterpolator.evaluate(power) : rightInterpolator.evaluate(power));

        if (previousVoltage < 0) {
            previousVoltage = currentVoltage;
            return;
        }

        double voltageDifference = currentVoltage - previousVoltage;
        double circularDifference = circularDelta(currentVoltage, previousVoltage);

        // filter out large voltage spikes
        if (Math.abs(circularDifference) > DV_THRESHOLD) {
            numRejections++;
            return;
        }

        // Detect wraparound
        if (voltageDifference > MAX_VOLTAGE / 2) {
            totalRotations -= 1.0;
        } else if (voltageDifference < -MAX_VOLTAGE / 2) {
            totalRotations += 1.0;
        }

        previousVoltage = currentVoltage;
    }

    public double getAbsolutePositionRadians() {
        double fractionalRotation = (previousVoltage - homeVoltage) / MAX_VOLTAGE;
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

