package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Actions.TurnTable;
import org.firstinspires.ftc.teamcode.Actions.Wrist;

public class EndEffectorPosition {
    // values in inches
    public static class RobotConstants {
        public static double ARM_LENGTH = 350.52 / 25.4;
        public static double WRIST_LENGTH = 68.65 / 25.4;
        public static double ARM_HEIGHT = 269.352 / 25.4;
        public static double ARM_DISTANCE_TO_EDGE = 35.16 / 25.4;
        public static double COLOR_SENSOR_THICKNESS = 16.8 / 25.4;
    }

    public static class ArmConstants {
        static final double SERVO_RANGE_DEGREES = 255;
        static final double GEAR_RATIO = 40d / 48;
        static final double ARM_DEGREES = SERVO_RANGE_DEGREES / GEAR_RATIO;
        static final double ARM_RADIANS = Math.toRadians(ARM_DEGREES);

        public static double SERVO_ZERO_POSITION = 0.6; // todo set value
    }

    public static class WristConstants {
        static final double SERVO_RANGE_DEGREES = 255;
        static final double GEAR_RATIO = 40d / 48;

        static final double WRIST_DEGREES = SERVO_RANGE_DEGREES / GEAR_RATIO;
        static final double WRIST_RADIANS = Math.toRadians(WRIST_DEGREES);

        public static double SERVO_ZERO_POSITION = 0.66; // todo set value
    }

    public static class TurntableConstants {
        static final double SERVO_ZERO = TurnTable.NEUTRAL_POS;
        static final double SERVO_LEFT = TurnTable.LEFT_BOUND;

        static final double SERVO_RANGE = Math.PI / 2 / (SERVO_LEFT - SERVO_ZERO);
    }

    public static class SlidesConstants {
        public static final double WINCH_RADIUS = 17.5 / 25.4;
        public static final double WINCH_DIAMETER = WINCH_RADIUS * 2 * Math.PI;
        public static final double GEAR_RATIO = 1 / 15.2;
        public static final double TICKS_PER_REV = 28 / GEAR_RATIO;

        public static double IN_PER_TICK = WINCH_DIAMETER / TICKS_PER_REV;
    }

    public Pose2d updatePosition(double slidesTicks, double armServoPosition, double wristServoPosition, double turnTableServoPosition) {
        double armOrientation = (ArmConstants.SERVO_ZERO_POSITION - armServoPosition) * ArmConstants.ARM_RADIANS;
        double wristOrientation = (wristServoPosition - WristConstants.SERVO_ZERO_POSITION) * WristConstants.WRIST_RADIANS;

        double posX = RobotConstants.ARM_DISTANCE_TO_EDGE + Math.cos(armOrientation) * RobotConstants.ARM_LENGTH +
                      Math.cos(wristOrientation) * (RobotConstants.WRIST_LENGTH + RobotConstants.COLOR_SENSOR_THICKNESS);

        double posY = slidesTicks * SlidesConstants.IN_PER_TICK +
                      RobotConstants.ARM_HEIGHT + Math.sin(armOrientation) * RobotConstants.ARM_LENGTH +
                      Math.sin(wristOrientation) * (RobotConstants.WRIST_LENGTH + RobotConstants.COLOR_SENSOR_THICKNESS);

        return new Pose2d(posX, posY, wristOrientation);
    }
}
