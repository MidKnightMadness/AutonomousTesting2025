package org.firstinspires.ftc.teamcode.Kinematics;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.TurnTable;
import org.firstinspires.ftc.teamcode.Mechanisms.VerticalSlides;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;

@Config
public class Kinematics {
    // values in inches
    public static Pose2d endEffectorPosition;

    public static class RobotConstants {
        public static double ARM_LENGTH = 350.52 / 25.4;
        public static double WRIST_LENGTH = 68.65 / 25.4;
        public static double ARM_HEIGHT = 269.352 / 25.4;
        public static double ARM_DISTANCE_TO_EDGE = 35.16 / 25.4;
        public static double COLOR_SENSOR_THICKNESS = 0 / 25.4; // 16.8mm but flush with surface

        public static double ROBOT_WIDTH = 14;
        public static double ROBOT_LENGTH = 16;
        public static double ROBOT_DIAGONAL = Math.hypot(ROBOT_LENGTH, ROBOT_WIDTH);
    }

    public static class ArmConstants {
        static final double SERVO_RANGE_DEGREES = 255;

        static final double GEAR_RATIO = 40d / 48;
        static final double ARM_DEGREES = SERVO_RANGE_DEGREES / GEAR_RATIO;
        static final double ARM_RADIANS = Math.toRadians(ARM_DEGREES);

        public static double SERVO_ZERO_POSITION = 0.631; // todo set value

        public static double SERVO_MAX = 1;
        public static double SERVO_MIN = 0;
    }

    public static class WristConstants {
        static final double SERVO_RANGE_DEGREES = 300;
        static final double GEAR_RATIO = 40d / 48;

        static final double WRIST_DEGREES = SERVO_RANGE_DEGREES / GEAR_RATIO;
        static final double WRIST_RADIANS = Math.toRadians(WRIST_DEGREES);

        public static double SERVO_ZERO_POSITION = 0.503; // todo set value
        public static double SERVO_MAX = 1;
        public static double SERVO_MIN = 0.2316;
    }

    public static class TurntableConstants {
        static final double SERVO_ZERO = TurnTable.NEUTRAL_POS;
        static final double SERVO_MAX = TurnTable.LEFT_BOUND;
        static final double SERVO_MIN = TurnTable.RIGHT_BOUND;

        static final double SERVO_RANGE_DEGREES = 300;
        static final double TURNTABLE_RADIANS = Math.toRadians(SERVO_RANGE_DEGREES);
    }

    public static class SlidesConstants {
        public static final double WINCH_RADIUS = 17.5 / 25.4;
        public static final double WINCH_DIAMETER = WINCH_RADIUS * 2 * Math.PI;
        public static final double GEAR_RATIO = 1 / 15.2;
        public static final double TICKS_PER_REV = 28 / GEAR_RATIO;

        public static double IN_PER_TICK = WINCH_DIAMETER / TICKS_PER_REV;
    }

    public static Pose2d updatePosition(double slidesTicks, double armServoPosition, double wristServoPosition, double turnTableServoPosition) {
        double armOrientation = armPositionToOrientation(armServoPosition);
        double wristOrientation = wristPositionToOrientation(wristServoPosition);

        double posX = RobotConstants.ARM_DISTANCE_TO_EDGE + Math.cos(armOrientation) * RobotConstants.ARM_LENGTH +
                      Math.cos(wristOrientation) * (RobotConstants.WRIST_LENGTH + RobotConstants.COLOR_SENSOR_THICKNESS);

        double posY = slidesTicks * SlidesConstants.IN_PER_TICK +
                      RobotConstants.ARM_HEIGHT + Math.sin(armOrientation) * RobotConstants.ARM_LENGTH +
                      Math.sin(wristOrientation) * (RobotConstants.WRIST_LENGTH + RobotConstants.COLOR_SENSOR_THICKNESS);

        endEffectorPosition = new Pose2d(posX, posY, wristOrientation + armOrientation);

        return endEffectorPosition;
    }


    public static Pose2d updatePosition(VerticalSlides slides, Arm arm, Wrist wrist, TurnTable turntable) {
        return updatePosition((slides.getLeftMotor().getCurrentPosition() + slides.getRightMotor().getCurrentPosition()) / 2d,
                                arm.leftServo.getPosition(),
                                wrist.servo.getPosition(),
                                turntable.servo.getPosition());
    }

    public static double armPositionToOrientation(double position) {
        return (ArmConstants.SERVO_ZERO_POSITION - position) * ArmConstants.ARM_RADIANS;
    }

    public static double wristPositionToOrientation(double position) {
        return (position - WristConstants.SERVO_ZERO_POSITION) * WristConstants.WRIST_RADIANS;
    }


    public static double armOrientationToPosition(double orientation) {
        double position = ArmConstants.SERVO_ZERO_POSITION - orientation / ArmConstants.ARM_RADIANS;
        if (position > 1) position = 1;
        if (position < 0) position = 0;

        return position;
    }

    public static double turnTableOrientationToPosition(double orientation) {
        double position = TurntableConstants.SERVO_ZERO + orientation / TurntableConstants.TURNTABLE_RADIANS;
        if (position > TurntableConstants.SERVO_MAX) position = TurntableConstants.SERVO_MAX;
        if (position <  TurntableConstants.SERVO_MIN) position = TurntableConstants.SERVO_MIN;

        return position;
    }

    public static double turnTablePositionToOrientation(double orientation) {
        return TurntableConstants.SERVO_ZERO + orientation / TurntableConstants.TURNTABLE_RADIANS;
    }

    public static double wristOrientationToPosition(double orientation) {
        double position = WristConstants.SERVO_ZERO_POSITION + orientation / WristConstants.WRIST_RADIANS;
        if (position > WristConstants.SERVO_MAX) position = WristConstants.SERVO_MAX;
        if (position < WristConstants.SERVO_MIN) position = WristConstants.SERVO_MIN;

        return position;
    }

    public static double slidesInchesToTicks(double inches) {
        return inches / SlidesConstants.IN_PER_TICK;
    }
}
