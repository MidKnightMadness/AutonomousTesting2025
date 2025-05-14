package org.firstinspires.ftc.teamcode.Kinematics;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.OutdatedPrograms.OldArm;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.Kinematics.Kinematics.RobotConstants;

public class InverseKinematics {


    public static class IKResult {
        public boolean isReachable;
        public double armServoPosition;
        public double wristServoPosition;
        public double slidesTicks;
        public double turntablePosition;
        public Pose2d robotPose;
        public String message;

        public IKResult(boolean isReachable, double armServoPosition, double wristServoPosition, double turntablePosition, double slidesTicks, Pose2d driveTrainPosition) {
            this.isReachable = isReachable;
            this.armServoPosition = armServoPosition;
            this.wristServoPosition = wristServoPosition;
            this.turntablePosition = turntablePosition;
            this.slidesTicks = slidesTicks;
            this.robotPose = driveTrainPosition;
        }
        public IKResult(String message) {
            this();
            this.message = message;
        }

        public IKResult() {
            this.isReachable = false;
            this.armServoPosition = 0;
            this.wristServoPosition = 0;
            this.slidesTicks = 0;
            this.robotPose = new Pose2d(0, 0, 0);
        }

    }

    public Action setArmOrientationWithEndEffectorAngle(OldArm arm, Wrist wrist, double armOrientation, double endEffectorOrientation) {
        return new ParallelAction(
                arm.setPositionSmooth(Kinematics.armOrientationToPosition(armOrientation)),
                wrist.setPosition(Kinematics.wristOrientationToPosition(endEffectorOrientation - armOrientation))
        );
    }

    public static class FieldConstants {
        public static double barYCoordinate = -18.75;
        public static double subXLowerBound = 41;
        public static double subXUpperBound = 89;
    }

    public static double pickUpRadius = 14.53;  // in inches
    public static double pickUpArmOrientation = Math.toRadians(-31.824);
    public static double pickUpWristOrientation = Math.toRadians(-58.184);


    public static double distance(Vector2d a, Vector2d b) {
        return Math.hypot(a.x - b.x, a.y - b.y);
    }

    public static IKResult solve(Pose2d samplePose) {
        // within submersible bounds
        if (!isBetween(samplePose.position.x, FieldConstants.subXLowerBound, FieldConstants.subXUpperBound) || samplePose.position.y > FieldConstants.barYCoordinate) {
            return new IKResult("Position not within submersible zone bounds");
        }

        // sample too far to reach
        if (samplePose.position.y < FieldConstants.barYCoordinate - pickUpRadius) {
            return new IKResult("Sample position unreachable: coordinate too far");  // TODO: increase radius by allowing wrist extension
        }

        if (distance(samplePose.position, new Vector2d(FieldConstants.subXLowerBound + RobotConstants.ROBOT_WIDTH, FieldConstants.barYCoordinate)) > pickUpRadius &&
                distance(samplePose.position, new Vector2d(FieldConstants.subXUpperBound - RobotConstants.ROBOT_WIDTH, FieldConstants.barYCoordinate)) > pickUpRadius) {
            return new IKResult("Sample position unreachable: coordinate too far");
        }

        // simple case: pickup with robot heading -90 deg
        if (isBetween(samplePose.position.x, FieldConstants.subXLowerBound + RobotConstants.ROBOT_WIDTH / 2, FieldConstants.subXUpperBound - RobotConstants.ROBOT_WIDTH / 2)) {
            return new IKResult(true, Kinematics.armOrientationToPosition(pickUpArmOrientation), Kinematics.wristOrientationToPosition(pickUpWristOrientation),
                    Kinematics.turnTableOrientationToPosition(samplePose.heading.toDouble() + Math.toRadians(90)), 0,
                    new Pose2d(samplePose.position.x, samplePose.position.y + pickUpRadius + RobotConstants.ROBOT_LENGTH / 2, Math.toRadians(-90)));
        }

        // variable robot heading
        double r = pickUpRadius;
        double w = RobotConstants.ROBOT_WIDTH / 2;

        double R = Math.hypot(r, w);
        double cY = FieldConstants.barYCoordinate - samplePose.position.y;
        double cX = FieldConstants.subXLowerBound - samplePose.position.x;

        double theta1;
        double theta2;
        double robotHeading;

        // left side
        if (samplePose.position.x < FieldConstants.subXLowerBound + w) {
            theta1 = Math.atan(-r / w) - Math.acos(cY / R) + 2 * Math.PI; // left corner sub x lower bound
            theta2 = Math.atan(-w / r) + Math.acos(cX / R) + Math.PI;  // right corner on bar
            robotHeading = (theta1 + theta2) / 2;
        }
        else if (samplePose.position.x > FieldConstants.subXUpperBound - w) {
            theta1 = -Math.atan(w / r) + Math.asin(-cY / R) + 2 * Math.PI; // left corner on bar
            theta2 = Math.atan(w / r) + Math.acos(cX / R) + Math.PI; // right corner on sub x upper bound
            robotHeading = (theta1 + theta2) / 2;
        }
        else {
            return new IKResult("Unexpected error: sample position not within pickup position");
        }

        // check math errors
        if (Double.isNaN(robotHeading) || Double.isInfinite(robotHeading)) return new IKResult("Invalid robot heading from calculation");

        // check if corners of robot intersect with submersible bounds
        Vector2d armPos = samplePose.position.plus(new Vector2d(-Math.cos(robotHeading), -Math.sin(robotHeading)).times(r));
        Vector2d leftCorner = armPos.plus(new Vector2d(-Math.sin(robotHeading), Math.cos(robotHeading)).times(w));
        Vector2d rightCorner = armPos.plus(new Vector2d(Math.sin(robotHeading), -Math.cos(robotHeading)).times(w));

        if (!isBetween(leftCorner.x, FieldConstants.subXLowerBound, FieldConstants.subXUpperBound) || leftCorner.y < FieldConstants.barYCoordinate) {
            return new IKResult("Sample position unreachable: left corner outside of zone");
        }

        if (!isBetween(rightCorner.x, FieldConstants.subXLowerBound, FieldConstants.subXUpperBound) || rightCorner.y < FieldConstants.barYCoordinate) {
            return new IKResult("Sample position unreachable: right corner outside of zone");
        }

        Vector2d robotPos = armPos.plus(new Vector2d(-Math.cos(robotHeading), -Math.sin(robotHeading)).times(RobotConstants.ROBOT_LENGTH / 2));

        return new IKResult(true, Kinematics.armOrientationToPosition(pickUpArmOrientation), Kinematics.wristOrientationToPosition(pickUpWristOrientation),
                Kinematics.turnTableOrientationToPosition(samplePose.heading.toDouble() - (robotHeading - 2 * Math.PI)), 0,
                new Pose2d(robotPos, robotHeading));
    }


    static boolean isBetween(double val, double lower, double upper) {
        return val > lower && val < upper;
    }
}
