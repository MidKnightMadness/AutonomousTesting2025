package org.firstinspires.ftc.teamcode.Kinematics;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.Kinematics.Kinematics.RobotConstants;

public class InverseKinematics {

    // Tolerance for checking if the target lies on the reachable circle (in inches)
    private static final double TOLERANCE = 0.5;

    public static class IKResult {
        public boolean isReachable;
        public double armServoPosition;
        public double wristServoPosition;
        public double slidesTicks;
        public Pose2d driveTrainPosition;

        public IKResult(boolean isReachable, double armServoPosition, double wristServoPosition, double slidesTicks, Pose2d driveTrainPosition) {
            this.isReachable = isReachable;
            this.armServoPosition = armServoPosition;
            this.wristServoPosition = wristServoPosition;
            this.slidesTicks = slidesTicks;
            this.driveTrainPosition = driveTrainPosition;
        }

        public IKResult() {
            this.isReachable = false;
            this.armServoPosition = 0;
            this.wristServoPosition = 0;
            this.slidesTicks = 0;
            this.driveTrainPosition = new Pose2d(0, 0, 0);
        }

    }

    public Action setArmOrientationWithEndEffectorAngle(Arm arm, Wrist wrist, double armOrientation, double endEffectorOrientation) {

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

    public IKResult solve(Pose2d samplePose) {
        // validate reachability
        if (samplePose.position.x > FieldConstants.subXUpperBound || samplePose.position.x < FieldConstants.subXLowerBound) {
            return new IKResult();
        }

        if (samplePose.position.y < FieldConstants.barYCoordinate - pickUpRadius) {
            return new IKResult();  // TODO: increase radius by allowing wrist extension
        }

        if (distance(samplePose.position, new Vector2d(FieldConstants.subXLowerBound + RobotConstants.ROBOT_WIDTH, FieldConstants.barYCoordinate)) > pickUpRadius &&
                distance(samplePose.position, new Vector2d(FieldConstants.subXUpperBound - RobotConstants.ROBOT_WIDTH, FieldConstants.barYCoordinate)) > pickUpRadius) {
            return new IKResult();
        }

        // simple case: pickup with robot heading -90 deg
        if (samplePose.position.x < FieldConstants.subXUpperBound - RobotConstants.ROBOT_WIDTH || samplePose.position.x > FieldConstants.subXLowerBound + RobotConstants.ROBOT_WIDTH) {
            return new IKResult(true, Kinematics.armOrientationToPosition(pickUpArmOrientation), Kinematics.wristOrientationToPosition(pickUpWristOrientation), 0,
                    new Pose2d(samplePose.position.x, samplePose.position.y - pickUpRadius, Math.toRadians(-90)));
        }

        // TODO: variable robot heading
        double diagonalAngle = Math.atan(RobotConstants.ROBOT_WIDTH / RobotConstants.ROBOT_LENGTH);
        double changeHeading = 0;

        // robot corners
        double yDistanceFromCenter = -RobotConstants.ROBOT_DIAGONAL / 2 * Math.cos(diagonalAngle + changeHeading);
        double xDistanceFromCenter = RobotConstants.ROBOT_DIAGONAL / 2 * Math.sin(diagonalAngle + changeHeading);

        // Check if the sample is within reach from the left or right edges
        Vector2d leftEdge = new Vector2d(FieldConstants.subXLowerBound + RobotConstants.ROBOT_WIDTH, FieldConstants.barYCoordinate);
        Vector2d rightEdge = new Vector2d(FieldConstants.subXUpperBound - RobotConstants.ROBOT_WIDTH, FieldConstants.barYCoordinate);
        if (distance(samplePose.position, leftEdge) > pickUpRadius && distance(samplePose.position, rightEdge) > pickUpRadius) {
            return new IKResult();
        }

        // Simple case: center region, approach with -90 degrees heading
        if (samplePose.position.x <= FieldConstants.subXUpperBound - RobotConstants.ROBOT_WIDTH && samplePose.position.x >= FieldConstants.subXLowerBound + RobotConstants.ROBOT_WIDTH) {
            return new IKResult(true, Kinematics.armOrientationToPosition(pickUpArmOrientation), Kinematics.wristOrientationToPosition(pickUpWristOrientation), 0,
                    new Pose2d(samplePose.position.x, samplePose.position.y - pickUpRadius, Math.toRadians(-90)));
        }

        // Edge case: calculate required heading
        boolean isLeftEdge = samplePose.position.x < FieldConstants.subXLowerBound + RobotConstants.ROBOT_WIDTH;
        Vector2d edgePoint = isLeftEdge ? leftEdge : rightEdge;

        // Vector from edge to sample
        Vector2d toSample = samplePose.position.minus(edgePoint);
        double edgeDistance = toSample.norm();
        if (edgeDistance > pickUpRadius) {
            return new IKResult();
        }

        // Calculate the angle from the edge to the sample
        double theta = Math.atan2(toSample.y, toSample.x);
        // Adjust robot's heading to point towards the sample while avoiding the bar
        double robotHeading = isLeftEdge ? (-Math.PI/2 + theta) : (Math.PI/2 + theta);

        // Calculate robot's position: move from the edge point along theta by the robot's half diagonal
        double diagonalHalf = Math.hypot(RobotConstants.ROBOT_LENGTH, RobotConstants.ROBOT_WIDTH) / 2;
        double robotX = edgePoint.x + diagonalHalf * Math.cos(robotHeading);
        double robotY = edgePoint.y + diagonalHalf * Math.sin(robotHeading);

        // Check if robot's position keeps all corners above the bar
        double minY = robotY - (RobotConstants.ROBOT_LENGTH * Math.sin(robotHeading) + (RobotConstants.ROBOT_WIDTH/2) * Math.abs(Math.cos(robotHeading)));
        if (minY < FieldConstants.barYCoordinate) {
            return new IKResult();
        }

        return new IKResult(true, Kinematics.armOrientationToPosition(pickUpArmOrientation), Kinematics.wristOrientationToPosition(pickUpWristOrientation), 0,
                new Pose2d(robotX, robotY, robotHeading));
    }
}
