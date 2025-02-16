package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;

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
        public static double barYCoordinate;
        public static double subXLowerBound;
        public static double subXUpperBound;
    }

    public static double pickUpRadius = 14.53;  // in inches

    public IKResult solve(Pose2d samplePose) {
        // validate reachability
        if (samplePose.position.x > FieldConstants.subXUpperBound || samplePose.position.x < FieldConstants.subXLowerBound) {
            return new IKResult();
        }

        if (samplePose.position.y < FieldConstants.barYCoordinate - pickUpRadius) {
            return new IKResult();  // todo: increase radius by allowing wrist extension
        }

        return new IKResult();
    }
}
