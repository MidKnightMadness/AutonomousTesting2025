package org.firstinspires.ftc.teamcode.Camera;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class VisionCoordTransformation {

    //Transforms Robot coordinates relative from the center of the front panel of the robot to the robot center
    public static double[] transformSampleCoordFromRobotCenter(double[] samplePoseRelToRobotFrontCenter, double robotRadius){
        double x = samplePoseRelToRobotFrontCenter[0];
        double y = samplePoseRelToRobotFrontCenter[1] +  robotRadius;
        //simply offsetting the y value by the radius
        return new double[]{x, y};
    }

    public static double[] transformRelativeCoordToWorld(double[] sampleRelCoord, Pose2d robotPose){
        double[] sampleWorldCoord = new double[]{0,0};
        sampleWorldCoord[0] = robotPose.position.x + Math.cos(robotPose.heading.toDouble()) * sampleRelCoord[0] - Math.sin(robotPose.heading.toDouble()) * sampleRelCoord[1];
        sampleWorldCoord[1] = robotPose.position.y + Math.sin(robotPose.heading.toDouble()) * sampleRelCoord[0] + Math.cos(robotPose.heading.toDouble()) * sampleRelCoord[1];
        return sampleWorldCoord;
    }


}
