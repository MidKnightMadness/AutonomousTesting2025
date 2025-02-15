package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;


//Class to define sample in sub zone
public class Area {
    Pose2d samplePos;
    Vector2d[] areaCoordinates;//TL, TR, BL, BR
    Rotation2d turnTableHeading;
    //Coordinate System from bottom left of submarine defined at (0,0)

    //Define an area by samplePos and a tolerance
    public Area(Pose2d samplePos, double xTolerance, double yTolerance){
        this.samplePos = samplePos;

        turnTableHeading = samplePos.heading;
    }

    public Area(Vector2d frontLeft, Vector2d frontRight, Vector2d backLeft, Vector2d backRight, Rotation2d heading){
        areaCoordinates[0] = frontLeft;
        areaCoordinates[1] = frontRight;
        areaCoordinates[2] = backLeft;
        areaCoordinates[3] = backRight;
        turnTableHeading = heading;

    }


    public Vector2d[] getAreaCoordinates() {
        return areaCoordinates;
    }
    public Pose2d getSamplePos(){
        return samplePos;
    }

    public String toString(){
        return samplePos.toString();
    }
}
