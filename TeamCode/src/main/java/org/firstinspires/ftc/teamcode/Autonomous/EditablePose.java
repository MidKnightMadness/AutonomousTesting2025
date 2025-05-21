package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class EditablePose {

    public double x;
    public double y;
    public double heading;

    public Vector2d vector2d;
    public Pose2d pose2d;

    public EditablePose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = Math.toRadians(heading);
        this.vector2d = new Vector2d(x, y);
        this.pose2d = new Pose2d(x, y, this.heading);
    }
}
