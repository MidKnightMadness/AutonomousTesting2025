package org.firstinspires.ftc.teamcode.Localization.messages;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Autonomous.FiveSampleAuto;

public final class Drawing {
    private Drawing() {}


    public static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

//        t = new Pose2d(t.position.x -FiveSampleAuto.sampleOffset.x, t.position.y - FiveSampleAuto.sampleOffset.y, t.heading.toDouble());

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }
}
