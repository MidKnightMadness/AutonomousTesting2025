package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.Kinematics.Kinematics;

@Config
public class TurnTable {
    //Wrist should be all throughout driver controlled in teleop except couple buttons for specimen and sample preset positions

    Timer timer;
    public static double RIGHT_BOUND = Kinematics.turnTableOrientationToPosition(Math.toRadians(-90));;
    public static double LEFT_BOUND = Kinematics.turnTableOrientationToPosition(Math.toRadians(90));
    public static double THIRD_SAMPLE_POS = Kinematics.turnTableOrientationToPosition(Math.toRadians(-45));;
    public static double NEUTRAL_POS = Kinematics.turnTableOrientationToPosition(0);;

    public Servo servo;

    public TurnTable(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "Turn Table");

        timer = new Timer();
    }

    public Action setPosition(double position) {
        return new TurnTable.SetPosition(position);
    }

    public Action setPositionSmooth(double position, double movementTime){
        return new TurnTable.SetPosition(position, movementTime);
    }

    public void setInitPosition() {
        servo.setPosition(LEFT_BOUND);
    }


    public class SetPosition implements Action {
        private final double targetPosition;
        private final double movementTime;
        double startTime;
        double startPosition;
        boolean initialized = false;

        public SetPosition(double position) {
            this.targetPosition = position;
            this.movementTime = 0;
        }

        public SetPosition(double position, double movementTime) {
            this.targetPosition = position;
            this.movementTime = movementTime;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                startTime = timer.updateTime();
                startPosition = servo.getPosition();
                initialized = true;
            }

            if (movementTime != 0) {
                double timeSinceStart = timer.updateTime() - startTime;
                double percentOfMovement = Math.min(1, timeSinceStart / movementTime);
                double intermediatePoint = (targetPosition - startPosition) * percentOfMovement + startPosition;

                servo.setPosition(intermediatePoint);
                return timeSinceStart < movementTime;
            } else {
                servo.setPosition(targetPosition);
                return false;
            }
        }
    }
}
