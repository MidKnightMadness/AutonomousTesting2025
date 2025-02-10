package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Timer;

@Config
public class TurnTable {
    //Wrist should be all throughout driver controlled in teleop except couple buttons for specimen and sample preset positions

    Timer timer;
    public static double RIGHT_BOUND = 0.086;
    public static double LEFT_BOUND = 0.81;
    public static double THIRD_SAMPLE_POS = 0.28;
    public static double NEUTRAL_POS = 0.475; //Parallel to odo left and rightwheels

    public Servo servo;
    ElapsedTime elapsedTime;

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

            packet.addLine("Time: " + (timer.updateTime() - startTime));

            if (movementTime != 0) {
                double timeSinceStart = timer.updateTime() - startTime;
                double percentOfMovement = Math.min(1, timeSinceStart / movementTime);
                double intermediatePoint = (targetPosition - startPosition) * percentOfMovement + startPosition;


                packet.addLine("Position " + intermediatePoint);

                servo.setPosition(intermediatePoint);
                return timeSinceStart < movementTime;
            } else {
                packet.addLine("Position " + targetPosition);
                servo.setPosition(targetPosition);
                return false;
            }
        }
    }
}
