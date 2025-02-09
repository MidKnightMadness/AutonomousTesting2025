package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Timer;

@Config()
public class SampleClaw {
    final Vector2d CLAW_SERVO_BOUNDS = new Vector2d(0,1);
    public static double RELEASE_POSITION = 0.46;
    public static double GRAB_POSITION = 0.73;


    public Servo servo;
    Timer timer;

    public SampleClaw(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "Sample Claw");
        timer = new Timer();
    }


    public Action grabAction(double waitTime) {
        return new SetPosition(GRAB_POSITION, waitTime);
    }

    public void grab() {
        servo.setPosition(GRAB_POSITION);
    }

    public Action releaseAction(double waitTime) {
        return new SetPosition(RELEASE_POSITION, waitTime);
    }

    public void release() {
        servo.setPosition(RELEASE_POSITION);
    }


    public Action setPositionSmooth(double position, double movementTime) {
        return new SetPosition(position, movementTime);
    }
    public Action setPosition(double position) {
        return new SetPosition(position);
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



            //TODO: CHECK IF movement time = 0
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
