package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {
    final Vector2d CLAW_SERVO_BOUNDS = new Vector2d(0,1);
    public static double RELEASE_POSITION = 0.46;
    public static double GRAB_POSITION = 0.65;

    public Servo servo;
    ElapsedTime elapsedTime;

    public Claw(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "clawServo");
        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.startTime();
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


    public Action setPosition(double position, double waitTime) {
        return new SetPosition(position, waitTime);
    }

    public class SetPosition implements Action {
        private final double position;
        private final double waitTime;

        double startTime;
        boolean initialized = false;

        public SetPosition(double position, double waitTime) {
            this.position = position;
            this.waitTime = waitTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                startTime = elapsedTime.time();
                servo.setPosition(position);
                initialized = true;
            }

            return (elapsedTime.time() - startTime) > waitTime;
        }
    }


}
