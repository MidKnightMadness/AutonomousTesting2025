package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {
    final double OPEN_POSITION = 0;
    final double CLOSED_POSITION = 0;

    public Servo servo;
    ElapsedTime elapsedTime;

    public Claw(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "clawServo");
        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.startTime();
    }

    public Action open(double waitTime) {
        return new SetPosition(OPEN_POSITION, waitTime);
    }

    public Action close(double waitTime) {
        return new SetPosition(CLOSED_POSITION, waitTime);
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
            }

            return (elapsedTime.time() - startTime) / 1000d > waitTime;
        }
    }


}
