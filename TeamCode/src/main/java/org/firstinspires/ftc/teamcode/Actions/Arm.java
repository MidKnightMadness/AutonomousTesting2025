package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {
    final Vector2d RIGHT_SERVO_BOUNDS = new Vector2d(0, 1);
    final Vector2d LEFT_SERVO_BOUNDS = new Vector2d(0, 1);

    public Servo leftServo;
    public Servo rightServo;
    ElapsedTime elapsedTime;

    public Arm(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "leftArm");
        rightServo = hardwareMap.get(Servo.class, "rightArm");

        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.startTime();
    }

    public Action open(double waitTime) {
        return new SetPosition(LEFT_SERVO_BOUNDS.y, RIGHT_SERVO_BOUNDS.y, waitTime);
    }

    public Action close(double waitTime) {
        return new SetPosition(LEFT_SERVO_BOUNDS.x, RIGHT_SERVO_BOUNDS.x, waitTime);
    }

    public Action setPosition(double leftPosition, double rightPosition, double waitTime) {
        return new SetPosition(leftPosition, rightPosition, waitTime);
    }

    public class SetPosition implements Action {
        private final double leftPosition;
        private final double rightPosition;
        private final double waitTime;

        double startTime;
        boolean initialized = false;

        public SetPosition(double leftPosition, double rightPosition, double waitTime) {
            this.leftPosition = leftPosition;
            this.rightPosition = rightPosition;
            this.waitTime = waitTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                startTime = elapsedTime.time();
                leftServo.setPosition(leftPosition);
                rightServo.setPosition(rightPosition);
            }

            return (elapsedTime.time() - startTime) / 1000d > waitTime;
        }
    }


}
