package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.Timer;

@Config
public class Arm {
    public static double TIME_FULL_ROTATION = 1.5;
    public static double ZERO_POSITION = 0.1;

    public static double SERVO_DEGREES = 355;

    public Servo leftServo;
    public Servo rightServo;

    double lastSetPosition = ZERO_POSITION;
    Timer timer;

    public Arm(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "Left Arm");
        rightServo = hardwareMap.get(Servo.class, "Right Arm");
        timer = new Timer();
    }

    public static double degreesToPosition(double degrees) {
        return ZERO_POSITION + degrees * 2 / SERVO_DEGREES;   // 2 to 1 gear reduction on servos
    }

    public Action setPositionSmooth(double position, double movementTime) {
        lastSetPosition = position;
        return new SetPositionAction(position, movementTime);
    }

    public Action setPositionSmooth(double position){
        double lastPos = lastSetPosition;
        lastSetPosition = position;

        return new SetPositionAction(position, Math.abs(position - lastPos) * TIME_FULL_ROTATION);
    }

    public Action setAngleSmooth(double angle, double movementTime) {
        return setPositionSmooth(Arm.degreesToPosition(angle), movementTime);
    }

    public Action setAngleSmooth(double angle) {
        return setPositionSmooth(Arm.degreesToPosition(angle));
    }

    public void setPosition(double position) {
        lastSetPosition = position;
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    public void setAngle(double angle) {
        setPosition(Arm.degreesToPosition(angle));
    }

    public void setInitPosition() {
        lastSetPosition = ZERO_POSITION;
        leftServo.setPosition(ZERO_POSITION);
        rightServo.setPosition(ZERO_POSITION);
    }

    public class SetPositionAction implements Action {
        private final double targetPosition;
        private final double movementTime;

        double startTime;
        boolean initialized = false;
        double startPosition;

        public SetPositionAction(double targetPosition, double movementTime) {
            this.targetPosition = targetPosition;
            this.movementTime = movementTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                startTime = timer.updateTime();
                startPosition = leftServo.getPosition();
                initialized = true;
            }

            if (movementTime != 0) {
                double timeSinceStart = timer.updateTime() - startTime;
                double percentOfMovement = (Math.min(1, timeSinceStart / movementTime));  // square root curve movement
                double intermediatePoint = (targetPosition - startPosition) * percentOfMovement + startPosition;

                leftServo.setPosition(intermediatePoint);
                rightServo.setPosition(intermediatePoint);

                if (timeSinceStart > movementTime) {
                    leftServo.setPosition(targetPosition);
                    rightServo.setPosition(targetPosition);
                    return false;
                }

                return timeSinceStart <= movementTime;
            } else {
                leftServo.setPosition(targetPosition);
                rightServo.setPosition(targetPosition);

                return false;
            }
        }
    }
}

