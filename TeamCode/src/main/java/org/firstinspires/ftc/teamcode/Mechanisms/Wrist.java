package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.Components.Timer;

@Config
public class Wrist {
    Timer timer;
    public static double SAMPLE_PICKUP_POSITION = Kinematics.wristOrientationToPosition(Math.toRadians(-58.184)); // sample pickup in autonomous
    public static double THIRD_SAMPLE = Kinematics.wristOrientationToPosition(Math.toRadians(-65)); // sample pickup in autonomous
    public static double BASKET_POSITION = Kinematics.wristOrientationToPosition(Math.toRadians(-58.184));; // sample dropoff in basket(top basket) at certain arm position
    public static double INIT_POSITION = Kinematics.wristOrientationToPosition(Math.toRadians(-80));
    public static double STRAIGHT_POSITION = Kinematics.wristOrientationToPosition(Math.toRadians(0));

    public static double SPECIMEN_INTAKE_POSITION = 0.63; //specimen position
    public static double SPECIMEN_OUTAKE_POSITION = 0;

    public Servo servo;

    public Wrist(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "Wrist");
        timer = new Timer();
    }

    public Action setPosition(double position) {
        return new Wrist.SetPosition(position);
    }

    public Action setPositionSmooth(double position, double movementTime){
        return new Wrist.SetPosition(position, movementTime);
    }

    public void setInitPosition() {
        servo.setPosition(INIT_POSITION);
    }

    public void setPositionDirect(double position) {
        servo.setPosition(position);
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
