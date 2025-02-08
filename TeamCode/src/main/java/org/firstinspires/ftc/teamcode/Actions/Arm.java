package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Timer;

@Config
public class Arm {

    public static double INIT_AUTO_POS = 0.07;
    public static double END = 1;
    public static double SAMPLE_INTAKE = 0.74; //set already
    public static double BASKET_POSITION = 0.42;

    public static double STRAIGHT_UP_POSITION = 0.30;
    public static double PERPENDICULAR = 0.69;

    public static double ARM_TO_BAR = 0.44;


    public static double TIME_FULL_ROTATION = 2;

    public Servo leftServo;
    public Servo rightServo;
    Timer timer;


    public Arm(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "Arm Left");
        rightServo = hardwareMap.get(Servo.class, "Arm Right");
        timer = new Timer();
    }

    boolean smooth = false;
    double interval = 0.05;
    //Still being used:
    public Action setPositionSmooth(double position, double movementTime) {
        lastSetPosition = position;
        return new SetPosition(position, movementTime);
    }

    double lastSetPosition = Arm.INIT_AUTO_POS;

    public Action setPositionSmooth(double position){
        double lastPos = lastSetPosition;
        lastSetPosition = position;

        return new SetPosition(position, Math.abs(position - lastPos) * TIME_FULL_ROTATION);
    }

    public Action setPosition(double position){
        lastSetPosition = position;
        return new SetPosition(position);
    }

    public void setInitPosition() {
        lastSetPosition = Arm.INIT_AUTO_POS;
        leftServo.setPosition(Arm.INIT_AUTO_POS);
        rightServo.setPosition(Arm.INIT_AUTO_POS);
    }


    public class SetPosition implements Action {
        private final double targetPosition;
        private final double movementTime;

        double startTime;
        boolean initialized = false;
        double startPosition;

        public SetPosition(double position) {
            this.targetPosition = position;
            this.movementTime = 0;
        }

        public SetPosition(double targetPosition, double movementTime) {
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

            packet.addLine("Time: " + (timer.updateTime() - startTime));


            if (movementTime != 0) {
                double timeSinceStart = timer.updateTime() - startTime;
                double percentOfMovement = (Math.min(1, timeSinceStart / movementTime));  // square root curve movement
                double intermediatePoint = (targetPosition - startPosition) * percentOfMovement + startPosition;

                packet.addLine("Position " + intermediatePoint);

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
