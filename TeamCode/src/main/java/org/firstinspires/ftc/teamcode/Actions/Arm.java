package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Timer;

import java.util.concurrent.TimeUnit;

@Config
public class Arm {

        public static double INIT_AUTO_POS = 0.07 - 0.03;
        public static double END = 1;
        public static double SAMPLE_INTAKE_AUTO = 0.78 - 0.07; //set already
        public static double INTERMEDIATE_DOWN = 0.66 - 0.07;
        public static double BASKET_POSITION_AUTO = 0.44 - 0.07;


        public static double STRAIGHT_UP_POSITION = 0.33 - 0.07;
        public static double PERPENDICULAR = 0.69;

        public static double ARM_TO_BAR = 0.44 - 0.07;

        //TELEOP

        public static double SAMPLE_INTAKE_POSITION_MAIN = 0.875;
        public static double BASKET_POSITION_MAIN = 0.575;

        public static double SPECIMEN_OUTAKE_POSITION_MAIN = 0;
        public static double SPECIMEN_INTAKE_POSITION_MAIN = 0;


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
    public Action setPositionSmooth(double position, double movementTime){
        return new SetPosition(position, movementTime);
    }

    public Action setPosition(double position){
        return new SetPosition(position);
    }

    public void setInitPosition() {
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
                double percentOfMovement = Math.sqrt(Math.min(1, timeSinceStart / movementTime));  // square root curve movement
                double intermediatePoint = (targetPosition - startPosition) * percentOfMovement + startPosition;


                packet.addLine("Position " + intermediatePoint);

                leftServo.setPosition(intermediatePoint);
                rightServo.setPosition(intermediatePoint);
                return timeSinceStart < movementTime;
            } else {
                packet.addLine("Position " + targetPosition);
                leftServo.setPosition(targetPosition);
                rightServo.setPosition(targetPosition);

                return false;
            }
        }
    }


}
