package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.Components.Timer;

@Deprecated
public class Spintake {

    RevColorSensorV3 colorSensor;
    Rev2mDistanceSensor distanceSensor;
    public static double INTAKE_POS = 1.0;
    public static double OUTAKE_POS = 0.0;

    public static double INTAKE_THRESHOLD = 0.6;
    public static double OUTAKE_THRESHOLD = 2;


    public Servo leftServo;
    public Servo rightServo;


    Timer timer;

    double lastSetPosition = 0.5;
    public Spintake(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "Left Arm");
        rightServo = hardwareMap.get(Servo.class, "Right Arm");

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "Inside Color");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "Outside Distance");

        timer = new Timer();
    }

    public Action setPositionSmooth(double position, double movementTime) {
        lastSetPosition = position;
        return new SetPosition(position, movementTime);
    }


    public Action setPosition(double position){
        lastSetPosition = position;
        return new SetPosition(position);
    }

    public void setPositionDirect(double position) {
        lastSetPosition = position;
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    public void setInitPosition() {
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);
    }

    public class SetPosition implements Action {
        private final double continuousPosition;
        private final double movementTime;

        double startTime;
        boolean initialized = false;
        double startPosition;

        public SetPosition(double continuousPosition) {
            this.continuousPosition = continuousPosition;
            this.movementTime = 0;
        }

        public SetPosition(double continuousPosition, double movementTime) {
            this.continuousPosition = continuousPosition;
            this.movementTime = movementTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                startTime = timer.updateTime();
                initialized = true;
            }

            if (movementTime != 0) {
                double currentTime = timer.updateTime();
                if (currentTime - startTime < movementTime) {
                    if (continuousPosition > 0.5 && ((distanceSensor.getDistance(DistanceUnit.INCH) > INTAKE_THRESHOLD)
                            || (colorSensor.getDistance(DistanceUnit.INCH) > INTAKE_THRESHOLD))) {
                        leftServo.setPosition(continuousPosition);
                        rightServo.setPosition(continuousPosition);
                        return true;
                    } else if (continuousPosition < 0.5 && ((distanceSensor.getDistance(DistanceUnit.INCH) < OUTAKE_THRESHOLD) ||
                            (colorSensor.getDistance(DistanceUnit.INCH) < OUTAKE_THRESHOLD))) {
                        leftServo.setPosition(continuousPosition);
                        rightServo.setPosition(continuousPosition);
                        return true;
                    } else {
                        leftServo.setPosition(0.5);
                        rightServo.setPosition(0.5);
                        return false;
                    }
                }
            }
            leftServo.setPosition(0.5);
            rightServo.setPosition(0.5);
            return false;
        }
    }
}
