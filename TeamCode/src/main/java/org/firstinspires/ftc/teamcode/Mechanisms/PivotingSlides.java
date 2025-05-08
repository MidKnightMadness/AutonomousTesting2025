package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.Components.Util;

@Config
public class PivotingSlides {


    Timer timer;
    public static double RETRACT_SERVO_POSITION = 0.467;
    public static double EXTEND_SERVO_POSITION = 0.856;
    public static double MAX_EXTENSION_LENGTH = 200 / 25.4;
    public static double INIT_POSITION  = 0.467;

    Servo leftServo;
    Servo rightServo;

    private double currentExtensionLength = 0;

    public PivotingSlides(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "Left Pivoting Slides");
        rightServo = hardwareMap.get(Servo.class, "Right Pivoting Slides");

        leftServo.setDirection(Servo.Direction.REVERSE);

        setExtensionLengthDirect(currentExtensionLength);
        timer = new Timer();
    }
    public Servo getLeftServo(){
        return leftServo;
    }

    public Servo getRightServo(){
        return rightServo;
    }

    public void setInitPosition(){
        currentExtensionLength = 0;
        leftServo.setPosition(INIT_POSITION);
    }

    //Sets Extension Length to a certain extension length - Returns null
    public void setExtensionLengthDirect(double extensionLength) {
        currentExtensionLength = Util.clamp(extensionLength, 0, MAX_EXTENSION_LENGTH);
        leftServo.setPosition(RETRACT_SERVO_POSITION + currentExtensionLength / MAX_EXTENSION_LENGTH * (EXTEND_SERVO_POSITION - RETRACT_SERVO_POSITION));
    }

    //Sets Extension Length to a certain extension length - Returns Action(Autonomous usecase)
    public Action setExtensionLength(double extensionLength){
        currentExtensionLength = Util.clamp(extensionLength, 0, MAX_EXTENSION_LENGTH);
        return new SetPosition(RETRACT_SERVO_POSITION + currentExtensionLength / MAX_EXTENSION_LENGTH * (EXTEND_SERVO_POSITION - RETRACT_SERVO_POSITION));
    }

    //Sets Extension Length to a certain extension length while smoothing using square root curve - Returns Action(Autonomous usecase)
    public Action setExtensionLengthSmooth(double extensionLength, double movementTime){
        currentExtensionLength = Util.clamp(extensionLength, 0, MAX_EXTENSION_LENGTH);
        return new SetPosition(RETRACT_SERVO_POSITION + currentExtensionLength / MAX_EXTENSION_LENGTH * (EXTEND_SERVO_POSITION - RETRACT_SERVO_POSITION), movementTime);
    }


    public double getExtensionLength() {
        return currentExtensionLength;
    }

    public class SetPosition implements Action{
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

            if (movementTime != 0) {
                double timeSinceStart = timer.updateTime() - startTime;
                double percentOfMovement = (Math.min(1, timeSinceStart / movementTime));  // square root curve movement
                double intermediatePoint = (targetPosition - startPosition) * percentOfMovement + startPosition;

                leftServo.setPosition(intermediatePoint);
//                rightServo.setPosition(intermediatePoint);

                if (timeSinceStart > movementTime) {
                    leftServo.setPosition(targetPosition);
//                    rightServo.setPosition(targetPosition);
                    return false;
                }

                return timeSinceStart <= movementTime;
            } else {
                leftServo.setPosition(targetPosition);
//                rightServo.setPosition(targetPosition);

                return false;
            }
        }
    }

}
