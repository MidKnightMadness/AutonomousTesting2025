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

    //LEFT SIDE:
    public static class Left {
        public static double INIT_AUTO_POS = 0.2;
        public static double FACING_DOWN_POSITION_AUTO = 0.9; //set already
        public static double BASKET_POSITION_AUTO = 0.575;
        public static double SAMPLE_INTAKE_INTERMEDIATE = 0.6;


        public static double STRAIGHT_UP_POSITION = 0.5;
        public static double STRAIGHT_UP_BENT = 0.4;

        //TELEOP

        public static double SAMPLE_INTAKE_POSITION = 0.875;
        public static double BASKET_POSITION = 0.575;

        public static double SPECIMEN_OUTAKE_POSITION = 0;
        public static double SPECIMEN_INTAKE_POSITION = 0;

    }

    //RIGHT:
    public static class Right{
        public static double INIT_AUTO_POS = 0.2;
        public static double FACING_DOWN_POSITION_AUTO = 0.9; //set already
        public static double BASKET_POSITION_AUTO = 0.575;
        public static double SAMPLE_INTAKE_INTERMEDIATE = 0.6;


        public static double STRAIGHT_UP_POSITION = 0.5;
        public static double STRAIGHT_UP_BENT = 0.4;

        //TELEOP

        public static double SAMPLE_INTAKE_POSITION = 0.875;
        public static double BASKET_POSITION = 0.575;

        public static double SPECIMEN_OUTAKE_POSITION = 0;
        public static double SPECIMEN_INTAKE_POSITION = 0;

    }

    public Servo leftServo;
    public Servo rightServo;
    Timer timer;


    public Arm(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightServo = hardwareMap.get(Servo.class, "rightArmServo");

        timer = new Timer();
    }


    boolean smooth = false;
    double interval = 0.05;
    public Action setBasketPositionAction(double waitTime) {
        return new SetPosition(Arm.Left.BASKET_POSITION_AUTO, waitTime);
    }

    public Action setStraightUp(double waitTime)  {
        return new SetPosition(Arm.Left.STRAIGHT_UP_POSITION, waitTime);
    }

    public Action setStraightUpBent(double waitTime){
        return new SetPosition(Arm.Left.STRAIGHT_UP_BENT, waitTime);
    }

    public Action setPositionSmooth(double position, double movementTime){
        return new SetPosition(position, movementTime);
    }

    public Action setSamplePositionAction(double waitTime){
        return new SetPosition(Arm.Left.FACING_DOWN_POSITION_AUTO, waitTime);
    }

    public Action setSampleIntermediate(double waitTime) {
        return new SetPosition(Arm.Left.SAMPLE_INTAKE_INTERMEDIATE, waitTime);
    }

    public Action setSamplePositionAuto(double waitTime){
        return new SetPosition(Arm.Left.FACING_DOWN_POSITION_AUTO, waitTime);
    }

    public void setSamplePositionTeleOp(){
        leftServo.setPosition(Arm.Left.SAMPLE_INTAKE_POSITION);
    }

    public void setBasketPositionTeleOp(){
        leftServo.setPosition(Arm.Left.BASKET_POSITION);
    }


    public void setSpecimenOutakePosition(){
        leftServo.setPosition(Arm.Left.SPECIMEN_OUTAKE_POSITION);
    }

    public void setSpecimenIntakePosition(){
        leftServo.setPosition(Arm.Left.SPECIMEN_INTAKE_POSITION);
    }

    public Action setPosition(double leftPosition, double waitTime) {
        return new SetPosition(leftPosition, waitTime);
    }
    public void setInitPosition(){
        leftServo.setPosition(Arm.Left.INIT_AUTO_POS);
    }

    public Action setInitPositionAction(double waitTime) {
        return new SetPosition(Arm.Left.INIT_AUTO_POS, waitTime);
    }


    public class SetPosition implements Action {
        private final double leftPosition;
        private final double rightPosition;
        private final double movementTime;


        double startTime;
        boolean initialized = false;
        double startPosition;

        public SetPosition(double leftPosition, double rightPosition) {
            this.leftPosition = leftPosition;
            this.rightPosition = rightPosition;
            this.movementTime = 0;
        }

        public SetPosition(double leftPosition, double rightPosition, double movementTime){
            this.leftPosition = leftPosition;
            this.rightPosition = rightPosition;
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

            double timeSinceStart = timer.updateTime() - startTime;
            double percentOfMovement = Math.min(1, timeSinceStart / movementTime);
            double intermediatePoint = (leftPosition - startPosition) * percentOfMovement + startPosition;

            packet.addLine("Position " + intermediatePoint);

            leftServo.setPosition(intermediatePoint);

            return timeSinceStart < movementTime;
        }
    }


}
