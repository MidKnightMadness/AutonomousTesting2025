package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Config
public class Arm {

    public static double INIT_AUTO_POS = 0.12;
    public static double FACING_DOWN_POSITION_AUTO = 0.875; //set already
    public static double SPECIMEN_OUTAKE_POSITION_AUTO = 0; //not set
    public static double SPECIMEN_INTAKE_POSITION_AUTO = 0; //not set
    public static double BASKET_POSITION_AUTO = 0.5;
    public static double SAMPLE_INTAKE_INTERMEDIATE = 0.68;

    public static double STRAIGHT_UP_POSITION = 0.45;

    //TELEOP

    public static double SAMPLE_INTAKE_POSITION = 0.875;
    public static double BASKET_POSITION = 0.4;

    public static double SPECIMEN_OUTAKE_POSITION = 0;
    public static double SPECIMEN_INTAKE_POSITION = 0;

    public Servo leftServo;
    public Servo rightServo;
    ElapsedTime elapsedTime;

    public Arm(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "leftArmServo");
//        rightServo = hardwareMap.get(Servo.class, "rightArmServo");

        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.startTime();
    }


    public Action setBasketPositionAction(double waitTime ) {
        return new SetPosition(BASKET_POSITION_AUTO, waitTime);
    }

    public Action setStraightUp(double waitTime ) {
        return new SetPosition(STRAIGHT_UP_POSITION, waitTime);
    }

    public Action setBasketPositionAutoSmooth(double interval, double waitTime){
        return new SetPosition(BASKET_POSITION_AUTO, waitTime, interval);
    }

    public Action setSamplePositionAction(double waitTime){
        return new SetPosition(FACING_DOWN_POSITION_AUTO, waitTime);
    }

    public Action setSampleIntermediate(double waitTime) {
        return new SetPosition(SAMPLE_INTAKE_INTERMEDIATE, waitTime);
    }

    public Action setSamplePositionAuto(double waitTime){
        return new SetPosition(FACING_DOWN_POSITION_AUTO, waitTime);
    }
    public void setSamplePositionAuto(){
        leftServo.setPosition(FACING_DOWN_POSITION_AUTO);
    }

    public void setBasketPositionAuto(){
        leftServo.setPosition(BASKET_POSITION_AUTO);
    }


    public void setSamplePositionTeleOp(){
        leftServo.setPosition(SAMPLE_INTAKE_POSITION);
    }

    public void setBasketPositionTeleOp(){
        leftServo.setPosition(BASKET_POSITION);
    }


    public void setSpecimenOutakePosition(){
        leftServo.setPosition(SPECIMEN_OUTAKE_POSITION);
    }

    public void setSpecimenIntakePosition(){
        leftServo.setPosition(SPECIMEN_INTAKE_POSITION);
    }

    public Action setPosition(double leftPosition, double waitTime) {
        return new SetPosition(leftPosition, waitTime);
    }
    public void setInitPosition(){
        leftServo.setPosition(INIT_AUTO_POS);
    }

    public Action setInitPositionAction(double waitTime) {
        return new SetPosition(INIT_AUTO_POS, waitTime);
    }


    public class SetPosition implements Action {
        private final double leftPosition;
        private final double waitTime;


        double startTime;
        boolean initialized = false;

        public SetPosition(double leftPosition, double waitTime) {
            this.leftPosition = leftPosition;
            this.waitTime = waitTime;

        }


        public SetPosition(double leftPosition, double waitTime, double interval){
            this.leftPosition = leftPosition;
            this.waitTime = waitTime;

        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {

                startTime = elapsedTime.time();
                leftServo.setPosition(leftPosition);
//                if(smooth == false) {
//                    leftServo.setPosition(leftPosition);
//                }
//                if(smooth == true){
//                    if(leftServo.getPosition() > leftPosition){
//                        leftServo.setPosition(leftServo.getPosition() - interval);
//                    }
//                    else {
//                        leftServo.setPosition(leftServo.getPosition() + interval);
//                    }
//
//                }
//                rightServo.setPosition(rightPosition);
                initialized = true;
            }

            return false;
        }
    }


}
