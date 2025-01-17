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
    public static double FACING_DOWN_POSITION_AUTO = 0.8; //set already
    public static double SUBMERSIBLE_POSITION_AUTO = 0; //not set
    public static double BASKET_POSITION_AUTO = 0.4;

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

    public Action setSamplePositionAction(double waitTime){
        return new SetPosition(FACING_DOWN_POSITION_AUTO, waitTime);
    }
    public void setSamplePosition(){
        leftServo.setPosition(FACING_DOWN_POSITION_AUTO);
    }

    public void setBasketPosition(){
        leftServo.setPosition(BASKET_POSITION_AUTO);
    }


    public Action setPosition(double leftPosition, double waitTime) {
        return new SetPosition(leftPosition, waitTime);
    }
    public void setInitPosition(){
        leftServo.setPosition(0);
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

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                startTime = elapsedTime.time();
                leftServo.setPosition(leftPosition);
//                rightServo.setPosition(rightPosition);
                initialized = true;
            }

            return false;
        }
    }


}
