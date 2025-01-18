package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Wrist {
    public static double SAMPLE_LINE_POSITION_AUTO = 0.46; //sample pickup in autonomous
    public static double SAMPLE_SUB_POSITION = 0.46;
    public static double BASKET_POSITION_AUTO = 0.4; //sample dropoff in basket(top basket) at certain arm position
    public static double INIT_POSITION = 0.8;
    public static double SPECIMEN_INTAKE_POSITION = 0.63; //specimen position
    public static double SPECIMEN_OUTAKE_POSITION = 0;
    public Servo servo;
    ElapsedTime elapsedTime;

    public Wrist(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "wristServo");
        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.startTime();
    }


    public Action setSampleLinePos(double waitTime) {
        return new Wrist.SetPosition(SAMPLE_LINE_POSITION_AUTO, waitTime);
    }
    public void setSampleSubPos(){
        servo.setPosition(SAMPLE_SUB_POSITION);
    }

    public void setBasketPos(){
       servo.setPosition(BASKET_POSITION_AUTO);
    }


    public Action setPosition(double position, double waitTime) {
        return new Wrist.SetPosition(position, waitTime);
    }

    public void setInitPosition() {
        servo.setPosition(INIT_POSITION);
    }

    public Action setBasketPositionAction(double waitTime) {
        return new Wrist.SetPosition(BASKET_POSITION_AUTO, waitTime);
    }


    public class SetPosition implements Action {
        private final double position;
        private final double waitTime;

        double startTime;
        boolean initialized = false;

        public SetPosition(double position, double waitTime) {
            this.position = position;
            this.waitTime = waitTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                startTime = elapsedTime.time();
                servo.setPosition(position);
                initialized = true;
            }

            return false;
        }
    }
}
