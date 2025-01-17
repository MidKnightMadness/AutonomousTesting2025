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
    public static double SAMPLE_POSITION_AUTO = 0 ; //sample pickup in autonomous
    public static double BASKET_POSITION_AUTO = 0; //sample dropoff in basket(top basket) at certain arm position
    public static double SPECIMEN_INTAKE_POSITION = 0; //specimen position
    public static double SPECIMEN_OUTAKE_POSITION = 0;
    public Servo servo;
    ElapsedTime elapsedTime;

    public Wrist(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "wristServo");
        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.startTime();
    }


    public Action setSamplePos(double waitTime) {
        return new Wrist.SetPosition(SAMPLE_POSITION_AUTO, waitTime);
    }

    public Action setBasketPos(double waitTime){
        return new Wrist.SetPosition(BASKET_POSITION_AUTO, waitTime);
    }

    public Action setPosition(double position, double waitTime) {
        return new Wrist.SetPosition(position, waitTime);
    }

    public void setInitPosition() {
        servo.setPosition(0);
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
