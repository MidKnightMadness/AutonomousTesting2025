package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {
    final Vector2d LEFT_SERVO_BOUNDS = new Vector2d(0, 1);

                                                        //Left, Right
    final double FACING_DOWN_POSITION_AUTO = 0.835; //set already
    final double SUBMERSIBLE_POSITION_AUTO = 0; //not set
    final double BASKET_POSITION_AUTO = 0;

    public Servo leftServo;
    public Servo rightServo;
    ElapsedTime elapsedTime;

    public Arm(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "leftArm");

        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.startTime();
    }

    public Action open(double waitTime) {
        return new SetPosition(LEFT_SERVO_BOUNDS.y, waitTime);
    }

    public Action close(double waitTime) {
        return new SetPosition(LEFT_SERVO_BOUNDS.x, waitTime);
    }

    public Action setSamplePosition(double waitTime){//pickup sample
        return new Arm.SetPosition(FACING_DOWN_POSITION_AUTO, waitTime);
    }

    public Action setSubPosition(double waitTime){//pickup sample from submersible
        return new Arm.SetPosition(SUBMERSIBLE_POSITION_AUTO, waitTime);
    }

    public Action setBasketPosition(double waitTime){
        return new Arm.SetPosition(BASKET_POSITION_AUTO,  waitTime);
    }

    public Action setPosition(double leftPosition, double waitTime) {
        return new SetPosition(leftPosition, waitTime);
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
            }

            return (elapsedTime.time() - startTime) / 1000d > waitTime;
        }
    }


}
