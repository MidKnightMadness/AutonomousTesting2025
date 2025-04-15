package org.firstinspires.ftc.teamcode.TouchSensor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Components.Timer;

@TeleOp(name = "TouchSensorSpintake", group = "Sensor")
public class TouchSensorTest extends OpMode {
    TouchSensor touchSensor;
    Servo servoLeft;
    Servo servoRight;
    Timer timer;
    double power = 1;
    final double DIRECTION_LEFT = 1;
    final double DIRECTION_RIGHT = -1;

    boolean touchPressed;

    @Override
    public void init() {
        servoLeft = hardwareMap.get(Servo.class, "leftServo");
        servoRight = hardwareMap.get(Servo.class, "rightServo");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        timer = new Timer();
    }

    @Override
    public void loop() {
        if(!touchPressed) {
            if (gamepad1.left_bumper) {//intaking in
                servoLeft.setPosition(DIRECTION_LEFT * power);
                servoRight.setPosition(DIRECTION_RIGHT * power);
            }
        }

        if (gamepad1.right_bumper) {//intaking out
            servoLeft.setPosition(-DIRECTION_LEFT * power);
            servoRight.setPosition(-DIRECTION_RIGHT * power);
        }

        if(touchSensor.isPressed()){
           servoLeft.setPosition(0);
           servoRight.setPosition(0);
           touchPressed = true;
        }

        telemetry.addData("Left Servo Pos", servoLeft.getPosition());
        telemetry.addData("Right Servo Pos", servoRight.getPosition());
        telemetry.addData("Touch Pressed", touchPressed);
        telemetry.addData("Power", power);
        telemetry.addData("Time", timer.updateTime());

    }
}
