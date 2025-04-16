package org.firstinspires.ftc.teamcode.TouchSensor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;

import org.firstinspires.ftc.teamcode.Components.Timer;

@TeleOp(name = "TouchSensorSpintake", group = "Sensor")
public class TouchSensorTest extends OpMode {
    RevTouchSensor touchSensor;
    Servo servoLeft;
    Servo servoRight;
    Timer timer;
    double power = 1;
    final double DIRECTION_LEFT = 1;
    final double DIRECTION_RIGHT = 0;

    boolean touchPressed;

    @Override
    public void init() {
        servoLeft = hardwareMap.get(Servo.class, "leftServo");
        servoRight = hardwareMap.get(Servo.class, "rightServo");
        touchSensor = hardwareMap.get(RevTouchSensor.class, "touchSensor");

        timer = new Timer();
    }

    double count = 0;
    @Override
    public void loop() {
        if(touchSensor.isPressed()){
            servoLeft.setPosition(0.5);
            servoRight.setPosition(0.5);
            touchPressed = true;
            count++;
        }

        if(!touchPressed) {
            if (gamepad1.left_bumper) {//intaking in
                servoLeft.setPosition(0.8);
                servoRight.setPosition(0.2);
            }
        }

        if (gamepad1.right_bumper) {//intaking out
            servoLeft.setPosition(0);
            servoRight.setPosition(gamepad1.left_stick_x);
        }


        telemetry.addData("Left Servo Pos", servoLeft.getPosition());
        telemetry.addData("Right Servo Pos", servoRight.getPosition());
        telemetry.addData("Touch Pressed", touchPressed);
        telemetry.addData("Touch Amount", count);
        telemetry.addData("Power", power);
        telemetry.addData("Time", timer.updateTime());
        telemetry.update();
    }
}
