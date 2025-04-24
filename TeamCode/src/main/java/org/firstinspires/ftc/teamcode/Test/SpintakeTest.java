package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Timer;

@TeleOp(group="Test")
public class SpintakeTest extends OpMode {

    RevColorSensorV3 outsideColorSensor;
    RevColorSensorV3 insideColorSensor;
    Servo servoLeft;
    Servo servoRight;
    Timer timer;

    boolean intaking;
    boolean outtaking;

    @Override
    public void init() {
        servoLeft = hardwareMap.get(Servo.class, "leftServo");
        servoRight = hardwareMap.get(Servo.class, "rightServo");
        servoRight.setDirection(Servo.Direction.REVERSE);

        outsideColorSensor = hardwareMap.get(RevColorSensorV3.class, "outsideColorSensor");
        insideColorSensor = hardwareMap.get(RevColorSensorV3.class, "insideColorSensor");

        timer = new Timer();
    }


    final double INTAKE_THRESHOLD  = 0.6;
    final double OUTTAKE_THRESHOLD = 2;

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            intaking = true;
        }

        if (gamepad1.right_bumper) {
            outtaking = true;
        }

        if (intaking) {
            servoLeft.setPosition(1);
            servoRight.setPosition(1);
            intaking = (outsideColorSensor.getDistance(DistanceUnit.INCH) > INTAKE_THRESHOLD)
                    || (insideColorSensor.getDistance(DistanceUnit.INCH) > INTAKE_THRESHOLD);
        }
        else if (outtaking) {
            servoLeft.setPosition(0);
            servoRight.setPosition(0);
            outtaking = (outsideColorSensor.getDistance(DistanceUnit.INCH) < OUTTAKE_THRESHOLD)
                     || (insideColorSensor.getDistance(DistanceUnit.INCH) < OUTTAKE_THRESHOLD);
        }
        else {
            servoLeft.setPosition(0.5);
            servoRight.setPosition(0.5);
        }

        telemetry.addData("Left Servo Pos", servoLeft.getPosition());
        telemetry.addData("Right Servo Pos", servoRight.getPosition());
        telemetry.addData("Outside color sensor distance", outsideColorSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Inside color sensor distance", insideColorSensor.getDistance(DistanceUnit.INCH));

        telemetry.addData("Inside color sensor optical", insideColorSensor.getRawLightDetected());
        telemetry.addData("Outside color sensor optical", outsideColorSensor.getRawLightDetected());

        telemetry.addData("Inside color sensor raw", insideColorSensor.rawOptical());
        telemetry.addData("outside color sensor raw", outsideColorSensor.rawOptical());

        telemetry.addData("Time", timer.updateTime());
        telemetry.update();
    }
}
