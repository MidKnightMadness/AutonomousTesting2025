package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.Mechanisms.Spintake;

@TeleOp(group="Test")
public class SpintakeTest extends OpMode {

    Spintake spintake;

    Timer timer;
    boolean intaking;
    boolean outtaking;

    @Override
    public void init() {
        spintake = new Spintake(hardwareMap);

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
            spintake.setPower(1);
            intaking = (spintake.colorSensor.getDistance(DistanceUnit.INCH) > INTAKE_THRESHOLD)
                    && (spintake.distanceSensor.getDistance(DistanceUnit.INCH) > INTAKE_THRESHOLD);
        }
        else if (outtaking) {
            spintake.setPower(-1);
            outtaking = ((spintake.colorSensor.getDistance(DistanceUnit.INCH) < OUTTAKE_THRESHOLD)
                     && ((spintake.distanceSensor.getDistance(DistanceUnit.INCH) < OUTTAKE_THRESHOLD)));
        }
        else {
            spintake.setPower(0);
        }

        telemetry.addData("Left Servo Power", spintake.leftServo.getPower());
        telemetry.addData("Right Servo Power", spintake.rightServo.getPower());
        telemetry.addData("Color  sensor distance", spintake.colorSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Distance sensor distance", spintake.distanceSensor.getDistance(DistanceUnit.INCH));

        telemetry.addData("Inside color sensor optical", spintake.colorSensor.getRawLightDetected());

        telemetry.addData("Inside color sensor raw", spintake.colorSensor.rawOptical());

        telemetry.addData("Time", timer.updateTime());
        telemetry.update();
    }
}
