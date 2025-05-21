package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.Components.Timer;

public class Spintake {

    public RevColorSensorV3 colorSensor;
    public Rev2mDistanceSensor distanceSensor;

    public static double INTAKE_THRESHOLD = 0.6;
    public static double OUTAKE_THRESHOLD = 2;

    public CRServo leftServo;
    public CRServo rightServo;


    Timer timer;

    double lastSetPosition = 0.5;
    public Spintake(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(CRServo.class, "Left Spintake");
        rightServo = hardwareMap.get(CRServo.class, "Right Spintake");
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "Inside Color");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "Outside Distance");

        timer = new Timer();
    }


    public void setPower(double power) {
        leftServo.setPower(power);
        rightServo.setPower(power);
    }

    public Action outtake() {
        return new Outtake();
    }

    public Action intake() {
        return new Intake();
    }

    public class Outtake implements Action {
        public double timeout = 2;

        double startTime;
        boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                startTime = timer.updateTime();
                initialized = true;
            }

            setPower(-1);

            if (timer.updateTime() - startTime < timeout) {
                boolean run = (colorSensor.getDistance(DistanceUnit.INCH) < Spintake.OUTAKE_THRESHOLD)
                        || ((distanceSensor.getDistance(DistanceUnit.INCH) < Spintake.OUTAKE_THRESHOLD));

                if (!run) setPower(0);
                return run;
            }

            setPower(0);
            return false;
        }
    }

    public class Intake implements Action {
        public double timeout = 3;

        double startTime;
        boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                startTime = timer.updateTime();
                initialized = true;
            }

            setPower(1);

            if (timer.updateTime() - startTime < timeout) {
                boolean run = (colorSensor.getDistance(DistanceUnit.INCH) > INTAKE_THRESHOLD)
                        && (distanceSensor.getDistance(DistanceUnit.INCH) > INTAKE_THRESHOLD);

                if (!run) setPower(0);
                return run;
            }

            setPower(0);
            return false;
        }
    }
}
