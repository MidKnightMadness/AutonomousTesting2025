package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.Timer;

@Config
public class Wrist {
    Timer timer;

    public static double SERVO_MAX = 0.633;
    public static double SERVO_MIN = 0;
    public static double OUTTAKE_POSITION = SERVO_MAX;
    public static double INTAKE_POSITION = degreesToPosition(90);
    public static double STRAIGHT_POSITION = 0.5117;

    public Servo leftServo;
    public Servo rightServo;

    public Wrist(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "Left Wrist");
        rightServo = hardwareMap.get(Servo.class, "Right Wrist");
        leftServo.setDirection(Servo.Direction.REVERSE);

        timer = new Timer();
    }

    public Action setPositionAction(double position) {
        return new InstantAction(() -> setPosition(position));
    }

    public Action setDegreesAction(double degrees) {
        return setPositionAction(degreesToPosition(degrees));
    }

    public static double degreesToPosition(double degrees) {
        return STRAIGHT_POSITION - degrees / 300;
    }

    public void setInitPosition() {
        leftServo.setPosition(SERVO_MIN);
        rightServo.setPosition(SERVO_MIN);
    }

    public void setPosition(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }
}
