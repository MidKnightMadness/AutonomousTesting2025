package org.firstinspires.ftc.teamcode.Mechanisms;

import static com.acmerobotics.roadrunner.Math.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.Util;

@Config
public class PivotingSlides {

    public static double SERVO_RANGE = Math.toRadians(300);
    public static double RETRACT_SERVO_POSITION = 0.4649;
    public static double EXTEND_SERVO_POSITION = 0.856;
    public static double MAX_EXTENSION_LENGTH = 240 / 25.4;

    // offset slider-crank mechanism
    public static double LEVER_LENGTH = 240;
    public static double COUPLER_LENGTH = 23;
    public static double OFFSET = 20;


    public static double currentExtensionLength = 0;

    // degree 6 polynomial approximation for function of extension length from servo angle
    public static double[] powerSeriesCoefficients = new double[] {
            98.43355, -2.73596, 0.016785, -0.000000134805, 0.0000000605797, -0.000000000790336
    };

    Servo leftServo;
    Servo rightServo;

    public PivotingSlides(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "leftSlideServo");
        rightServo = hardwareMap.get(Servo.class, "rightSlideServo");

        leftServo.setDirection(Servo.Direction.REVERSE);

        setExtensionLength(currentExtensionLength);
    }
    
    public void setExtensionLength(double extensionLength) {
        currentExtensionLength = Util.clamp(extensionLength, 0, MAX_EXTENSION_LENGTH);

        leftServo.setPosition(RETRACT_SERVO_POSITION + currentExtensionLength / MAX_EXTENSION_LENGTH * (EXTEND_SERVO_POSITION - RETRACT_SERVO_POSITION));
    }

    public double getExtensionLength() {
        return currentExtensionLength;
    }

    public double servoPositionToAngle(double position) {
        return position;
    }
}
