package org.firstinspires.ftc.teamcode.Mechanisms;

import static com.acmerobotics.roadrunner.Math.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.Util;

@Config
public class PivotingSlides {

    public static double RETRACT_SERVO_POSITION = 0.4649;
    public static double EXTEND_SERVO_POSITION = 0.856;
    public static double MAX_EXTENSION_LENGTH = 200 / 25.4;

    Servo leftServo;
    Servo rightServo;

    private double currentExtensionLength = 0;

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
}
