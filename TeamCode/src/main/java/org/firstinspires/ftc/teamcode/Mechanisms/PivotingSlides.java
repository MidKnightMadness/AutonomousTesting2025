package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.PolynomialApproximator;
import org.firstinspires.ftc.teamcode.Components.Util;

@Config
public class PivotingSlides {

    public static double RETRACT_SERVO_POSITION = 0.137;
    public static double EXTEND_SERVO_POSITION = 0.6;
    public static double MAX_EXTENSION_LENGTH = 240;

    double[] extensionToAngleCoefficients = {  // input: extension, output: theta servo (zero defined as zero extension)
            -0.5087, 1.44263, -0.0182803, 0.000177068, -9.83349e-7, 2.83774e-9, -3.24382e-12
    };

    PolynomialApproximator angleToExtension = new PolynomialApproximator(extensionToAngleCoefficients, 0, MAX_EXTENSION_LENGTH);

    private static double SERVO_DEGREES = 240;
    public static double currentExtensionLength = 0;

    public Servo leftServo;
    public Servo rightServo;

    public double targetAngle;
    public double targetPos;

    public PivotingSlides(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "Left Pivoting Slides");
        rightServo = hardwareMap.get(Servo.class, "Right Pivoting Slides");
        rightServo.setDirection(Servo.Direction.REVERSE);
    }
    
    public void setExtension(double extensionLength) {
        currentExtensionLength = Util.clamp(extensionLength, 0, MAX_EXTENSION_LENGTH);

        double angle = angleToExtension.evaluate(currentExtensionLength);
        double position = RETRACT_SERVO_POSITION + angle / SERVO_DEGREES;

        targetAngle = angle;
        targetPos = position;

        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    public double getExtensionLength() {
        return currentExtensionLength;
    }

    public Action setExtensionAction(double extension) {
        return new InstantAction(() -> setExtension(extension));
    }
}
