package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.AxonEncoder;
import org.firstinspires.ftc.teamcode.Components.PIDController;
import org.firstinspires.ftc.teamcode.Components.PolynomialApproximator;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.Components.Util;

@Config
public class Arm {

    public final CRServo leftServo;
    public final CRServo rightServo;

    public final AxonEncoder leftEncoder;
    public final AxonEncoder rightEncoder;

    public static double kP;
    public static double kI;
    public static double kD;

    private final Timer timer;

    private double minOutput;
    private double maxOutput;

    public static double deadband = 1;

    public PIDController leftController;
    public PIDController rightController;
    public PIDController armPID;

    public static double p;
    public static double i;
    public static double d;

    Telemetry telemetry;


    private final double[] cgExtensionCoefficients = new double[] { 209.90601, 0.484398 };
    PolynomialApproximator cgExtension = new PolynomialApproximator(cgExtensionCoefficients, 0, PivotingSlides.MAX_EXTENSION_LENGTH);

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {

        this.leftServo = hardwareMap.get(CRServo.class, "Left Arm");
        this.rightServo = hardwareMap.get(CRServo.class, "Right Arm");

        this.leftEncoder = new AxonEncoder(hardwareMap, "Arm Left Encoder");
        this.rightEncoder = new AxonEncoder(hardwareMap, "Arm Right Encoder");
        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightEncoder.setDirection(DcMotorSimple.Direction.FORWARD);

        leftController = new PIDController(p, i, d);
        rightController = new PIDController(p, i, d);
        armPID = new PIDController(kP, kI, kD);

        this.timer = new Timer();
        this.telemetry = telemetry;
    }

    public void setIndividualPositions(double left, double right) {
        timer.updateTime();

        leftEncoder.update();
        rightEncoder.update();

        leftController.setCoefficients(p, i, d);
        rightController.setCoefficients(p, i, d);

        double leftError = left - leftEncoder.getAbsolutePositionDegrees();
        double rightError = right - rightEncoder.getAbsolutePositionDegrees();

        double leftPow = leftController.update(normalizeDegrees(leftError), timer.getDeltaTime());
        double rightPow = rightController.update(normalizeDegrees(rightError), timer.getDeltaTime());

        telemetry.addData("LPower", leftPow);
        telemetry.addData("RPower", rightPow);

        leftServo.setPower(leftPow);
        rightServo.setPower(rightPow);
    }

    double normalizeDegrees(double angle) {
        return ((angle + 180) % 360) - 180;
    }


    public void update(double targetAngleDegrees) {
        timer.updateTime();

        leftEncoder.update();
        rightEncoder.update();

        double armOrientation = (leftEncoder.getAbsolutePositionDegrees() + rightEncoder.getAbsolutePositionDegrees()) / 2;
        telemetry.addData("Current angle", armOrientation);
        double error = normalizeDegrees(targetAngleDegrees - armOrientation);

        telemetry.addData("Error", error);
        double output = armPID.update(error, timer.getDeltaTime());

        if (Math.abs(error) < deadband) {
            output = 0.0;
        }

        output = Math.max(minOutput, Math.min(maxOutput, output));

        //        leftServo.setPower(output);
        //        rightServo.setPower(output);
    }

    public double calculateGravityTorque() {
        double extensionLength = Util.clamp(PivotingSlides.currentExtensionLength, 0, PivotingSlides.MAX_EXTENSION_LENGTH);


        return 0;
    }

    public void stop() {
        leftServo.setPower(0.0);
    }

    public void resetEncoder() {
        leftEncoder.reset();
    }

    public double getCurrentAngleDegrees() {
        return leftEncoder.getAbsolutePositionDegrees();
    }
}

