package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

    public static double leftZeroVoltage = 0;
    public static double rightZeroVoltage = 0;

    public final AxonEncoder leftEncoder;
    public final AxonEncoder rightEncoder;

    public static double combinedStallTorque = 1.8071592 + 2.05351793766;
    public static double kPTorque = 0.04687;

    public static double kP = 0.02;
    public static double kI = 0;
    public static double kD = 0.0001;

    private final Timer timer;

    static double minOutput = 0;
    static double maxOutput = 1;
    public static double minPower = 0.051;

    public static double deadband = 0;

    public PIDController leftController;
    public PIDController rightController;
    public PIDController armPID;

    public static double p;
    public static double i;
    public static double d;

    Telemetry telemetry;

    private double targetAngle;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {

        this.leftServo = hardwareMap.get(CRServo.class, "Left Arm");
        this.rightServo = hardwareMap.get(CRServo.class, "Right Arm");
        leftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftServo.setPower(0); rightServo.setPower(0);

        this.leftEncoder = new AxonEncoder(hardwareMap, "Arm Left Encoder");
        this.rightEncoder = new AxonEncoder(hardwareMap, "Arm Right Encoder");
        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightEncoder.setDirection(DcMotorSimple.Direction.FORWARD);

        leftController = new PIDController(p, i, d);
        rightController = new PIDController(p, i, d);
        armPID = new PIDController(kP, kI, kD);
        armPID.setOutputLimits(0, 1);

        this.timer = new Timer();
        this.telemetry = telemetry;

        homeEncoders(leftZeroVoltage, rightZeroVoltage);
    }

    public void setIndividualPositions(double left, double right) {
        timer.updateTime();

        updateEncoders();

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

    public double update(double targetAngleDegrees) {
        targetAngle = targetAngleDegrees;
        armPID.setCoefficients(kP, kI, kD);
        timer.updateTime();

        // 2 : 1 gear ratio
        double armOrientation = getCurrentAngleDegrees();
        telemetry.addData("Current angle", armOrientation);
        double error = normalizeDegrees(targetAngleDegrees - armOrientation);

        telemetry.addData("Error", error);
        // 0.15 power for 3.2Nm
        double gravityFeedForward = -calculateGravityTorque() * kPTorque;

        double controlOutput = armPID.update(error, timer.getDeltaTime());

        if (Math.abs(error) < deadband) {
            controlOutput = 0.0;
        }
        else {
            double sign = Math.signum(controlOutput);
            double absOutput = Math.abs(controlOutput);

            if (absOutput < minPower) {
                absOutput = minPower;
            }

            absOutput = Util.clamp(absOutput, minOutput, maxOutput);
            controlOutput = sign * absOutput;
        }

        telemetry.addData("Control output", controlOutput);
        telemetry.addData("Gravity torque", calculateGravityTorque());
        telemetry.addData("FF power", gravityFeedForward);


        leftServo.setPower(controlOutput + gravityFeedForward);
        rightServo.setPower(controlOutput + gravityFeedForward);

        return error;
    }

    private final double[] cgExtensionCoefficients = new double[] { 209.90601, 0.484398 };
    PolynomialApproximator cgExtension = new PolynomialApproximator(cgExtensionCoefficients, 0, PivotingSlides.MAX_EXTENSION_LENGTH);

    double massArm = 1.568;
    double homeAngle = 141;

    // Returns torque in Newton-meters
    public double calculateGravityTorque() {
        double extensionLength = Util.clamp(PivotingSlides.currentExtensionLength, 0, PivotingSlides.MAX_EXTENSION_LENGTH);
        double rotation = homeAngle - getCurrentAngleDegrees();
        double cgDistance = cgExtension.evaluate(extensionLength);

        return cgDistance / 1000 * massArm * 9.8 * Math.cos(Math.toRadians(rotation));
    }

    public void stop() {
        leftServo.setPower(0.0);
        rightServo.setPower(0.0);
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void updateEncoders() {
        leftEncoder.update();
        rightEncoder.update();
    }

    public void homeEncoders(double left, double right) {
        leftEncoder.setHome(left);
        rightEncoder.setHome(right);
    }

    public void homeEncoders() {
        leftEncoder.home();
        rightEncoder.home();
    }

    public double getCurrentAngleDegrees() {
        updateEncoders();
        return (rightEncoder.getAbsolutePositionDegrees() + leftEncoder.getAbsolutePositionDegrees()) / 4d;
    }

    public void setPowerWithFF(double power) {
        double gravityFeedForward = -calculateGravityTorque() * kPTorque;

        telemetry.addData("Arm set power", power + gravityFeedForward);
        leftServo.setPower(power + gravityFeedForward);
        rightServo.setPower(power + gravityFeedForward);
    }

    public double getTargetAngle() { return targetAngle; }

    public final double setAngleToleranceDegrees = 3;

    public class SetAngle implements Action {
        private final double targetAngleDegrees;

        public SetAngle(double targetAngleDegrees) {
            this.targetAngleDegrees = targetAngleDegrees;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double error = update(targetAngleDegrees);
            return Math.abs(error) > setAngleToleranceDegrees;
        }
    }

    public Action setAngle(double angle) {
        return new SetAngle(angle);
    }
}

