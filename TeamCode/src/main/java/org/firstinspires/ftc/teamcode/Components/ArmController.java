package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmController {

    private final CRServo leftServo;
    private final CRServo rightServo;

    public final AxonEncoder leftEncoder;
    public final AxonEncoder rightEncoder;

    public static double leftEncoderOffset;
    public static double rightEncoderOffset;

    // PID coefficients
    private final double kP;
    private final double kI;
    private final double kD;

    // PID state
    private double integral = 0.0;
    private double previousError = 0.0;
    private final Timer timer;

    // Output limits
    private final double minOutput;
    private final double maxOutput;

    // Deadband threshold
    private final double deadband;

    public PIDController leftController;
    public PIDController rightController;

    public static double p;
    public static double i;
    public static double d;

    Telemetry telemetry;

    public ArmController(HardwareMap hardwareMap,
                         double kP, double kI, double kD,
                         double minOutput, double maxOutput, double deadband, Telemetry telemetry) {

        this.leftServo = hardwareMap.get(CRServo.class, "Left Arm");
        this.rightServo = hardwareMap.get(CRServo.class, "Right Arm");

        this.leftEncoder = new AxonEncoder(hardwareMap, "Arm Left Encoder", leftEncoderOffset);
        this.rightEncoder = new AxonEncoder(hardwareMap, "Arm Right Encoder", rightEncoderOffset);
        rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        leftController = new PIDController(p, i, d);
        rightController = new PIDController(p, i, d);

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        this.deadband = deadband;

        this.timer = new Timer();
        this.telemetry = telemetry;
    }

    public void setIndividualPositions(double left, double right) {
        timer.updateTime();

        leftEncoder.update();
        rightEncoder.update();

        leftController.setCoefficients(p, i, d);
        rightController.setCoefficients(p, i, d);

        double leftPow = leftController.update(left, leftEncoder.getAbsolutePositionDegrees(), timer.getDeltaTime());
        double rightPow = rightController.update(right, rightEncoder.getAbsolutePositionDegrees(), timer.getDeltaTime());

        telemetry.addData("LPower", leftPow);
        telemetry.addData("RPower", rightPow);

        leftServo.setPower(leftPow);
        rightServo.setPower(rightPow);
    }

    public void update(double targetAngleDegrees) {
        timer.updateTime();

        // Update encoder reading
        leftEncoder.update();
        double currentAngle = leftEncoder.getAbsolutePositionDegrees();

        // Calculate error
        double error = targetAngleDegrees - currentAngle;

        // Normalize error to [-180, 180] range
        error = ((error + 180) % 360) - 180;

        // PID calculations
        integral += error * timer.getDeltaTime();
        double derivative = (error - previousError) / timer.getDeltaTime();
        previousError = error;

        double output = kP * error + kI * integral + kD * derivative;

        // Apply deadband
        if (Math.abs(error) < deadband) {
            output = 0.0;
        }

        // Clamp output to min/max
        output = Math.max(minOutput, Math.min(maxOutput, output));

        // Set servo power
        leftServo.setPower(output);
    }

    /**
     * Stops the servo.
     */
    public void stop() {
        leftServo.setPower(0.0);
    }

    /**
     * Resets the encoder's position tracking.
     */
    public void resetEncoder() {
        leftEncoder.reset();
    }

    /**
     * Retrieves the current angle in degrees.
     * @return Current angle in degrees.
     */
    public double getCurrentAngleDegrees() {
        return leftEncoder.getAbsolutePositionDegrees();
    }
}

