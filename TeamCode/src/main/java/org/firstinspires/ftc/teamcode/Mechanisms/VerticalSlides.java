package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class VerticalSlides {
    DcMotorEx rightMotor;
    DcMotorEx leftMotor;

    public static double BASKET_SCORING_POSITION = 2150;
    public static double DOWN_POSITION = 10;

    public static double SPECIMEN_INTAKE = 700;
    public static double SPECIMEN_OUTTAKE = 2200;

    public static double feedforwardPower = 0.05;

    double inactivePower = 0;

    //HANG:
    public VerticalSlides(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotorEx.class, "Right Slide");
        leftMotor = hardwareMap.get(DcMotorEx.class, "Left Slide");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void enableFeedforward() {
        inactivePower = feedforwardPower;
        leftMotor.setPower(inactivePower);
        rightMotor.setPower(inactivePower);
    }

    public void disableFeedforward() {
        inactivePower = 0;
        leftMotor.setPower(inactivePower);
        rightMotor.setPower(inactivePower);
    }

    public Action setPosition(double targetPosition, double power) {
        return new Lift(targetPosition, power);
    }

    public Action liftUp(double power) {
        return new Lift(BASKET_SCORING_POSITION, power);
    }

    public Action bringDown(double power) {
        return new Lift(DOWN_POSITION, power);
    }

    public class Lift implements Action {
        private boolean initialized = false;
        private final double targetPosition;
        private final double power;

        public Lift(double targetPosition, double power) {
            this.targetPosition = targetPosition;
            this.power = power;
        }

        double leftDirection;
        double rightDirection;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftDirection = Math.signum(targetPosition - leftMotor.getCurrentPosition());
                rightDirection = Math.signum(targetPosition - rightMotor.getCurrentPosition());

                rightMotor.setPower(power * rightDirection);
                leftMotor.setPower(power * leftDirection);

                initialized = true;
            }

            double leftPos = leftMotor.getCurrentPosition();
            double rightPos = rightMotor.getCurrentPosition();

            boolean leftCompleted = leftDirection == 1 ? (leftPos > targetPosition) : (leftPos < targetPosition);
            boolean rightCompleted = rightDirection == 1 ? (rightPos > targetPosition) : (leftPos < targetPosition);

            if (leftCompleted) leftMotor.setPower(inactivePower);
            if (rightCompleted) rightMotor.setPower(inactivePower);

            return !(rightCompleted && leftCompleted);
        }
    }

    public DcMotorEx getLeftMotor() {
        return leftMotor;
    }

    public DcMotorEx getRightMotor() {
        return rightMotor;
    }

    public void resetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
