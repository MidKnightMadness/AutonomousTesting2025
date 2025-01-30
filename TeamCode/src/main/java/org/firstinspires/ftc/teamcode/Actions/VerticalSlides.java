package org.firstinspires.ftc.teamcode.Actions;

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
    private static final double INTERMEDIATE_POSITION = 800;
    DcMotorEx rightMotor;
    DcMotorEx leftMotor;

    public static double power = 0.8;
    public static double bringDownPower = 0.8;
    public static double BASKET_SCORING_POSITION = 2150;
    public static double DOWN_POSITION = 10;

    public VerticalSlides(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotorEx.class, "Right Vertical Slides");
        leftMotor = hardwareMap.get(DcMotorEx.class, "Left Vertical Slides");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();


        power = 0.8;
    }

    public Action setPosition(double targetPosition) {
        return new Lift(targetPosition);
    }
    public Action liftUp() {
        return new Lift(BASKET_SCORING_POSITION);
    }

    public Action bringDown(double power) {
        bringDownPower = power;
        return new Lift(DOWN_POSITION);
    }

    //TODO: Figure out way to smooth or whether we really need an intermediate lift up
    public Action intermediateLiftUp() {
        return new Lift(INTERMEDIATE_POSITION);
    }

    public class Lift implements Action {
        private boolean initialized = false;
        private final double targetPosition;

        public Lift(double targetPosition) {
            this.targetPosition = targetPosition;
        }

        double leftDirection;
        double rightDirection;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftDirection = Math.signum(targetPosition - leftMotor.getCurrentPosition());
                rightDirection = Math.signum(targetPosition - rightMotor.getCurrentPosition());
                if (rightDirection == -1) {
                    rightMotor.setPower(bringDownPower * rightDirection);
                    leftMotor.setPower(bringDownPower * leftDirection);
                }
                else {
                    rightMotor.setPower(power * rightDirection);
                    leftMotor.setPower(power * leftDirection);
                }

                initialized = true;
            }

            double leftPos = leftMotor.getCurrentPosition();
            double rightPos = rightMotor.getCurrentPosition();

            packet.put("leftLiftPosition", leftPos);
            packet.put("rightLiftPosition", rightPos);

            boolean leftCompleted = leftDirection == 1 ? (leftPos > targetPosition) : (leftPos < targetPosition);
            boolean rightCompleted = rightDirection == 1 ? (rightPos > targetPosition) : (leftPos < targetPosition);

            if (leftCompleted) leftMotor.setPower(0);
            if (rightCompleted) rightMotor.setPower(0);

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
