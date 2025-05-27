package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.CRArm;

@TeleOp
@Config
public class VoltageOffsetData extends OpMode {
    CRArm controller;

    public static double startPower = 0;
    public static int numTrials = 5;

    public static int SAME_VOLTAGE_COUNT = 5;
    public static double SAME_VOLTAGE_THRESHOLD = 0.005;

    public static double endPower = 0.5;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new CRArm(hardwareMap, telemetry);

//        telemetry.setAutoClear(false);
        telemetry.addLine("Initialized");
        telemetry.update();
    }


    double leftDiffTotal;
    double rightDiffTotal;

    @Override
    public void start() {

        for (int i = 0; i < numTrials; i++) {
            double[] trial = runTrial();

            leftDiffTotal += trial[0];
            rightDiffTotal += trial[1];
        }

        telemetry.addData("Avg Left Diff", leftDiffTotal / numTrials);
        telemetry.addData("Avg Right Diff", rightDiffTotal / numTrials);
        telemetry.update();
    }

    public double[] runTrial() {
        double leftStartVoltage;
        double rightStartVoltage;

        double leftEndVoltage;
        double rightEndVoltage;

        controller.rightServo.setPower(startPower);
        controller.leftServo.setPower(startPower);

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        double[] results = getStabilizedVoltages();
        leftStartVoltage = results[0];
        rightStartVoltage = results[1];

        telemetry.addData("Left  start (power = " + startPower + ")", results[0]);
        telemetry.addData("Right start (power = " + startPower + ")", results[1]);

        controller.rightServo.setPower(endPower);
        controller.leftServo.setPower(endPower);

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        results = getStabilizedVoltages();
        leftEndVoltage = results[0];
        rightEndVoltage = results[1];

        telemetry.addData("Left   end (power = " + endPower + ")", results[0]);
        telemetry.addData("Right  end (power = " + endPower + ")", results[1]);

        telemetry.addData("Left voltage diff", leftEndVoltage - leftStartVoltage);
        telemetry.addData("Right voltage diff", rightEndVoltage - rightStartVoltage);

        telemetry.update();

        controller.leftServo.setPower(0);
        controller.rightServo.setPower(0);


        return new double[] { leftEndVoltage - leftStartVoltage, rightEndVoltage - rightStartVoltage};
    }

    public double[] getStabilizedVoltages() {
        double finalLeft = 0;
        double finalRight = 0;

        int leftSameCount = 0;
        int rightSameCount = 0;

        double lastLeftVoltage = 0;
        double lastRightVoltage = 0;


        while (rightSameCount < SAME_VOLTAGE_COUNT || leftSameCount < SAME_VOLTAGE_COUNT) {
            double leftVoltage = controller.leftEncoder.getVoltage();
            double rightVoltage = controller.rightEncoder.getVoltage();

            if (Math.abs(lastRightVoltage - rightVoltage) < SAME_VOLTAGE_THRESHOLD) {
                rightSameCount++;
            }
            else {
                lastRightVoltage = rightVoltage;
                rightSameCount = 0;
            }

            if (Math.abs(lastLeftVoltage - leftVoltage) < SAME_VOLTAGE_THRESHOLD) {
                leftSameCount++;
            }
            else {
                leftSameCount = 0;
                lastLeftVoltage = leftVoltage;
            }

            if (leftSameCount >= SAME_VOLTAGE_COUNT) finalLeft = leftVoltage;
            if (rightSameCount >= SAME_VOLTAGE_COUNT) finalRight = rightVoltage;
        }

        return new double[] {finalLeft, finalRight};
    }



    @Override
    public void loop() {


    }
}
