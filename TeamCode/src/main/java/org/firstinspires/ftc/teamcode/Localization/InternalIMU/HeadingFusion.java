package org.firstinspires.ftc.teamcode.Localization.InternalIMU;

@Deprecated
public class HeadingFusion {
    private double integratedError = 0.0;
    private final double correctionGain;
    private final double lambda;
    private final double integralDecayFactor;

    private double imuGain;

    double fusedHeading;

    public HeadingFusion(double correctionGain, double lambda, double integralDecayFactor) {
        this.correctionGain = correctionGain;
        this.lambda = lambda;
        this.integralDecayFactor = integralDecayFactor;
    }

    public double update(double imuHeading, double odoHeading, double dt) {
        double error = odoHeading - fusedHeading;

        double alpha = Math.exp(-lambda * Math.abs(error));

        imuGain = alpha;

        // Complementary filter with adaptive weighting
        fusedHeading = alpha * imuHeading + (1 - alpha) * odoHeading;

        integratedError = (1 - integralDecayFactor) * integratedError;
        integratedError += error * dt;
        fusedHeading += correctionGain * integratedError;

        return fusedHeading;
    }

    public double getIMUGain() {
        return imuGain;
    }

    public double getIntegratedError() {
        return integratedError;
    }
}

