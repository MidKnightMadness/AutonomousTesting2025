package org.firstinspires.ftc.teamcode.tuning;

@Deprecated
public class IMUOdometryFusion {
    private double imuHeading;
    private final double kf;
    private final double alpha;

    public IMUOdometryFusion(double kf, double alpha) {
        this.kf = kf;
        this.alpha = alpha;
    }

    public double update(double imuHeading, double odometryHeading) {
        double error = odometryHeading - imuHeading;

        imuHeading += error * kf;

        return alpha * imuHeading + (1 - imuHeading) * odometryHeading;
    }
}
