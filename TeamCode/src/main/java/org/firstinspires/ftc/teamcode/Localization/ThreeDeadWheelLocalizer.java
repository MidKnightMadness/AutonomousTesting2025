package org.firstinspires.ftc.teamcode.Localization;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.RunOptions;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    public static class Params {
        public double par0YTicks = -2768.62492919;
        public double par1YTicks = 2762.17518481;
        public double perpXTicks = 656.514140254;

        // IMU Fusion Parameters
        public int imuUpdateInterval = 1;
        public double imuDifferenceThreshold = 0.1; // rad/s
        public double encoderImuDifferenceThreshold = 0.2; // rad
        public double imuWeight = 0.7;
        public double encoderWeight = 0.3;
        public double maxOmega = Math.toRadians(720); // Max expected angular velocity (rad/s)
    }

    public static Params PARAMS = new Params();

    private final Encoder par0, par1, perp;
    private final double inPerTick;
    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private boolean initialized;
    private Pose2d pose;
    private final IMU imuExpansion, imuControl;
    private final Timer timer;
    private int loopCount = 0;

    Telemetry telemetry;

    double lastControlYaw;
    double lastExpansionYaw;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, IMU imuControl, IMU imuExpansion, double inPerTick, Pose2d initialPose, Telemetry telemetry) {
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "Left Encoder")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "FR")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "Front Encoder")));
        par0.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;
        this.inPerTick = inPerTick;
        this.imuControl = imuControl;
        this.imuExpansion = imuExpansion;
        this.pose = initialPose;
        this.timer = new Timer();
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public PoseVelocity2d update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        timer.updateTime();

        if (!initialized) {
            initialized = true;
            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;
            return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        loopCount++;
        if (loopCount < PARAMS.imuUpdateInterval) {
            pose = pose.plus(twist.value());
            return twist.velocity().value();
        }

        loopCount = 0;

        // IMU Fusion
        double deltaTime = timer.getDeltaTime();
        double encoderDeltaHeading = twist.angle.get(0);
        double omegaControl = Double.NaN, omegaExpansion = Double.NaN;

        try {
            AngularVelocity controlAngVel = imuControl.getRobotAngularVelocity(AngleUnit.RADIANS);
            omegaControl = controlAngVel.zRotationRate;
        } catch (Exception e) { /* Handle error */ }

        try {
            AngularVelocity expansionAngVel = imuExpansion.getRobotAngularVelocity(AngleUnit.RADIANS);
            omegaExpansion = expansionAngVel.zRotationRate;
        } catch (Exception e) { /* Handle error */ }

        boolean controlValid = !Double.isNaN(omegaControl) && Math.abs(omegaControl) <= PARAMS.maxOmega;
        boolean expansionValid = !Double.isNaN(omegaExpansion) && Math.abs(omegaExpansion) <= PARAMS.maxOmega;

        double avgOmega;
        if (controlValid && expansionValid) {
            avgOmega = (omegaControl + omegaExpansion) / 2.0;
        } else if (controlValid) {
            avgOmega = omegaControl;
        } else if (expansionValid) {
            avgOmega = omegaExpansion;
        } else {
            avgOmega = twist.angle.get(1);
        }

        double imuDeltaHeading = avgOmega * deltaTime;
        double fusedDeltaHeading = encoderDeltaHeading;
        double fusedOmega = avgOmega;

        if (controlValid || expansionValid) {
            // normal case, all 3 readings good
            if (controlValid && expansionValid) {
                double imuDiff = Math.abs(omegaControl - omegaExpansion);
                if (imuDiff > PARAMS.imuDifferenceThreshold) {
                    double controlDelta = omegaControl * deltaTime;
                    double expansionDelta = omegaExpansion * deltaTime;
                    double controlEncoderDiff = Math.abs(controlDelta - encoderDeltaHeading);
                    double expansionEncoderDiff = Math.abs(expansionDelta - encoderDeltaHeading);

                    if (controlEncoderDiff < expansionEncoderDiff) {
                        fusedDeltaHeading = controlDelta;
                        fusedOmega = omegaControl;
                    } else {
                        fusedDeltaHeading = expansionDelta;
                        fusedOmega = omegaExpansion;
                    }
                } else {
                    if (RunOptions.enableTelemetry) {
                        telemetry.addData("Difference In IMUs", String.format("Control - %f, Expansion - %f, Diff - %f", omegaControl, omegaExpansion, imuDiff));
                    }
                    double encoderDiff = Math.abs(encoderDeltaHeading - imuDeltaHeading);
                    if (encoderDiff > PARAMS.encoderImuDifferenceThreshold) {
                        fusedDeltaHeading = imuDeltaHeading;
                    } else {
                        fusedDeltaHeading = PARAMS.imuWeight * imuDeltaHeading + PARAMS.encoderWeight * encoderDeltaHeading;
                        fusedOmega = PARAMS.imuWeight * avgOmega + PARAMS.encoderWeight * twist.angle.get(1);
                    }
                }
            } else {
                // If either control or expansion isn't valid
                double imuOmega = controlValid ? omegaControl : omegaExpansion;
                double imuDelta = imuOmega * deltaTime;
                double encoderDiff = Math.abs(imuDelta - encoderDeltaHeading);

                // if encoder value is an outlier
                if (encoderDiff > PARAMS.encoderImuDifferenceThreshold) {
                    fusedDeltaHeading = imuDelta;
                    fusedOmega = imuOmega;
                } else {
                    fusedDeltaHeading = PARAMS.imuWeight * imuDelta + PARAMS.encoderWeight * encoderDeltaHeading;
                    fusedOmega = PARAMS.imuWeight * imuOmega + PARAMS.encoderWeight * twist.angle.get(1);
                }
            }
        }

        Twist2dDual<Time> fusedTwist = new Twist2dDual<>(
                twist.line,
                new DualNum<>(new double[]{fusedDeltaHeading, fusedOmega})
        );

        pose = pose.plus(fusedTwist.value());

        return new PoseVelocity2d(twist.velocity().linearVel.value(), fusedOmega);
    }
}
