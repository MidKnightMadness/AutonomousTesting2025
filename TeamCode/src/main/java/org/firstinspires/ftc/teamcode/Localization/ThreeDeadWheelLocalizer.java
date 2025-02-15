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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.tuning.HeadingFusion;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    // theoretical values from CAD
    public static class Params {
        public double par0YTicks = -2768.62492919;
        public double par1YTicks = 2762.17518481;
        public double perpXTicks = 656.514140254;
    }

    // measured values from AngularRampLogger
//    public static class Params {
//        public double par0YTicks = -2742.8111015606496; // y position of the first parallel encoder (in tick units)
//        public double par1YTicks = 2804.674862264565; // y position of the second parallel encoder (in tick units)
//        public double perpXTicks = 656.514140254; // x position of the perpendicular encoder (in tick units)
//    }

    public static Params PARAMS = new Params();

    public final Encoder par0, par1, perp;

    public final double inPerTick;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private boolean initialized;
    private Pose2d pose;

    HeadingFusion headingFusion;
    HeadingFusion headingVelocity;
    HeadingFusion headingDelta;

    double headingOffset;

    Timer timer;
    BNO055IMU imu;

    Telemetry tel;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, BNO055IMU imu, double inPerTick, Pose2d initialPose, Telemetry tel) {
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "Left Encoder")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "FR")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "Front Encoder")));

        this.tel = tel;

        headingFusion = new HeadingFusion(0.05, 5, 0.05);
        headingVelocity = new HeadingFusion(0.05, 5, 0.05);
        headingDelta = new HeadingFusion(0.05, 5, 0.05);

        this.imu = imu;

        par0.setDirection(DcMotorSimple.Direction.REVERSE);

        this.inPerTick = inPerTick;

        timer = new Timer();
        // FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);

        pose = initialPose;
        headingOffset = initialPose.heading.toDouble();
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    double fusedHeading;
    double lastIMUHeading = 0;

    @Override
    public PoseVelocity2d update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        // FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

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

        pose = pose.plus(twist.value());

        double imuHeading = headingOffset + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        fusedHeading = headingFusion.update(imuHeading, pose.heading.toDouble(), timer.getDeltaTime());

        pose = new Pose2d(pose.position, fusedHeading);
        lastIMUHeading = imuHeading;

        tel.addData("imuHeading", imuHeading);
        tel.addData("dt", timer.getDeltaTime());

        return twist.velocity().value();
    }
}