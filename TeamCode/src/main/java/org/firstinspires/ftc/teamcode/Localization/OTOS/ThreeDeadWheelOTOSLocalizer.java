package org.firstinspires.ftc.teamcode.Localization.OTOS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Localization.Localizer;
import org.firstinspires.ftc.teamcode.Localization.ThreeDeadWheelLocalizer;

public final class ThreeDeadWheelOTOSLocalizer implements Localizer {
    public static class Params {
        public double par0YTicks = ThreeDeadWheelLocalizer.PARAMS.par0YTicks; // 138.874mm
        public double par1YTicks = ThreeDeadWheelLocalizer.PARAMS.par1YTicks; // 138.874mm
        public double perpXTicks = ThreeDeadWheelLocalizer.PARAMS.perpXTicks; // 33mm
    }

    public static Params PARAMS = new Params();

    public final Encoder par0, par1, perp;

    public final double inPerTick;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private boolean initialized;

    private Rotation2d lastHeading;
    private double lastRawHeadingVel, headingVelOffset;

    private Pose2d pose;

    SparkFunOTOS otos;
    public ThreeDeadWheelOTOSLocalizer(HardwareMap hardwareMap, SparkFunOTOS otos, double inPerTick, Pose2d pose) {
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "Left Encoder")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "FR")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "Front Encoder")));

        this.otos = otos;
        par0.setDirection(DcMotorSimple.Direction.REVERSE);

        this.inPerTick = inPerTick;

//        FlightRecorder.write("THREE_DEAD_WHEEL_IMU_PARAMS", PARAMS);

        this.pose = pose;
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    double lastHeadingDelta;

    @Override
    public Pose2d getPose() {
        return pose;
    }
    @Override
    public PoseVelocity2d update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        // FlightRecorder.write("THREE_DEAD_WHEEL_IMU_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

        Rotation2d heading = Rotation2d.exp(otos.getPosition().h);


        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        double rawHeadingVel = otos.getVelocity().h;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }

        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            lastHeading = heading;

            return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        // IMU failure, reboot
//        if (Math.abs(headingDelta) > Math.toRadians(0.5) && headingVel < 0.01) {
//            imu.initialize(DualIMU.parameters);
//            headingDelta = lastHeadingDelta;
//
//        }

        lastHeadingDelta = headingDelta;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
//                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks), // todo
                                (double) (par1PosDelta + par0PosDelta) / 2,
//                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                (double) (par1PosVel.velocity + par0PosVel.velocity) / 2
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
//                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
//                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        lastHeading = heading;
        pose = pose.plus(twist.value());

//        pose = new Pose2d(new Vector2d(pose.position.x + (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks) * inPerTick,
//                pose.position.y + (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta) * inPerTick)
//                , headingDelta);

        return twist.velocity().value();
    }
}
