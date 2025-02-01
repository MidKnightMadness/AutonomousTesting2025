package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Timer;

@Config
public class Wrist {
    //TODO: Fix whether want seperate positions in auto and teleop or same
    //Wrist should be all throughout driver controlled in teleop except couple buttons for specimen and sample preset positions


    //AUTO:

    Timer timer;
    public static double SAMPLE_LINE_POSITION_AUTO = 0.56; //sample pickup in autonomous
    public static double SAMPLE_SUB_POSITION = 0.56;
    public static double BASKET_POSITION_AUTO = 0.56; //sample dropoff in basket(top basket) at certain arm position
    public static double INIT_POSITION = 0.117;

    public static double SPECIMEN_INTAKE_POSITION = 0.63; //specimen position
    public static double SPECIMEN_OUTAKE_POSITION = 0;

    public Servo servo;
    ElapsedTime elapsedTime;

    public Wrist(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "Wrist");

        timer = new Timer();
    }

    public Action setPosition(double position) {
        return new Wrist.SetPosition(position);
    }

    public Action setPositionSmooth(double position, double movementTime){
        return new Wrist.SetPosition(position, movementTime);
    }

    public void setInitPosition() {
        servo.setPosition(INIT_POSITION);
    }


    public class SetPosition implements Action {
        private final double targetPosition;
        private final double movementTime;
        double startTime;
        double startPosition;
        boolean initialized = false;

        public SetPosition(double position) {
            this.targetPosition = position;
            this.movementTime = 0;
        }

        public SetPosition(double position, double movementTime) {
            this.targetPosition = position;
            this.movementTime = movementTime;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                startTime = timer.updateTime();
                startPosition = servo.getPosition();
                initialized = true;
            }

            packet.addLine("Time: " + (timer.updateTime() - startTime));



            //TODO: CHECK IF movement time = 0
            if (movementTime != 0) {
                double timeSinceStart = timer.updateTime() - startTime;
                double percentOfMovement = Math.min(1, timeSinceStart / movementTime);
                double intermediatePoint = (targetPosition - startPosition) * percentOfMovement + startPosition;


                packet.addLine("Position " + intermediatePoint);

                servo.setPosition(intermediatePoint);
                return timeSinceStart < movementTime;
            } else {
                packet.addLine("Position " + targetPosition);
                servo.setPosition(targetPosition);
                return false;
            }
        }
    }
}
