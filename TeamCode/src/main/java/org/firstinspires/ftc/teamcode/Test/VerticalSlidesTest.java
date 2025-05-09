package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.VerticalSlides;

public class VerticalSlidesTest extends OpMode {
    VerticalSlides verticalSlides;


    @Override
    public void init() {
        verticalSlides = new VerticalSlides(hardwareMap);
    }

    double targetHeight = 0;
    @Override
    public void loop() {
        if(gamepad1.dpad_up){
            targetHeight += 0.5;
        }
        if(gamepad1.dpad_down){
            targetHeight -= 0.5;
        }

        if(gamepad1.a){
            verticalSlides.setPosition(targetHeight, 0.6);
        }
        else if(gamepad1.b){
            verticalSlides.setPosition(0, 0.6);
        }

    }
}
