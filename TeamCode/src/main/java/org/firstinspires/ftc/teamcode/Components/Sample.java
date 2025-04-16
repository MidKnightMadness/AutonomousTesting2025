package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;

public class Sample {
    SampleColors color;
    double robotXCoordinate;
    double robotYCoordinate;
    double sampleRotation;

    public Sample(SampleColors color, double robotXCoordinate, double robotYCoordinate, double sampleRotation){
        this.color = color;
        this.robotXCoordinate = robotXCoordinate;
        this.robotYCoordinate = robotYCoordinate;
        this.sampleRotation = sampleRotation;
    }
}
