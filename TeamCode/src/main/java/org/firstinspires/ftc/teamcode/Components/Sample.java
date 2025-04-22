package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;

public class Sample {
    private SampleColors color;
    private double fieldX; //Center
    private double fieldY; //Center
    private Rotation sampleRotation; //closer to Vertical or horizontal
    private double confidence;
    public enum Rotation{
        Vertical,
        Horizontal,
        None
    }

    public Sample(SampleColors color, double fieldX, double fieldY, Rotation sampleRotation, double confidence){
        this.color = color;
        this.fieldX = fieldX;
        this.fieldY = fieldY;
        this.sampleRotation = sampleRotation;
        this.confidence = confidence;
    }

    public Sample(){
        this.color = SampleColors.NONE;
        this.fieldX = 0;
        this.fieldY = 0;
        this.sampleRotation = Rotation.None;
        this.confidence = 0;
    }

    public SampleColors getColor(){
        return color;
    }

    public double getFieldXRelativeToCam(){
        return fieldX;
    }
    public double getFieldYRelativeToCam(){
        return fieldY;
    }

    public double getConfidence(){
        return confidence;
    }

    public String toString(){
        return "Sample: { " + color + ", Field Coordinates(Relative to Cam): [ " + fieldX + ", " + fieldY + "], Rotation: " + sampleRotation + ", Confidence " + confidence + "}";
    }
}
