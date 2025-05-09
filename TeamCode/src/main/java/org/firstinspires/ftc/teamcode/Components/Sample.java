package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;
import org.firstinspires.ftc.teamcode.Kinematics.InverseKinematics;


public class Sample {
    private SampleColors color;
    private double relativeX; //Center
    private double relativeY; //Center

    private double worldX;
    private double worldY;
    private double confidence;
    private Sample.Rotation relativesSampleRotation;
    private InverseKinematics.IKResult IKR;
    public enum Rotation{
        Vertical,
        Horizontal,
        None
    }

    public Sample(SampleColors color, double[] relativeCoordinates, double[] worldCoordinates, Sample.Rotation relativesSampleRotation, double confidence, InverseKinematics.IKResult ikResult){
        this.color = color;
        this.relativeX = relativeCoordinates[0];
        this.relativeY = relativeCoordinates[1];
        this.worldX = worldCoordinates[0];
        this.worldY = worldCoordinates[1];
        this.confidence = confidence;
        this.relativesSampleRotation = relativesSampleRotation;
        this.IKR = ikResult;
    }

    public Sample(){
        this.color = SampleColors.NONE;
        this.relativeX = 0;
        this.relativeY = 0;
        this.worldX = 0;
        this.worldY = 0;
        this.confidence = 0;
        this.relativesSampleRotation = Rotation.Vertical;
        this.IKR = new InverseKinematics.IKResult();
    }

    public SampleColors getColor(){
        return color;
    }

    public double getRelativeX(){
        return relativeX;
    }
    public double getRelativeY(){
        return relativeY;
    }

    public double getWorldX(){
        return worldX;
    }
    public double getWorldY(){
        return worldY;
    }

    public double getConfidence(){
        return confidence;
    }

    public Sample.Rotation getSampleRotation(){
        return relativesSampleRotation;
    }

    public InverseKinematics.IKResult getIKRResult(){
        return IKR;
    }

}
