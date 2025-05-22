package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;
import org.firstinspires.ftc.teamcode.Kinematics.InverseKinematics;


public class Sample {
    private SampleColors color;
    private double relativeX; //Center
    private double relativeY; //Center

//    private double worldX;
//    private double worldY;
    private double confidence;
    private double hyp;
    private double sampleTheta;
//    private InverseKinematics.IKResult IKR;
    public enum Rotation{
        Vertical,
        Horizontal,
        None
    }

    public Sample(SampleColors color, double[] relativeCoordinates, double hyp, double sampleTheta, double confidence){
        this.color = color;
        this.relativeX = relativeCoordinates[0];
        this.relativeY = relativeCoordinates[1];
//        this.worldX = worldCoordinates[0];
//        this.worldY = worldCoordinates[1];
        this.hyp = hyp;
        this.sampleTheta = sampleTheta;
        this.confidence = confidence;
    }

    public Sample(){
        this.color = SampleColors.NONE;
        this.relativeX = 0;
        this.relativeY = 0;
//        this.worldX = 0;
//        this.worldY = 0;
        this.hyp = 0;
        this.sampleTheta = 0;
        this.confidence = 0;
//        this.IKR = new InverseKinematics.IKResult();
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

//    public double getWorldX(){
//        return worldX;
//    }
//    public double getWorldY(){
//        return worldY;
//    }

    public double getSampleDistance(){
        return hyp;
    }
    public double getSampleTheta(){//Radians
        return sampleTheta;
    }


    public double getConfidence(){
        return confidence;
    }

//    public InverseKinematics.IKResult getIKRResult(){
//        return IKR;
//    }

}
