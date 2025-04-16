package org.firstinspires.ftc.teamcode.Camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Sample;

import java.util.ArrayList;
import java.util.List;

public class Vision {
    Limelight3A limelight;
    boolean detectorTrue;
    Telemetry telemetry;
    ArrayList<Sample> sampleList;



    public Vision(HardwareMap hardwareMap, Telemetry telemetry, boolean detectorTrue){
        this.detectorTrue = detectorTrue;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if(detectorTrue){
            limelight.pipelineSwitch(3);//detector pipeline
        }
        limelight.start();



        this.telemetry = telemetry;
    }

    public void update(){
        //Process result of pipeline
        LLResult llResult = limelight.getLatestResult();
        List<DetectorResult> detectorResults = llResult.getDetectorResults();
        double sampleCount = 0;
        for(DetectorResult result : detectorResults){
            sampleCount++;

            String sampleType = result.getClassName();
            double confidence = result.getConfidence();


            double x = result.getTargetXDegrees();
            double y = result.getTargetYDegrees();
            double area = result.getTargetArea();

            //create sample and add it to array list
            telemetry.addLine("----------------------------");
            telemetry.addData("Sample Count", sampleCount);
            telemetry.addData("Sample Color", sampleType);
            telemetry.addData("Confidence", confidence);
            telemetry.addData("CameraX", x);
            telemetry.addData("CameraY", y);
            telemetry.addData("CameraArea", area);


        }
    }
}

