package org.firstinspires.ftc.teamcode.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;
import org.firstinspires.ftc.teamcode.Components.Sample;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
public class Vision {
    Limelight3A limelight;
    boolean detectorTrue;
    Telemetry telemetry;
    ArrayList<Sample> sampleList;

    //CONSTANTS
    public static double HEIGHT_CAMERA = 15; //INCH
    public static double MOUNT_ANGLE = 25; //DEGREES

    Timer timer;
    Timer gameTimer;
    FtcDashboard dashboard;


    Point[] pixelPoints = new Point[4];//untransformed(pixel coordinates)
    Point[] worldPoints = new Point[4];//transformed(undistorts camera warp)
    MatOfPoint2f pixelMat;
    MatOfPoint2f worldMat;
    Mat transformMatrix;

    public static double WINDOW_HORIZONTAL_OFFSET = 0; //CM
    public static double WINDOW_VERTICAL_OFFSET = 0; //CM

    ArrayList<Sample> samples = new ArrayList<>();

    //Sample side ratio
    public static double SIDE_RATIO = 7/3;
    public Vision(HardwareMap hardwareMap, Telemetry telemetry, boolean detectorTrue){
        this.detectorTrue = detectorTrue;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if(detectorTrue){
            limelight.pipelineSwitch(3);//detector pipeline
        }
        limelight.start();

        gameTimer = new Timer();//for camera screenshots
        timer = new Timer();//for update rate

        //Get corners of the zone that we want to apply the transformation to(corners of the 4 samples
        pixelPoints[0] = new Point(1071.0, 114.5);//top right
        pixelPoints[1] = new Point(264.5, 104.5);//top left
        pixelPoints[2] = new Point(1185.5, 565.0);//bottom right
        pixelPoints[3] = new Point(142.2, 564.5);//bottom left

//        //CM
        worldPoints[0] = new Point(6 * 2.54 + WINDOW_HORIZONTAL_OFFSET,18 * 2.54 + WINDOW_VERTICAL_OFFSET);//top right
        worldPoints[1] = new Point(-6 * 2.54 + WINDOW_HORIZONTAL_OFFSET,18 * 2.54 + WINDOW_VERTICAL_OFFSET);//top left
        worldPoints[2] = new Point(6 * 2.54 + WINDOW_HORIZONTAL_OFFSET,10 * 2.54 + WINDOW_VERTICAL_OFFSET); //bottom right
        worldPoints[3] = new Point(-6 * 2.54 + WINDOW_HORIZONTAL_OFFSET,10 * 2.54 + WINDOW_VERTICAL_OFFSET);//bottom left

        pixelMat = new MatOfPoint2f(pixelPoints);
        worldMat = new MatOfPoint2f(worldPoints);
        transformMatrix = Imgproc.getPerspectiveTransform(pixelMat, worldMat);

        samples = new ArrayList<>();
        this.telemetry = telemetry;
    }

    public String getSampleRotation(List<List<Double>> sampleCornerPixels){
        Point[] corners = new Point[4];
        corners[0] = new Point(sampleCornerPixels.get(0).get(0), sampleCornerPixels.get(0).get(1));//top left
        corners[1] = new Point(sampleCornerPixels.get(1).get(0), sampleCornerPixels.get(1).get(1));//top right
        corners[2] = new Point(sampleCornerPixels.get(2).get(0), sampleCornerPixels.get(2).get(1));//bottom right
        corners[3] = new Point(sampleCornerPixels.get(3).get(0), sampleCornerPixels.get(3).get(1));//bottom left

        //Change pixel corners to robot coordinates
        double[][] robotCoordinates = new double[4][2];
        for(int i = 0; i < 4; i++){
            Mat pointMat = new Mat(1, 1, CvType.CV_64FC2);//Make a point map with 1 row 1 col, 64 bit/double 2 channels(x, y)
            pointMat.put(0, 0, new double[]{corners[i].x,corners[i].y});//add the point
            Mat resultMat = new Mat();
            Core.perspectiveTransform(pointMat, resultMat, transformMatrix);//applied transformation
            robotCoordinates[i] = resultMat.get(0,0);
            //free up memory
            pointMat.release();
            resultMat.release();
        }

        //calculate the width and height of the bounding box
        double width = Math.hypot(robotCoordinates[1][0] - robotCoordinates[0][0], robotCoordinates[1][1] - robotCoordinates[0][1]);
        double height = Math.hypot(robotCoordinates[3][0] - robotCoordinates[0][0], robotCoordinates[3][1] - robotCoordinates[0][1]);

        double lengthRatio = height/width;

        telemetry.addData("Width", width);
        telemetry.addData("Height", height);
        telemetry.addData("Sides Ratio", lengthRatio);

        if(lengthRatio < 1){
            return "Horizontal";//closer to horizontal than vertical
        }
        else{
            return "Vertical";//closer to vertical than horizontal
        }

    }

    public void update(){
        limelight.captureSnapshot("Time:" + gameTimer.updateTime());
        //reupdate samples list
        samples = new ArrayList<>();
        timer.updateTime();
        double deltaTime = timer.getDeltaTime();

        //Process result of pipeline
        LLResult llResult = limelight.getLatestResult();
        List<DetectorResult> detectorResults = llResult.getDetectorResults();
        double sampleCount = 0;

        for(DetectorResult result : detectorResults){
            sampleCount++;
            String sampleType = result.getClassName();
            double confidence = result.getConfidence();

            SampleColors sampleColor = SampleColors.NONE;
            String type = result.getClassName();
//
            if(type.equals("blue sample")){
                sampleColor = SampleColors.BLUE;
            }
            else if(type.equals("red sample")){
                sampleColor = SampleColors.RED;
            }
            else{
                sampleColor = SampleColors.YELLOW;
            }

            //Trig implementation
            double xAngle = result.getTargetXDegrees();
            double yAngle = result.getTargetYDegrees();
//            double theta = MOUNT_ANGLE + y;
//
//            double horizontalDistance = HEIGHT_CAMERA / Math.tan(Math.toRadians(theta));
//            double verticalDistance = Math.tan(Math.toRadians(x)) * horizontalDistance;
//
//            double area = result.getTargetArea();

            //Corners coordinates of sample-> to calculate sample rotation
            List<List<Double>> corners = result.getTargetCorners();
            telemetry.addData("Corner World Coordinates", corners.toString());
            String rotation = getSampleRotation(corners);
            telemetry.addData("Sample Closer To:", rotation);
            Sample.Rotation rot;
            if(rotation.equals(Sample.Rotation.Horizontal)){
                rot = Sample.Rotation.Horizontal;
            }
            else{
                rot = Sample.Rotation.Vertical;
            }
            //Center of sample -> transform to world coordinates
            double x = result.getTargetXPixels();
            double y = result.getTargetYPixels();
//
            Mat pointMat = new Mat(1, 1, CvType.CV_64FC2);//Make a point map with 1 row 1 col, 64 bit/double 2 channels(x, y)
            pointMat.put(0, 0, new double[]{x,y});//add the point
            Mat resultMat = new Mat();
            Core.perspectiveTransform(pointMat, resultMat, transformMatrix);//applied transformation
            double[] sampleFieldCoord = resultMat.get(0,0);
            sampleFieldCoord = new double[]{sampleFieldCoord[0]/2.54, sampleFieldCoord[1]/2.54};

            Sample sample = new Sample(sampleColor, sampleFieldCoord[0] / 2.54, sampleFieldCoord[1] / 2.54, rot, confidence);

//            //TODO: Check if is reachable using IK before adding to samples
//
//            //Autonomous: have the entire space but want to sort by weights closest to the center
//
            samples.add(sample);

            //create sample and add it to array list

            telemetry.addLine("----------------------------");
            telemetry.addLine("Sample " + sampleCount);
            telemetry.addData("Update Rate(Hz)", 1/deltaTime);

            telemetry.addLine("PixelCoord: { " + x + ", " + y + "} ");
            telemetry.addLine("FieldCoord(Relative to Cam): { " + sampleFieldCoord[0] + ", " + sampleFieldCoord[1] + "} ");
            telemetry.addData("Color", sampleType);
//            telemetry.addData("Hor Distance", horizontalDistance);
//            telemetry.addData("Vert Distance", verticalDistance);

            telemetry.addData("Confidence", confidence);
            telemetry.addData("CameraXAngle", xAngle);
            telemetry.addData("CameraYAngle", yAngle);
//            telemetry.addData("CameraArea", area);

            //Free up memory
            pointMat.release();
            resultMat.release();
        }

    }

    //Get Closest Sample of a given color to Center of camera's fov
    public Sample getClosestSample(SampleColors sampleColors, double bufferTime){//bufferTime in seconds
        double startTime = gameTimer.updateTime();
        while(timer.updateTime() < startTime + bufferTime) {
            limelight.captureSnapshot("Time:" + gameTimer.updateTime());
            for (Sample sample : samples) {
                if (sample.getColor() == sampleColors) {
                    return sample;
                }
            }
        }
        return null; //return null if can't find
    }


    public void clearScreenshots(){
        limelight.deleteSnapshots();
        telemetry.addLine("Cleared all screenshots");
    }
}

