package org.firstinspires.ftc.teamcode.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;
import org.firstinspires.ftc.teamcode.Components.Sample;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.Kinematics.InverseKinematics;
import org.firstinspires.ftc.teamcode.Kinematics.Kinematics;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;

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


    //initially 4 points -> 6 points test
    Point[] pixelPoints = new Point[4];//untransformed(pixel coordinates)
    Point[] worldPoints = new Point[4];//transformed(undistorts camera warp)
    MatOfPoint2f pixelMat;
    MatOfPoint2f worldMat;//Mat values calibrated to (0,0) being from the center of the front of the robot chassis
    //Positive x pixels/inch -> right, Positive y pixels/inch -> down
    Mat transformMatrix;

    public static double WINDOW_HORIZONTAL_OFFSET = 1; //CM
    public static double WINDOW_VERTICAL_OFFSET = 0; //CM
    public static double WINDOW_SQUISH_OFFSET = 3;//CM

    public static double ROBOT_RADIUS = 10;// INCH from robot center in front of chassis to the center
    public static double VISION_MID = 17;//Encourages sample being closer to vision mid when sorting
    public static double X_WEIGHT = 0.5;//Penalizes sorting if sample is on the left, encourages sample on the right after being close to mid

    public static double Y_WEIGHT = 2;

    public static double CONFIDENCE_THRESHOLD = 50;
    public static double SUB_CENTER_X = 72;//INCH
    public static double SUB_CENTER_Y = -20;//INCH
    public static double subCenterXOffset = 0;//INCH
    public static double subCenterYOffset = 0;//INCH
    public double robotX;
    public double robotY;
    //ROBOT X and Y values(from localization

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
        pixelPoints[0] = new Point(1047.0, 99.0);//top right
        pixelPoints[1] = new Point(214.0, 86.0);//top left
        pixelPoints[2] = new Point(1175.5, 530.5);//bottom right
        pixelPoints[3] = new Point(60.5, 533.5);//bottom left
//        pixelPoints[4] = new Point(366.5, 342.0);//center left
//        pixelPoints[5] = new Point(805.5, 354.5);//center right

//        //CM
        worldPoints[0] = new Point( -1* 2.54 + WINDOW_HORIZONTAL_OFFSET, 18 * 2.54 + WINDOW_VERTICAL_OFFSET - WINDOW_SQUISH_OFFSET);//top right
        worldPoints[1] = new Point( -16* 2.54 + WINDOW_HORIZONTAL_OFFSET, 18 * 2.54 + WINDOW_VERTICAL_OFFSET - WINDOW_SQUISH_OFFSET);//top left
        worldPoints[2] = new Point(-1 * 2.54 + WINDOW_HORIZONTAL_OFFSET, 8 * 2.54 + WINDOW_VERTICAL_OFFSET); //bottom right
        worldPoints[3] = new Point(-16 * 2.54 + WINDOW_HORIZONTAL_OFFSET,8 * 2.54 + WINDOW_VERTICAL_OFFSET);//bottom left
//        worldPoints[4] = new Point(-3 * 2.54 + WINDOW_HORIZONTAL_OFFSET, 14 * 2.54 + WINDOW_VERTICAL_OFFSET);//center left
//        worldPoints[5] = new Point(3 * 2.54 + WINDOW_HORIZONTAL_OFFSET, 14 * 2.54 + WINDOW_VERTICAL_OFFSET);//center right

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

    //pass in the current localization pose of the robot
    public void update(Pose2d robotPose){//robot theta = theta value assuming 0 = x axis and counterclockwise

        LLStatus status = limelight.getStatus();
        limelight.captureSnapshot("Time:" + gameTimer.updateTime());
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());


        //reupdate samples list
        samples = new ArrayList<>();
        timer.updateTime();
        double deltaTime = timer.getDeltaTime();

        //Process result of pipeline
        LLResult llResult = limelight.getLatestResult();
        List<DetectorResult> detectorResults = llResult.getDetectorResults();
        double sampleCount = 0;

        for(DetectorResult result : detectorResults){
            //ignore samples that are below our confidence threshold(lighting)
            if(result.getConfidence() < CONFIDENCE_THRESHOLD){
                continue;
            }

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


            //Center of sample -> transform to world coordinates
            double pixelX = result.getTargetXPixels();
            double pixelY = result.getTargetYPixels();
//
            Mat pointMat = new Mat(1, 1, CvType.CV_64FC2);//Make a point map with 1 row 1 col, 64 bit/double 2 channels(x, y)
            pointMat.put(0, 0, new double[]{pixelX,pixelY});//add the point
            Mat resultMat = new Mat();
            Core.perspectiveTransform(pointMat, resultMat, transformMatrix);//applied transformation
            double[] sampleRelCoord = resultMat.get(0,0);

            sampleRelCoord = new double[]{sampleRelCoord[0]/2.54, sampleRelCoord[1]/2.54};

            //All units here are inches, plug in degrees in radians
            sampleRelCoord = VisionCoordTransformation.transformSampleCoordFromRobotCenter(sampleRelCoord, ROBOT_RADIUS);

            double[] sampleWorldCoord = VisionCoordTransformation.transformRelativeCoordToWorld(sampleRelCoord, robotPose);

            //TODO: Plug in sample coordinates and robot coordinates into NEW IKR to produce minimalistic IKR as possible
//            InverseKinematics.IKResult IKRResult = InverseKinematics.solve(new Pose2d(sampleRelCoord[0], sampleRelCoord[1], sampleRotationValue));
//
//            telemetry.addData("Is Position Reachable", IKRResult.isReachable);
//
//            //Only the far bounds/side bounds should not be reachable
//            if (IKRResult.isReachable) {
//                telemetry.addData("Robot position", IKRResult.robotPose.position);
//                telemetry.addData("Robot heading", Math.toDegrees(IKRResult.robotPose.heading.toDouble()));
//                Sample sample = new Sample(sampleColor, sampleRelCoord, sampleWorldCoord, rot, confidence, IKRResult);
//                samples.add(sample);
//
//            }
//            else{
//                telemetry.addData("Message", IKRResult.message);
//            }


//            //Autonomous: have the entire space but want to sort by weights closest to the center

            telemetry.addLine("----------------------------");
            telemetry.addLine("Sample " + sampleCount);
            telemetry.addData("Update Rate(Hz)", 1/deltaTime);

            telemetry.addLine("PixelCoord: { " + pixelX + ", " + pixelY + "} ");
            telemetry.addLine("RelativeCoord(Relative to Center of Robot): { " + sampleRelCoord[0] + ", " + sampleRelCoord[1] + "} ");
            telemetry.addLine("WorldCoord: { " + sampleWorldCoord[0] + ", " + sampleWorldCoord[1] + "} ");
            telemetry.addData("Color", sampleType.toString());
//            telemetry.addData("Hor Distance", horizontalDistance);
//            telemetry.addData("Vert Distance", verticalDistance);

            telemetry.addData("Confidence", confidence);
            telemetry.addData("CameraXAngle", xAngle);
            telemetry.addData("CameraYAngle", yAngle);
//            telemetry.addData("CameraArea", area);

            //sort samples array

            //Free up memory
            pointMat.release();
            resultMat.release();
        }

        //SORTING SAMPLES
        //prioritizes samples close to the center and to the right, lower the value the more higher priority
        if(!samples.isEmpty()){
            samples.sort((s1, s2)->{
                if((Math.abs(s1.getRelativeY() - VISION_MID) * Y_WEIGHT - s1.getRelativeX() * X_WEIGHT) <
                        (Math.abs(s2.getRelativeY() - VISION_MID) * Y_WEIGHT - s2.getRelativeX() * X_WEIGHT)){
                    return -1;
                }
                else if((Math.abs(s1.getRelativeY() - VISION_MID) * Y_WEIGHT - s1.getRelativeX() * X_WEIGHT) >
                        (Math.abs(s2.getRelativeY() - VISION_MID) * Y_WEIGHT - s2.getRelativeX() * X_WEIGHT)){
                    return 1;
                }
                else{
                    return Double.compare(s1.getRelativeX(), s2.getRelativeX());
                }
            });
        }

        telemetry.addData("Samples List:", samples.toString());

    }

    //Get Closest Sample of a given color to Center of camera's fov
    public Sample getClosestSample(SampleColors sampleColors, double bufferTime){//bufferTime in seconds
        if(sampleList.isEmpty()){
            return null;
        }
        double startTime = gameTimer.updateTime();
        while(timer.updateTime() < startTime + bufferTime) {
            limelight.captureSnapshot("Time:" + gameTimer.updateTime());
            for (Sample sample : samples) {
                if (sample.getColor() == sampleColors) {
                    return sample;
                }
            }
        }
        return null;
    }




    public ArrayList<Sample> getSortedSamples(){
        return samples;
    }

    public ArrayList<Sample> getSortedSamplesOfColor(SampleColors sampleColors, double bufferTime){
        ArrayList<Sample> sampleListOfColor = new ArrayList<>();

        if(sampleList.isEmpty()){
            return null;
        }
        double starTime = gameTimer.updateTime();
        while(timer.updateTime() < starTime + bufferTime){
            limelight.captureSnapshot("Time:" + gameTimer.updateTime());
            for(Sample sample: samples){
                if(sample.getColor() == sampleColors){
                    sampleListOfColor.add(sample);
                }
            }
        }
        return sampleListOfColor;
    }
    public void clearScreenshots(){
        limelight.deleteSnapshots();
        telemetry.addLine("Cleared all screenshots");
    }
}

