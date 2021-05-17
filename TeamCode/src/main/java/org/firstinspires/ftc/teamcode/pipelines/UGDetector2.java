package org.firstinspires.ftc.teamcode.pipelines;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.vision.UGRectRingPipeline;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
@Config
public class UGDetector2 {

    public static double MIN = 105;
    public static double MAX = 110;
    private OpenCvCamera camera;
    private boolean isUsingWebcam;
    private String webcamName;
    private HardwareMap hardwareMap;
    private UGRectRingPipeline ftclibPipeline;
    private Telemetry tl;

    //The constructor is overloaded to allow the use of webcam instead of the phone camera
    public UGDetector2(HardwareMap hMap, Telemetry tl) {
        hardwareMap = hMap;
        this.tl = tl;
    }

    public UGDetector2(HardwareMap hMap, String webcamName, Telemetry tl, int viewId) {
        hardwareMap = hMap;
        isUsingWebcam = true;
        this.webcamName = webcamName;
        this.tl = tl;

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), viewId);
        camera.openCameraDevice();
        camera.setPipeline(ftclibPipeline = new UGRectRingPipeline());
    }

    public void init() {
        //This will instantiate an OpenCvCamera object for the camera we'll be using
        //Set the pipeline the camera should use and start streaming
        tl.addData("Camera1", camera);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }


    public void setTopRectangle(double topRectHeightPercentage, double topRectWidthPercentage) {
        ftclibPipeline.setTopRectHeightPercentage(topRectHeightPercentage);
        ftclibPipeline.setTopRectWidthPercentage(topRectWidthPercentage);
    }

    public void setBottomRectangle(double bottomRectHeightPercentage, double bottomRectWidthPercentage) {
        ftclibPipeline.setBottomRectHeightPercentage(bottomRectHeightPercentage);
        ftclibPipeline.setBottomRectWidthPercentage(bottomRectWidthPercentage);
    }

    public void setRectangleSize(int rectangleWidth, int rectangleHeight){
        ftclibPipeline.setRectangleHeight(rectangleHeight);
        ftclibPipeline.setRectangleWidth(rectangleWidth);
    }

    public Stack getStack() {
        if (Math.abs(ftclibPipeline.getTopAverage() - ftclibPipeline.getBottomAverage()) < ftclibPipeline.getThreshold() && (ftclibPipeline.getTopAverage() <= MAX && ftclibPipeline.getBottomAverage() <= MAX)) {
            return Stack.FOUR;
        } else if (Math.abs(ftclibPipeline.getTopAverage() - ftclibPipeline.getBottomAverage()) < ftclibPipeline.getThreshold() && (ftclibPipeline.getTopAverage() >= MAX && ftclibPipeline.getBottomAverage() >= MAX)) {
            return Stack.ZERO;
        } else {
            return Stack.ONE;
        }
    }

    public void setThreshold(int threshold) {
        ftclibPipeline.setThreshold(threshold);
    }

    public double getTopAverage() {
        return ftclibPipeline.getTopAverage();
    }

    public double getBottomAverage() {
        return ftclibPipeline.getBottomAverage();
    }

    public enum Stack {
        ZERO,
        ONE,
        FOUR,
    }

}
