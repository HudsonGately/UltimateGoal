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
        camera.setPipeline(ftclibPipeline = new UGRectRingPipeline());
    }

    public void init() {
        //This will instantiate an OpenCvCamera object for the camera we'll be using
        //Set the pipeline the camera should use and start streaming
        tl.addData("Camera1", camera);
        camera.openCameraDeviceAsync(() -> {
            /*
             * Tell the webcam to start streaming images to us! Note that you must make sure
             * the resolution you specify is supported by the camera. If it is not, an exception
             * will be thrown.
             *
             * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
             * supports streaming from the webcam in the uncompressed YUV image format. This means
             * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
             * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
             *
             * Also, we specify the rotation that the webcam is used in. This is so that the image
             * from the camera sensor can be rotated such that it is always displayed with the image upright.
             * For a front facing camera, rotation is defined assuming the user is looking at the screen.
             * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
             * away from the user.
             */
            camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        });
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
