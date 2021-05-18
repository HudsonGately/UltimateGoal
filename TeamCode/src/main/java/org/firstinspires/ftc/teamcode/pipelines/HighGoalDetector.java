package org.firstinspires.ftc.teamcode.pipelines;

import com.arcrobotics.ftclib.vision.UGRectRingPipeline;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class HighGoalDetector {
    private OpenCvCamera camera;
    private boolean isUsingWebcam;
    private String webcamName;
    private HardwareMap hardwareMap;
    private UGBasicHighGoalPipeline.Mode color;
    private Telemetry tl;
    private UGAngleHighGoalPipeline pipeline;
    public HighGoalDetector(HardwareMap hMap, String webcamName, Telemetry tl, UGBasicHighGoalPipeline.Mode color, int viewId) {
        hardwareMap = hMap;
        isUsingWebcam = true;
        this.webcamName = webcamName;
        this.tl = tl;
        this.color = color;

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), viewId);
        camera.setPipeline(pipeline = new UGAngleHighGoalPipeline(55, color));

    }

    public double getTargetAngle() {
        return pipeline.calculateYaw(color);
    }
    public double getTargetPitch() {
        return pipeline.calculatePitch(color);
    }
    public boolean isTargetVisible() {
        if (color == UGBasicHighGoalPipeline.Mode.RED_ONLY)
            return pipeline.isRedVisible();
        return pipeline.isBlueVisible();
    }
    public void init() {
        //Set the pipeline the camera should use and start streaming
        tl.addData("Camera", camera);
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
        //camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

}
