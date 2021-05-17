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
        camera.openCameraDevice();
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
        //camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

}
