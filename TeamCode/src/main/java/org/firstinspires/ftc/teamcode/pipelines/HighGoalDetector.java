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
    private Telemetry tl;
    private UGBasicHighGoalPipeline.Mode color;

    private UGAngleHighGoalPipeline pipeline;
    public HighGoalDetector(HardwareMap hMap, String webcamName, Telemetry tl, UGBasicHighGoalPipeline.Mode color) {
        hardwareMap = hMap;
        isUsingWebcam = true;
        this.webcamName = webcamName;
        this.tl = tl;
        this.color = color;
    }

    public double getTargetAngle() {
        return pipeline.calculateYaw(color);
    }
    public double getTargetPitch() {
        return pipeline.calculatePitch(color);
    }

    public void init(int viewId) {
        //This will instantiate an OpenCvCamera object for the camera we'll be using
        if (isUsingWebcam) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), viewId);
        } else {
            camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, viewId);
        }

        tl.addData("Camera:", camera);
        //Set the pipeline the camera should use and start streaming
        camera.setPipeline(pipeline = new UGAngleHighGoalPipeline(55, color));
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

}
