package org.firstinspires.ftc.teamcode.inperson.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Vision Test")
public class VisionTest extends MatchOpMode {
    public static double startPoseX = -62.5;
    public static double startPoseY = 0;
    public static double startPoseHeading = 180;
    public static double RED_CAMERA_WIDTH = .02;
    private OpenCvCamera camera;


    @Override
    public void robotInit() {
        //This will instantiate an OpenCvCamera object for the camera we'll be using
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId);
        telemetry.addData("Camera:", camera);
        //Set the pipeline the camera should use and start streaming
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void matchStart() {


    }
}
