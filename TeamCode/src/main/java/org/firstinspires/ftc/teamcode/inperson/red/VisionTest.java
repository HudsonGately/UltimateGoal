package org.firstinspires.ftc.teamcode.inperson.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.pipelines.UGDetector2;
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
    UGDetector2 stackDetector;

    @Override
    public void robotInit() {
        //This will instantiate an OpenCvCamera object for the camera we'll be using
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        stackDetector = new UGDetector2(hardwareMap, "webcam", telemetry, cameraMonitorViewId);
        stackDetector.init();
        stackDetector.setBottomRectangle(0.58, .02);
        stackDetector.setTopRectangle(0.45, .02);

        stackDetector.setRectangleSize(5, 5);

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void matchStart() {


    }
}
