package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pipelines.HighGoalDetector;
import org.firstinspires.ftc.teamcode.pipelines.UGBasicHighGoalPipeline;
import org.firstinspires.ftc.teamcode.pipelines.UGDetector2;
import org.firstinspires.ftc.teamcode.Util;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.logging.Level;
@Config
public class VisionStack extends SubsystemBase {

    public static double TOP_PERCENT = 0.39;
    public static double BOTTOM_PERCENT = 0.52;
    public static double WIDTH_PERCENT = 0.9;
    private Telemetry telemetry;
    private UGDetector2 ringDetector;
    private UGDetector2.Stack currentStack;


    public VisionStack(HardwareMap hw, String ringWebcam, Telemetry tl, double top, double bottom, double width) {
        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally
        ringDetector = new UGDetector2(hw, ringWebcam, tl, viewportContainerIds[0]);
        ringDetector.init();
        telemetry = tl;

        currentStack = ringDetector.getStack();
        ringDetector.setBottomRectangle(bottom, width);
        ringDetector.setTopRectangle(top, width);

        ringDetector.setRectangleSize(5, 5);

    }

    public void setBottomPercent(double bottomPercent, double width) {
        ringDetector.setBottomRectangle(bottomPercent, width);
    }

    public void setTopPercent(double topPercent, double width) {
        ringDetector.setBottomRectangle(topPercent, width);
    }

    @Override
    public void periodic() {
        try {
            currentStack = ringDetector.getStack();

            Util.logger(this, Level.INFO, "Current Stack", currentStack);
            Util.logger(this, Level.INFO, "Bottom", ringDetector.getBottomAverage());
            Util.logger(this, Level.INFO, "Top", ringDetector.getTopAverage());
        } catch(Exception e) {
            telemetry.addData("Ring Camera:", "Note yet online");
        }

    }

    public double getTopAverage() {
        return ringDetector.getTopAverage();
    }
    public double getBottomAverage() {
        return ringDetector.getTopAverage();
    }
    public UGDetector2.Stack getCurrentStack() {
        return ringDetector.getStack();
    }
}
