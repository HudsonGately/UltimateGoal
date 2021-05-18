package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.pipelines.HighGoalDetector;
import org.firstinspires.ftc.teamcode.pipelines.UGBasicHighGoalPipeline;
import org.firstinspires.ftc.teamcode.pipelines.UGDetector2;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.logging.Level;

@Config
public class VisionHG extends SubsystemBase {

    private Telemetry telemetry;

    private HighGoalDetector goalDetector;
    private UGBasicHighGoalPipeline.Mode color;

    public VisionHG(HardwareMap hw, String goalWebcam, Telemetry tl,  UGBasicHighGoalPipeline.Mode color) {
        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally
        goalDetector = new HighGoalDetector(hw, goalWebcam, tl, color, viewportContainerIds[1]);
        goalDetector.init();
        this.color = color;
        telemetry = tl;

        goalDetector.getTargetAngle();
    }

    @Override
    public void periodic() {
        try {
            Util.logger(this, Level.INFO, "Goal yaw (0 if not visible)", goalDetector.getTargetAngle());
            Util.logger(this, Level.INFO, "Goal pitch (0 if not visible)", goalDetector.getTargetPitch());
        } catch(Exception e) {
            telemetry.addData("Goal Camera:", "Note yet online");
        }
    }
    public void setOffset(int offset) {
        goalDetector.setOffset(offset);
    }
    public double getHighGoalAngle() {
        return goalDetector.getTargetAngle(); }
    public boolean isTargetVisible() {
        return goalDetector.isTargetVisible();
    }
}
