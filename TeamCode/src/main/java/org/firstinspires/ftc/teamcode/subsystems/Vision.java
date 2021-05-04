package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pipelines.HighGoalDetector;
import org.firstinspires.ftc.teamcode.pipelines.UGBasicHighGoalPipeline;
import org.firstinspires.ftc.teamcode.pipelines.UGDetector2;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;
@Config
public class Vision extends SubsystemBase {

    public static double TOP_PERCENT = 0.39;
    public static double BOTTOM_PERCENT = 0.52;
    public static double WIDTH_PERCENT = 0.9;
    private Telemetry telemetry;
    private UGDetector2 ringDetector;
    private UGDetector2.Stack currentStack;

    private HighGoalDetector goalDetector;

    public Vision(HardwareMap hw, String ringWebcam, String goalWebcam, Telemetry tl, double top, double bottom, double width, UGBasicHighGoalPipeline.Mode color) {
        ringDetector = new UGDetector2(hw, ringWebcam, tl);
        ringDetector.init();

        goalDetector = new HighGoalDetector(hw, goalWebcam, tl, color);
        goalDetector.init();

        telemetry = tl;

        goalDetector.getTargetAngle();
        currentStack = ringDetector.getStack();
        ringDetector.setBottomRectangle(bottom, width);
        ringDetector.setTopRectangle(top, width);

        ringDetector.setRectangleSize(5, 5);

    }


    @Override
    public void periodic() {
        currentStack = ringDetector.getStack();

        Util.logger(this, telemetry, Level.INFO, "Current Stack", currentStack);
        Util.logger(this, telemetry, Level.INFO, "Bottom", ringDetector.getBottomAverage());
        Util.logger(this, telemetry, Level.INFO, "Top", ringDetector.getTopAverage());
        Util.logger(this, telemetry, Level.INFO, "Goal yaw (0 if not visible)", goalDetector.getTargetAngle());
        Util.logger(this, telemetry, Level.INFO, "Goal pitch (0 if not visible)", goalDetector.getTargetPitch());

    }

    public double getTopAverage() {
        return ringDetector.getTopAverage();
    }

    public double getBottomAverage() {
        return ringDetector.getTopAverage();
    }

    public double getHighGoalAngle() { return goalDetector.getTargetAngle(); }

    public UGDetector2.Stack getCurrentStack() {
        return ringDetector.getStack();
    }
}
