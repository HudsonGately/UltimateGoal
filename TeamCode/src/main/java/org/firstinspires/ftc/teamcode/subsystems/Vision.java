package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.vision.UGRectDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Vision extends SubsystemBase {
    private Telemetry telemetry;
    private UGRectDetector ringDetector;
    private UGRectDetector.Stack currentStack;

    public Vision(HardwareMap hw, String webcamName, Telemetry tl) {
        ringDetector = new UGRectDetector(hw, webcamName);
        ringDetector.init();

        ringDetector.setBottomRectangle(.1, 0.1);
        ringDetector.setTopRectangle(.1, 0.1);
        ringDetector.setRectangleSize(100, 100);
        telemetry = tl;
        currentStack = ringDetector.getStack();
    }

    @Override
    public void periodic() {
        currentStack = ringDetector.getStack();
        telemetry.addData("Current Stack:", currentStack);
    }

    public UGRectDetector.Stack getCurrentStack() {
        return currentStack;
    }
}
