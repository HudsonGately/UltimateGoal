package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UGDetector2;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;

public class Vision extends SubsystemBase {
    private Telemetry telemetry;
    private UGDetector2 ringDetector;
    private UGDetector2.Stack currentStack;

    public Vision(HardwareMap hw, String webcamName, Telemetry tl) {
        ringDetector = new UGDetector2(hw, webcamName, tl);
        ringDetector.init();

        telemetry = tl;
        currentStack = ringDetector.getStack();
        ringDetector.setBottomRectangle(.8, .5);
        ringDetector.setTopRectangle(.2, .5);

    }

    @Override
    public void periodic() {
        currentStack = ringDetector.getStack();

        Util.logger(this, telemetry, Level.INFO, "Current Stack", currentStack);
        Util.logger(this, telemetry, Level.INFO, "Bottom", ringDetector.getBottomAverage());
        Util.logger(this, telemetry, Level.INFO, "Top", ringDetector.getTopAverage());

    }

    public UGDetector2.Stack getCurrentStack() {
        return currentStack;
    }
}
