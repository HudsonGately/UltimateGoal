package org.firstinspires.ftc.teamcode.inperson;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.pipelines.UGBasicHighGoalPipeline;
import org.firstinspires.ftc.teamcode.subsystems.VisionStack;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.logging.Level;

@TeleOp(name = "Starterstack Vision Test")
public class RingDetectorOpMode extends MatchOpMode {
    public static double startPoseX = -62.5;
    public static double startPoseY = 0;
    public static double startPoseHeading = 180;
    public double width = .02;
    public double topHeight = 0.38;
    public double bottomHeight = 0.56;
    private OpenCvCamera camera;
    GamepadEx driverGamepad;
    VisionStack visionStack;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        //This will instantiate an OpenCvCamera object for the camera we'll be using

        visionStack = new VisionStack(hardwareMap, "webcam",  telemetry, topHeight, bottomHeight, width);

    }

    @Override
    public void configureButtons() {
        (new GamepadButton(driverGamepad, GamepadKeys.Button.Y)).whenPressed(() -> {
            topHeight -= 0.02;
            visionStack.setTopPercent(topHeight, width);
        });
        (new GamepadButton(driverGamepad, GamepadKeys.Button.X)).whenPressed(() -> {
            topHeight += 0.02;
            visionStack.setTopPercent(topHeight, width);
        });

        (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)).whenPressed(() -> {
            bottomHeight -= 0.02;
            visionStack.setTopPercent(bottomHeight, width);
        });
        (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)).whenPressed(() -> {
            bottomHeight += 0.02;
            visionStack.setTopPercent(bottomHeight, width);
        });

        (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT)).whenPressed(() -> {
            width -= 0.01;
            visionStack.setTopPercent(topHeight, width);
            visionStack.setBottomPercent(bottomHeight, width);
        });

        (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT)).whenPressed(() -> {
            width += 0.01;
            visionStack.setTopPercent(topHeight, width);
            visionStack.setBottomPercent(bottomHeight, width);
        });

        (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)).whenPressed(() -> {
            width = 0.02;
            visionStack.setTopPercent(topHeight, width);
            visionStack.setBottomPercent(bottomHeight, width);
        });

        (new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)).whenPressed(() -> {
            width = 0.92;
            visionStack.setTopPercent(topHeight, width);
            visionStack.setBottomPercent(bottomHeight, width);
        });
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void robotPeriodic() {
        Util.logger(this, Level.INFO, "Top Y Height", topHeight);
        Util.logger(this, Level.INFO, "bottom Y Height", bottomHeight);
        Util.logger(this, Level.INFO, "width", width);
        Util.logger(this, Level.INFO, "num rings", visionStack.getCurrentStack());
    }

    @Override
    public void matchStart() {


    }
}
