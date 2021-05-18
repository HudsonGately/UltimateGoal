package org.firstinspires.ftc.teamcode.inperson;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.pipelines.UGBasicHighGoalPipeline;
import org.firstinspires.ftc.teamcode.subsystems.VisionHG;
import org.firstinspires.ftc.teamcode.subsystems.VisionStack;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.logging.Level;

@TeleOp(name = "High Goal (RED) Vision Test")
public class HighDetectorOpMode extends MatchOpMode {
    public static double startPoseX = -62.5;
    public static double startPoseY = 0;
    public static double startPoseHeading = 180;
    public int offset = 0;
    private OpenCvCamera camera;
    GamepadEx driverGamepad;
    VisionHG visionHG;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        //This will instantiate an OpenCvCamera object for the camera we'll be using

        visionHG = new VisionHG(hardwareMap, "webcam1", telemetry, UGBasicHighGoalPipeline.Mode.RED_ONLY);

    }

    @Override
    public void configureButtons() {
        (new GamepadButton(driverGamepad, GamepadKeys.Button.A)).whenPressed(() -> {
            offset -= 5;
            visionHG.setOffset(offset);
        });
        (new GamepadButton(driverGamepad, GamepadKeys.Button.B)).whenPressed(() -> {
            offset += 5;
            visionHG.setOffset(offset);
        });

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void robotPeriodic() {
        Util.logger(this, Level.INFO, "Offset", offset);
    }

    @Override
    public void matchStart() {


    }
}
