package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.VisionHG;
import org.firstinspires.ftc.teamcode.subsystems.VisionStack;

import java.util.logging.Level;

@Config
public class VisionCommand extends CommandBase {
    private VisionHG vision;
    private Drivetrain drivetrain;

    private PIDController turningController;
    private double possibleRange;

    public static double VISION_P = 0.038;
    public static double VISION_I = 0.000001;
    public static double VISION_D = 0.00001;
    public static double MAX_SPEED = 0.42;
    public static double TOLERANCE = 0.5;

    public VisionCommand(Drivetrain drivetrain, VisionHG vision, double range) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.possibleRange = range;

        turningController = new PIDController(VISION_P, VISION_I, VISION_D);
        turningController.setTolerance(TOLERANCE);
        // f(error) => output
        // f(error) = kP * error (-15 to 15)
        // output (-1 to 1)
        // (-0.5 to 0.5)


    }

    @Override
    public void initialize() {
        Util.logger(this, Level.WARNING, "Target visible", vision.isTargetVisible());
    }

    @Override
    public void execute() {
        double output = turningController.calculate(-vision.getHighGoalAngle(), 0);
        if (output < -MAX_SPEED)
            output = -MAX_SPEED;
        else if (output > MAX_SPEED)
            output = MAX_SPEED;

        drivetrain.arcadeDrive(0, output);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(vision.getHighGoalAngle()) > possibleRange || turningController.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
