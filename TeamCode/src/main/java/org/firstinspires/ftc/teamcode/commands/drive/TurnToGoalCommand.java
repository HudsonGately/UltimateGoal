package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Config
public class TurnToGoalCommand extends CommandBase {
    private Vision vision;
    private Drivetrain drivetrain;

    private PIDController turningController;
    private double possibleRange;

    public static double VISION_P = 0.0001;
    public static double VISION_I = 0.0000001;
    public static double VISION_D = 0.00001;
    public static double MAX_SPEED = 0.3;
    public static double TOLERANCE = 1;

    public TurnToGoalCommand(Drivetrain drivetrain, Vision vision, double range) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.possibleRange = range;

        turningController = new PIDController(VISION_P, VISION_I, VISION_D);
    }

    @Override
    public void execute() {
        double output = turningController.calculate(vision.getHighGoalAngle(), 0);
        if (output < -MAX_SPEED)
            output = -MAX_SPEED;
        else if (output > MAX_SPEED)
            output = MAX_SPEED;

        drivetrain.arcadeDrive(0, output);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(vision.getHighGoalAngle()) > possibleRange || Math.abs(vision.getHighGoalAngle()) < TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
