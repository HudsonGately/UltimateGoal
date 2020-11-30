package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PDController;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShooterAngleCommand extends CommandBase {
    private Shooter shooter;
    private double angle, maxSpeed;

    private PDController controlLoop;

    public ShooterAngleCommand(Shooter shooter, double angle, double maxSpeed) {
        this.shooter = shooter;
        this.angle = angle;
        this.maxSpeed = maxSpeed;

        this.addRequirements(shooter);

        controlLoop = new PDController(Constants.ANGLER_P, Constants.ANGLER_D, angle, shooter.getShooterAngle());
    }

    @Override
    public void initialize() {
        controlLoop.setTolerance(1);
        controlLoop.calculate(shooter.getShooterAngle());
    }

    @Override
    public void execute() {
        double output = controlLoop.calculate(shooter.getShooterAngle());

        if (Math.abs(output) > maxSpeed) {
            output = Math.signum(output);
        }

        shooter.setAngler(output);
    }

    @Override
    public boolean isFinished() {
        return controlLoop.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAngler();
    }
}
