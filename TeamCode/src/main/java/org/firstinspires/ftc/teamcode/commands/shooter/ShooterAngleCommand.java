package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PDController;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterAngler;

public class ShooterAngleCommand extends CommandBase {
    private ShooterAngler shooterAngler;
    private double angle, maxSpeed;

    private PDController controlLoop;

    public ShooterAngleCommand(ShooterAngler shooterAngler, double angle, double maxSpeed) {
        this.shooterAngler = shooterAngler;
        this.angle = angle;
        this.maxSpeed = maxSpeed;

        this.addRequirements(shooterAngler);

        controlLoop = new PDController(Constants.ANGLER_P, Constants.ANGLER_D, angle, shooterAngler.getShooterAngle());
    }

    @Override
    public void initialize() {
        controlLoop.setTolerance(1);
        controlLoop.calculate(shooterAngler.getShooterAngle());
    }

    @Override
    public void execute() {
        double output = controlLoop.calculate(shooterAngler.getShooterAngle());

        if (Math.abs(output) > maxSpeed) {
            output = Math.signum(output) * maxSpeed;
        }
        if (controlLoop.atSetPoint()) {
            output = 0;
        }
        shooterAngler.setAngler(output);
    }

    @Override
    public void end(boolean interrupted) {
        shooterAngler.stopAngler();
    }
}
