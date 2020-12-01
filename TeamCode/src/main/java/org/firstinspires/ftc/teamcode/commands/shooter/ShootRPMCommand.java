package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShootRPMCommand extends CommandBase {
    private Shooter shooter;
    private double rpm, error, feedforward;


    public ShootRPMCommand(Shooter shooter, double rpm) {
        this.shooter = shooter;
        this.rpm = rpm;

        this.addRequirements(shooter);
    }

    @Override
    public void execute() {
        error = rpm - shooter.getShooterRPM();
        feedforward = rpm * Constants.SHOOTER_F;
        double output = error * Constants.SHOOTER_P + feedforward;

        if (Math.abs(output) > 1) {
            output = Math.signum(output);
        }
        shooter.setShooter(output);
    }

    public void end(boolean interrupted) {
        shooter.setShooter(0);
    }
}
