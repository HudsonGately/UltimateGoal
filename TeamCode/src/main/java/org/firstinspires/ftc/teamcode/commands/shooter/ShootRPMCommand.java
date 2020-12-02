package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;

public class ShootRPMCommand extends CommandBase {
    private ShooterWheels shooterWheels;
    private double rpm, error, feedforward;


    public ShootRPMCommand(ShooterWheels shooterWheels, double rpm) {
        this.shooterWheels = shooterWheels;
        this.rpm = rpm;

        this.addRequirements(shooterWheels);
    }

    @Override
    public void execute() {
        error = rpm - shooterWheels.getShooterRPM();
        feedforward = rpm * Constants.SHOOTER_F;
        double output = error * Constants.SHOOTER_P + feedforward;

        if (Math.abs(output) > 1) {
            output = Math.signum(output);
        }
        shooterWheels.setShooter(output);
    }

    public void end(boolean interrupted) {
        shooterWheels.setShooter(0);
    }
}
