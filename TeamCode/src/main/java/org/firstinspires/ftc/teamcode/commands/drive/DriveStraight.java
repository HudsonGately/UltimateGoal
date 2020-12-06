package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriveStraight extends CommandBase {
    Drivetrain drivetrain;
    private double distance;
    public DriveStraight(Drivetrain drivetrain, double distance) {
        this.drivetrain = drivetrain;
        this.distance = distance;
        this.addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetEncoders();
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(0.8 * Math.signum(distance), 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getAverageDistance()) >= Math.abs(distance);
    }

    private double getAverageDistance() {
       return (drivetrain.getLeftDistance() + drivetrain.getRightDistance()) / 2.0;
    }
}
