package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class TurnToCommand extends CommandBase {

    private final Drivetrain drive;
    private final double angle;

    public TurnToCommand(Drivetrain drive, double angle) {
        this.drive = drive;
        this.angle = angle;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.turnTo(Math.toRadians(angle));
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}

