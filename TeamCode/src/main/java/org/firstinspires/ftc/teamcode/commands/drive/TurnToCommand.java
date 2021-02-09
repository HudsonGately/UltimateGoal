package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.logging.Level;

public class TurnToCommand extends CommandBase {

    private final Drivetrain drive;
    private final double angle;
    double desired, firstAngle;
    Telemetry tl;
    public TurnToCommand(Drivetrain drive, double angle, Telemetry tl) {
        this.drive = drive;
        this.angle = angle;
        this.tl = tl;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        double firstAngle = drive.getHeading();
        if (firstAngle > 180) firstAngle = firstAngle - 360;
        desired = angle - firstAngle;

        drive.turn(Math.toRadians(desired));
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

