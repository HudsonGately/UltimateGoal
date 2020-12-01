package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PDController;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriveTurnCommand extends CommandBase {

    private Drivetrain drive;

    private double angle;
    private double speed;
    private double error;

    /**
     * Creates a new DriveDistance.
     *
     * @param speed The speed at which the robot will drive
     * @param drive The drive subsystem on which this command will run
     */
    public DriveTurnCommand(Drivetrain drive, double angle, double speed) {
        this.angle = angle;
        this.speed = speed;
        this.drive = drive;
        this.addRequirements(drive);

    }

    @Override
    public void initialize() {
        drive.resetAngle();
    }

    @Override
    public void execute() {
        error = Util.getModulusError(angle, drive.getAngle(), -180, 180);
        double output = error * Constants.DRIVE_GYRO_P;

        if (Math.abs(output) > speed) {
            output = Math.signum(output) * speed;
        }
        drive.arcadeDrive(0, output);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(error) <= 1.5;
    }

}

