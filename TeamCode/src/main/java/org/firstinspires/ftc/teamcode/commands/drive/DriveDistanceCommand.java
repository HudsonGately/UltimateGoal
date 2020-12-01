package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PDController;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriveDistanceCommand extends CommandBase {

    private Drivetrain drive;

    private PDController controlLoop;
    private double distance;
    private double speed;

    /**
     * Creates a new DriveDistance.
     *
     * @param inches The number of inches the robot will drive
     * @param speed The speed at which the robot will drive
     * @param drive The drive subsystem on which this command will run
     */
    public DriveDistanceCommand(Drivetrain drive, double inches, double speed) {
        this.distance = inches;
        this.speed = speed;
        this.drive = drive;
        this.addRequirements(drive);

        controlLoop = new PDController(Constants.DRIVE_STRAIGHT_P, Constants.DRIVE_STRAIGHT_D,
                inches, 0);
    }

    @Override
    public void initialize() {
        drive.resetEncoders();

        controlLoop.setTolerance(1);
        controlLoop.calculate(0);

    }

    @Override
    public void execute() {

        double output = controlLoop.calculate(getAverageDistance());

        if (Math.abs(output) > speed) {
            output = Math.signum(output) * speed;
        }
        drive.arcadeDrive(output, 0);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getAverageDistance()) >= distance;
    }

    private double getAverageDistance() {

        return drive.getRightDistance() / 2;
    }
}

