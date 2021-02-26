package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Config
public class DriveForwardCommand extends CommandBase{

    Drivetrain drive;
    double distance;
    Trajectory trajectory;
    public DriveForwardCommand(Drivetrain drive, double distance) {
        this.drive = drive;
        this.distance = distance;

        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        if (distance < 0)
            trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), Trajectories.slowConstraint, Trajectories.accelConstraint).back(-distance).build();
        else
            trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), Trajectories.slowConstraint, Trajectories.accelConstraint).forward(distance).build();

        drive.followTrajectory(trajectory);

    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }
}
