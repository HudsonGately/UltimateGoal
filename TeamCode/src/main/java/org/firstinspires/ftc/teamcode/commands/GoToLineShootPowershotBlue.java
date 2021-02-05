package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.teamcode.commands.drive.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;

import java.util.logging.Logger;

@Config
public class GoToLineShootPowershotBlue extends SequentialCommandGroup {
    private Drivetrain drivetrain;
    private ShooterWheels shooterWheels;
    private ShooterFeeder feeder;

    public static int DELAY_FEED = 75;
    public static int DELAY_SHOT =80;
    public static double TURN_ANGLE = -21;
    public static double TURN_OFFSET = 5;
    public static double SPEED = 2675;

    public GoToLineShootPowershotBlue(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder) {
        addCommands(
                new TrajectoryFollowerCommand(drivetrain, Trajectories.driveToShoot),
                new InstantCommand(() -> shooterWheels.setShooterRPM(SPEED)),
                new TurnCommand(drivetrain, TURN_ANGLE),
                new ShootRingsCommand(shooterWheels, feeder, SPEED, 1),
                new TurnCommand(drivetrain, TURN_OFFSET),
                new ShootRingsCommand(shooterWheels, feeder, SPEED, 1),
                new TurnCommand(drivetrain, TURN_OFFSET),
                new ShootRingsCommand(shooterWheels, feeder, SPEED, 1),
                new TurnCommand(drivetrain, -1 * (TURN_ANGLE + 2 *TURN_OFFSET))

                );

    }
}
