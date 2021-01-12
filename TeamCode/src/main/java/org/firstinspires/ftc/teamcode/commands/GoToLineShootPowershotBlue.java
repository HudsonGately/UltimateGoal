package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.drive.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;

public class GoToLineShootPowershotBlue extends SequentialCommandGroup {
    private Drivetrain drivetrain;
    private ShooterWheels shooterWheels;
    private ShooterFeeder feeder;

    public GoToLineShootPowershotBlue(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder) {
        addCommands(
                new TrajectoryFollowerCommand(drivetrain, drivetrain.trajectoryBuilder(new Pose2d()).back(48).build()),
                new TurnCommand(drivetrain, -5.75),
                new ShootRingsCommand(shooterWheels, feeder, 2600, 1),
                new TurnCommand(drivetrain, -3.5),
                new ShootRingsCommand(shooterWheels, feeder, 2750, 0),
                new TurnCommand(drivetrain, -3),
                new ShootRingsCommand(shooterWheels, feeder, 2750, 0),
                new TurnCommand(drivetrain, 10),
                new InstantCommand(() -> drivetrain.setPoseEstimate(new Pose2d()))
                );
    }
}
