package org.firstinspires.ftc.teamcode.redautos;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.teamcode.commands.PlaceWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.drive.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

public class RedFourHGCommand extends SequentialCommandGroup {

    public RedFourHGCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Telemetry telemetry) {
        addCommands(
                // Setup
                new InstantCommand(wobbleGoalArm::setTurretMiddle),
                new InstantCommand(wobbleGoalArm::closeClaw),
                new InstantCommand(feeder::retractFeed),
                // Spin up wheels
                new InstantCommand(() -> shooterWheels.setShooterRPM(2950), shooterWheels),
                // Drive to Spot
                new ParallelCommandGroup(new DriveForwardCommand(drivetrain, -60),
                        new WaitCommand(200).andThen(new InstantCommand(wobbleGoalArm::midWobbleGoal, wobbleGoalArm))),
                new TurnToCommand(drivetrain, 177),
                // Shoot rings
                new FeedRingsCommand(feeder, 3),
                // Shoot powershots
                new InstantCommand(() -> shooterWheels.setShooterRPM(2850), shooterWheels),
                new TurnToCommand(drivetrain, 180),
                new TurnCommand(drivetrain, 40),

                // Shoot Powershots
                new InstantCommand(intake::intake, intake),
                new DriveForwardCommand(drivetrain, 30, Trajectories.slowConstraint),
                new TurnCommand(drivetrain, -15),
                new FeedRingsCommand(feeder, 1),
                new TurnCommand(drivetrain, 40),

                // Shoot HG
                new DriveForwardCommand(drivetrain, 11, Trajectories.slowConstraint),
                new TurnToCommand(drivetrain, 195),
                new InstantCommand(() -> shooterWheels.setShooterRPM(2850), shooterWheels),
                new FeedRingsCommand(feeder, 3),

                new TurnToCommand(drivetrain, 180),
                new InstantCommand(shooterWheels::stopShooter, shooterWheels),
                new InstantCommand(intake::stop, intake),

                // Go to FourSquare
                new InstantCommand(wobbleGoalArm::setTurretRight, wobbleGoalArm),
                new SplineCommand(drivetrain, new Vector2d(65, -25.5), 0, true),
                new TurnToCommand(drivetrain, 180, false),
                new PlaceWobbleGoal(wobbleGoalArm),


                // Setup arm for 2nd Wobble Goal
                new InstantCommand(wobbleGoalArm::setTurretMiddle, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::placeWobbleGoal, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::openClaw, wobbleGoalArm),
                // Grab 2nd Wobble
                new SplineCommand(drivetrain, new Vector2d(-30, -26), Math.toRadians(180)),
                new InstantCommand(wobbleGoalArm::closeClaw, wobbleGoalArm),
                new WaitCommand(500),
                new InstantCommand(wobbleGoalArm::midWobbleGoal, wobbleGoalArm),

                // Place 2nd Wobble Goal
                new SplineCommand(drivetrain, new Vector2d(67, -20), 0, true),
                new TurnCommand(drivetrain, 90),
                new InstantCommand(wobbleGoalArm::placeWobbleGoal, wobbleGoalArm),
                new WaitCommand(300),
                new InstantCommand(wobbleGoalArm::openClaw, wobbleGoalArm),
                new SplineCommand(drivetrain, new Vector2d(15, 0), Math.toRadians(180), true)


                );
    }
}
