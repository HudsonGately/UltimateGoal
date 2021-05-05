package org.firstinspires.ftc.teamcode.redautos;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.HomeWobbleArm;
import org.firstinspires.ftc.teamcode.commands.PlaceWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.drive.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

public class RedOneHGCommand extends SequentialCommandGroup {

    public RedOneHGCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Telemetry telemetry) {
        addCommands(
                // Setup
                new InstantCommand(wobbleGoalArm::setTurretMiddle),
                new InstantCommand(wobbleGoalArm::closeClaw),
                new InstantCommand(feeder::retractFeed),
                new HomeWobbleArm(wobbleGoalArm),
                // Spin up wheels
                new InstantCommand(() -> shooterWheels.setShooterRPM(2950), shooterWheels),
                // Drive to Spot
                new ParallelCommandGroup(new DriveForwardCommand(drivetrain, -60),
                        new WaitCommand(200).andThen(new InstantCommand(wobbleGoalArm::midWobbleGoal, wobbleGoalArm))),
                new TurnToCommand(drivetrain, 177),
                // Shoot rings
                new FeedRingsCommand(feeder, 3),
                new InstantCommand(shooterWheels::stopShooter),
                new TurnToCommand(drivetrain, 180),
                // Go to OneSquare
                new InstantCommand(wobbleGoalArm::setTurretRight, wobbleGoalArm),
                new DriveForwardCommand(drivetrain, -45),
                new PlaceWobbleGoal(wobbleGoalArm),
                // Setup arm for 2nd Wobble Goal
                new InstantCommand(wobbleGoalArm::setTurretMiddle, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::placeWobbleGoal, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::openClaw, wobbleGoalArm),
                // Go to 2nd Wobble Goal while intaking
                new InstantCommand(intake::intake, intake),
                new SplineCommand(drivetrain, new Vector2d(-33, -34), Math.toRadians(220)),
                // Grab wobble goal
                new InstantCommand(intake::stop, intake),
                // Start shooter
                new InstantCommand(() -> shooterWheels.setShooterRPM(2800), shooterWheels),
                new WaitCommand(800),
                new InstantCommand(wobbleGoalArm::closeClaw, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::liftWobbleGoal, wobbleGoalArm),
                new ShootRingsCommand(shooterWheels, feeder, 2800, 3),
                // Go to 1 Ring square
                new DriveForwardCommand(drivetrain, -50),
                new TurnCommand(drivetrain, 165),
                new PlaceWobbleGoal(wobbleGoalArm),
                new DriveForwardCommand(drivetrain, -5)









                );
    }
}
