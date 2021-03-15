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
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

public class RedFourHGAltCommand extends SequentialCommandGroup {

    public RedFourHGAltCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Telemetry telemetry) {
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
                new TurnToCommand(drivetrain, 177, telemetry),
                // Shoot rings
                new FeedRingsCommand(feeder, 3),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels),
                new TurnToCommand(drivetrain, 180, telemetry),

                // Place 1st Wobble Goal
                new InstantCommand(wobbleGoalArm::setTurretRight, wobbleGoalArm),
                new SplineCommand(drivetrain, new Vector2d(60, -25.5), 0, true),
                new TurnToCommand(drivetrain, 180, false, telemetry),
                new PlaceWobbleGoal(wobbleGoalArm),

                // Setup arm for 2nd Wobble Goal
                new InstantCommand(wobbleGoalArm::setTurretMiddle, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::placeWobbleGoal, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::openClaw, wobbleGoalArm),
                // Grab 2nd Wobble
                new SplineCommand(drivetrain, new Vector2d(-32, -30), Math.toRadians(180)),
                new InstantCommand(wobbleGoalArm::closeClaw, wobbleGoalArm),
                new WaitCommand(500),
                new InstantCommand(wobbleGoalArm::midWobbleGoal, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::setTurretFarRight, wobbleGoalArm),

                // Place 2nd kkkblk Goalk
                new SplineCommand(drivetrain, new Vector2d(64, -20), 0, true),
                new InstantCommand(wobbleGoalArm::placeWobbleGoal, wobbleGoalArm),
                new WaitCommand(300),
                new InstantCommand(wobbleGoalArm::openClaw, wobbleGoalArm),
                new InstantCommand(intake::intake, intake),
                new InstantCommand(() -> shooterWheels.setShooterRPM(2900), shooterWheels),
                new SplineCommand(drivetrain, new Vector2d(-18, -12), Math.toRadians(180)),
                new InstantCommand(wobbleGoalArm::setTurretMiddle, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::liftWobbleGoal, wobbleGoalArm),
                // HGkkkkk
                new TurnToCommand(drivetrain, 188, telemetry),
                new FeedRingsCommand(feeder, 2),
                new TurnToCommand(drivetrain, 170, telemetry),
                new DriveForwardCommand(drivetrain, 34),
                new DriveForwardCommand(drivetrain, -34),
                new TurnToCommand(drivetrain, 188, telemetry),
                new FeedRingsCommand(feeder, 6),
                new InstantCommand(intake::stop, intake),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels),
                new DriveForwardCommand(drivetrain, -28)




                );
    }
}
