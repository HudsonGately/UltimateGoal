package org.firstinspires.ftc.teamcode.redautos;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.apache.commons.math3.stat.descriptive.moment.VectorialCovariance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Trajectories;
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
                new TurnToCommand(drivetrain, 177, telemetry),
                // Shoot rings
                new FeedRingsCommand(feeder, 3),
                // Shoot powershots
                new InstantCommand(() -> shooterWheels.setShooterRPM(2850), shooterWheels),
                new TurnToCommand(drivetrain, 180, telemetry),
                new TurnCommand(drivetrain, 45),
                new InstantCommand(intake::intake, intake),
                new DriveForwardCommand(drivetrain, 30, Trajectories.slowConstraint),
                new TurnCommand(drivetrain, -19),
                new WaitCommand(500),
                new FeedRingsCommand(feeder, 2),

                new DriveForwardCommand(drivetrain, 12),
                new InstantCommand(intake::stop),
                new InstantCommand(() -> shooterWheels.setShooterRPM(2900), shooterWheels),

                new TurnToCommand(drivetrain, 194, false, telemetry),
                new InstantCommand(intake::intake, intake),
                new WaitCommand(300),
                new FeedRingsCommand(feeder, 5),
                new InstantCommand(intake::stop),
                new TurnToCommand(drivetrain, 180, telemetry),
                new InstantCommand(shooterWheels::stopShooter),


                // Go to FourSquare
                new InstantCommand(wobbleGoalArm::setTurretRight, wobbleGoalArm),
                new SplineCommand(drivetrain, new Vector2d(60, -26), 0, true),
                new TurnToCommand(drivetrain, 180, false, telemetry),
                new PlaceWobbleGoal(wobbleGoalArm),


                // Setup arm for 2nd Wobble Goal
                new InstantCommand(wobbleGoalArm::setTurretMiddle, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::placeWobbleGoal, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::openClaw, wobbleGoalArm),
                // Grab 2nd Wobble
                new SplineCommand(drivetrain, new Vector2d(-33, -27), Math.toRadians(180)),
                new InstantCommand(wobbleGoalArm::closeClaw, wobbleGoalArm),
                new WaitCommand(300),
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
