package org.firstinspires.ftc.teamcode.redautos;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.teamcode.commands.HomeWobbleArm;
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

import static org.firstinspires.ftc.teamcode.Trajectories.BlueCloseTape.highGoalX;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueCloseTape.highGoalY;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueCloseTape.wobbleGoalSquareDistance;

public class RedZeroHGCommand extends SequentialCommandGroup {

    public RedZeroHGCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Telemetry telemetry) {
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
                new TurnToCommand(drivetrain, 177, telemetry),

                // Shoot rings
                new FeedRingsCommand(feeder, 3),
                new InstantCommand(shooterWheels::stopShooter),
                new TurnToCommand(drivetrain, 180, telemetry),

                // Go to ZeroSquare
                new TurnCommand(drivetrain, 110),
                new DriveForwardCommand(drivetrain, 25),
                new PlaceWobbleGoal(wobbleGoalArm),

                // Go to Second Wobble Goal
                new TurnToCommand(drivetrain, 180, telemetry),
                new InstantCommand(wobbleGoalArm::placeWobbleGoal, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::openClaw),
                new SplineCommand(drivetrain, new Vector2d(-33, -24), Math.toRadians(180)),
                new InstantCommand(wobbleGoalArm::closeClaw),
                new WaitCommand(800),
                new InstantCommand(wobbleGoalArm::liftWobbleGoal, wobbleGoalArm),

                // Drop off wobble goal
                new TurnToCommand(drivetrain, 180, telemetry),
                new SplineCommand(drivetrain, new Vector2d(9, -22), Math.toRadians(0), true),
                new TurnCommand(drivetrain, 90),
                new PlaceWobbleGoal(wobbleGoalArm),
                new DriveForwardCommand(drivetrain, -5)
                );
    }
}
