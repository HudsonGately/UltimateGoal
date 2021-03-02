package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

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

import static org.firstinspires.ftc.teamcode.Trajectories.BlueCloseTape.highGoalX;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueCloseTape.highGoalY;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueCloseTape.wobbleGoalSquareDistance;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueMid.shootDistance;

public class OneRingCommand extends SequentialCommandGroup {

    public OneRingCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Telemetry telemetry) {
        addCommands(
                new InstantCommand(wobbleGoalArm::setTurretMiddle),
                new InstantCommand(wobbleGoalArm::closeClaw),
                new InstantCommand(() -> shooterWheels.setShooterRPM(2900)),
                new ParallelCommandGroup(new WaitCommand(750).andThen(new InstantCommand(() -> wobbleGoalArm.setWobbleGoal(-90))).andThen(new WaitCommand(500)).andThen(new InstantCommand(wobbleGoalArm::setTurretRight)), new DriveForwardCommand(drivetrain, -shootDistance)),
                // turn and shoot
                new TurnToCommand(drivetrain, 178, telemetry),
                new FeedRingsCommand(feeder, 4, 75),
                new InstantCommand(intake::stop),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0)),
                new TurnToCommand(drivetrain, 180, telemetry),
                //place wobble goal
                new DriveForwardCommand(drivetrain, -Trajectories.BlueMid.wobbleGoalSquareDistance),
                new PlaceWobbleGoal(wobbleGoalArm),
                //go to ring
                new InstantCommand(intake::intake),
                new InstantCommand(() -> shooterWheels.setShooterRPM(2750)),
                new SplineCommand(drivetrain, new Vector2d(Trajectories.BlueMid.ringX, Trajectories.BlueMid.ringY), Math.toRadians(180)),
                //align for powershot
                new TurnToCommand(drivetrain, 180, telemetry),
                new FeedRingsCommand(feeder, 4, 75),
                new InstantCommand(intake::stop),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0)),
                //wobble goal
                new TurnToCommand(drivetrain, Trajectories.BlueMid.wobbleAngle, telemetry),
                new InstantCommand(wobbleGoalArm::openClaw),
                new InstantCommand(wobbleGoalArm::setTurretMiddle),
                // lower arm
                new DriveForwardCommand(drivetrain, -2, Trajectories.slowConstraint),
                new InstantCommand(wobbleGoalArm::placeWobbleGoal, wobbleGoalArm),
                new WaitCommand(2000),
                new DriveForwardCommand(drivetrain, Trajectories.BlueMid.wobbleDistance, Trajectories.slowConstraint),
                new WaitCommand(500),
                // grab wobble gaol
                new InstantCommand(wobbleGoalArm::closeClaw),
                new WaitCommand(1000),
                new InstantCommand(wobbleGoalArm::liftWobbleGoal),
                new ParallelCommandGroup(
                        new SplineCommand(drivetrain, Trajectories.velConstraint, true, new Vector2d(45, 13), Math.toRadians(0)),
                        new InstantCommand(wobbleGoalArm::setTurretLeft)
                ),
                new TurnToCommand(drivetrain, 180, telemetry),

                new PlaceWobbleGoal(wobbleGoalArm),

                new DriveForwardCommand(drivetrain, 30, Trajectories.slowConstraint),
                new TurnCommand(drivetrain, 180)

        );
    }
}
