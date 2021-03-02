package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
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

public class ZeroRingCommand extends SequentialCommandGroup {

    public ZeroRingCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Telemetry telemetry) {
        addCommands(
                new InstantCommand(wobbleGoalArm::setTurretMiddle),
                new InstantCommand(wobbleGoalArm::closeClaw),
                new ParallelCommandGroup(new WaitCommand(750).andThen(new InstantCommand(() -> wobbleGoalArm.setWobbleGoal(-90))).andThen(new WaitCommand(500)).andThen(new InstantCommand(wobbleGoalArm::setTurretLeft)), new DriveForwardCommand(drivetrain, -wobbleGoalSquareDistance)),
                new TurnToCommand(drivetrain, 180, telemetry),
                new PlaceWobbleGoal(wobbleGoalArm),
                new InstantCommand(() -> shooterWheels.setShooterRPM(2900)),
                new SplineCommand(drivetrain, new Vector2d(highGoalX, highGoalY), Math.toRadians(180)),
                //turn and shoot
                new TurnToCommand(drivetrain, 195, telemetry),
                new FeedRingsCommand(feeder, 4, 75),
                new InstantCommand(intake::stop),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0)),
                //turn towards wobble goal
                new TurnToCommand(drivetrain, Trajectories.BlueCloseTape.wobbleAngle, telemetry),
                new InstantCommand(wobbleGoalArm::openClaw),
                new InstantCommand(wobbleGoalArm::setTurretMiddle),
                new InstantCommand(() -> wobbleGoalArm.setWobbleGoal(4)),
                new ParallelCommandGroup(new DriveForwardCommand(drivetrain, Trajectories.BlueCloseTape.wobbleDistance, Trajectories.velConstraint), new WaitCommand(1000).andThen(new InstantCommand(wobbleGoalArm::closeClaw))),
                //go to wobble goal
                new WaitCommand(1000),
                new InstantCommand(wobbleGoalArm::liftWobbleGoal, wobbleGoalArm),
                new DriveForwardCommand(drivetrain, -48),
                new TurnToCommand(drivetrain,90, telemetry),
                new DriveForwardCommand(drivetrain, 10),

                new InstantCommand(wobbleGoalArm::placeWobbleGoal, wobbleGoalArm),
                new WaitCommand(1000),
                new InstantCommand(wobbleGoalArm::openClaw, wobbleGoalArm),
                new DriveForwardCommand(drivetrain, -4),
                new InstantCommand(() -> wobbleGoalArm.setWobbleGoal(-127), wobbleGoalArm)

        );
    }
}
