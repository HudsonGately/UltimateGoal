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
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

import static org.firstinspires.ftc.teamcode.Trajectories.BlueLeftTape.highGoalX;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueLeftTape.highGoalY;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueLeftTape.intakeDistance;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueLeftTape.intakeFirst;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueLeftTape.shootMoreDistance;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueLeftTape.wobbleGoalX;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueLeftTape.wobbleGoalY;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueMid.shootDistance;

public class FourRingCommand extends SequentialCommandGroup {

    public FourRingCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Telemetry telemetry) {
        addCommands(
                new InstantCommand(wobbleGoalArm::setTurretMiddle),
                new InstantCommand(wobbleGoalArm::closeClaw),

                new InstantCommand(() -> shooterWheels.setShooterRPM(2900)),
                new DriveForwardCommand(drivetrain, -60),

                new InstantCommand(() -> wobbleGoalArm.setWobbleGoal(-85)),
                //turn and shoot
                // turn and shoot
                new TurnToCommand(drivetrain, 178, telemetry),
                new FeedRingsCommand(feeder, 4, 75),
                new InstantCommand(intake::stop),
                new TurnToCommand(drivetrain, 180, telemetry),

                new ParallelCommandGroup(new WaitCommand(200).andThen(new InstantCommand(wobbleGoalArm::setTurretLeft)), new SplineCommand(drivetrain, Trajectories.velConstraint, true, new Vector2d(wobbleGoalX, wobbleGoalY), Math.toDegrees(0))),
                new TurnToCommand(drivetrain, 180, telemetry),
                new PlaceWobbleGoal(wobbleGoalArm),
                new SplineCommand(drivetrain, new Vector2d(highGoalX, highGoalY), Math.toRadians(180)),
                new TurnToCommand(drivetrain, 180, telemetry),
                //go to rings
                new InstantCommand(intake::intake, intake),
                new InstantCommand(() -> shooterWheels.setShooterRPM(3000)),
                new DriveForwardCommand(drivetrain, intakeFirst),
                new TurnToCommand(drivetrain, 188, telemetry),
                new FeedRingsCommand(feeder, 5, 80),
                new InstantCommand(wobbleGoalArm::setTurretMiddle),
                new DriveForwardCommand(drivetrain, intakeDistance),
                new TurnToCommand(drivetrain, 185, telemetry),
                new InstantCommand(wobbleGoalArm::placeWobbleGoal, wobbleGoalArm),
                new DriveForwardCommand(drivetrain, -shootMoreDistance),
                new FeedRingsCommand(feeder, 3, 50),
                new TurnToCommand(drivetrain, 180, telemetry),
                new InstantCommand(intake::stop),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0)),
                new InstantCommand(wobbleGoalArm::openClaw),
                new TurnToCommand(drivetrain, Trajectories.BlueLeftTape.wobbleAngle, telemetry),
                new ParallelCommandGroup(new DriveForwardCommand(drivetrain, Trajectories.BlueLeftTape.wobbleDistance, Trajectories.slowConstraint), new WaitCommand(500).andThen(new InstantCommand(wobbleGoalArm::closeClaw))),
                new WaitCommand(500),
                new InstantCommand(() -> wobbleGoalArm.setWobbleGoal(-100)),
                new ParallelCommandGroup(new WaitCommand(1000).andThen(new InstantCommand(wobbleGoalArm::setTurretLeft, wobbleGoalArm)), new SplineCommand(drivetrain, Trajectories.velConstraint, true, new Vector2d(wobbleGoalX+6, 28), Math.toDegrees(0))),
                new TurnToCommand(drivetrain, 180, telemetry),
                new PlaceWobbleGoal(wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::setTurretMiddle),
                new InstantCommand(wobbleGoalArm::placeWobbleGoal),
                new DriveForwardCommand(drivetrain, 30)



                );
    }
}
