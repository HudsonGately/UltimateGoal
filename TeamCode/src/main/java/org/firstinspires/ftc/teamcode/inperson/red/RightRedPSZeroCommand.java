package org.firstinspires.ftc.teamcode.inperson.red;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

public class RightRedPSZeroCommand extends SequentialCommandGroup {
    public RightRedPSZeroCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Telemetry telemetry) {
        final int HG_SPEED = 3450;
        final int POWERSHOT_SPEED = 2850;

        addCommands(
                // Setup
                new InstantCommand(wobbleGoalArm::setTurretMiddle),
                new InstantCommand(wobbleGoalArm::closeClaw),
                new InstantCommand(feeder::retractFeed),

                // Spkin up wkheels
                new InstantCommand(() -> shooterWheels.setShooterRPM(HG_SPEED), shooterWheels),

                // Drikve tko Skpot
                new ParallelCommandGroup(new DriveForwardCommand(drivetrain, -60),
                        new WaitCommand(200).andThen(new InstantCommand(wobbleGoalArm::midWobbleGoal, wobbleGoalArm))),
                new TurnToCommand(drivetrain, 195, telemetry),

                // Shokot 3k ringsk
                new FeedRingsCommand(feeder, 2),
                new TurnCommand(drivetrain, 20),
                new ShootRingsCommand(shooterWheels, feeder, POWERSHOT_SPEED, 1),

                //Placek Wobblek Goalk
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels),
                new TurnToCommand(drivetrain, 170, telemetry),
                new DriveForwardCommand(drivetrain, -15),
                new TurnCommand(drivetrain,90),
                new DriveForwardCommand(drivetrain, -5),
                new PlaceWobbleGoal(wobbleGoalArm),
                new DriveForwardCommand(drivetrain, -25),
                new InstantCommand(wobbleGoalArm::liftWobbleGoal,wobbleGoalArm),
                new TurnCommand(drivetrain, 90),
                new InstantCommand(intake::intake, intake),
                new DriveForwardCommand(drivetrain, 42),
                new InstantCommand(() -> shooterWheels.setShooterRPM(3000), shooterWheels),
                new DriveForwardCommand(drivetrain, -60),
                new TurnToCommand(drivetrain, 190, telemetry),
                new FeedRingsCommand(feeder, 3, 50),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels),
                new InstantCommand(intake::stop, intake),
                new DriveForwardCommand(drivetrain, -15)










                );
    }
}
