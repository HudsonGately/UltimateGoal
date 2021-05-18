package org.firstinspires.ftc.teamcode.inperson.blue.fellowship;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.PlaceWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.drive.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

public class FellowshipBlueZeroCommand extends SequentialCommandGroup {
    public FellowshipBlueZeroCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Telemetry telemetry) {
        final int HG_SPEED = 3450;
        final int POWERSHOT_SPEED = 3000;

        addCommands(
                // Setup
                new InstantCommand(wobbleGoalArm::setTurretMiddle),
                new InstantCommand(wobbleGoalArm::closeClaw),
                new InstantCommand(feeder::retractFeed),


                // Spin up wheels
                new InstantCommand(() -> shooterWheels.setShooterRPM(HG_SPEED), shooterWheels),

                // Drive to Spot
                new ParallelCommandGroup(new DriveForwardCommand(drivetrain, -60),
                        new WaitCommand(200).andThen(new InstantCommand(wobbleGoalArm::midWobbleGoal, wobbleGoalArm))),
                //new TurnToGoalCommand(drivetrain, vision, 195),
                new TurnToCommand(drivetrain, 170),

                // Shoot 3 rings
                new FeedRingsCommand(feeder, 3),
                //Place Wobble Goal
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels),
                new TurnToCommand(drivetrain, 180),
                new DriveForwardCommand(drivetrain, -15),
                new InstantCommand(wobbleGoalArm::setTurretLeft,wobbleGoalArm),
                new WaitCommand(500),
                new PlaceWobbleGoal(wobbleGoalArm),
                new WaitCommand(500),
                new InstantCommand(wobbleGoalArm::openClaw, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::liftWobbleGoal, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::setTurretMiddle, wobbleGoalArm),
                new TurnToCommand(drivetrain, 180),


                //get out of the way
                new DriveForwardCommand(drivetrain, -40),
                new TurnCommand(drivetrain,90),
                //wait a bit
                new WaitCommand(5000),
                new InstantCommand(intake::intake, intake),
                new DriveForwardCommand(drivetrain, 40),
                new TurnToCommand(drivetrain, 180),
                new DriveForwardCommand(drivetrain, 58),

                new InstantCommand(intake::stop, intake),

                // Spin up wheels (probably take out later)
                new InstantCommand(() -> shooterWheels.setShooterRPM(HG_SPEED), shooterWheels),
                new TurnToCommand(drivetrain, 205),

                // Shoot ring(s)
                new FeedRingsCommand(feeder, 3),
                new WaitCommand(250),
                new DriveForwardCommand(drivetrain, -10)










        );
    }
}
