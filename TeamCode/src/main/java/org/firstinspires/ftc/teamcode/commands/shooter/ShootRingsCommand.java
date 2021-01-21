package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;

public class ShootRingsCommand extends SequentialCommandGroup {
    public ShootRingsCommand(ShooterWheels shooter, ShooterFeeder feeder, double rpm, int numRings) {
        addCommands(
                    new InstantCommand(() -> shooter.setShooterRPM(rpm)),
                    new WaitUntilCommand(shooter::atSetpoint),
                    new FeedRingsCommand(feeder, numRings, 150),
                    new WaitCommand(500),
                    new InstantCommand(() -> shooter.setShooterRPM(0))
        );
    }
}
