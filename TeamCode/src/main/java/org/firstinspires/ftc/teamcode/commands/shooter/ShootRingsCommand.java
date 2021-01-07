package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;

public class ShootRingsCommand extends SequentialCommandGroup {
    public ShootRingsCommand(ShooterWheels shooter, ShooterFeeder feeder, double rpm, int numRings) {
        addCommands(
                    new InstantCommand(() -> shooter.setShooterRPM(rpm)),
                    new WaitCommand(1000),
                    new FeedRingsCommand(feeder, numRings),
                    new InstantCommand(() -> shooter.setShooterRPM(0))
        );
    }
}
