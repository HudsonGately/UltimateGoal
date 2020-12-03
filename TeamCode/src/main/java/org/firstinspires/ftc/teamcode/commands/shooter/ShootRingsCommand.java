package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ShooterAngler;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;

public class ShootRingsCommand extends SequentialCommandGroup {
    public ShootRingsCommand(ShooterWheels shooter, ShooterAngler angler, ShooterFeeder feeder, double rpm, int numRings) {
        addCommands(
                    new InstantCommand(() -> shooter.setShooterRPM(rpm)),
                    new FeedRingsCommand(feeder, numRings),
                    new InstantCommand(() -> shooter.setShooterRPM(0))
        );
    }
}
