package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ShooterAngler;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;

public class ShootRingsCommand extends SequentialCommandGroup {
    public ShootRingsCommand(ShooterWheels shooter, ShooterAngler angler, ShooterFeeder feeder, double rpm, double angle, int numRings) {
        addCommands(
                    new InstantCommand(() -> angler.setShooterAngle(30)),
                    new InstantCommand(() -> shooter.setShooterRPM(rpm)),
                    new WaitCommand(500),
                    new FeedRingsCommand(feeder, numRings),
                    new InstantCommand(() -> shooter.setShooterRPM(0))
        );
    }
}
