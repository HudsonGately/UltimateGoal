package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;

public class FeedRingsCommand extends SequentialCommandGroup {
    public FeedRingsCommand(ShooterFeeder feeder, int numRings) {
        for (int i = 0; i < numRings; i++) {
            addCommands(
                new WaitCommand(75),
                new InstantCommand(feeder::feedShooter),
                new WaitCommand(75),
                new InstantCommand(feeder::retractFeed)

            );
        }
    }
}
