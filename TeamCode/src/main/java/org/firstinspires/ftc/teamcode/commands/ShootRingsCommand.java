package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShootRingsCommand extends SequentialCommandGroup {
    public ShootRingsCommand(Shooter shooter, Intake intake, double angle, double rpm, int numRings) {
        addCommands(
                new ShooterAngleCommand(shooter, angle, .5),
                new ParallelRaceGroup(
                        new ShootRPMCommand(shooter, rpm),
                        new FeedRingsCommand(intake, numRings)
                )
        );
    }
}
