package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.intake.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;

public class ShootRingsCommand extends SequentialCommandGroup {
    public ShootRingsCommand(ShooterWheels shooterWheels, Intake intake, double angle, double rpm, int numRings) {
        addCommands(
                new ShooterAngleCommand(shooterWheels, angle, .5),
                new ParallelRaceGroup(
                        new ShootRPMCommand(shooterWheels, rpm),
                        new FeedRingsCommand(intake, numRings)
                )
        );
    }
}
