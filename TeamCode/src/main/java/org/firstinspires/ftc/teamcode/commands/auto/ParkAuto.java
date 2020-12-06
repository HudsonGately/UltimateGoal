package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.drive.DriveStraight;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class ParkAuto extends SequentialCommandGroup {
    public ParkAuto(Drivetrain drivetrain) {
        addCommands(
            new DriveStraight(drivetrain, 12)
        );
    }
}
