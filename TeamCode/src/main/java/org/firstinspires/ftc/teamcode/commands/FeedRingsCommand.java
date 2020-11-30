package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class FeedRingsCommand extends SequentialCommandGroup {
    public FeedRingsCommand(Intake intake, int numRings) {
        for (int i = 0; i < numRings; i++) {
            addCommands(new FeedCommand(intake, Constants.SERVO_POSITION_SHOOT),
                    new WaitCommand(250),
                    new FeedCommand(intake, Constants.SERVO_POSITION_HOME));
        }
        addCommands(new WaitCommand(500));
    }
}
