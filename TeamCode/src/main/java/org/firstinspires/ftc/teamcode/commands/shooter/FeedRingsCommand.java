package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;

public class FeedRingsCommand extends SequentialCommandGroup {
    public FeedRingsCommand(ShooterFeeder feeder, int numRings) {
        for (int i = 0; i < numRings; i++) {
            addCommands(new FeedCommand(feeder, Constants.SERVO_POSITION_SHOOT),
                    new WaitCommand(50),
                    new FeedCommand(feeder, Constants.SERVO_POSITION_HOME));
        }
    }
}
