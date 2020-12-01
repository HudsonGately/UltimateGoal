package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class FeedCommand extends CommandBase {
    private Intake intake;
    private double position;

    public FeedCommand(Intake intake, double position) {
        this.intake = intake;
        this.position = position;
        
        this.addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setFeedServo(position);
    }
}
