package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeCommand extends CommandBase {
    Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
        this.addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        intake.intake();
    }

    public void end(boolean interrupted) {
        intake.stop();
    }
}
