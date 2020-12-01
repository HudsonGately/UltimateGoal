package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class OuttakeCommand extends CommandBase {
    Intake intake;

    OuttakeCommand(Intake intake) {
        this.intake = intake;
        this.addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        intake.outtake();
    }

    public void end(boolean interrupted) {
        intake.stop();
    }
}
