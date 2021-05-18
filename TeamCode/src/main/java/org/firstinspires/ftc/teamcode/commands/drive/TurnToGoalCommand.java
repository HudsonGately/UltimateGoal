package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.VisionHG;
import org.firstinspires.ftc.teamcode.subsystems.VisionStack;

@Config
public class TurnToGoalCommand extends ConditionalCommand {

    public TurnToGoalCommand(Drivetrain drivetrain, VisionHG vision, double defaultTurnToAngle) {
        super(new VisionCommand(drivetrain, vision, 20), new TurnToCommand(drivetrain, defaultTurnToAngle), () -> vision.isTargetVisible());
    }

}
