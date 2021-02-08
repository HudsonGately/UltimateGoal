package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

public class MoveWobbleGoalUp extends SequentialCommandGroup {
    private WobbleGoalArm wobbleGoal;

    public MoveWobbleGoalUp(WobbleGoalArm wobbleGoal) {
        addCommands(
                new InstantCommand(() -> wobbleGoal.setArmSpeed(11), wobbleGoal),
                new WaitCommand(2000),
                new InstantCommand(wobbleGoal::stopArm, wobbleGoal)
        );
    }
}
