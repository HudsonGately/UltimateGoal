package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

public class MoveWobbleGoalDown extends SequentialCommandGroup {
    private WobbleGoalArm wobbleGoal;

    public MoveWobbleGoalDown(WobbleGoalArm wobbleGoal) {
        addCommands(
                new InstantCommand(() -> wobbleGoal.setArmSpeed(-1), wobbleGoal),
                new WaitCommand(2175),
                new InstantCommand(wobbleGoal::stopArm, wobbleGoal)
        );
    }
}
