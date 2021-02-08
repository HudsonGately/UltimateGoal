package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

public class PickupWobbleGoal extends SequentialCommandGroup {
    private WobbleGoalArm wobbleGoal;

    public PickupWobbleGoal(WobbleGoalArm wobbleGoal) {
        addCommands(
                new InstantCommand(() -> wobbleGoal.openClaw(), wobbleGoal),
                new InstantCommand(() -> wobbleGoal.setArmSpeed(-1), wobbleGoal),
                new WaitCommand(2000),
                new InstantCommand(wobbleGoal::stopArm, wobbleGoal),
                new InstantCommand(() -> wobbleGoal.closeClaw(), wobbleGoal),
                new WaitCommand(800),
                new InstantCommand(() -> wobbleGoal.setArmSpeed(1), wobbleGoal),
                new WaitCommand(2000),
                new InstantCommand(() -> wobbleGoal.setArmSpeed(0.0), wobbleGoal)
        );
    }
}
