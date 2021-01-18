package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

public class PlaceWobbleGoal extends SequentialCommandGroup {
    private WobbleGoalArm wobbleGoal;

    public PlaceWobbleGoal(WobbleGoalArm wobbleGoal) {
        addCommands(
                new InstantCommand(() -> wobbleGoal.setArmSpeed(-1), wobbleGoal),
                new WaitCommand(2400),
                new InstantCommand(wobbleGoal::stopArm, wobbleGoal),
                new WaitCommand(500),
                new InstantCommand(() -> wobbleGoal.openClaw()),
                new WaitCommand(500),
                new InstantCommand(() -> wobbleGoal.setArmSpeed(1), wobbleGoal),
                new WaitCommand(1000),
                new InstantCommand(() -> wobbleGoal.setArmSpeed(0.0), wobbleGoal),
                new InstantCommand(() -> wobbleGoal.closeClaw())
        );
    }
}
