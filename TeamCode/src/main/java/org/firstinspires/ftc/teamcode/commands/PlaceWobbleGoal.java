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
                new WaitCommand(1900),
                new InstantCommand(wobbleGoal::stopArm, wobbleGoal),
                new InstantCommand(() -> wobbleGoal.openClaw(), wobbleGoal),
                new WaitCommand(800),
                new InstantCommand(() -> wobbleGoal.setArmSpeed(1), wobbleGoal),
                new WaitCommand(1900),
                new InstantCommand(() -> wobbleGoal.setArmSpeed(0.0), wobbleGoal)
        );
    }
}
