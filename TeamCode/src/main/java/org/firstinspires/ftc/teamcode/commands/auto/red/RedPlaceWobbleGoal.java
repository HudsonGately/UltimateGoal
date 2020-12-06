package org.firstinspires.ftc.teamcode.commands.auto.red;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.vision.UGRectDetector;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class RedPlaceWobbleGoal extends SequentialCommandGroup {
    public RedPlaceWobbleGoal(Drivetrain drivetrain, Supplier<Object> currentRing) {
        Map<Object, Command> mapSelector = new HashMap<Object, Command>();
        mapSelector.put(UGRectDetector.Stack.ZERO, new InstantCommand());
        mapSelector.put(UGRectDetector.Stack.ONE, new InstantCommand());
        mapSelector.put(UGRectDetector.Stack.FOUR, new InstantCommand());
        addCommands(
                new SelectCommand(
                        mapSelector,
                        currentRing
                )
        );
    }
}
