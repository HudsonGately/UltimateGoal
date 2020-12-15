package org.firstinspires.ftc.teamcode.commands.auto.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.vision.UGRectDetector;

import org.firstinspires.ftc.teamcode.commands.drive.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.ShooterAngler;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class RedAuto extends SequentialCommandGroup {

    Trajectory autoTarget;
    public RedAuto(Drivetrain drivetrain, ShooterWheels shooter, ShooterAngler angler, ShooterFeeder feeder, Supplier<Object> currentRing) {
        Map<Object, Command> mapSelector = new HashMap<Object, Command>();
        Pose2d startPose = new Pose2d(-62, 50, 0);
        drivetrain.setPoseEstimate(startPose);
        mapSelector.put(UGRectDetector.Stack.ZERO, new SequentialCommandGroup(
                new InstantCommand(() -> {
                    autoTarget = drivetrain.trajectoryBuilder(startPose).splineTo(new Vector2d(12, 50), 0).build();
                }),
                new TrajectoryFollowerCommand(drivetrain, autoTarget)
           // Middle

        ));
        mapSelector.put(UGRectDetector.Stack.ONE, new SequentialCommandGroup(

        ));
        mapSelector.put(UGRectDetector.Stack.FOUR, new SequentialCommandGroup(

        ));
        addCommands(
                new SelectCommand(
                        mapSelector,
                        currentRing
                ),
                // Put down wobble goal, code later
                // TODO WobbleGoalCommand
                new TrajectoryFollowerCommand(drivetrain, drivetrain.trajectoryBuilder(autoTarget.end(), true).splineTo(new Vector2d(-5.0, 50.0), Math.toRadians(156.31)).build()),
                new InstantCommand(() -> angler.setShooterAngle(30), angler),
                new WaitUntilCommand(() -> angler.atSetpoint()),
                new ShootRingsCommand(shooter, feeder, 4500, 3)
        );
    }
}
