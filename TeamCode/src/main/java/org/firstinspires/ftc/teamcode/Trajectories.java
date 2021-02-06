package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

public class Trajectories {

    public static MinVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(MAX_ANG_VEL),
                new TankVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
    public static MinVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(MAX_ANG_VEL),
            new TankVelocityConstraint(30, TRACK_WIDTH)
    ));
    public static ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);
    public static Pose2d startPose = new Pose2d();

    // Move To line
    public static Trajectory driveToShoot = new TrajectoryBuilder(startPose, velConstraint, accelConstraint).back(48).build();

    public static Trajectory shootToZeroSquare = new TrajectoryBuilder(driveToShoot.end(), velConstraint, accelConstraint).back(30).build();
    public static Trajectory shootToFourSquare = new TrajectoryBuilder(driveToShoot.end(), velConstraint, accelConstraint).back(70).build();
    public static Trajectory shootToOneSquare = new TrajectoryBuilder(driveToShoot.end(), true, velConstraint, accelConstraint)
            .splineTo(new Vector2d(-76, 32), Math.toRadians(180)).back(24).build();

    public static Trajectory fourSquareToLine = new TrajectoryBuilder(shootToFourSquare.end(), velConstraint, accelConstraint).forward(48).build();
    public static Trajectory oneSquareToLine = new TrajectoryBuilder(shootToOneSquare.end(), velConstraint, accelConstraint)
            .splineTo(new Vector2d(-66, 0), Math.toRadians(0)).build();
    public static Trajectory lineToIntake = new TrajectoryBuilder(fourSquareToLine.end(), slowConstraint, accelConstraint).splineTo(new Vector2d(-20, 20), Math.toRadians(0), slowConstraint, accelConstraint).build();
    public static Trajectory intakeToShoot = new TrajectoryBuilder(lineToIntake.end(), true, slowConstraint, accelConstraint).splineTo(new Vector2d(-48, 0), Math.toRadians(180), slowConstraint, accelConstraint).build();

    public static Trajectory lineToSecondWobbleGoal = new TrajectoryBuilder(oneSquareToLine.end(), velConstraint, accelConstraint)
            .splineTo(new Vector2d(-20, 44), Math.toRadians(0)).build();
    public static Trajectory secondWobbleGoalToLine = new TrajectoryBuilder(lineToSecondWobbleGoal.end(), velConstraint, accelConstraint)
            .splineTo(new Vector2d(-64, 0), Math.toRadians(0)).build();

}
