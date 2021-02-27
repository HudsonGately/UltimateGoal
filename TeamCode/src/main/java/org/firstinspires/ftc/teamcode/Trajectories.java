package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;

import java.util.Arrays;
import java.util.Vector;

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
            new TankVelocityConstraint(12, TRACK_WIDTH)
    ));


    public static ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);
    public static ProfileAccelerationConstraint slowAccelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);

    @Config
    public static class BlueLeftTape {
        public static Pose2d startPose = new Pose2d(-62.5, 52, Math.toRadians(180));

        public static double wobbleGoalX = 66;
        public static double wobbleGoalY = 40;

        public static double highGoalX = -3;
        public static double highGoalY = 38;
        public static double intakeFirst = 36;
        public static double intakeDistance = 12;
        public static double shootMoreDistance = 24;
        public static double wobbleDistance = 8.5;
        public static double wobbleAngle = 223;

        public static Trajectory driveToWobble = new TrajectoryBuilder(startPose, true, velConstraint, accelConstraint).splineTo(new Vector2d(wobbleGoalX, wobbleGoalY), Math.toRadians(0)).build();
        public static Trajectory wobbleToHighgoal = new TrajectoryBuilder(driveToWobble.end(), velConstraint, accelConstraint).splineTo(new Vector2d(highGoalX, highGoalY), Math.toRadians(180)).build();
        public static Trajectory highGoalHitIntake = new TrajectoryBuilder(wobbleToHighgoal.end(), velConstraint, accelConstraint).forward(intakeFirst, velConstraint, accelConstraint).build();
        public static Trajectory intakeRings = new TrajectoryBuilder(highGoalHitIntake.end(), velConstraint, accelConstraint).forward(intakeDistance, velConstraint, accelConstraint).build();
        public static Trajectory shootMoreRings = new TrajectoryBuilder(intakeRings.end(), velConstraint, accelConstraint).back(shootMoreDistance, velConstraint, accelConstraint).build();

        public static Trajectory ringsToWobble = new TrajectoryBuilder(shootMoreRings.end().plus(new Pose2d(0, 0, Math.toRadians(wobbleAngle))), slowConstraint, slowAccelConstraint).forward(wobbleDistance, slowConstraint, slowAccelConstraint).build();



    }


}
