package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.commands.GoToLineShootHighGoal;
import org.firstinspires.ftc.teamcode.commands.GoToLineShootPowershotBlue;
import org.firstinspires.ftc.teamcode.commands.PlaceWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.drive.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRingsCommand;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

import java.util.HashMap;
import java.util.logging.Level;

@Autonomous(name = "HighGoalBlue")
public class HighGoalAuto extends MatchOpMode {
    // Motors
    private MotorEx intakeMotor;
    private DcMotorEx shooterMotorFront, shooterMotorBack;
    private CRServo arm;
    private ServoEx feedServo, clawServo;

    private ServoEx releaseShooter;
    // Gamepad
    private GamepadEx driverGamepad;

    // Subsystems
    private Drivetrain drivetrain;
    private ShooterWheels shooterWheels;
    private ShooterFeeder feeder;
    private Intake intake;
    private WobbleGoalArm wobbleGoalArm;
    private Vision vision;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void robotInit() {

        // Drivetrain Hardware Initializations
        // Intake hardware Initializations
        intakeMotor = new MotorEx(hardwareMap, "intake");

        // Shooter hardware initializations
        shooterMotorBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter_back");
        shooterMotorFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter_front");
        feedServo = new SimpleServo(hardwareMap, "feed_servo", 0, 230);

        // Wobble Harware initializations
        arm = hardwareMap.get(CRServo.class, "arm");
        clawServo = new SimpleServo(hardwareMap, "claw_servo", 0, 230);

        releaseShooter = new SimpleServo(hardwareMap, "release_servo", 0, 180);

        // Subsystems
        drivetrain = new Drivetrain(new SampleTankDrive(hardwareMap, packet), telemetry, packet);
        intake = new Intake(intakeMotor, telemetry, packet);
        shooterWheels = new ShooterWheels(shooterMotorFront, shooterMotorBack, telemetry, packet);
        feeder = new ShooterFeeder(feedServo, telemetry, packet);
        wobbleGoalArm = new WobbleGoalArm(arm, clawServo, telemetry, packet);
        vision = new Vision(hardwareMap, "webcam", telemetry, packet);
    }

    @Override
    public void robotPeriodic() {
       Util.logger(this, telemetry, Level.INFO, "Current Stack", vision.getCurrentStack());
    }

    @Override
    public void matchStart() {
        Trajectory trajectory = drivetrain.trajectoryBuilder(new Pose2d())
                .back(48)
                .build();
        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> releaseShooter.setPosition(0.2)),
                        new InstantCommand(() -> drivetrain.setPoseEstimate(new Pose2d())),
                        new InstantCommand(feeder::retractFeed),
                        new SelectCommand(new HashMap<Object, Command>() {{
                            put(UGDetector2.Stack.FOUR, new SequentialCommandGroup(
                                    new GoToLineShootHighGoal(drivetrain, shooterWheels, feeder),
                                    new InstantCommand(() -> drivetrain.setPoseEstimate(new Pose2d())),
                                    new TrajectoryFollowerCommand(drivetrain, drivetrain.trajectoryBuilder(new Pose2d()).back(70).build()),
                                    new PlaceWobbleGoal(wobbleGoalArm),
                                    new InstantCommand(() -> drivetrain.setPoseEstimate(new Pose2d())),
                                    new TrajectoryFollowerCommand(drivetrain, drivetrain.trajectoryBuilder(new Pose2d()).forward(48).build())
                            ));
                            put(UGDetector2.Stack.ONE, new SequentialCommandGroup(
                                    new GoToLineShootHighGoal(drivetrain, shooterWheels, feeder),
                                    new TurnCommand(drivetrain, -30),
                                    new WaitCommand(500),
                                    new InstantCommand(() -> drivetrain.setPoseEstimate(new Pose2d())),
                                    new TrajectoryFollowerCommand(drivetrain, drivetrain.trajectoryBuilder(new Pose2d()).back(48).build()),
                                    new PlaceWobbleGoal(wobbleGoalArm),
                                    new InstantCommand(() -> drivetrain.setPoseEstimate(new Pose2d())),
                                    new TrajectoryFollowerCommand(drivetrain, drivetrain.trajectoryBuilder(new Pose2d()).forward(24).build())
                            ));
                            put(UGDetector2.Stack.ZERO, new SequentialCommandGroup(
                                    new GoToLineShootHighGoal(drivetrain, shooterWheels, feeder),
                                    new InstantCommand(() -> drivetrain.setPoseEstimate(new Pose2d())),
                                    new TrajectoryFollowerCommand(drivetrain, drivetrain.trajectoryBuilder(new Pose2d()).back(32).build()),
                                    new PlaceWobbleGoal(wobbleGoalArm)
                            ));
                        }}, vision::getCurrentStack),
                        new InstantCommand(this::stop)
                )
        );
    }
}
