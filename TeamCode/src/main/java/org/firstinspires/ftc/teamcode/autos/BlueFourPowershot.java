package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.teamcode.commands.PlaceWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.drive.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRingsCommand;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

@Autonomous(name = "BlueFour Powershot")
public class BlueFourPowershot extends MatchOpMode {
    // Motors
    private MotorEx leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor;
    private MotorEx intakeMotor;
    private DcMotorEx shooterMotorFront, shooterMotorBack;
    private MotorEx arm;
    private ServoEx feedServo, clawServo, lazySusanServo;

    // Gamepad
    private GamepadEx driverGamepad;

    // Subsystems
    private Drivetrain drivetrain;
    private ShooterWheels shooterWheels;
    private ShooterFeeder feeder;
    private Intake intake;
    private WobbleGoalArm wobbleGoalArm;

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
        arm = new MotorEx(hardwareMap, "arm", Motor.GoBILDA.RPM_60);
        clawServo = new SimpleServo(hardwareMap, "claw_servo", 0, 230);
        lazySusanServo = new SimpleServo(hardwareMap, "lazy_susan", 0, 360);


        // Subsystems
        drivetrain = new Drivetrain(new SampleTankDrive(hardwareMap), telemetry);
        drivetrain.init();
        intake = new Intake(intakeMotor, telemetry);
        shooterWheels = new ShooterWheels(shooterMotorFront, shooterMotorBack, telemetry);
        feeder = new ShooterFeeder(feedServo, telemetry);
        wobbleGoalArm = new WobbleGoalArm(arm, lazySusanServo, clawServo, telemetry);
        drivetrain.setPoseEstimate(Trajectories.BlueLeftTape.startPose);

    }

    @Override
    public void matchStart() {
        feeder.retractFeed();
        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> wobbleGoalArm.setLazySusanPosition(.543)),
                        new InstantCommand(() -> wobbleGoalArm.closeClaw()),
                        new InstantCommand(() -> wobbleGoalArm.setWobbleGoal(-90)),
                        new WaitUntilCommand(wobbleGoalArm::atTargetAngle),
                        new ParallelCommandGroup(new WaitCommand(1000).andThen(new InstantCommand(() -> wobbleGoalArm.setLazySusanPosition(0.95))), new TrajectoryFollowerCommand(drivetrain, Trajectories.BlueLeftTape.driveToWobble)),
                        new TurnToCommand(drivetrain, 180, telemetry),
                        new PlaceWobbleGoal(wobbleGoalArm),
                        new InstantCommand(() -> shooterWheels.setShooterRPM(2900)),
                        new TrajectoryFollowerCommand(drivetrain, Trajectories.BlueLeftTape.wobbleToHighgoal),
                        //turn and shoot
                        new ConditionalCommand( new TurnToCommand(drivetrain, 187, telemetry),  new TurnToCommand(drivetrain, 195, telemetry), () -> drivetrain.getPoseEstimate().getY() > 23),
                        new TurnToCommand(drivetrain, 187, telemetry),
                        new FeedRingsCommand(feeder, 4, 75),
                        new TurnToCommand(drivetrain, 180, telemetry),
                        //go to rings
                        new InstantCommand(intake::intake, intake),
                        new TrajectoryFollowerCommand(drivetrain, Trajectories.BlueLeftTape.highGoalHitIntake),
                        new TurnToCommand(drivetrain, 189, telemetry),
                        new InstantCommand(() -> shooterWheels.setShooterRPM(2900)),
                        new FeedRingsCommand(feeder, 5, 50),
                        new TrajectoryFollowerCommand(drivetrain, Trajectories.BlueLeftTape.intakeRings),
                        new TurnToCommand(drivetrain, 180, telemetry),
                        new InstantCommand(() -> wobbleGoalArm.setWobbleGoal(0)),
                        new TrajectoryFollowerCommand(drivetrain, Trajectories.BlueLeftTape.shootMoreRings),
                        new FeedRingsCommand(feeder, 3, 50),
                        new InstantCommand(intake::stop),
                        new InstantCommand(() -> shooterWheels.setShooterRPM(0)),
                        new InstantCommand(wobbleGoalArm::openClaw),
                        new InstantCommand(() -> wobbleGoalArm.setLazySusanPosition(.543)),

                        new TurnToCommand(drivetrain, Trajectories.BlueLeftTape.wobbleAngle, telemetry),
                        new TrajectoryFollowerCommand(drivetrain, Trajectories.BlueLeftTape.ringsToWobble),
                        new InstantCommand(wobbleGoalArm::closeClaw)
                )
        );

    }
}
