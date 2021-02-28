package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.teamcode.commands.PlaceWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.drive.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
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

import static org.firstinspires.ftc.teamcode.Trajectories.BlueMid.shootDistance;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueMid.wobbleGoalSquareDistance;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueMid.ringX;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueMid.ringY;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueMid.intakeDistance;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueMid.intakeFirst;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueMid.shootMoreDistance;


@Autonomous(name = "Middle")
public class MiddleStartingPosition extends MatchOpMode {
    // Motors
    private MotorEx leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor;
    private MotorEx intakeMotor;
    private DcMotorEx shooterMotorFront, shooterMotorBack;
    private MotorEx arm;
    private ServoEx feedServo, clawServo, lazySusanServo;
    private TouchSensor wobbleTouchSensor;

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
        wobbleTouchSensor = hardwareMap.get(TouchSensor.class, "Touch");


        // Subsystems
        drivetrain = new Drivetrain(new SampleTankDrive(hardwareMap), telemetry);
        drivetrain.init();
        intake = new Intake(intakeMotor, telemetry);
        shooterWheels = new ShooterWheels(shooterMotorFront, shooterMotorBack, telemetry);
        feeder = new ShooterFeeder(feedServo, telemetry);
        wobbleGoalArm = new WobbleGoalArm(arm, lazySusanServo, clawServo, wobbleTouchSensor, telemetry);
        drivetrain.setPoseEstimate(Trajectories.BlueMid.startPose);

    }

    @Override
    public void matchStart() {
        feeder.retractFeed();
        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(wobbleGoalArm::setTurretMiddle),
                        new InstantCommand(wobbleGoalArm::closeClaw),
                        new InstantCommand(() -> wobbleGoalArm.setWobbleGoal(-90)),
                        new WaitUntilCommand(wobbleGoalArm::atTargetAngle),
                        new InstantCommand(() -> shooterWheels.setShooterRPM(2900)),
                        new ParallelCommandGroup(new WaitCommand(1000).andThen(new InstantCommand(wobbleGoalArm::setTurretRight)), new DriveForwardCommand(drivetrain, -shootDistance)),
                        // turn and shoot
                        new TurnToCommand(drivetrain, 178, telemetry),
                        new FeedRingsCommand(feeder, 4, 75),
                        new InstantCommand(intake::stop),
                        new InstantCommand(() -> shooterWheels.setShooterRPM(0)),
                        new TurnToCommand(drivetrain, 180, telemetry),
                        //place wobble goal
                        new DriveForwardCommand(drivetrain, -wobbleGoalSquareDistance),
                        new PlaceWobbleGoal(wobbleGoalArm),
                        //go to ring
                        new InstantCommand(intake::intake),
                        new InstantCommand(() -> shooterWheels.setShooterRPM(2750)),
                        new SplineCommand(drivetrain, new Vector2d(Trajectories.BlueMid.ringX, Trajectories.BlueMid.ringY), Math.toRadians(180)),
                        //align for powershot
                        new TurnToCommand(drivetrain, 180, telemetry),
                        new FeedRingsCommand(feeder, 4, 75),
                        new InstantCommand(intake::stop),
                        new InstantCommand(() -> shooterWheels.setShooterRPM(0)),
                        //wobble goal
                        new TurnToCommand(drivetrain, Trajectories.BlueMid.wobbleAngle, telemetry),
                        new InstantCommand(wobbleGoalArm::openClaw),
                        new InstantCommand(wobbleGoalArm::setTurretMiddle),
                        // lower arm
                        new DriveForwardCommand(drivetrain, -2, Trajectories.slowConstraint),
                        new InstantCommand(() -> wobbleGoalArm.setWobbleGoal(4)),
                        new WaitCommand(2000),
                        new DriveForwardCommand(drivetrain, Trajectories.BlueMid.wobbleDistance, Trajectories.slowConstraint),
                        new WaitCommand(500),
                        // grab wobble gaol
                        new InstantCommand(wobbleGoalArm::closeClaw),
                        new WaitCommand(1000),
                        new InstantCommand(wobbleGoalArm::liftWobbleGoal),
                        new ParallelCommandGroup(
                            new SplineCommand(drivetrain, Trajectories.velConstraint, true, new Vector2d(40, 12), Math.toRadians(0)),
                            new InstantCommand(wobbleGoalArm::setTurretLeft)
                        ),
                        new TurnToCommand(drivetrain, 180, telemetry),

                        new PlaceWobbleGoal(wobbleGoalArm),
                        new DriveForwardCommand(drivetrain, 24, Trajectories.slowConstraint),
                        new TurnCommand(drivetrain, 180)

              )
        );

    }
}
