package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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
import org.firstinspires.ftc.teamcode.commands.drive.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

import static org.firstinspires.ftc.teamcode.Trajectories.BlueCloseTape.highGoalX;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueCloseTape.highGoalY;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueCloseTape.wobbleGoalSquareDistance;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueCloseTape.wobbleGoalX;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueCloseTape.wobbleGoalY;

@Autonomous(name = "Front")
public class FrontWobblePosition extends MatchOpMode {
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
        drivetrain.setPoseEstimate(Trajectories.BlueCloseTape.startPose);

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
                        new ParallelCommandGroup(new WaitCommand(1000).andThen(new InstantCommand(wobbleGoalArm::setTurretLeft)), new DriveForwardCommand(drivetrain, -wobbleGoalSquareDistance)),
                        new TurnToCommand(drivetrain, 180, telemetry),
                        new PlaceWobbleGoal(wobbleGoalArm),
                        new InstantCommand(() -> shooterWheels.setShooterRPM(2900)),
                        new SplineCommand(drivetrain, new Vector2d(highGoalX, highGoalY), Math.toRadians(180)),
                        //turn and shoot
                        new TurnToCommand(drivetrain, 195, telemetry),
                        new FeedRingsCommand(feeder, 4, 75),
                        new InstantCommand(intake::stop),
                        new InstantCommand(() -> shooterWheels.setShooterRPM(0)),
                        //turn towards wobble goal
                        new TurnToCommand(drivetrain, Trajectories.BlueCloseTape.wobbleAngle, telemetry),
                        new InstantCommand(wobbleGoalArm::openClaw),
                        new InstantCommand(wobbleGoalArm::setTurretMiddle),
                        new InstantCommand(() -> wobbleGoalArm.setWobbleGoal(4)),
                        new ParallelCommandGroup(new DriveForwardCommand(drivetrain, Trajectories.BlueCloseTape.wobbleDistance, Trajectories.velConstraint), new WaitCommand(1000).andThen(new InstantCommand(wobbleGoalArm::closeClaw))),
                        //go to wobble goal
                        new WaitCommand(1000),
                        new InstantCommand(wobbleGoalArm::liftWobbleGoal, wobbleGoalArm),
                        new DriveForwardCommand(drivetrain, -50),
                        new TurnToCommand(drivetrain,90, telemetry),
                        new DriveForwardCommand(drivetrain, 10),

                        new InstantCommand(wobbleGoalArm::placeWobbleGoal, wobbleGoalArm),
                        new WaitCommand(1000),
                        new InstantCommand(wobbleGoalArm::openClaw, wobbleGoalArm),
                        new DriveForwardCommand(drivetrain, -4),
                        new InstantCommand(() -> wobbleGoalArm.setWobbleGoal(-127), wobbleGoalArm)




                    /*
                        new InstantCommand(wobbleGoalArm::openClaw),
                        new InstantCommand(wobbleGoalArm::setTurretMiddle),
                        new TurnToCommand(drivetrain, Trajectories.BlueCloseTape.wobbleAngle, telemetry),
                        new ParallelCommandGroup(new DriveForwardCommand(drivetrain, Trajectories.BlueCloseTape.wobbleDistance, Trajectories.slowConstraint), new WaitCommand(500).andThen(new InstantCommand(wobbleGoalArm::closeClaw))),

                        new WaitCommand(500),
                        new InstantCommand(() -> wobbleGoalArm.setWobbleGoal(-100)),
                        new ParallelCommandGroup(new WaitCommand(1000).andThen(new InstantCommand(wobbleGoalArm::setTurretLeft, wobbleGoalArm)), new SplineCommand(drivetrain, Trajectories.velConstraint, true, new Vector2d(wobbleGoalX + 4, 30), Math.toDegrees(0))),
                        new TurnToCommand(drivetrain, 90, telemetry),
                        new PlaceWobbleGoal(wobbleGoalArm),
                        new SplineCommand(drivetrain, Trajectories.velConstraint, true, new Vector2d(20, 30), Math.toRadians(180))
                        */

                )
        );

    }
}
