package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drive.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRPMCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterAngleCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterAngler;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

@TeleOp(name = "TeleOp")
public class TeleopTest extends CommandOpMode {

    // Motors
    private MotorEx leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor;
    private MotorEx intakeMotor;
    private MotorEx shooterMotorFront, shooterMotorBack, anglerMotor;
    private CRServo arm;
    private ServoEx feedServo, clawServo;

    // Gyro
    private GyroEx gyro;

    // Gamepad
    private GamepadEx driverGamepad;

    // Subsystems
    private Drivetrain drivetrain;
    private ShooterWheels shooterWheels;
    private ShooterAngler shooterAngler;
    private ShooterFeeder feeder;
    private Intake intake;
    private WobbleGoalArm wobbleGoalArm;

    // Commands

    /*
        ✅Right Trigger - Slow Mode
        ✅Left Trigger - Single Shot
        ✅Right Bumper - Intake
        ✅Left Bumper - Triple Shot
        ✅Y - Start/Stop Shooter
        ✅X - Switch Shooter Angles
        ✅A - Wobble Goal Claw Open/Close
        ✅B - Outtake
        ✅ D-Pad Up - Wobble Goal Arm Up
    */
    // Buttons
    private Button intakeButton, outtakeButton, shootButton, slowModeTrigger, liftArmButton, lowerArmButton, toggleClawButton, tripleShotButton, singleShotButton, angleToggleButton;

    @Override
    public void initialize() {

        // Drivetrain Hardware Initializations
        leftBackDriveMotor = new MotorEx(hardwareMap, "rear_drive_left");
        leftFrontDriveMotor = new MotorEx(hardwareMap, "front_drive_left");
        rightBackDriveMotor = new MotorEx(hardwareMap, "rear_drive_right");
        rightFrontDriveMotor = new MotorEx(hardwareMap, "front_drive_right");
        gyro = new RevIMU(hardwareMap);

        // Intake hardware Initializations
        intakeMotor = new MotorEx(hardwareMap, "intake");

        // Shooter hardware initializations
        shooterMotorBack = new MotorEx(hardwareMap, "shooter_back");
        shooterMotorFront = new MotorEx(hardwareMap, "shooter_front");
        anglerMotor = new MotorEx(hardwareMap, "angler");
        feedServo = new SimpleServo(hardwareMap, "feed_servo");

        // Wobble Harware initializations
        arm = new CRServo(hardwareMap, "arm");
        clawServo = new SimpleServo(hardwareMap, "claw_servo");


        // Subsystems
        drivetrain = new Drivetrain(leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor, gyro);
        intake = new Intake(intakeMotor);
        shooterWheels = new ShooterWheels(shooterMotorFront, shooterMotorBack);
        shooterAngler = new ShooterAngler(anglerMotor);
        feeder = new ShooterFeeder(feedServo);
        wobbleGoalArm = new WobbleGoalArm(arm, clawServo);

        // Gamepad
        driverGamepad = new GamepadEx(gamepad1);

        // Buttons/commands
        intakeButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)).whenPressed(new InstantCommand(intake::intake));
        outtakeButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.B)).whenPressed(new InstantCommand(intake::outtake));
        slowModeTrigger = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)).whenPressed(new SlowDriveCommand(drivetrain, driverGamepad));
        tripleShotButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)).whenPressed(new FeedRingsCommand(feeder, 3));
        singleShotButton = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)).whenPressed(new FeedRingsCommand(feeder, 1));

        liftArmButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)).whenHeld(new InstantCommand(wobbleGoalArm::liftArm));
        lowerArmButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)).whenHeld(new InstantCommand(wobbleGoalArm::lowerArm));

        // Make a button to toggle the claw
        toggleClawButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.A))
                .whenPressed(new InstantCommand(wobbleGoalArm::toggleClaw));
        // Toggles shooter from 300 RPM to stopped (default command)
        shootButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.Y))
                .toggleWhenPressed(new ShootRPMCommand(shooterWheels, 300));
        // Toggles angle
        angleToggleButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.X))
                .toggleWhenPressed(new ShooterAngleCommand(shooterAngler, 30, 0.75));


        intake.setDefaultCommand(new InstantCommand(intake::stop));
        wobbleGoalArm.setDefaultCommand(new InstantCommand(wobbleGoalArm::openClaw));
        shooterWheels.setDefaultCommand(new InstantCommand(shooterWheels::stopShooter));
        shooterAngler.setDefaultCommand(new ShooterAngleCommand(shooterAngler, 0, 0.75));
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad));
        register(drivetrain, shooterWheels, shooterAngler, feeder, intake, wobbleGoalArm);

    }

}