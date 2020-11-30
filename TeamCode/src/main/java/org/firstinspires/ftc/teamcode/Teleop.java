package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShootRPMCommand;
import org.firstinspires.ftc.teamcode.commands.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "TeleOp")
public class Teleop extends CommandOpMode {

    // Motors
    private MotorEx leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor;
    private MotorEx intakeMotor;
    private MotorEx shooterMotorFront, shooterMotorBack, anglerMotor;
    private ServoEx feedServo;

    // Gyro
    private GyroEx gyro;

    // Gamepad
    private GamepadEx driverGamepad;

    // Subsystems
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Intake intake;

    // Commands

    // Buttons
    private Button intakeButton, slowButton, shootFastButton;

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

        // Subsystems
        drivetrain = new Drivetrain(leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor, gyro);
        intake = new Intake(intakeMotor);
        shooter = new Shooter(shooterMotorFront, shooterMotorBack, anglerMotor, feedServo);

        // Gamepad
        driverGamepad = new GamepadEx(gamepad1);

        // Buttons/commands
        intakeButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.A)).whenPressed(new IntakeCommand(intake));
        slowButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)).whenPressed(new SlowDriveCommand(drivetrain, driverGamepad));
        shootFastButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.A)).whenHeld(new ShootRPMCommand(shooter, 300));

        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad));
        register(drivetrain, intake);
    }

}