package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
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
import org.firstinspires.ftc.teamcode.commands.intake.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.intake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRPMCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRingsCommand;
import org.firstinspires.ftc.teamcode.commands.wobblegoal.MoveWobbleClaw;
import org.firstinspires.ftc.teamcode.commands.wobblegoal.MoveWobbleGoal;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
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
    private Shooter shooter;
    private Intake intake;
    private WobbleGoalArm wobbleGoalArm;

    // Commands

    // Buttons
    private Button intakeButton, outtakeButton, shootButton, liftArmButton, lowerArmButton, toggleClawButton, tripleShotButton;
    private Trigger slowModeTrigger;
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
        intake = new Intake(intakeMotor, feedServo);
        shooter = new Shooter(shooterMotorFront, shooterMotorBack, anglerMotor);
        wobbleGoalArm = new WobbleGoalArm(arm, clawServo);

        // Gamepad
        driverGamepad = new GamepadEx(gamepad1);

        // Buttons/commands
        intakeButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)).whenPressed(new IntakeCommand(intake));
        outtakeButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.B)).whenPressed(new OuttakeCommand(intake));
        // slowButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.)).whenPressed(new SlowDriveCommand(drivetrain, driverGamepad));
        shootButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.Y)).whenHeld(new ShootRPMCommand(shooter, 300));
        tripleShotButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)).whenPressed(new FeedRingsCommand(intake, 3));

        liftArmButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)).whenHeld(new MoveWobbleGoal(wobbleGoalArm, 1));
        lowerArmButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)).whenHeld(new MoveWobbleGoal(wobbleGoalArm, -1));

        toggleClawButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.A)).toggleWhenPressed(new MoveWobbleClaw(wobbleGoalArm, 0.5));

        shooter.setDefaultCommand(new ShootRPMCommand(shooter, 0));
        wobbleGoalArm.setDefaultCommand(new MoveWobbleClaw(wobbleGoalArm, 0));
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad));
        register(drivetrain, shooter, intake, wobbleGoalArm);

    }/*
    Right Trigger - Slow Mode
Left Trigger - Single Shot
Right Bumper - Intake
Left Bumper - Triple Shot
Y - Start/Stop Shooter
X - Switch Shooter Angles
A - Wobble Goal Claw Open/Close
B - Outtake
D-Pad Up - Wobble Goal Arm Up
    */

}