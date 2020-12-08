package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.commands.StartEndCommand;
import org.firstinspires.ftc.teamcode.commands.drive.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterAngler;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

import java.time.Instant;

@TeleOp(name = "TeleOp-test")
public class TeleopTest extends CommandOpMode {
    // Motors
    private MotorEx leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor;
    private MotorEx intakeMotor;
    private MotorEx anglerMotor;
    private DcMotorEx shooterMotorFront, shooterMotorBack;
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
    private Button slowModeTrigger, tripleShotButton, shootRingsButton, shootButton, anglerUpButton, anglerDownButton,toggleClawButton, liftArmButton, lowerArmButton;
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
        shooterMotorBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter_back");
        shooterMotorFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter_front");
        anglerMotor = new MotorEx(hardwareMap, "angler");
        feedServo = new SimpleServo(hardwareMap, "feed_servo");

        // Wobble Harware initializations
        arm = new CRServo(hardwareMap, "arm");
        clawServo = new SimpleServo(hardwareMap, "claw_servo");


        // Subsystems
        drivetrain = new Drivetrain(leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor, gyro, telemetry);
        // intake = new Intake(intakeMotor, telemetry);
        shooterWheels = new ShooterWheels(shooterMotorFront, shooterMotorBack, telemetry);
        shooterAngler = new ShooterAngler(anglerMotor, telemetry, true);
        feeder = new ShooterFeeder(feedServo, telemetry);
        wobbleGoalArm = new WobbleGoalArm(arm, clawServo, telemetry);

        driverGamepad = new GamepadEx(gamepad1);

        slowModeTrigger = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)).whileHeld(new SlowDriveCommand(drivetrain, driverGamepad));
        shootRingsButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.X)).whenPressed(new FeedRingsCommand(feeder, 1));
        tripleShotButton = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)).whenPressed(new FeedRingsCommand(feeder, 3));

        // intakeButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)).whileHeld(intake::intake).whenReleased(intake::stop);

        shootButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.Y)).toggleWhenPressed(new StartEndCommand(
                () -> {
                    shooterWheels.setShooterRPM(4500);
                },
                () -> {
                    shooterWheels.setShooterRPM(0);
                }, shooterWheels
        ));

        anglerUpButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)).whileHeld(() -> shooterAngler.setAngler(0.3)).whenReleased(() -> shooterAngler.setAngler(0));
        anglerDownButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)).whileHeld(() -> shooterAngler.setAngler(-0.3)).whenReleased(() -> shooterAngler.setAngler(0));
        toggleClawButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.A)).toggleWhenPressed(new StartEndCommand(
                wobbleGoalArm::openClaw,
                wobbleGoalArm::closeClaw,
                wobbleGoalArm
        ));
        // outtakeButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.B)).whileHeld(intake::outtake).whenReleased(intake::stop);

        liftArmButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)).whileHeld(wobbleGoalArm::liftArm).whenReleased(wobbleGoalArm::stopArm);
        lowerArmButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)).whileHeld(wobbleGoalArm::lowerArm).whenReleased(wobbleGoalArm::stopArm);

        // Gamepad
        schedule(new RunCommand(telemetry::update));
        /* schedule(new RunCommand(() -> {
            feeder.setFeedServo(driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        }, feeder));
        */
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad));
    }
}
