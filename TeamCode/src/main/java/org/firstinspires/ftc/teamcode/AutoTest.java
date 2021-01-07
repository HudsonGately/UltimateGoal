package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.commands.auto.red.RedAuto;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

@Autonomous(name = "Auto-test")
public class AutoTest extends CommandOpMode {
    // Motors
    private MotorEx leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor;
    private MotorEx intakeMotor;
    private MotorEx anglerMotor;
    private DcMotorEx shooterMotorFront, shooterMotorBack;
    private CRServo arm;
    private ServoEx feedServo, clawServo;

    // Gamepad
    private GamepadEx driverGamepad;

    // Subsystems
    private Drivetrain drivetrain;
    private ShooterWheels shooterWheels;
    private ShooterAngler shooterAngler;
    private ShooterFeeder feeder;
    private Intake intake;
    private WobbleGoalArm wobbleGoalArm;
    private Vision vision;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void initialize() {

        // Drivetrain Hardware Initializations
        // Intake hardware Initializations
        intakeMotor = new MotorEx(hardwareMap, "intake");

        // Shooter hardware initializations
        shooterMotorBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter_back");
        shooterMotorFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter_front");
        anglerMotor = new MotorEx(hardwareMap, "angler");
        feedServo = new SimpleServo(hardwareMap, "feed_servo", 0, 230);

        // Wobble Harware initializations
        arm = new CRServo(hardwareMap, "arm");
        clawServo = new SimpleServo(hardwareMap, "claw_servo", 0, 230);


        // Subsystems
        drivetrain = new Drivetrain(new SampleTankDrive(hardwareMap),telemetry);
        // intake = new Intake(intakeMotor, telemetry);
        shooterWheels = new ShooterWheels(shooterMotorFront, shooterMotorBack, telemetry);
        shooterAngler = new ShooterAngler(anglerMotor, telemetry, true);
        feeder = new ShooterFeeder(feedServo, telemetry);
        wobbleGoalArm = new WobbleGoalArm(arm, clawServo, telemetry);
        vision = new Vision(hardwareMap, "webcam", telemetry);


        driverGamepad = new GamepadEx(gamepad1);

        // Gamepad
        schedule(new RunCommand(telemetry::update));
        schedule(new RedAuto(drivetrain, shooterWheels, shooterAngler, feeder, vision::getCurrentStack));
    }
}
