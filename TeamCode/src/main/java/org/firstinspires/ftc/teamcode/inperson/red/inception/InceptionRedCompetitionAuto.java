package org.firstinspires.ftc.teamcode.inperson.red.inception;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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
import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.inperson.blue.inception.InceptionBlueZeroCommand;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.pipelines.UGBasicHighGoalPipeline;
import org.firstinspires.ftc.teamcode.pipelines.UGDetector2;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.VisionStack;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

import java.util.HashMap;
import java.util.logging.Level;

@Autonomous(name = "Inception Competition Autonomous (Red)", group = "Red")
public class InceptionRedCompetitionAuto extends MatchOpMode {
    public static double startPoseX = -62.5;
    public static double startPoseY = 0;
    public static double startPoseHeading = 180;
    public static double RED_CAMERA_WIDTH = .02;
    // Motors
    private MotorEx leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor;
    private MotorEx intakeMotor;
    private DcMotorEx shooterMotorFront, shooterMotorBack;
    private MotorEx arm;
    private ServoEx feedServo, clawServo, lazySusanServo;
    private TouchSensor wobbleTouchSensor;
    private ServoEx intakeServo;

    // Gamepad
    private GamepadEx driverGamepad;

    // Subsystems
    private Drivetrain drivetrain;
    private ShooterWheels shooterWheels;
    private ShooterFeeder feeder;
    private Intake intake;
    private WobbleGoalArm wobbleGoalArm;
    private VisionStack visionStack;

    @Override
    public void robotInit() {
        // Drivetrain Hardware Initializations
        // Intake hardware Initializations
        intakeMotor = new MotorEx(hardwareMap, "intake");
        intakeServo = new SimpleServo(hardwareMap, "intake_wall_servo", 0, 180);

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
        intake = new Intake(intakeMotor, intakeServo, telemetry);
        shooterWheels = new ShooterWheels(shooterMotorFront, shooterMotorBack, telemetry);
        feeder = new ShooterFeeder(feedServo, telemetry);
        wobbleGoalArm = new WobbleGoalArm(arm, lazySusanServo, clawServo, wobbleTouchSensor, telemetry);
        drivetrain.setPoseEstimate(Trajectories.BlueLeftTape.startPose);
        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));
        visionStack = new VisionStack(hardwareMap, "webcam",  telemetry, 0.43, 0.56, RED_CAMERA_WIDTH);

    }

    @Override
    public void disabledPeriodic() {
        Util.logger(this, telemetry, Level.INFO, "Current Stack", visionStack.getCurrentStack());
    }

    @Override
    public void matchStart() {
        feeder.retractFeed();
        wobbleGoalArm.setOffset();
        schedule(
                new SelectCommand(new HashMap<Object, Command>() {{
                    put(UGDetector2.Stack.FOUR, new SequentialCommandGroup(
                            new InceptionLeftRedFourCommand(drivetrain, shooterWheels, feeder, intake, wobbleGoalArm, telemetry)
                    ));
                    put(UGDetector2.Stack.ONE, new SequentialCommandGroup(
                            new InceptionLeftRedOneCommand(drivetrain, shooterWheels, feeder, intake, wobbleGoalArm, telemetry)
                    ));
                    put(UGDetector2.Stack.ZERO, new SequentialCommandGroup(
                            new InceptionBlueZeroCommand(drivetrain, shooterWheels, feeder, intake, wobbleGoalArm, telemetry)
                    ));
                }}, visionStack::getCurrentStack)
        );

    }
}
